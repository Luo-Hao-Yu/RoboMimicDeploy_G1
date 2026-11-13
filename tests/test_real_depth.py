"""Offline tests for real depth acquisition contracts; never open hardware."""
import io
import json
import math
from pathlib import Path
import subprocess
import sys
import time
from types import SimpleNamespace
import unittest
from unittest import mock

import cv2
import numpy as np

from deploy_real.depth_source import camera_maps, create_depth_source, preprocess_depth
from deploy_real.ssh_depth import RemoteDepth, RemoteCameraStartupError, SSHDepth, calibration_maps

ROOT = Path(__file__).resolve().parents[1]
CAMERA = dict(source='ssh', ssh_host='unitree@192.168.123.164', remote_python='python3',
              serial='233722073805', width=848, height=480, fps=60,
              virtual_width=64, virtual_height=36, virtual_fovy_deg=58.29,
              stream_timeout_s=.15)
DEPTH = dict(render_height=36, render_width=64, crop_region=[18, 0, 16, 16],
             range=[0., 2.5], min_distance=.1, blur_sigma=1.)

# A separate local process speaks the same framing protocol without SSH/camera.
WORKER = r'''
import json, math, struct, sys, time, zlib
sys.path.insert(0, sys.argv[1])
from deploy_real.camera_stream import send_calibration, write_frame, FRAME, MAGIC
settings = json.loads(sys.stdin.buffer.readline())
focal = 36 / (2 * math.tan(math.radians(58.29)/2))
calibration = dict(protocol='G1_DEPTH_1',width=64,height=36,fx=focal,fy=focal,
                   ppx=31.5,ppy=17.5,scale=.001,format='z16',rectified=True)
mode=sys.argv[2]
if mode=='busy':
    send_calibration(sys.stdout.buffer,dict(protocol='G1_DEPTH_1',error_kind='device_busy',error='Device or resource busy'))
    sys.exit(1)
send_calibration(sys.stdout.buffer,calibration)
number=0
while sys.stdin.buffer.read(1)==b'F':
    if mode=='eof':break
    if mode!='repeat' or number==0:number+=1
    if mode=='regress' and number==2:number=0
    if mode=='slow':time.sleep(.2)
    if mode in ('corrupt','oversized'):
        payload=b'!' if mode=='corrupt' else zlib.compress(b'\0'*(64*36*2+1))
        sys.stdout.buffer.write(FRAME.pack(MAGIC,number,0,len(payload))+payload)
        sys.stdout.buffer.flush()
    else:
        age=.5 if mode=='stale' else .01
        write_frame(sys.stdout.buffer,number,time.monotonic_ns()-int(age*1e9),struct.pack('<H',1500)*(64*36))
'''


class DepthPreprocessingTests(unittest.TestCase):
    def test_simulation_equivalence_for_invalid_near_and_far_pixels(self):
        from deploy_mujoco.deploy_mujoco_env import _cropped_normalized_depth_image
        image = np.random.default_rng(4).uniform(0, 4, (36, 64)).astype(np.float32)
        image[18:23, 16:22] = np.array([np.nan, np.inf, -np.inf, 0., .05, .1], np.float32)
        renderer = mock.Mock()
        renderer.render.return_value = image
        expected = _cropped_normalized_depth_image(
            renderer, object(), 'depth_cam', crop_region=tuple(DEPTH['crop_region']),
            depth_range=tuple(DEPTH['range']), blur_sigma=1., min_distance=.1)
        actual = preprocess_depth(image, DEPTH)
        np.testing.assert_array_equal(actual, expected)
        self.assertEqual(actual.shape, (18, 32))
        self.assertEqual(actual.dtype, np.float32)
        self.assertTrue(np.isfinite(actual).all())

    def test_invalid_and_near_pixels_become_far_not_inpainted(self):
        cfg = dict(DEPTH, blur_sigma=0)
        for value in (0., .09, np.nan, np.inf, -1.):
            with self.subTest(value=value):
                np.testing.assert_array_equal(preprocess_depth(np.full((36, 64), value), cfg), 1.)
        np.testing.assert_allclose(preprocess_depth(np.full((36, 64), .1), cfg), .04)
        np.testing.assert_array_equal(preprocess_depth(np.full((36, 64), 5.), cfg), 1.)

    def test_shape_and_configuration_fail_before_processing(self):
        for shape in ((270, 480), (18, 32), (36, 64, 1)):
            with self.subTest(shape=shape), self.assertRaises(ValueError):
                preprocess_depth(np.ones(shape), DEPTH)
        for change in ({'crop_region':[36,0,0,0]}, {'range':[2.5,0]}, {'blur_sigma':float('nan')}):
            with self.subTest(change=change), self.assertRaises(ValueError):
                preprocess_depth(np.ones((36,64)),dict(DEPTH,**change))

    def test_pinhole_reprojection_preserves_optical_z(self):
        focal = 36 / (2 * math.tan(math.radians(58.29)/2))
        intr = SimpleNamespace(fx=focal, fy=focal, ppx=31.5, ppy=17.5)
        x,y = camera_maps(intr,CAMERA)
        yy,xx=np.mgrid[:36,:64]
        np.testing.assert_allclose(x,xx,atol=3e-6)
        np.testing.assert_allclose(y,yy,atol=3e-6)
        image=np.full((36,64),1.5,np.float32)
        np.testing.assert_array_equal(cv2.remap(image,x,y,cv2.INTER_NEAREST),image)

    def test_invalid_source_configuration_does_not_touch_hardware(self):
        changes = ({'source':'udp'},{'virtual_width':480},{'width':0},{'fps':0},
                   {'fps':120},{'stream_timeout_s':0},{'virtual_fovy_deg':180})
        with mock.patch('deploy_real.depth_source.RealSense') as camera:
            for change in changes:
                with self.subTest(change=change),self.assertRaises(ValueError):
                    create_depth_source(dict(CAMERA,**change))
            camera.assert_not_called()


class RemoteDepthTests(unittest.TestCase):
    def setUp(self):
        self.camera=None
        self.process=None

    def tearDown(self):
        if self.camera:self.camera.close()
        if self.process and self.process.poll() is None:
            self.process.terminate()
            self.process.wait(timeout=2)
        if self.process:
            for stream in (self.process.stdin, self.process.stdout, self.process.stderr):
                if stream: stream.close()

    def start(self,mode):
        self.process=subprocess.Popen([sys.executable,'-u','-c',WORKER,str(ROOT),mode],
                                      stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=subprocess.PIPE,bufsize=0)
        self.camera=RemoteDepth(CAMERA,self.process)
        return self.camera

    def until(self,callback):
        end=time.monotonic()+2
        while time.monotonic()<end:
            result=callback()
            if result:return result
            time.sleep(.005)
        self.fail('No mock camera update')

    def test_lossless_z16_metres_and_stable_age_on_repeated_read(self):
        camera=self.start('normal')
        stamp,image=self.until(camera.read)
        self.assertEqual(image.shape,(36,64))
        np.testing.assert_allclose(image,1.5,atol=2e-7)
        self.assertGreater(stamp,0)
        self.assertGreaterEqual(time.monotonic_ns()-stamp,int(.01*1e9))
        # Read snapshots under the capture lock; no getter fabricates freshness.
        with camera.lock:
            stamp_before=camera.latest[0]
        for _ in range(10):
            with camera.lock:
                previous=camera.latest
            observed=camera.read()
            if observed is previous:
                self.assertEqual(observed[0],previous[0])
        camera.close()
        self.assertIsNotNone(self.process.poll())
        self.assertFalse(camera.thread.is_alive())

    def test_invalid_repeated_regressed_stale_corrupt_and_oversize_frames_fail(self):
        for mode in ('repeat','regress','stale','slow','corrupt','oversized','eof'):
            with self.subTest(mode=mode):
                camera=self.start(mode)
                try:
                    # First frame can be valid; wait for the second invalid packet.
                    def wait_error():
                        camera.read()
                        return False
                    with self.assertRaises(RuntimeError):self.until(wait_error)
                finally:
                    camera.close()
                    if self.process.stderr:self.process.stderr.close()
                    self.camera=None

    def test_camera_busy_is_reported_without_stopping_owner(self):
        self.process=subprocess.Popen([sys.executable,'-u','-c',WORKER,str(ROOT),'busy'],
                                      stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=subprocess.PIPE,bufsize=0)
        with self.assertRaises(RemoteCameraStartupError) as raised:
            RemoteDepth(CAMERA,self.process)
        self.assertEqual(raised.exception.kind,'device_busy')
        self.assertIn('不会停止相机占用者',str(raised.exception))
        self.assertIsNotNone(self.process.poll())

    def test_calibration_rejects_wrong_units_geometry_or_unrectified_stream(self):
        valid=dict(protocol='G1_DEPTH_1',format='z16',rectified=True,width=64,height=36,
                   fx=30.,fy=30.,ppx=31.5,ppy=17.5,scale=.001)
        for change in ({'rectified':False},{'scale':0},{'scale':1},{'fx':float('nan')},{'width':0}):
            with self.subTest(change=change),self.assertRaises(ValueError):
                calibration_maps(dict(valid,**change),CAMERA)

    def test_ssh_validates_host_and_requires_known_host_verification(self):
        with mock.patch('deploy_real.ssh_depth.subprocess.Popen') as process:
            with self.assertRaises(ValueError):SSHDepth(dict(CAMERA,ssh_host='-oBad'))
            process.assert_not_called()
        with mock.patch.object(RemoteDepth,'__init__',return_value=None), \
             mock.patch('deploy_real.ssh_depth.subprocess.Popen') as process:
            SSHDepth(dict(CAMERA,identity_file='/tmp/test_key',known_hosts='/tmp/test_known_hosts'))
            args=process.call_args.args[0]
            self.assertIn('StrictHostKeyChecking=yes',args)
            self.assertIn('BatchMode=yes',args)
            self.assertIn('/tmp/test_key',args)
            self.assertIn('UserKnownHostsFile=/tmp/test_known_hosts',args)


if __name__=='__main__':unittest.main()
