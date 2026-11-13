"""Read robot-attached RealSense frames over SSH while inference stays local."""
import json
import math
import os
from pathlib import Path
import re
import select
import shlex
import subprocess
import threading
import time
from types import SimpleNamespace
import zlib

import cv2
import numpy as np

from .camera_stream import FRAME, LENGTH, MAGIC, MAX_PAYLOAD, MAX_PIXELS
from .depth_source import camera_maps


def read_exact(stream, length, timeout, stopped):
    deadline = time.monotonic() + timeout
    output = bytearray()
    while len(output) < length:
        if stopped.is_set():
            raise InterruptedError('Depth stream stopped')
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            raise TimeoutError('SSH camera response timed out')
        if not select.select([stream], [], [], min(remaining, .1))[0]:
            continue
        block = os.read(stream.fileno(), length - len(output))
        if not block:
            raise ConnectionError('SSH camera stream closed; inspect SSH/camera errors above')
        output.extend(block)
    return bytes(output)


def calibration_maps(calibration, cfg):
    c = calibration
    if c.get('protocol') != 'G1_DEPTH_1' or c.get('format') != 'z16' or c.get('rectified') is not True:
        raise ValueError('Unsupported remote camera protocol/calibration')
    w, h = c['width'], c['height']
    if not isinstance(w, int) or not isinstance(h, int) or min(w, h) <= 0 or w * h > MAX_PIXELS:
        raise ValueError('Invalid remote depth dimensions')
    for k in ('fx', 'fy', 'ppx', 'ppy', 'scale'):
        if not math.isfinite(c[k]):
            raise ValueError('Non-finite camera calibration')
    if min(c['fx'], c['fy'], c['scale']) <= 0 or c['scale'] > .1:
        raise ValueError('Invalid camera focal length/scale')
    return camera_maps(SimpleNamespace(**c), cfg)


class RemoteCameraStartupError(ConnectionError):
    """The SSH command ran, but its camera failed before stream initialization."""
    def __init__(self, kind, remote_error):
        self.kind = kind
        self.remote_error = remote_error
        if kind == 'device_busy':
            message = ('SSH 已连通，但远端 RealSense 相机被占用。请确认并正常停止占用相机的进程，'
                       '再重试；无需重装 SSH 密钥。本程序不会停止相机占用者。远端原始错误：' + remote_error)
        else:
            message = 'SSH 远端命令已执行，但相机初始化失败。远端原始错误：' + remote_error
        super().__init__(message)


class RemoteDepth:
    def __init__(self, cfg, process):
        self.process = process
        self.cfg = cfg
        self.lock = threading.Lock()
        self.latest = None
        self.error = None
        self.stop_event = threading.Event()
        self.thread = None
        try:
            settings = {k: cfg[k] for k in ('serial', 'width', 'height', 'fps')}
            process.stdin.write(json.dumps(settings).encode() + b'\n')
            process.stdin.flush()
            length = LENGTH.unpack(read_exact(process.stdout, LENGTH.size, 15, self.stop_event))[0]
            if not 0 < length <= 4096:
                raise ValueError('Invalid SSH camera handshake')
            self.calibration = json.loads(read_exact(process.stdout, length, 2, self.stop_event))
            if isinstance(self.calibration, dict) and 'error_kind' in self.calibration:
                if (self.calibration.get('protocol') != 'G1_DEPTH_1' or
                    not isinstance(self.calibration['error_kind'], str) or
                    not isinstance(self.calibration.get('error'), str)):
                    raise ValueError('Invalid remote camera startup error')
                raise RemoteCameraStartupError(self.calibration['error_kind'], self.calibration['error'])
            self.maps = calibration_maps(self.calibration, cfg)
            print('Remote RealSense calibration: %s' % self.calibration, flush=True)
            self.thread = threading.Thread(target=self._capture, daemon=True)
            self.thread.start()
        except BaseException:
            self.close()
            raise

    def _capture(self):
        last_number = None
        try:
            while not self.stop_event.is_set():
                start = time.monotonic()
                request_ns = time.monotonic_ns()
                self.process.stdin.write(b'F')
                self.process.stdin.flush()
                raw_header = read_exact(self.process.stdout, FRAME.size, 2, self.stop_event)
                magic, number, remote_age_ns, size = FRAME.unpack(raw_header)
                if (magic != MAGIC or (last_number is not None and number <= last_number)
                        or not 0 < size <= MAX_PAYLOAD):
                    raise ValueError('Invalid/repeated remote depth frame')
                payload = read_exact(self.process.stdout, size, 2, self.stop_event)
                # Bound decompression before allocating an image.
                c = self.calibration
                expected = c['width'] * c['height'] * 2
                decompressor = zlib.decompressobj()
                pixels = decompressor.decompress(payload, expected + 1)
                if len(pixels) != expected or not decompressor.eof or decompressor.unused_data or decompressor.unconsumed_tail:
                    raise ValueError('Invalid compressed remote depth image')
                # Remote and local monotonic epochs are unrelated. Conservatively
                # include the full request/response time plus sender capture age.
                stamp_ns = request_ns - remote_age_ns
                if stamp_ns <= 0 or (time.monotonic_ns() - stamp_ns) * 1e-9 > self.cfg['stream_timeout_s']:
                    raise TimeoutError('Remote depth exceeded the configured age limit')
                raw = np.frombuffer(pixels, dtype='<u2').reshape(c['height'], c['width'])
                metres = raw.astype(np.float32) * c['scale']
                image = cv2.remap(metres, *self.maps, interpolation=cv2.INTER_NEAREST,
                                  borderMode=cv2.BORDER_CONSTANT, borderValue=0)
                last_number = number
                with self.lock:
                    self.latest = (stamp_ns, image)
                self.stop_event.wait(max(0., .02 - (time.monotonic() - start)))
        except BaseException as exc:
            if not self.stop_event.is_set():
                with self.lock:
                    self.error = exc

    def read(self):
        with self.lock:
            if self.error:
                raise RuntimeError('Remote camera failed: %s' % self.error) from self.error
            return self.latest

    def close(self):
        self.stop_event.set()
        if self.thread and self.thread is not threading.current_thread():
            self.thread.join(timeout=.3)
        # EOF lets the camera-only worker leave its request loop and stop the
        # RealSense pipeline normally. Terminate SSH only if graceful exit stalls.
        if self.process.stdin:
            try:
                self.process.stdin.close()
            except (OSError, ValueError):
                pass
        try:
            self.process.wait(timeout=3.5)
        except subprocess.TimeoutExpired:
            if self.process.poll() is None:
                self.process.terminate()
            try:
                self.process.wait(timeout=2)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.wait()
        for stream in (self.process.stdin, self.process.stdout):
            if stream:
                try:
                    stream.close()
                except (OSError, ValueError):
                    pass
        if self.thread and self.thread is not threading.current_thread() and self.thread.is_alive():
            self.thread.join(timeout=.3)


class SSHDepth(RemoteDepth):
    def __init__(self, cfg):
        host = cfg['ssh_host']
        if not re.fullmatch(r'[A-Za-z0-9_.@:-]+', host) or host.startswith('-'):
            raise ValueError('Invalid camera SSH host')
        root = Path(__file__).resolve().parents[1]
        key_dir = root / '.g1-ssh'
        args = ['ssh', '-T', '-o', 'BatchMode=yes', '-o', 'StrictHostKeyChecking=yes',
                '-o', 'ConnectTimeout=5', '-o', 'ServerAliveInterval=2', '-o', 'ServerAliveCountMax=2']
        if cfg.get('identity_file'):
            args += ['-i', str(Path(cfg['identity_file']).expanduser()), '-o', 'IdentitiesOnly=yes']
        elif (key_dir / 'id_ed25519').exists():
            args += ['-i', str(key_dir / 'id_ed25519'), '-o', 'IdentitiesOnly=yes',
                     '-o', 'UserKnownHostsFile=' + str(key_dir / 'known_hosts')]
        if cfg.get('known_hosts'):
            args += ['-o', 'UserKnownHostsFile=' + str(Path(cfg['known_hosts']).expanduser())]
        source = (Path(__file__).parent / 'camera_stream.py').read_text()
        command = shlex.quote(cfg['remote_python']) + ' -u -c ' + shlex.quote(source)
        print('Starting camera-only worker on %s; ONNX and motor bridge stay on this computer.' % host, flush=True)
        process = subprocess.Popen(args + [host, command], stdin=subprocess.PIPE,
                                   stdout=subprocess.PIPE, bufsize=0)
        try:
            super().__init__(cfg, process)
        except RemoteCameraStartupError:
            raise
        except (ConnectionError, TimeoutError) as exc:
            raise ConnectionError('SSH 深度会话未建立或已中断（%s）：%s。可能是 SSH 认证/网络故障，'
                                  '也可能是远端相机进程在返回标定前退出；请优先查看上方具体报错。' %
                                  (host, exc)) from exc
