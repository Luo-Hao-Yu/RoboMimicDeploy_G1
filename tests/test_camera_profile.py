import unittest
from unittest.mock import Mock
from types import SimpleNamespace
from deploy_real.camera_stream import check_depth_profile, CameraProfileError
from deploy_real.ssh_depth import RemoteCameraStartupError

class ProfileTests(unittest.TestCase):
    def sdk(self, profiles, usb='2.1'):
        rs=Mock()
        rs.camera_info=SimpleNamespace(serial_number='serial',usb_type_descriptor='usb')
        rs.stream.depth='depth';rs.format.z16='z16'
        device=Mock();device.supports.return_value=True
        device.get_info.side_effect=lambda key: {'serial':'233722073805','usb':usb}[key]
        rs.context.return_value.query_devices.return_value=[device]
        result=[]
        for w,h,fps in profiles:
            p=Mock();p.stream_type.return_value='depth';p.format.return_value='z16'
            v=p.as_video_stream_profile.return_value
            v.width.return_value=w;v.height.return_value=h;v.fps.return_value=fps
            result.append(p)
        sensor=Mock();sensor.get_stream_profiles.return_value=result
        device.query_sensors.return_value=[sensor]
        return rs

    def test_usb2_incompatible_profile_reports_usb_and_available_fps(self):
        rs=self.sdk([(848,480,6),(848,480,10),(480,270,60)])
        with self.assertRaises(CameraProfileError) as error:
            check_depth_profile(rs,dict(serial='233722073805',width=848,height=480,fps=60))
        self.assertIn('USB=2.1',str(error.exception));self.assertIn('[6, 10]',str(error.exception))
        rs.pipeline.assert_not_called()

    def test_supported_profile_is_kept_without_fallback(self):
        rs=self.sdk([(848,480,60)],usb='3.2')
        self.assertEqual(check_depth_profile(rs,dict(serial='233722073805',width=848,height=480,fps=60)),'3.2')
        rs.pipeline.assert_not_called()

    def test_wrong_serial_rejected(self):
        with self.assertRaisesRegex(RuntimeError,'found 0'):
            check_depth_profile(self.sdk([]),dict(serial='wrong',width=848,height=480,fps=60))

    def test_remote_error_identifies_profile_not_ssh(self):
        error=RemoteCameraStartupError('unsupported_profile','USB=2.1 requested 848x480@60')
        self.assertIn('USB',str(error));self.assertEqual(error.kind,'unsupported_profile')

    def test_camera_start_failure_overwrites_stale_pass_report(self):
        import json,tempfile
        from pathlib import Path
        from unittest.mock import patch
        from deploy_real.deploy_real import camera_check
        from deploy_real.real_config import load_config
        with tempfile.TemporaryDirectory() as directory:
            report=Path(directory)/'camera.json'
            report.write_text('{"camera_check":"PASS"}')
            error=RemoteCameraStartupError('unsupported_profile','USB=2.1')
            with patch('deploy_real.deploy_real.create_depth_source',side_effect=error):
                with self.assertRaises(RemoteCameraStartupError):
                    camera_check(Mock(),load_config(),5,report)
            actual=json.loads(report.read_text())
            self.assertEqual(actual['camera_check'],'FAIL')
            self.assertEqual(actual['error_kind'],'unsupported_profile')
