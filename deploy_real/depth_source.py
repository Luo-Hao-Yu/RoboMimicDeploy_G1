"""RealSense/SSH optical-Z depth acquisition and sim-matched preprocessing.

Camera imports are lazy. Importing this module never opens hardware or SSH.
"""
from __future__ import annotations

import math
import threading
import time

import cv2
import numpy as np

def camera_maps(intrinsics, cfg):
    """Resample the camera's pinhole rays to the reference virtual FOV/aspect."""
    w, h = cfg['virtual_width'], cfg['virtual_height']
    fovy = float(cfg['virtual_fovy_deg'])
    if (not isinstance(w, int) or not isinstance(h, int) or min(w, h) <= 0
            or not math.isfinite(fovy) or not 1 < fovy < 170):
        raise ValueError('Invalid reference camera geometry')
    if not np.isfinite([intrinsics.fx, intrinsics.fy, intrinsics.ppx, intrinsics.ppy]).all() or min(intrinsics.fx, intrinsics.fy) <= 0:
        raise ValueError('Invalid camera intrinsics')
    focal = h / (2 * math.tan(math.radians(fovy) / 2))
    yy, xx = np.mgrid[:h, :w].astype(np.float32)
    x = (xx - (w - 1) / 2) * intrinsics.fx / focal + intrinsics.ppx
    y = (yy - (h - 1) / 2) * intrinsics.fy / focal + intrinsics.ppy
    return x.astype(np.float32), y.astype(np.float32)


class RealSense:
    def __init__(self, cfg):
        # Keep hardware imports optional for offline tests and model checks.
        import pyrealsense2 as rs
        self.pipeline = rs.pipeline()
        try:
            settings = rs.config()
            if cfg.get('serial'):
                settings.enable_device(cfg['serial'])
            settings.enable_stream(rs.stream.depth, cfg['width'], cfg['height'], rs.format.z16, cfg['fps'])
            profile = self.pipeline.start(settings)
            self.scale = profile.get_device().first_depth_sensor().get_depth_scale()
            if not math.isfinite(self.scale) or not 0 < self.scale <= .1:
                raise ValueError('Invalid RealSense depth scale')
            stream = profile.get_stream(rs.stream.depth).as_video_stream_profile()
            intrinsic = stream.get_intrinsics()
            # D435 depth is rectified; non-pinhole calibration needs explicit correction.
            if intrinsic.model not in (rs.distortion.none, rs.distortion.brown_conrady) or any(abs(c) > 1e-7 for c in intrinsic.coeffs):
                raise ValueError('Depth stream is not rectified; rectify before policy preprocessing')
            self.maps = camera_maps(intrinsic, cfg)
            print('RealSense depth scale=%g m/unit, intrinsics=%s' % (self.scale, intrinsic), flush=True)
        except BaseException:
            try:
                self.pipeline.stop()
            except Exception:
                pass  # Cleanup must not replace the camera's original startup error.
            raise
        self.lock = threading.Lock()
        self.latest = None
        self.error = None
        self.stop_event = threading.Event()
        self.thread = threading.Thread(target=self._capture, daemon=True)
        self.thread.start()

    def _capture(self):
        last_number = None
        try:
            while not self.stop_event.is_set():
                frame = self.pipeline.wait_for_frames(1000).get_depth_frame()
                if not frame:
                    continue
                number = frame.get_frame_number()
                if last_number is not None and number <= last_number:
                    raise RuntimeError('RealSense frame number stopped advancing')
                last_number = number
                stamp = time.monotonic_ns()
                raw = np.asanyarray(frame.get_data()).astype(np.float32) * self.scale
                image = cv2.remap(raw, *self.maps, interpolation=cv2.INTER_NEAREST,
                                  borderMode=cv2.BORDER_CONSTANT, borderValue=0)
                with self.lock:
                    self.latest = (stamp, image)
        except BaseException as exc:
            with self.lock:
                self.error = exc

    def read(self):
        with self.lock:
            if self.error:
                raise RuntimeError('RealSense capture failed') from self.error
            return self.latest

    def close(self):
        self.stop_event.set()
        self.thread.join(timeout=1.2)
        self.pipeline.stop()


def preprocess_depth(metres, depth_cfg):
    """Apply exactly the configured simulation depth transform to metric Z.

    Input is already reprojected to the virtual 36x64 pinhole camera; no inpaint,
    random delay, or implicit 480x270 image resizing is performed here.
    """
    image = np.asarray(metres, dtype=np.float32)
    height, width = int(depth_cfg['render_height']), int(depth_cfg['render_width'])
    if image.shape != (height, width):
        raise ValueError('Expected virtual metric depth shape %s, got %s' % ((height, width), image.shape))
    near, far = (float(x) for x in depth_cfg['range'])
    min_distance = float(depth_cfg.get('min_distance', 0.0))
    sigma = float(depth_cfg.get('blur_sigma', 1.0))
    crop = depth_cfg['crop_region']
    if (len(crop) != 4 or any(not isinstance(x, int) or isinstance(x, bool) or x < 0 for x in crop)
            or not np.isfinite([near, far, min_distance, sigma]).all()
            or not 0 <= near < far or min_distance < 0 or sigma < 0):
        raise ValueError('Invalid depth preprocessing configuration')
    top, bottom, left, right = crop
    if top + bottom >= height or left + right >= width:
        raise ValueError('Depth crop removes the complete image')
    image = np.where(np.isfinite(image), image, far)
    if min_distance > 0:
        image = np.where(image < min_distance, far, image)
    image = image[top:height-bottom, left:width-right]
    if depth_cfg.get('resize_shape') is not None:
        target_h, target_w = depth_cfg['resize_shape']
        if min(target_h, target_w) <= 0:
            raise ValueError('Invalid depth resize shape')
        image = cv2.resize(image, (target_w, target_h), interpolation=cv2.INTER_AREA)
    if sigma > 0:
        image = cv2.GaussianBlur(image, (3, 3), sigmaX=sigma, sigmaY=sigma,
                                 borderType=cv2.BORDER_REFLECT_101)
    return ((np.clip(image, near, far) - near) / (far - near)).astype(np.float32)


def create_depth_source(camera_cfg):
    """Open an explicitly requested local or SSH camera with metric-Z output.

    ``read()`` returns None before the first frame, otherwise a stable
    ``(monotonic_stamp_ns, metres[virtual_height, virtual_width])`` pair.
    Re-reading a frame does not refresh its stamp. For SSH the stamp includes a
    conservative request/response bound plus the remote capture age.
    """
    cfg = dict(camera_cfg)
    source = cfg.get('source', 'ssh')
    if source not in ('ssh', 'realsense'):
        raise ValueError('Unknown depth source: ' + str(source))
    for key in ('width', 'height', 'fps', 'virtual_width', 'virtual_height'):
        if not isinstance(cfg.get(key), int) or isinstance(cfg[key], bool) or cfg[key] <= 0:
            raise ValueError('camera.%s must be a positive integer' % key)
    if cfg['virtual_width'] != 64 or cfg['virtual_height'] != 36:
        raise ValueError('Parkour virtual depth camera must be 64x36')
    if cfg['width'] * cfg['height'] > 1280 * 720 or cfg['fps'] > 90:
        raise ValueError('Unsupported depth stream dimensions/rate')
    for key in ('virtual_fovy_deg', 'stream_timeout_s'):
        if not math.isfinite(float(cfg.get(key, 0))) or float(cfg.get(key, 0)) <= 0:
            raise ValueError('camera.%s must be finite and positive' % key)
    if not 1 < float(cfg['virtual_fovy_deg']) < 170:
        raise ValueError('Invalid virtual camera field of view')
    cfg.setdefault('serial', '')
    cfg.setdefault('remote_python', 'python3')
    if source == 'realsense':
        return RealSense(cfg)
    from .ssh_depth import SSHDepth
    return SSHDepth(cfg)
