"""Camera-only SSH worker. No model, DDS, OpenCV or robot control dependencies.

The controller sends one F request per response, so a slow network cannot build
an unbounded frame queue. Pixels are losslessly compressed optical-Z Z16.
"""
import json
import struct
import sys
import threading
import time
import zlib

LENGTH = struct.Struct('<I')
FRAME = struct.Struct('<4sQQI')  # magic, frame id, capture age ns, compressed size
MAGIC = b'G1D1'
MAX_PIXELS = 1280 * 720
MAX_PAYLOAD = MAX_PIXELS * 2 + 4096


def send_calibration(output, calibration):
    data = json.dumps(calibration).encode('utf-8')
    output.write(LENGTH.pack(len(data)))
    output.write(data)
    output.flush()


def write_frame(output, number, stamp_ns, pixels):
    payload = zlib.compress(pixels, level=1)
    age = max(0, time.monotonic_ns() - stamp_ns)
    output.write(FRAME.pack(MAGIC, number, age, len(payload)))
    output.write(payload)
    output.flush()


def serve(settings, requests, output):
    pipeline = None
    stopped = threading.Event()
    condition = threading.Condition()
    shared = dict(frame=None, error=None)
    thread = None
    calibration_sent = False
    failure = None
    try:
        import numpy as np
        import pyrealsense2 as rs
        width, height, fps = (int(settings[k]) for k in ('width', 'height', 'fps'))
        if not 0 < width * height <= MAX_PIXELS or min(width, height, fps) <= 0 or fps > 90:
            raise ValueError('Unsupported camera stream dimensions/rate')
        pipeline = rs.pipeline()
        config = rs.config()
        if settings.get('serial'):
            config.enable_device(settings['serial'])
        config.enable_stream(rs.stream.depth, width, height, rs.format.z16, fps)
        profile = pipeline.start(config)
        scale = profile.get_device().first_depth_sensor().get_depth_scale()
        intrinsic = profile.get_stream(rs.stream.depth).as_video_stream_profile().get_intrinsics()
        if intrinsic.model not in (rs.distortion.none, rs.distortion.brown_conrady) or any(abs(c) > 1e-7 for c in intrinsic.coeffs):
            raise ValueError('RealSense depth must be rectified')
        calibration = dict(protocol='G1_DEPTH_1', width=intrinsic.width, height=intrinsic.height,
                           fx=intrinsic.fx, fy=intrinsic.fy, ppx=intrinsic.ppx, ppy=intrinsic.ppy,
                           scale=scale, format='z16', rectified=True)
        def capture():
            previous = None
            try:
                while not stopped.is_set():
                    frame = pipeline.wait_for_frames(1000).get_depth_frame()
                    if not frame or frame.get_frame_number() == previous:
                        continue
                    previous = frame.get_frame_number()
                    stamp = time.monotonic_ns()
                    pixels = np.asarray(frame.get_data(), dtype='<u2').tobytes()
                    with condition:
                        shared['frame'] = (previous, stamp, pixels)
                        condition.notify_all()
            except BaseException as exc:
                with condition:
                    shared['error'] = exc
                    condition.notify_all()
        thread = threading.Thread(target=capture, daemon=True)
        send_calibration(output, calibration)
        calibration_sent = True
        thread.start()
        last_sent = None
        while requests.read(1) == b'F':
            with condition:
                ready = condition.wait_for(lambda: shared['error'] is not None or
                    (shared['frame'] is not None and shared['frame'][0] != last_sent), timeout=2)
                if shared['error']:
                    raise RuntimeError('Camera capture failed') from shared['error']
                if not ready:
                    raise TimeoutError('Camera stopped producing new frames')
                number, stamp, pixels = shared['frame']
            write_frame(output, number, stamp, pixels)
            last_sent = number
    except BaseException as exc:
        failure = exc
        if not calibration_sent and isinstance(exc, Exception):
            message = str(exc)
            kind = 'device_busy' if ('device or resource busy' in message.lower() or getattr(exc, 'errno', None) == 16) else 'camera_startup'
            # Reuse the length-prefixed startup JSON; never insert it into frame data.
            # Bound escaping expansion so the client's 4096-byte handshake limit holds.
            try:
                send_calibration(output, dict(protocol='G1_DEPTH_1', error_kind=kind, error=message[:320]))
            except (OSError, ValueError):
                pass
        raise
    finally:
        stopped.set()
        if thread and thread.is_alive():
            thread.join(timeout=1.2)
        if pipeline is not None:
            try:
                pipeline.stop()
            except Exception:
                if failure is None:
                    raise


def main():
    line = sys.stdin.buffer.readline(4097)
    if len(line) > 4096 or not line.endswith(b'\n'):
        raise ValueError('Missing/bad camera settings')
    serve(json.loads(line), sys.stdin.buffer, sys.stdout.buffer)


if __name__ == '__main__':
    main()
