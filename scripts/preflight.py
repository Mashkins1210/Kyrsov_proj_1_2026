"""Read-only wheel diagnostics; lidar scanning and cameras; never arm()."""
import argparse
import json
from pathlib import Path
import sys
import time
import threading
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from src.hardware.power_status import read_status


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', default='data/preflight')
    parser.add_argument('--config', default='config/robot.json')
    args = parser.parse_args()
    root = Path(args.output); root.mkdir(parents=True, exist_ok=True)
    resources, result = [], {}
    power_samples=[]; power_stop=threading.Event()
    def monitor_power():
        while not power_stop.is_set():
            try:power_samples.append({'time':time.time(),**read_status()})
            except Exception as exc:power_samples.append({'time':time.time(),'error':str(exc),'current_fault':True})
            power_stop.wait(.5)
    power_thread=threading.Thread(target=monitor_power,daemon=True);power_thread.start()
    try:
        from src.hardware.motor_controller import MotorController
        motor = MotorController(); resources.append(motor)
        result['motors'] = motor.inspect()
        from src.perception.camera_controller import CameraController
        settings = json.loads(Path(args.config).read_text(encoding='utf-8'))
        camera = CameraController(devices=settings.get('camera_devices')); resources.append(camera)
        deadline = time.monotonic()+15
        while len(camera.get_frames()) < 3 and time.monotonic() < deadline:
            time.sleep(0.1)
        frames = camera.get_frames()
        import cv2
        result['cameras'] = [{'index': i, 'device': device, 'frame_available': i in frames,
                              'capture_mode':camera.capture_modes.get(i)}
                             for i, device in enumerate(camera.devices)]
        for index, frame in frames.items():
            cv2.imwrite(str(root/f'camera-{index}.jpg'), frame)
        from src.hardware.lidar_controller import LidarController
        lidar = LidarController(); resources.append(lidar)
        scan = lidar.get_fresh_scan(timeout=10)
        result['lidar'] = {'valid_points': len(scan or []), 'error': lidar.error}
        result['success'] = len(frames) == 3 and scan is not None and all(
            v['model'] == 320 and v['cw_limit'] == 0 and v['ccw_limit'] == 0 and v['torque_control'] == 0
            for v in result['motors'].values())
    except Exception as exc:
        result['success'] = False
        result['error'] = str(exc)
    finally:
        power_stop.set();power_thread.join(timeout=2)
        result['power']={'samples':power_samples,'current_fault_detected':any(s['current_fault'] for s in power_samples),'historical_undervoltage':any(s.get('undervoltage_occurred',False) for s in power_samples)}
        result['success']=bool(result.get('success')) and bool(power_samples) and not result['power']['current_fault_detected']
        for obj in reversed(resources):
            obj.shutdown()
        (root/'result.json').write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding='utf-8')
        print(json.dumps(result, ensure_ascii=False, indent=2))
    return 0 if result.get('success') else 1


if __name__ == '__main__':
    raise SystemExit(main())
