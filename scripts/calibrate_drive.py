"""Bounded physical check: 200 mm forward, +90/-90 degrees, 200 mm back.

Only run with operator authorization, on a clear level start cell, with an
emergency stop operator. This script does not certify external measurements.
"""
import argparse
import json
import math
from pathlib import Path
import sys
import time
import subprocess
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from scripts.mission import load_settings


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--drive',action='store_true')
    parser.add_argument('--front-verified',action='store_true')
    parser.add_argument('--forward-only',action='store_true',help='Bounded 200 mm recovery/validation movement only')
    parser.add_argument('--config',default='config/robot.json')
    parser.add_argument('--distance-mm',type=float,default=200)
    parser.add_argument('--turn-deg',type=float,default=90)
    args=parser.parse_args()
    if not args.drive or not args.front_verified:
        parser.error('Explicit --drive and physically checked lidar orientation --front-verified are required')
    if not math.isfinite(args.distance_mm) or not 20<=args.distance_mm<=200:
        parser.error('distance-mm must be 20..200')
    if not math.isfinite(args.turn_deg) or not 10<=args.turn_deg<=90:
        parser.error('turn-deg must be 10..90')
    settings=load_settings(args.config)
    from src.hardware.motor_controller import MotorController, LEFT_MOTORS, RIGHT_MOTORS
    from src.hardware.lidar_controller import LidarController
    from src.control.odometry import Odometry
    resources=[]; samples=[]; motor=None
    try:
        motor=MotorController(); resources.append(motor)
        lidar=LidarController(); resources.append(lidar)
        if not lidar.get_fresh_scan(timeout=5): raise RuntimeError('No lidar')
        motor.arm()
        # Initialize encoder timestamps after the potentially slow arm preflight.
        odom=Odometry(motor,LEFT_MOTORS,RIGHT_MOTORS,settings['wheel_diameter_mm'],settings['wheelbase_mm'],angular_scale=settings.get('odometry_angular_scale',1.0))
        resources.append(odom)

        def sense(direction=None):
            power=subprocess.check_output(['vcgencmd','get_throttled'],text=True,timeout=2).strip()
            if int(power.split('=')[1],16) & 15:
                raise RuntimeError(f'Current power/temperature throttling: {power}')
            if motor.fault: raise RuntimeError(motor.fault)
            scan=lidar.get_fresh_scan(timeout=0.1)
            if not scan: raise RuntimeError('Lidar stale')
            pose=odom.get_pose_meters()
            if direction is None:
                if min(d for q,a,d in scan if q>0)<200: raise RuntimeError('Insufficient rotation clearance')
            else:
                center=settings['lidar_front_deg']+(180 if direction=='back' else 0)
                zone=[d for q,a,d in scan if q>0 and abs((a-center+180)%360-180)<35]
                if len(zone)<5 or min(zone)<300: raise RuntimeError('Obstacle/unobserved motion sector')
            return pose

        def translate(direction):
            initial=sense(direction); deadline=time.monotonic()+8
            try:
                while time.monotonic()<deadline:
                    pose=sense(direction)
                    signed=(pose[0]-initial[0])*(1 if direction=='front' else -1)
                    if signed < -0.015: raise RuntimeError('Wheel/encoder direction mismatch')
                    yaw_delta=math.atan2(math.sin(pose[2]-initial[2]),math.cos(pose[2]-initial[2]))
                    if abs(pose[1]-initial[1])>.04 or abs(yaw_delta)>.15:
                        raise RuntimeError(f'Unexpected lateral/yaw deviation: direction={direction}, initial={initial}, pose={pose}')
                    if signed>=args.distance_mm/1000: break
                    motor.move_forward(60 if direction=='front' else -60)
                    time.sleep(0.05)
                else: raise RuntimeError('Translation timed out')
            finally: motor.stop_all()
            time.sleep(.25)
            samples.append({'stage':direction,'initial':initial,'pose':odom.get_pose_meters(),'encoders':motor.read_positions()})

        def rotate(target):
            deadline=time.monotonic()+8
            try:
                while time.monotonic()<deadline:
                    pose=sense()
                    error=math.atan2(math.sin(target-pose[2]),math.cos(target-pose[2]))
                    if abs(error)<.06: break
                    speed=min(60,max(20,int(80*abs(error))))
                    motor.rotate_left(speed if error>0 else -speed)
                    time.sleep(.05)
                else: raise RuntimeError('Rotation timed out')
            finally: motor.stop_all()
            time.sleep(.25)
            samples.append({'stage':'rotate','target':target,'pose':odom.get_pose_meters(),'encoders':motor.read_positions()})

        translate('front')
        if not args.forward_only:
            rotate(math.radians(args.turn_deg)); rotate(0); translate('back')
        result={'success':True,'samples':samples,'external_measurement_verified':False,
                'note':'Compare actual displacement/angles with a ruler; encoder agreement is not ground truth.'}
    except BaseException as exc:
        result={'success':False,'error':str(exc),'samples':samples}
        raise
    finally:
        if motor:
            try: motor.stop_all()
            except Exception: pass
        for obj in reversed(resources[1:]): obj.shutdown()
        if motor: motor.shutdown()
        Path('data').mkdir(exist_ok=True)
        Path('data/drive_calibration.json').write_text(json.dumps(result,indent=2),encoding='utf-8')
        print(json.dumps(result,indent=2))


if __name__=='__main__': main()
