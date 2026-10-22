"""MX-106 Protocol 1.0; reading hardware never arms the wheels."""
import math
import threading
import time
from dynamixel_sdk import (PortHandler, PacketHandler, GroupSyncWrite,
                          COMM_SUCCESS, DXL_LOBYTE, DXL_HIBYTE)
from src.utils.config import DXL_DEVICENAME, DXL_PROTOCOL_VERSION, DXL_BAUDRATE, DXL_IDS, BASE_SPEED, ROTATE_SPEED, WHEEL_FORWARD_SIGNS

LEFT_MOTORS = [9, 2]
RIGHT_MOTORS = [8, 7]
WHEEL_IDS = {'front_left': 9, 'front_right': 8, 'rear_left': 2, 'rear_right': 7}


class MotorController:
    def __init__(self, watchdog_timeout=0.5):
        self.portHandler = PortHandler(DXL_DEVICENAME)
        self.packetHandler = PacketHandler(DXL_PROTOCOL_VERSION)
        self.groupSyncWrite = GroupSyncWrite(self.portHandler, self.packetHandler, 32, 2)
        self._port_lock = threading.RLock()
        self._stop_event = threading.Event()
        self.armed = False
        self.fault = None
        self.last_speed_command = 0
        self._last_command_at = time.monotonic()
        self.watchdog_timeout = watchdog_timeout
        self._closed = False
        self.read_retry_count = 0
        self.write_retry_count = 0
        self._last_feedback_check = 0.0
        self.wheel_feedback = {}
        self.shutdown_errors = []
        if not self.portHandler.openPort():
            raise RuntimeError('Cannot open Dynamixel port')
        if not self.portHandler.setBaudRate(DXL_BAUDRATE):
            self.portHandler.closePort()
            raise RuntimeError('Cannot set Dynamixel baudrate')
        self._thread = threading.Thread(target=self._watchdog, daemon=True)
        self._thread.start()

    def _check(self, comm, error, context):
        if comm != COMM_SUCCESS or error:
            raise RuntimeError(f'{context}: {self.packetHandler.getTxRxResult(comm)} / {self.packetHandler.getRxPacketError(error)}')

    def _read(self, i, address, size=2):
        with self._port_lock:
            read = self.packetHandler.read1ByteTxRx if size == 1 else self.packetHandler.read2ByteTxRx
            for attempt in range(3):
                value, comm, error = read(self.portHandler, i, address)
                if comm == COMM_SUCCESS and not error:
                    return value
                # Retry only a missing/corrupt transport packet, never a device
                # alarm. No zero substitute and no indefinitely stale position.
                if attempt < 2 and comm != COMM_SUCCESS and not error:
                    self.read_retry_count = getattr(self,'read_retry_count',0)+1
                    self._recover_transport()
                    continue
                self._check(comm, error, f'ID {i}, register {address}')

    def _write(self, i, address, value, size=2):
        with self._port_lock:
            write = self.packetHandler.write1ByteTxRx if size == 1 else self.packetHandler.write2ByteTxRx
            for attempt in range(3):
                comm, error = write(self.portHandler, i, address, value)
                if comm == COMM_SUCCESS and not error:
                    return
                if attempt < 2 and comm != COMM_SUCCESS and not error:
                    self.write_retry_count = getattr(self,'write_retry_count',0)+1
                    self._recover_transport()
                    continue
                self._check(comm, error, f'ID {i}, register {address}')

    def _recover_transport(self):
        # SDK clearPort() only flushes outgoing bytes. Wait out the FTDI
        # receive latency before discarding late/corrupt status packets.
        self.portHandler.clearPort()
        time.sleep(.020)
        serial_port = getattr(self.portHandler, 'ser', None)
        if serial_port is not None:
            serial_port.reset_input_buffer()

    def inspect(self):
        return {i: {'model': self._read(i, 0), 'cw_limit': self._read(i, 6),
                    'ccw_limit': self._read(i, 8), 'torque': self._read(i, 24, 1),
                    'torque_control': self._read(i, 70, 1), 'position': self._read(i, 36)} for i in DXL_IDS}

    def arm(self):
        with self._port_lock:
            if self.fault:
                raise RuntimeError(self.fault)
            for i, status in self.inspect().items():
                if status['model'] != 320:
                    raise RuntimeError(f'ID {i}: expected MX-106 model 320')
                if status['cw_limit'] or status['ccw_limit'] or status['torque_control']:
                    raise RuntimeError(f'ID {i}: Wheel Mode / speed control not configured')
            try:
                for i in DXL_IDS:
                    self._write(i, 32, 0)
                for i in DXL_IDS:
                    self._write(i, 24, 1, 1)
                    if self._read(i,24,1) != 1:
                        raise RuntimeError(f'ID {i}: torque enable not confirmed')
                self.armed = True
                self._last_command_at = time.monotonic()
            except Exception:
                self.armed = True
                self.shutdown()
                raise

    @staticmethod
    def encode_velocity(i, velocity):
        if not isinstance(velocity, (int, float)) or not math.isfinite(velocity):
            raise ValueError('Wheel velocity must be finite')
        v = int(round(velocity)) * WHEEL_FORWARD_SIGNS[i]
        v = max(-1023, min(1023, v))
        return 1024-v if v < 0 else v

    def set_velocities_bulk(self, velocities_dict):
        if set(velocities_dict) != set(DXL_IDS):
            raise ValueError('Every command must specify all four wheels')
        encoded = {i: self.encode_velocity(i, v) for i, v in velocities_dict.items()}
        with self._port_lock:
            if not self.armed:
                if any(velocities_dict.values()):
                    raise RuntimeError('Motors are not armed; call arm() explicitly')
                return
            if self.fault and any(velocities_dict.values()):
                raise RuntimeError(self.fault)
            self.groupSyncWrite.clearParam()
            for i, value in encoded.items():
                if not self.groupSyncWrite.addParam(i, [DXL_LOBYTE(value), DXL_HIBYTE(value)]):
                    self.fault = 'Failed to encode motor command'
                    self._emergency_stop()
                    raise RuntimeError(self.fault)
            result = self.groupSyncWrite.txPacket()
            if result != COMM_SUCCESS:
                self.fault = self.packetHandler.getTxRxResult(result)
                self._emergency_stop()
                raise RuntimeError(self.fault)
            self._last_command_at = time.monotonic()
            self.last_speed_command = max(abs(v) for v in velocities_dict.values())
            if self.last_speed_command and time.monotonic()-self._last_feedback_check > .25:
                try:
                    self.verify_command(encoded)
                    self._last_feedback_check = time.monotonic()
                except Exception as exc:
                    self.fault = f'Drive feedback: {exc}'
                    self._emergency_stop()
                    raise RuntimeError(self.fault) from exc

    def verify_command(self, encoded):
        # A successful broadcast only confirms USB transmission, not reception
        # by every wheel. Verify each drive and repair a missed speed packet.
        feedback = {}
        for i, expected in encoded.items():
            torque = self._read(i,24,1)
            if torque != 1:
                raise RuntimeError(f'ID {i}: torque disabled during motion')
            actual = self._read(i,32)
            if actual != expected:
                self._write(i,32,expected)
                actual = self._read(i,32)
                if actual != expected:
                    raise RuntimeError(f'ID {i}: speed command not confirmed')
            feedback[i]={'torque':torque,'goal_speed':actual,'present_speed':self._read(i,38)}
        self.wheel_feedback = feedback
        return feedback

    def _emergency_stop(self):
        with self._port_lock:
            for i in DXL_IDS:
                try:
                    self._write(i, 32, 0)
                except Exception:
                    pass
            self.last_speed_command = 0

    def _watchdog(self):
        while not self._stop_event.wait(0.05):
            with self._port_lock:
                if self.armed and self.last_speed_command and time.monotonic()-self._last_command_at > self.watchdog_timeout:
                    self.fault = 'Motor command watchdog expired'
                    self._emergency_stop()

    def abort(self, reason):
        with self._port_lock:
            self.fault = reason
            self._emergency_stop()

    def read_speed(self, i):
        value = self._read(i, 38)
        return -(value & 1023) if value & 1024 else value

    def read_position(self, i):
        return self._read(i, 36)

    def read_positions(self):
        return {i: self.read_position(i) for i in DXL_IDS}

    def stop_all(self):
        self.set_velocities_bulk({i: 0 for i in DXL_IDS})

    def move_forward(self, speed):
        self.set_velocities_bulk({i: speed for i in DXL_IDS})

    def move_backward(self, speed):
        self.move_forward(-speed)

    def rotate_left(self, speed):
        self.set_velocities_bulk({i: -speed if i in LEFT_MOTORS else speed for i in DXL_IDS})

    def rotate_right(self, speed):
        self.rotate_left(-speed)

    def move_forward_with_arc(self, direction, forward_speed, correction_speed):
        if direction not in ('left', 'right'):
            raise ValueError('Invalid arc direction')
        left = forward_speed-correction_speed if direction == 'left' else forward_speed+correction_speed
        right = forward_speed+correction_speed if direction == 'left' else forward_speed-correction_speed
        self.set_velocities_bulk({i: left if i in LEFT_MOTORS else right for i in DXL_IDS})

    def turn_to_angle(self, target_angle_rad, current_angle_rad, tolerance=0.1):
        e = math.atan2(math.sin(target_angle_rad-current_angle_rad), math.cos(target_angle_rad-current_angle_rad))
        if abs(e) < tolerance:
            self.stop_all()
            return True
        self.rotate_left(ROTATE_SPEED if e > 0 else -ROTATE_SPEED)
        return False

    def move_forward_distance(self, distance_mm, speed=BASE_SPEED):
        raise RuntimeError('Timed distance motion disabled; use measured odometry navigation')

    def shutdown(self):
        self._stop_event.set()
        with self._port_lock:
            if self._closed:
                return
            if self.armed:
                self._emergency_stop()
                for i in DXL_IDS:
                    try:
                        self._write(i, 24, 0, 1)
                        if self._read(i,24,1) != 0:
                            raise RuntimeError(f'ID {i}: torque-off not confirmed')
                    except Exception as exc:
                        self.shutdown_errors.append(str(exc))
            self.armed = False
            self.portHandler.closePort()
            self._closed = True
