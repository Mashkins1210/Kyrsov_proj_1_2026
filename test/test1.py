import os
import time
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from dynamixel_sdk import *
from rplidar import RPLidar
import threading
from queue import Queue, Empty

# ==========================================
# 1. КОНФИГУРАЦИЯ УСТРОЙСТВ
# ==========================================
DXL_DEVICENAME = '/dev/ttyUSB0'
DXL_PROTOCOL_VERSION = 1.0
DXL_BAUDRATE = 57600
DXL_IDS = [2, 7, 8, 9]
ADDR_MX_TORQUE_ENABLE = 24
ADDR_MX_GOAL_VELOCITY = 32
ADDR_MX_PRESENT_VELOCITY = 38
DEGREES_PER_SECOND_AT_300 = 48.0

LIDAR_DEVICE_PATH = '/dev/ttyUSB1'
LIDAR_BAUD_RATE = 115200
LIDAR_TIMEOUT = 1

SAFE_DISTANCE_MM = 700
STOP_DISTANCE_MM = 300
BASE_SPEED = 400
ROTATE_SPEED = 300
BACKUP_SPEED = 250
TURN_ANGLE = 90

degrees_per_sec = DEGREES_PER_SECOND_AT_300 * (ROTATE_SPEED / 300.0)
TURN_DURATION = TURN_ANGLE / degrees_per_sec
print(f"📐 Turn calculation: {TURN_ANGLE}° / {degrees_per_sec:.1f}°/s = {TURN_DURATION:.2f}s")

# ==========================================
# [FIXED] ЗОНЫ АНАЛИЗА ЛИДАРА
# У вашего робота ПЕРЕД = 180°!
# ==========================================
FRONT_CENTER = 180    # Центр зоны "спереди"
REAR_CENTER = 0       # Центр зоны "сзади"
LEFT_CENTER = 270     # Центр зоны "слева"
RIGHT_CENTER = 90     # Центр зоны "справа"

FRONT_ANGLE_RANGE = 40   # Полуширина зоны спереди (±40° от 180° = [140°, 220°])
REAR_ANGLE_RANGE = 40    # Полуширина зоны сзади (±40° от 0° = [320°, 40°])
SIDE_ANGLE_RANGE = 45    # Полуширина зоны сбоку

print(f"📍 Lidar orientation: FRONT={FRONT_CENTER}°, REAR={REAR_CENTER}°")

# Состояния робота
STATE_MOVING = 'moving'
STATE_TURNING = 'turning'
STATE_RECOVERING = 'recovering'
STATE_BACKUP = 'backup'
STATE_ESCAPE = 'escape'
STATE_PAUSED = 'paused'

is_running = True
is_paused = False
robot_state = STATE_MOVING
turn_start_time = 0
recover_start_time = 0
backup_start_time = 0
escape_start_time = 0
RECOVER_DURATION = 1.0
BACKUP_DURATION = 1.5
ESCAPE_DURATION = 0
consecutive_turns = 0
MAX_CONSECUTIVE_TURNS = 2
last_speed_command = 0
escape_direction = 'right'
state_lock = threading.Lock()

# ==========================================
# 2. ИНИЦИАЛИЗАЦИЯ МОТОРОВ
# ==========================================
portHandler = PortHandler(DXL_DEVICENAME)
packetHandler = PacketHandler(DXL_PROTOCOL_VERSION)
groupSyncWrite = GroupSyncWrite(portHandler, packetHandler, ADDR_MX_GOAL_VELOCITY, 2)
groupSyncRead = GroupSyncRead(portHandler, packetHandler, ADDR_MX_PRESENT_VELOCITY, 2)

if not portHandler.openPort() or not portHandler.setBaudRate(DXL_BAUDRATE):
    print("Failed to open DXL port")
    quit()

for dxl_id in DXL_IDS:
    packetHandler.write1ByteTxRx(portHandler, dxl_id, ADDR_MX_TORQUE_ENABLE, 1)
    groupSyncRead.addParam(dxl_id)
print("✅ Dynamixels initialized")

# ==========================================
# 3. ФУНКЦИИ УПРАВЛЕНИЯ МОТОРАМИ
# ==========================================
def set_velocities_bulk(velocities_dict):
    global last_speed_command
    try:
        groupSyncWrite.clearParam()
        for dxl_id, velocity in velocities_dict.items():
            if dxl_id in [7, 8]:
                velocity = -velocity
            velocity = max(min(velocity, 1023), -1023)
            if velocity < 0:
                velocity = 1024 - velocity
            param = [DXL_LOBYTE(velocity), DXL_HIBYTE(velocity)]
            groupSyncWrite.addParam(dxl_id, param)
        
        result = groupSyncWrite.txPacket()
        if result != COMM_SUCCESS:
            print(f"️ Motor comm error: {packetHandler.getTxRxResult(result)}")
        else:
            last_speed_command = list(velocities_dict.values())[0] if velocities_dict else 0
    except Exception as e:
        print(f"⚠️ Motor error: {e}")

def stop_all():
    global last_speed_command
    last_speed_command = 0
    set_velocities_bulk({dxl_id: 0 for dxl_id in DXL_IDS})

def move_forward(speed):
    set_velocities_bulk({dxl_id: speed for dxl_id in DXL_IDS})

def move_backward(speed):
    set_velocities_bulk({dxl_id: -speed for dxl_id in DXL_IDS})

def rotate_left(speed):
    velocities = {2: -speed, 9: -speed, 7: speed, 8: speed}
    set_velocities_bulk(velocities)

def rotate_right(speed):
    velocities = {2: speed, 9: speed, 7: -speed, 8: -speed}
    set_velocities_bulk(velocities)

# ==========================================
# 4. ФУНКЦИИ АНАЛИЗА ОКРУЖЕНИЯ [FIXED]
# ==========================================
def get_zone_distances(scan_data, center_angle, angle_range):
    """
    Возвращает минимальное расстояние в заданной зоне.
    Правильно обрабатывает переход через 0°/360°.
    """
    if not scan_data or len(scan_data) == 0:
        return float('inf')
    
    scan_array = np.array(scan_data)
    angles = scan_array[:, 1]
    distances = scan_array[:, 2]
    
    diff = np.abs(angles - center_angle)
    diff = np.minimum(diff, 360 - diff)
    
    mask = diff <= angle_range
    zone_distances = distances[mask]
    zone_distances = zone_distances[zone_distances > 10]
    
    if len(zone_distances) == 0:
        return float('inf')
    
    return np.min(zone_distances)

def check_front_obstacle(scan_data):
    return get_zone_distances(scan_data, FRONT_CENTER, FRONT_ANGLE_RANGE)

def check_rear_obstacle(scan_data):
    return get_zone_distances(scan_data, REAR_CENTER, REAR_ANGLE_RANGE)

def check_left_obstacle(scan_data):
    return get_zone_distances(scan_data, LEFT_CENTER, SIDE_ANGLE_RANGE)

def check_right_obstacle(scan_data):
    return get_zone_distances(scan_data, RIGHT_CENTER, SIDE_ANGLE_RANGE)

def find_free_direction(scan_data):
    """
    [FIXED] Находит самое свободное направление.
    Возвращает: (направление, расстояние)
    """
    if not scan_data or len(scan_data) == 0:
        return 'right', float('inf')
    
    directions = {
        'front': FRONT_CENTER,
        'back': REAR_CENTER,
        'left': LEFT_CENTER,
        'right': RIGHT_CENTER
    }
    
    best_dir = 'right'
    best_dist = 0
    
    for dir_name, angle in directions.items():
        dist = get_zone_distances(scan_data, angle, SIDE_ANGLE_RANGE)
        if dist > best_dist:
            best_dist = dist
            best_dir = dir_name
    
    return best_dir, best_dist

def check_obstacle(scan_data):
    """Проверяет препятствие СПЕРЕДИ. Возвращает 'clear', 'slow' или 'obstacle'."""
    if not scan_data or len(scan_data) == 0:
        return 'clear'
    
    front_dist = check_front_obstacle(scan_data)
    
    if front_dist < STOP_DISTANCE_MM:
        return 'obstacle'
    elif front_dist < SAFE_DISTANCE_MM:
        return 'slow'
    return 'clear'

# ==========================================
# 5. ИНИЦИАЛИЗАЦИЯ ЛИДАРА И ПОТОКОВ
# ==========================================
lidar = RPLidar(port=LIDAR_DEVICE_PATH, baudrate=LIDAR_BAUD_RATE, timeout=LIDAR_TIMEOUT)
lidar.start_motor()
print("✅ RPLidar started")

stop_event = threading.Event()
scan_queue = Queue(maxsize=10)
lidar_error_count = 0
last_scan = None
scan_lock = threading.Lock()

def lidar_reader():
    global lidar_error_count, last_scan
    
    while not stop_event.is_set():
        try:
            for scan in lidar.iter_scans(max_buf_meas=8000):
                if stop_event.is_set():
                    break
                
                while not scan_queue.empty():
                    try: 
                        scan_queue.get_nowait()
                    except Empty: 
                        break
                
                scan_queue.put(scan)
                
                with scan_lock:
                    last_scan = scan
                
                lidar_error_count = 0
                
        except Exception as e:
            if not stop_event.is_set():
                lidar_error_count += 1
                print(f"⚠️ Lidar reader error #{lidar_error_count}: {e}")
                
                if lidar_error_count >= 5:
                    print("🔄 Attempting lidar reconnect...")
                    try:
                        lidar.stop()
                        time.sleep(0.5)
                        lidar.clear_buffer()
                        lidar.start_motor()
                        lidar_error_count = 0
                        print("✅ Lidar reconnected")
                    except Exception as reconnect_err:
                        print(f" Reconnect failed: {reconnect_err}")
                
                time.sleep(0.5)

lidar_thread = threading.Thread(target=lidar_reader, daemon=True)
lidar_thread.start()
print("✅ Lidar reader thread started")

# ==========================================
# 6. ФУНКЦИЯ КОРРЕКТНОГО ЗАВЕРШЕНИЯ
# ==========================================
def shutdown():
    global is_running
    
    if not is_running:
        return
    
    is_running = False
    print("\n🛑 Shutting down systems...")
    
    try:
        stop_all()
        for dxl_id in DXL_IDS:
            packetHandler.write1ByteTxRx(portHandler, dxl_id, ADDR_MX_TORQUE_ENABLE, 0)
        portHandler.closePort()
        print("✅ Motors stopped and port closed")
    except Exception as e:
        print(f"⚠️ Motor shutdown error: {e}")
    
    try:
        stop_event.set()
        if lidar_thread.is_alive():
            lidar_thread.join(timeout=2.0)
        lidar.stop()
        lidar.stop_motor()
        lidar.disconnect()
        print("✅ Lidar disconnected")
    except Exception as e:
        print(f"️ Lidar shutdown error: {e}")
    
    print("🏁 All systems stopped cleanly.")

# ==========================================
# 7. ГРАФИКА [FIXED] - ПОВОРОТ КАРТЫ
# ==========================================
plt.rcParams['toolbar'] = 'None'
fig = plt.figure("Robot Vision & Control")
ax = plt.subplot(111, projection='polar')
scatter = ax.scatter([], [], s=5, c=[], cmap=plt.cm.Greys_r, lw=0, vmin=0, vmax=50)
ax.set_rmax(500)
ax.grid(True)

ax.set_theta_offset(np.pi)

state_text = ax.text(0.05, 0.95, 'State: MOVING', transform=ax.transAxes, 
                     fontsize=14, color='green', fontweight='bold', verticalalignment='top')
info_text = ax.text(0.05, 0.88, '', transform=ax.transAxes, 
                    fontsize=10, color='blue', verticalalignment='top')
help_text = ax.text(0.5, 0.02, 
                    'Keys: [Q/Esc]=Quit  [S]=Stop  [P]=Pause  [R]=Resume',
                    transform=ax.transAxes, 
                    fontsize=9, color='gray', 
                    horizontalalignment='center', verticalalignment='bottom')

# ==========================================
# 8. ОБРАБОТКА НАЖАТИЙ КЛАВИШ
# ==========================================
def on_key_press(event):
    global robot_state, is_paused, turn_start_time, recover_start_time, consecutive_turns
    
    key = event.key
    print(f"🔑 Key pressed: '{key}'")
    
    if key in ['q', 'escape']:
        print("\n⏹️ Quit requested by user")
        shutdown()
        plt.close(fig)
    
    elif key == 's':
        print("🛑 Emergency stop!")
        stop_all()
        robot_state = STATE_PAUSED
        state_text.set_text('State: STOPPED (S)')
        state_text.set_color('red')
        info_text.set_text('Motors stopped. Press [R] to resume.')
    
    elif key == 'p':
        is_paused = not is_paused
        if is_paused:
            print("⏸️ Paused")
            stop_all()
            robot_state = STATE_PAUSED
            state_text.set_text('State: PAUSED (P)')
            state_text.set_color('yellow')
            info_text.set_text('Press [P] again to resume')
        else:
            print("▶️ Resumed")
            robot_state = STATE_MOVING
            consecutive_turns = 0
            state_text.set_text('State: MOVING')
            state_text.set_color('green')
            info_text.set_text('')
    
    elif key == 'r':
        print("🔄 Force resume to MOVING state")
        is_paused = False
        robot_state = STATE_MOVING
        consecutive_turns = 0
        state_text.set_text('State: MOVING')
        state_text.set_color('green')
        info_text.set_text('Manual override')

fig.canvas.mpl_connect('key_press_event', on_key_press)
print("🎮 Keyboard control enabled: [Q]=Quit [S]=Stop [P]=Pause [R]=Resume")

# ==========================================
# [NEW] ВСПОМОГАТЕЛЬНАЯ ФУНКЦИЯ ПОЛУЧЕНИЯ СВЕЖЕГО СКАНА
# ==========================================
def get_fresh_scan(timeout=0.1):
    """Получает свежий скан из очереди или возвращает последний известный"""
    global last_scan
    try:
        scan = scan_queue.get(timeout=timeout)
        with scan_lock:
            last_scan = scan
        return scan
    except Empty:
        with scan_lock:
            return last_scan.copy() if last_scan is not None else None

# ==========================================
# 9. ЦИКЛ АНИМАЦИИ С УМНЫМ ПОВЕДЕНИЕМ [FIXED]
# ==========================================
def update_frame(num):
    global is_running, robot_state, turn_start_time, recover_start_time
    global backup_start_time, escape_start_time, ESCAPE_DURATION
    global consecutive_turns, is_paused, escape_direction
    
    if not is_running:
        return scatter, state_text, info_text
    
    scan = get_fresh_scan(timeout=0.05)
    
    if scan is not None and len(scan) > 0:
        scan_array = np.array(scan)
        offsets = np.column_stack([np.radians(scan_array[:, 1]), scan_array[:, 2]])
        scatter.set_offsets(offsets)
        scatter.set_array(scan_array[:, 0])
    
    if is_paused or robot_state == STATE_PAUSED:
        return scatter, state_text, info_text
    
    current_time = time.time()
    
    # --- СОСТОЯНИЕ: TURNING ---
    if robot_state == STATE_TURNING:
        elapsed = current_time - turn_start_time
        remaining = TURN_DURATION - elapsed
        
        rotate_left(ROTATE_SPEED)
        
        state_text.set_text(f'State: TURNING {TURN_ANGLE}°')
        state_text.set_color('red')
        info_text.set_text(f'Turn #{consecutive_turns} | Remaining: {remaining:.1f}s')
        
        if elapsed >= TURN_DURATION:
            stop_all()
            robot_state = STATE_RECOVERING
            recover_start_time = current_time
            print(f"✅ Turn complete. Recovering...")
    
    # --- СОСТОЯНИЕ: RECOVERING ---
    elif robot_state == STATE_RECOVERING:
        elapsed = current_time - recover_start_time
        stop_all()
        
        state_text.set_text('State: RECOVERING')
        state_text.set_color('orange')
        info_text.set_text(f'Scanning: {elapsed:.1f}s | Turns: {consecutive_turns}')
        
        if elapsed >= RECOVER_DURATION:
            current_scan = get_fresh_scan(timeout=0.05)
            
            if consecutive_turns >= MAX_CONSECUTIVE_TURNS:
                # Анализируем окружение
                rear_dist = check_rear_obstacle(current_scan) if current_scan is not None else float('inf')
                print(f"🔍 Rear distance (0° zone): {rear_dist:.0f}mm")
                
                if rear_dist < STOP_DISTANCE_MM * 1.5:
                    # СЗАДИ СТЕНА! Ищем свободное направление
                    print(f"⚠️ Wall behind! Finding free direction...")
                    
                    if current_scan is not None and len(current_scan) > 0:
                        free_dir, free_dist = find_free_direction(current_scan)
                        print(f"🎯 Free direction: {free_dir} ({free_dist:.0f}mm)")
                        
                        escape_direction = free_dir
                        robot_state = STATE_ESCAPE
                        escape_start_time = current_time
                        
                        if free_dir == 'left':
                            ESCAPE_DURATION = TURN_DURATION
                            print(f"→ Turning LEFT 90°")
                        elif free_dir == 'right':
                            ESCAPE_DURATION = TURN_DURATION
                            print(f"→ Turning RIGHT 90°")
                        elif free_dir == 'back' or free_dir == 'front':
                            ESCAPE_DURATION = TURN_DURATION * 2
                            print(f"→ U-turn 180°")
                    else:
                        robot_state = STATE_TURNING
                        turn_start_time = current_time
                else:
                    # СЗАДИ СВОБОДНО — отъезжаем назад
                    print(f"✅ Rear is clear. Backing up...")
                    robot_state = STATE_BACKUP
                    backup_start_time = current_time
            else:
                # [FIXED] Сбрасываем счетчик и продолжаем движение вперед
                robot_state = STATE_MOVING
                consecutive_turns = 0
                print("✅ Recovery complete. Resuming forward movement.")
    
    # --- СОСТОЯНИЕ: BACKUP ---
    elif robot_state == STATE_BACKUP:
        elapsed = current_time - backup_start_time
        remaining = BACKUP_DURATION - elapsed
        
        move_backward(BACKUP_SPEED)
        
        state_text.set_text('State: BACKING UP')
        state_text.set_color('magenta')
        info_text.set_text(f'Backing up: {remaining:.1f}s | Turns: {consecutive_turns}')
        
        if elapsed >= BACKUP_DURATION:
            stop_all()
            # [FIXED] После отъезда назад - сбрасываем счетчик и продолжаем движение вперед
            print(f"✅ Backup complete. Resuming forward movement.")
            robot_state = STATE_MOVING
            consecutive_turns = 0
    
    # --- СОСТОЯНИЕ: ESCAPE [FIXED] ---
    elif robot_state == STATE_ESCAPE:
        elapsed = current_time - escape_start_time
        remaining = ESCAPE_DURATION - elapsed
        
        if escape_direction == 'right':
            rotate_right(ROTATE_SPEED)
            dir_label = 'RIGHT'
        elif escape_direction == 'left':
            rotate_left(ROTATE_SPEED)
            dir_label = 'LEFT'
        else:
            rotate_left(ROTATE_SPEED)
            dir_label = '180°'
        
        state_text.set_text(f'State: ESCAPING {dir_label}')
        state_text.set_color('cyan')
        info_text.set_text(f'Escape turn: {remaining:.1f}s | Turns: {consecutive_turns}')
        
        if elapsed >= ESCAPE_DURATION:
            stop_all()
            # [FIXED] После побега - сбрасываем счетчик и продолжаем движение вперед
            print(f"✅ Escape complete. Resuming forward movement.")
            robot_state = STATE_MOVING
            consecutive_turns = 0
    
    # --- СОСТОЯНИЕ: MOVING ---
    elif robot_state == STATE_MOVING:
        if scan is None or len(scan) == 0:
            if last_speed_command > 0:
                move_forward(last_speed_command)
                state_text.set_text('State: MOVING (no lidar)')
                state_text.set_color('yellow')
                info_text.set_text('⚠️ Lidar offline')
            else:
                move_forward(BASE_SPEED)
                state_text.set_text('State: MOVING (waiting lidar)')
                state_text.set_color('yellow')
                info_text.set_text('Starting...')
            return scatter, state_text, info_text
        
        obstacle_status = check_obstacle(scan)
        
        if obstacle_status == 'obstacle':
            robot_state = STATE_TURNING
            turn_start_time = current_time
            consecutive_turns += 1
            
            print(f"🚧 Obstacle! Turn #{consecutive_turns} (max {MAX_CONSECUTIVE_TURNS})")
            
            state_text.set_text(f'State: TURNING {TURN_ANGLE}°')
            state_text.set_color('red')
            info_text.set_text(f'Turn #{consecutive_turns}')
            
        elif obstacle_status == 'slow':
            move_forward(BASE_SPEED // 2)
            state_text.set_text('State: MOVING (SLOW)')
            state_text.set_color('orange')
            info_text.set_text('')
            consecutive_turns = 0
            
        else:
            move_forward(BASE_SPEED)
            state_text.set_text('State: MOVING')
            state_text.set_color('green')
            info_text.set_text('')
            consecutive_turns = 0
    
    return scatter, state_text, info_text

ani = animation.FuncAnimation(fig, update_frame, interval=100, blit=False, cache_frame_data=False)

# ==========================================
# 10. ЗАПУСК И БЕЗОПАСНОЕ ЗАВЕРШЕНИЕ
# ==========================================
try:
    plt.show()
except KeyboardInterrupt:
    print("\n⏹️ KeyboardInterrupt received.")
finally:
    shutdown()
    
    try:
        ani.event_source.stop()
    except:
        pass
    
    try:
        plt.close('all')
    except:
        pass