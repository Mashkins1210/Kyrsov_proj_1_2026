import time
import sys
import select
import tty
import termios
from rplidar import RPLidar

LIDAR_DEVICE_PATH = '/dev/ttyUSB1'
LIDAR_BAUD_RATE = 115200
LIDAR_TIMEOUT = 1

def get_key():
    if select.select([sys.stdin], [], [], 0.1)[0]:
        return sys.stdin.read(1)
    return None

def main():
    print("=== LIDAR DEBUG SCRIPT ===")
    print(f"Device: {LIDAR_DEVICE_PATH}")
    print(f"Baudrate: {LIDAR_BAUD_RATE}")
    print("Press 'q' to quit")
    print()

    lidar = None
    scan_count = 0
    error_count = 0
    last_scan_time = time.time()
    total_points = 0

    try:
        old_settings = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())
    except Exception:
        old_settings = None

    try:
        print("Initializing RPLidar...")
        lidar = RPLidar(
            port=LIDAR_DEVICE_PATH,
            baudrate=LIDAR_BAUD_RATE,
            timeout=LIDAR_TIMEOUT
        )
        lidar.start_motor()
        print("Motor started, waiting for first scan...")
        time.sleep(1.0)

        print("Reading scans...")
        print("-" * 60)

        for scan in lidar.iter_scans(max_buf_meas=8000):
            key = get_key()
            if key and key.lower() == 'q':
                print("\nQuit requested")
                break

            scan_count += 1
            total_points += len(scan)
            now = time.time()
            elapsed = now - last_scan_time

            distances = [point[2] for point in scan if point[2] > 10]

            if distances:
                min_dist = min(distances)
                max_dist = max(distances)
                avg_dist = sum(distances) / len(distances)
            else:
                min_dist = 0
                max_dist = 0
                avg_dist = 0

            angles = [point[1] for point in scan]
            if angles:
                angle_range = max(angles) - min(angles)
            else:
                angle_range = 0

            print(f"Scan #{scan_count} | Points: {len(scan):4d} | "
                  f"Valid: {len(distances):4d} | "
                  f"Min: {min_dist:6.0f}mm | Max: {max_dist:6.0f}mm | "
                  f"Avg: {avg_dist:6.0f}mm | "
                  f"Angle range: {angle_range:5.1f} deg | "
                  f"Time: {elapsed:.2f}s | "
                  f"Errors: {error_count}")

            last_scan_time = now
            error_count = 0

    except Exception as e:
        error_count += 1
        print(f"\nERROR: {type(e).__name__}: {e}")
        print(f"Total errors: {error_count}")

    finally:
        if old_settings is not None:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

        print()
        print("=" * 60)
        print("STATISTICS:")
        print(f"  Total scans: {scan_count}")
        print(f"  Total points: {total_points}")
        print(f"  Avg points per scan: {total_points / max(scan_count, 1):.0f}")
        print(f"  Total errors: {error_count}")
        print()

        if lidar is not None:
            try:
                print("Stopping lidar...")
                lidar.stop()
                lidar.stop_motor()
                lidar.disconnect()
                print("Lidar stopped successfully")
            except Exception as e:
                print(f"Error stopping lidar: {e}")

        print("Done")

if __name__ == "__main__":
    main()