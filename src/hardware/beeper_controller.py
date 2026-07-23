import RPi.GPIO as GPIO
import time
import threading

class BeeperController:
    def __init__(self, pin=18):
        self.pin = pin
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.pin, GPIO.OUT)
        self.stop_event = threading.Event()
        print("Beeper initialized")

    def beep(self, duration=0.5, frequency=1000):
        period = 1.0 / frequency
        half_period = period / 2.0
        end_time = time.time() + duration
        
        while time.time() < end_time and not self.stop_event.is_set():
            GPIO.output(self.pin, GPIO.HIGH)
            time.sleep(half_period)
            GPIO.output(self.pin, GPIO.LOW)
            time.sleep(half_period)

    def double_beep(self):
        self.beep(0.2)
        time.sleep(0.1)
        self.beep(0.2)

    def shutdown(self):
        self.stop_event.set()
        GPIO.output(self.pin, GPIO.LOW)
        GPIO.cleanup()
        print("Beeper shutdown")