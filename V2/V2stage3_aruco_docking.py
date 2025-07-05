import time
import smbus
import RPi.GPIO as GPIO
import numpy as np
import json
import threading
from queue import Queue
import socket
import struct

# Pin Configuration
IN1 = 17
IN2 = 27
ENA = 18

MPU9250_ADDR = 0x68
I2C_BUS = 1
GYRO_XOUT_H = 0x43
GYRO_XOUT_L = 0x44
GYRO_YOUT_H = 0x45
GYRO_YOUT_L = 0x46
GYRO_ZOUT_H = 0x47
GYRO_ZOUT_L = 0x48
PWR_MGMT_1 = 0x6B
GYRO_CONFIG = 0x1B

class AOCSArUcoDocking:
    def __init__(self, communication_port=8888):
        self.current_angle = 0.0
        self.target_angle = 0.0
        self.gyro_bias = 0.0
        self.is_moving = False
        self.is_docking = False
        self.communication_port = communication_port
        self.docking_tolerance = 1.0
        self.docking_max_speed = 25
        self.correction_interval = 1.0
        self.Kp = 0.8
        self.Ki = 0.02
        self.Kd = 0.1
        self.error_sum = 0
        self.last_error = 0
        self.alpha = 0.85
        self.filtered_gyro = 0.0
        self.vision_data = {"angle_error": 0.0, "distance": 0.0, "detected": False}
        self.last_vision_update = 0
        self._current_duty = 0
        self.setup_gpio()
        self.setup_i2c()
        self.initialize_mpu9250()
        self.load_initialization_data()
        self.comm_thread = threading.Thread(target=self.communication_handler, daemon=True)
        self.comm_thread.start()

    def setup_gpio(self):
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(ENA, GPIO.OUT)
        GPIO.setup(IN1, GPIO.OUT)
        GPIO.setup(IN2, GPIO.OUT)
        self.pwm = GPIO.PWM(ENA, 1000)
        self.pwm.start(0)
        self.stop_motor()

    def setup_i2c(self):
        self.bus = smbus.SMBus(I2C_BUS)

    def initialize_mpu9250(self):
        self.bus.write_byte_data(MPU9250_ADDR, PWR_MGMT_1, 0x00)
        self.bus.write_byte_data(MPU9250_ADDR, GYRO_CONFIG, 0x00)
        time.sleep(0.1)

    def load_initialization_data(self):
        try:
            with open("aocs_init_data.json", 'r') as f:
                init_data = json.load(f)
                self.gyro_bias = init_data.get("gyro_bias", 0.0)
                print(f"Loaded gyro bias: {self.gyro_bias:.3f} deg/s")
        except Exception as e:
            print(f"Warning: Could not load initialization data: {e}")
            self.gyro_bias = 0.0

    def ramp_to_duty(self, target_duty, ramp_time=1.0, step_delay=0.02):
        if target_duty == 0:
            self.pwm.ChangeDutyCycle(0)
            self._current_duty = 0
            return
        steps = max(1, int(ramp_time / step_delay))
        duty_step = (target_duty - self._current_duty) / steps
        for i in range(steps):
            duty = self._current_duty + duty_step * (i + 1)
            self.pwm.ChangeDutyCycle(duty)
            time.sleep(step_delay)
        self.pwm.ChangeDutyCycle(target_duty)
        self._current_duty = target_duty

    def set_motor_direction(self, direction):
        if direction > 0:
            GPIO.output(IN1, GPIO.HIGH)
            GPIO.output(IN2, GPIO.LOW)
        elif direction < 0:
            GPIO.output(IN1, GPIO.LOW)
            GPIO.output(IN2, GPIO.HIGH)
        else:
            GPIO.output(IN1, GPIO.LOW)
            GPIO.output(IN2, GPIO.LOW)

    def set_motor_speed(self, speed):
        if speed == 0:
            self.ramp_to_duty(0, ramp_time=0.3)
        else:
            effective_speed = max(35, min(100, abs(speed)))
            self.ramp_to_duty(effective_speed, ramp_time=0.8)

    def stop_motor(self):
        self.set_motor_direction(0)
        self.ramp_to_duty(0, ramp_time=0.5)

    def rotate_360_degrees(self, angular_velocity=20):
        print(f"Starting 360° rotation at {angular_velocity} deg/s for ArUco detection")
        start_angle = self.current_angle
        total_rotation = 0.0
        target_rotation = 360.0
        direction = 1 if angular_velocity > 0 else -1
        dt = 0.02
        self.is_moving = True
        aruco_detections = []

        try:
            while total_rotation < target_rotation and self.is_moving:
                start_time = time.time()
                current_gyro = self.update_current_angle(dt)

                if self.vision_data["detected"]:
                    detection_info = {
                        "angle": self.current_angle,
                        "angle_error": self.vision_data["angle_error"],
                        "distance": self.vision_data["distance"],
                        "timestamp": time.time()
                    }
                    aruco_detections.append(detection_info)
                    print(f"ArUco detected at angle {self.current_angle:.1f}°, error: {self.vision_data['angle_error']:.1f}°")

                angle_change = abs(self.current_angle - start_angle)
                if angle_change > 180:
                    angle_change = 360 - angle_change
                total_rotation = angle_change

                gyro_error = abs(angular_velocity) - abs(current_gyro)
                motor_speed = max(35, min(60, 40 + 2.0 * gyro_error))

                self.set_motor_direction(direction)
                self.set_motor_speed(motor_speed)

                print(f"Rotation: {total_rotation:.1f}°/{target_rotation}°, Gyro: {current_gyro:.1f} deg/s, ArUco: {self.vision_data['detected']}")

                elapsed = time.time() - start_time
                time.sleep(max(0, dt - elapsed))

        except KeyboardInterrupt:
            print("Rotation interrupted")
        finally:
            self.stop_motor()
            self.is_moving = False
            print(f"Rotation complete. Total detections: {len(aruco_detections)}")
            return aruco_detections

    def move_to_angle_vision_assisted(self, target_angle, max_speed=40):
        self.target_angle = self.normalize_angle(target_angle)
        print(f"Moving to angle: {self.target_angle:.1f}° with vision assistance")

        dt = 0.02
        self.is_moving = True
        self.error_sum = 0
        self.last_error = 0
        settled_count = 0
        required_settled_count = 25

        try:
            while self.is_moving:
                start_time = time.time()
                self.update_current_angle(dt)
                gyro_error = self.normalize_angle(self.target_angle - self.current_angle)

                final_error = gyro_error
                if self.vision_data["detected"] and time.time() - self.last_vision_update < 0.5:
                    vision_error = self.vision_data["angle_error"]
                    vision_weight = min(1.0, abs(gyro_error) / 10.0)
                    final_error = vision_weight * vision_error + (1 - vision_weight) * gyro_error
                    print(f"Vision-assisted: Gyro error: {gyro_error:.1f}°, Vision error: {vision_error:.1f}°, Final: {final_error:.1f}°")

                if abs(final_error) < self.docking_tolerance:
                    settled_count += 1
                    if settled_count >= required_settled_count:
                        print(f"Target reached at {self.current_angle:.1f}°")
                        break
                else:
                    settled_count = 0

                self.error_sum += final_error * dt
                self.error_sum = max(-30, min(30, self.error_sum))
                error_rate = (final_error - self.last_error) / dt
                self.last_error = final_error

                control_output = (self.Kp * final_error +
                                  self.Ki * self.error_sum +
                                  self.Kd * error_rate)

                if abs(control_output) < 5:
                    self.stop_motor()
                else:
                    direction = 1 if control_output > 0 else -1
                    speed = min(abs(control_output), max_speed)
                    speed = max(speed * 0.6 if abs(final_error) < 5 else speed, 35)
                    self.set_motor_direction(direction)
                    self.set_motor_speed(speed)

                print(f"Current: {self.current_angle:.1f}°, Target: {self.target_angle:.1f}°, Error: {final_error:.1f}°, Speed: {speed:.0f}%")

                elapsed = time.time() - start_time
                time.sleep(max(0, dt - elapsed))
        except KeyboardInterrupt:
            print("Movement interrupted")
        finally:
            self.stop_motor()
            self.is_moving = False

    def update_current_angle(self, dt):
        gyro_rate = self.read_gyro_z()
        self.filtered_gyro = self.alpha * self.filtered_gyro + (1 - self.alpha) * gyro_rate
        self.current_angle = self.normalize_angle(self.current_angle + self.filtered_gyro * dt)
        return self.filtered_gyro

    def read_gyro_z(self):
        high = self.bus.read_byte_data(MPU9250_ADDR, GYRO_ZOUT_H)
        low = self.bus.read_byte_data(MPU9250_ADDR, GYRO_ZOUT_L)
        value = struct.unpack('>h', bytes([high, low]))[0]
        return value / 131.0 - self.gyro_bias

    def normalize_angle(self, angle):
        return (angle + 360) % 360

    def communication_handler(self):
        pass  # Left intentionally blank for your implementation

    def cleanup(self):
        self.stop_motor()
        if hasattr(self, 'pwm'):
            self.pwm.stop()
        GPIO.cleanup()
        print("Cleanup completed")
