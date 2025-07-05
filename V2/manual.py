import time
import smbus
import RPi.GPIO as GPIO
import threading
import sys
import tty
import termios

# Pin Configuration
IN1 = 17     # GPIO17 (Pin 11) - Motor direction pin 1
IN2 = 27     # GPIO27 (Pin 13) - Motor direction pin 2
ENA = 18     # GPIO18 (Pin 12) - PWM pin for controlling motor speed (EnA)

# MPU9250 I2C Configuration
MPU9250_ADDR = 0x68
I2C_BUS = 1

# MPU9250 Register Addresses
GYRO_ZOUT_H = 0x47
PWR_MGMT_1 = 0x6B
GYRO_CONFIG = 0x1B

class ManualControl:
    def __init__(self):
        self.running = True
        self.current_angle = 0.0
        self.current_speed = 0
        self.motor_direction = 0
        self._current_duty = 0

        self.setup_gpio()
        self.setup_i2c()
        self.initialize_mpu9250()
        self.init_kalman()

        self.monitor_thread = threading.Thread(target=self.monitor_position, daemon=True)
        self.monitor_thread.start()

        print("Manual Control Initialized")
        print("Use 'a' to turn left, 'd' to turn right")
        print("Press 'q' to quit")

    def setup_gpio(self):
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(ENA, GPIO.OUT)
        GPIO.setup(IN1, GPIO.OUT)
        GPIO.setup(IN2, GPIO.OUT)
        self.pwm = GPIO.PWM(ENA, 1000)
        self.pwm.start(0)
        self.stop_motor()

    def setup_i2c(self):
        try:
            self.bus = smbus.SMBus(I2C_BUS)
        except Exception as e:
            print(f"Warning: Could not initialize I2C: {e}")
            self.bus = None

    def initialize_mpu9250(self):
        if self.bus is None:
            return
        try:
            self.bus.write_byte_data(MPU9250_ADDR, PWR_MGMT_1, 0x00)
            self.bus.write_byte_data(MPU9250_ADDR, GYRO_CONFIG, 0x00)
            time.sleep(0.1)
            print("MPU9250 initialized")
        except Exception as e:
            print(f"Warning: Could not initialize MPU9250: {e}")

    def read_gyro_z(self):
        if self.bus is None:
            return 0.0
        try:
            data = self.bus.read_i2c_block_data(MPU9250_ADDR, GYRO_ZOUT_H, 2)
            gyro_z = (data[0] << 8) | data[1]
            if gyro_z > 32767:
                gyro_z -= 65536
            return gyro_z / 131.0
        except Exception:
            return 0.0

    def init_kalman(self):
        self.angle_est = 0.0
        self.bias_est = 0.0
        self.P = [[1, 0], [0, 1]]
        self.Q_angle = 0.001
        self.Q_bias = 0.003
        self.R_measure = 0.03

    def kalman_update(self, new_rate, dt):
        rate = new_rate - self.bias_est
        self.angle_est += dt * rate

        self.P[0][0] += dt * (dt*self.P[1][1] - self.P[0][1] - self.P[1][0] + self.Q_angle)
        self.P[0][1] -= dt * self.P[1][1]
        self.P[1][0] -= dt * self.P[1][1]
        self.P[1][1] += self.Q_bias * dt

        S = self.P[0][0] + self.R_measure
        K = [self.P[0][0]/S, self.P[1][0]/S]

        y = new_rate - rate
        self.angle_est += K[0] * y
        self.bias_est += K[1] * y

        P00_temp = self.P[0][0]
        P01_temp = self.P[0][1]

        self.P[0][0] -= K[0] * P00_temp
        self.P[0][1] -= K[0] * P01_temp
        self.P[1][0] -= K[1] * P00_temp
        self.P[1][1] -= K[1] * P01_temp

    def monitor_position(self):
        dt = 0.05
        while self.running:
            try:
                gyro_z = self.read_gyro_z()
                self.kalman_update(gyro_z, dt)
                self.current_angle = self.angle_est % 360
                time.sleep(dt)
            except Exception:
                time.sleep(dt)

    def ramp_to_duty(self, target_duty, ramp_time=0.3, step_delay=0.02):
        if target_duty == 0:
            ramp_time = 0.2
        current_duty = self._current_duty
        steps = int(ramp_time / step_delay) or 1
        duty_step = (target_duty - current_duty) / steps
        for i in range(steps):
            duty = current_duty + duty_step * (i + 1)
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
        self.motor_direction = direction

    def set_motor_speed(self, speed):
        speed = max(0, min(100, speed))
        if speed == 0:
            self.ramp_to_duty(0, ramp_time=0.2)
        else:
            effective_speed = max(40, speed)
            self.ramp_to_duty(effective_speed, ramp_time=0.3)
        self.current_speed = speed

    def stop_motor(self):
        self.set_motor_direction(0)
        self.ramp_to_duty(0, ramp_time=0.2)
        self.current_speed = 0

    def turn_left(self, speed=50):
        self.set_motor_direction(-1)
        self.set_motor_speed(speed)
        print(f"Turning LEFT at {speed}% speed")

    def turn_right(self, speed=50):
        self.set_motor_direction(1)
        self.set_motor_speed(speed)
        print(f"Turning RIGHT at {speed}% speed")

    def get_char(self):
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            char = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return char

    def log_position(self):
        timestamp = time.strftime("%H:%M:%S")
        gyro_z = self.read_gyro_z()
        log_entry = f"[{timestamp}] Position: {self.current_angle:.1f}°, Gyro: {gyro_z:.1f} deg/s, Speed: {self.current_speed}%, Direction: {self.motor_direction}"
        print(log_entry)
        try:
            with open("position_log.txt", "a") as f:
                f.write(log_entry + "\n")
        except Exception as e:
            print(f"Warning: Could not write to log file: {e}")

    def run(self):
        try:
            print("\n=== Manual Control Active ===")
            print("Commands:")
            print("  'a' - Turn LEFT")
            print("  'd' - Turn RIGHT")
            print("  's' - STOP")
            print("  'p' - Show position")
            print("  SPACE - Log position to file")
            print("  'q' - Quit")
            print("\nPress keys (no Enter needed):")

            while self.running:
                try:
                    char = self.get_char()
                    if char.lower() == 'q':
                        print("\nQuitting...")
                        break
                    elif char.lower() == 'a':
                        self.turn_left()
                    elif char.lower() == 'd':
                        self.turn_right()
                    elif char.lower() == 's':
                        self.stop_motor()
                        print("STOPPED")
                    elif char.lower() == 'p':
                        print(f"Position: {self.current_angle:.1f}°, Speed: {self.current_speed}%, Direction: {self.motor_direction}")
                    elif char == ' ':
                        self.log_position()
                    elif char == '\x03':  # Ctrl+C
                        break
                except KeyboardInterrupt:
                    break
                except Exception as e:
                    print(f"Error: {e}")
        except Exception as e:
            print(f"Error in main loop: {e}")
        finally:
            self.cleanup()

    def cleanup(self):
        self.running = False
        self.stop_motor()
        time.sleep(0.3)
        if hasattr(self, 'pwm'):
            self.pwm.stop()
        GPIO.cleanup()
        print("Cleanup completed")

def main():
    try:
        controller = ManualControl()
        controller.run()
    except KeyboardInterrupt:
        print("\nInterrupted by user")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        GPIO.cleanup()

if __name__ == "__main__":
    main()
