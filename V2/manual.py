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
GYRO_XOUT_H = 0x43
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
        
        # Setup hardware
        self.setup_gpio()
        self.setup_i2c()
        self.initialize_mpu9250()
        
        # Start monitoring thread
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
            return gyro_z / 131.0  # Convert to degrees per second
        except Exception as e:
            return 0.0
    
    def ramp_to_duty(self, target_duty, ramp_time=0.3, step_delay=0.02):
        """Smoothly ramp PWM duty cycle to target value"""
        if target_duty == 0:
            ramp_time = 0.2  # Quick stop
        
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
        """Set motor direction: 1=right, -1=left, 0=stop"""
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
        """Set motor speed with ramping"""
        speed = max(0, min(100, speed))
        
        if speed == 0:
            self.ramp_to_duty(0, ramp_time=0.2)
        else:
            effective_speed = max(40, speed)  # Minimum speed for reliable operation
            self.ramp_to_duty(effective_speed, ramp_time=0.3)
        
        self.current_speed = speed
    
    def stop_motor(self):
        """Stop motor smoothly"""
        self.set_motor_direction(0)
        self.ramp_to_duty(0, ramp_time=0.2)
        self.current_speed = 0
    
    def turn_left(self, speed=50):
        """Turn left at specified speed"""
        self.set_motor_direction(-1)
        self.set_motor_speed(speed)
        print(f"Turning LEFT at {speed}% speed")
    
    def turn_right(self, speed=50):
        """Turn right at specified speed"""
        self.set_motor_direction(1)
        self.set_motor_speed(speed)
        print(f"Turning RIGHT at {speed}% speed")
    
    def monitor_position(self):
        """Monitor gyroscope and angle in background thread"""
        dt = 0.05  # 20Hz update rate
        gyro_filtered = 0.0
        alpha = 0.8
        
        while self.running:
            try:
                gyro_z = self.read_gyro_z()
                gyro_filtered = alpha * gyro_filtered + (1 - alpha) * gyro_z
                self.current_angle += gyro_filtered * dt
                
                # Normalize angle to -180 to 180 range
                while self.current_angle > 180:
                    self.current_angle -= 360
                while self.current_angle < -180:
                    self.current_angle += 360
                
                time.sleep(dt)
            except Exception as e:
                time.sleep(dt)
    
    def get_char(self):
        """Get a single character from stdin without pressing Enter"""
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            char = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return char
    
    def log_position(self):
        """Log current position with timestamp"""
        timestamp = time.strftime("%H:%M:%S")
        gyro_z = self.read_gyro_z()
        log_entry = f"[{timestamp}] Position: {self.current_angle:.1f}°, Gyro: {gyro_z:.1f} deg/s, Speed: {self.current_speed}%, Direction: {self.motor_direction}"
        print(log_entry)
        
        # Also save to file
        try:
            with open("position_log.txt", "a") as f:
                f.write(log_entry + "\n")
        except Exception as e:
            print(f"Warning: Could not write to log file: {e}")
    
    def run(self):
        """Main control loop"""
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
                    
                    if char == 'q' or char == 'Q':
                        print("\nQuitting...")
                        break
                    elif char == 'a' or char == 'A':
                        self.turn_left()
                    elif char == 'd' or char == 'D':
                        self.turn_right()
                    elif char == 's' or char == 'S':
                        self.stop_motor()
                        print("STOPPED")
                    elif char == 'p' or char == 'P':
                        print(f"Position: {self.current_angle:.1f}°, Speed: {self.current_speed}%, Direction: {self.motor_direction}")
                    elif char == ' ':  # Space bar
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
        """Clean up GPIO and stop threads"""
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