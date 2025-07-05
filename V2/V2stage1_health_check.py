import time
import smbus
import RPi.GPIO as GPIO
import numpy as np
import json

# Pin Configuration
IN1 = 17
IN2 = 27
ENA = 18

# MPU9250 I2C Configuration
MPU9250_ADDR = 0x68
I2C_BUS = 1

# MPU9250 Register Addresses
GYRO_XOUT_H = 0x43
GYRO_XOUT_L = 0x44
GYRO_YOUT_H = 0x45
GYRO_YOUT_L = 0x46
GYRO_ZOUT_H = 0x47
GYRO_ZOUT_L = 0x48
PWR_MGMT_1 = 0x6B
GYRO_CONFIG = 0x1B

class AOCSHealthCheck:
    def __init__(self):
        self.status = 0
        self.initial_angle = 0.0
        self.calibration_samples = []
        self.gyro_bias = 0.0
        self._current_duty = 0
        self.setup_gpio()
        self.setup_i2c()

    def setup_gpio(self):
        try:
            GPIO.setmode(GPIO.BCM)
            GPIO.setup(ENA, GPIO.OUT)
            GPIO.setup(IN1, GPIO.OUT)
            GPIO.setup(IN2, GPIO.OUT)
            self.pwm = GPIO.PWM(ENA, 1000)
            self.pwm.start(0)
            GPIO.output(IN1, GPIO.LOW)
            GPIO.output(IN2, GPIO.LOW)
            print("GPIO setup successful")
            return True
        except Exception as e:
            print(f"GPIO setup failed: {e}")
            return False

    def setup_i2c(self):
        try:
            self.bus = smbus.SMBus(I2C_BUS)
            return True
        except Exception as e:
            print(f"I2C setup failed: {e}")
            return False

    def initialize_mpu9250(self):
        try:
            self.bus.write_byte_data(MPU9250_ADDR, PWR_MGMT_1, 0x00)
            self.bus.write_byte_data(MPU9250_ADDR, GYRO_CONFIG, 0x00)
            time.sleep(0.1)
            print("MPU9250 IMU initialized successfully")
            return True
        except Exception as e:
            print(f"Failed to initialize MPU9250 IMU: {e}")
            return False

    def read_gyro_data(self):
        try:
            data = self.bus.read_i2c_block_data(MPU9250_ADDR, GYRO_XOUT_H, 6)
            gyro_x = (data[0] << 8) | data[1]
            gyro_y = (data[2] << 8) | data[3]
            gyro_z = (data[4] << 8) | data[5]
            if gyro_x > 32767: gyro_x -= 65536
            if gyro_y > 32767: gyro_y -= 65536
            if gyro_z > 32767: gyro_z -= 65536
            return {'x': gyro_x / 131.0, 'y': gyro_y / 131.0, 'z': gyro_z / 131.0}
        except Exception as e:
            print(f"Error reading gyroscope data: {e}")
            return None

    def ramp_to_duty(self, target_duty, ramp_time=2.0, step_delay=0.05):
        if target_duty == 0:
            self.pwm.ChangeDutyCycle(0)
            return
        steps = int(ramp_time / step_delay)
        for i in range(steps + 1):
            duty = (target_duty / steps) * i
            self.pwm.ChangeDutyCycle(duty)
            time.sleep(step_delay)
        self.pwm.ChangeDutyCycle(target_duty)

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
            self.ramp_to_duty(0, ramp_time=0.5)
        else:
            effective_speed = max(35, min(100, abs(speed)))
            self.ramp_to_duty(effective_speed, ramp_time=1.0)

    def stop_motor(self):
        self.set_motor_direction(0)
        self.ramp_to_duty(0, ramp_time=0.5)

    def motor_health_check(self):
        print("Testing motor functionality...")
        try:
            print("Testing forward direction...")
            self.set_motor_direction(1)
            self.set_motor_speed(40)
            time.sleep(2.0)

            print("Testing reverse direction...")
            self.set_motor_direction(-1)
            self.set_motor_speed(40)
            time.sleep(2.0)

            print("Stopping motor...")
            self.stop_motor()

            print("Motor health check passed")
            return True
        except Exception as e:
            print(f"Motor health check failed: {e}")
            return False
    
    def calibrate_gyroscope(self, samples=100):
        """Calibrate gyroscope by calculating bias"""
        print(f"Calibrating gyroscope with {samples} samples...")
        print("Please keep the CubeSat stationary during calibration...")
        
        gyro_sum = 0.0
        valid_samples = 0
        
        for i in range(samples):
            gyro_data = self.read_gyro_data()
            if gyro_data is not None:
                gyro_sum += gyro_data['z']
                valid_samples += 1
                print(f"Calibration progress: {i+1}/{samples}", end='\r')
            time.sleep(0.01)
        
        if valid_samples > samples * 0.8:  # At least 80% valid samples
            self.gyro_bias = gyro_sum / valid_samples
            print(f"\nGyroscope calibration complete. Bias: {self.gyro_bias:.3f} deg/s")
            return True
        else:
            print(f"\nGyroscope calibration failed. Only {valid_samples}/{samples} valid samples")
            return False
    
    def system_health_check(self):
        """Comprehensive system health check"""
        print("=== AOCS Health Check Started ===")
        
        # Check 1: IMU Initialization
        if not self.initialize_mpu9250():
            self.status = -1
            return self.status
        
        # Check 2: Gyroscope readings
        gyro_test_count = 0
        for _ in range(10):
            gyro_data = self.read_gyro_data()
            if gyro_data is not None:
                gyro_test_count += 1
            time.sleep(0.1)
        
        if gyro_test_count < 8:
            print("Gyroscope readings unstable")
            self.status = -1
            return self.status
        
        # Check 3: Motor functionality
        if not self.motor_health_check():
            self.status = -1
            return self.status
        
        # Check 4: Gyroscope calibration
        if not self.calibrate_gyroscope():
            self.status = -1
            return self.status
        
        print("=== All Health Checks Passed ===")
        self.status = 0  # Ready to initialize
        return self.status
    
    def initialize_reference_position(self):
        """Initialize the current position as reference (0 degrees)"""
        print("Initializing reference position...")
        
        # Take several readings to establish initial position
        angle_readings = []
        for _ in range(50):
            gyro_data = self.read_gyro_data()
            if gyro_data is not None:
                # Correct for bias
                corrected_gyro = gyro_data['z'] - self.gyro_bias
                angle_readings.append(corrected_gyro)
            time.sleep(0.02)
        
        if len(angle_readings) > 40:
            # Check if system is relatively stationary
            gyro_std = np.std(angle_readings)
            if gyro_std < 2.0:  # Less than 2 deg/s standard deviation
                self.initial_angle = 0.0  # Set current position as reference
                print(f"Reference position initialized. Gyro stability: ±{gyro_std:.2f} deg/s")
                self.status = 1  # Initialization complete
                return self.status
            else:
                print(f"System not stable enough for initialization. Gyro std: {gyro_std:.2f} deg/s")
                self.status = -1
                return self.status
        else:
            print("Insufficient readings for initialization")
            self.status = -1
            return self.status
    
    def get_status(self):
        """Return current status"""
        return self.status
    
    def get_status_message(self):
        """Return human-readable status message"""
        status_messages = {
            -1: "FAILED - System not ready",
            0: "READY - Ready to initialize",
            1: "DONE - Initialization complete"
        }
        return status_messages.get(self.status, "UNKNOWN")
    
    def cleanup(self):
        """Clean up GPIO and resources"""
        try:
            if hasattr(self, 'pwm'):
                self.pwm.stop()
            GPIO.cleanup()
            print("Cleanup completed")
        except Exception as e:
            print(f"Cleanup error: {e}")
    
    def save_initialization_data(self, filename="aocs_init_data.json"):
        """Save initialization data to file"""
        init_data = {
            "timestamp": time.time(),
            "initial_angle": self.initial_angle,
            "gyro_bias": self.gyro_bias,
            "status": self.status
        }
        
        try:
            with open(filename, 'w') as f:
                json.dump(init_data, f, indent=2)
            print(f"Initialization data saved to {filename}")
        except Exception as e:
            print(f"Failed to save initialization data: {e}")

def main():
    aocs = AOCSHealthCheck()
    
    try:
        # Step 1: Health Check
        status = aocs.system_health_check()
        print(f"Health Check Status: {aocs.get_status_message()}")
        
        if status == 0:  # Ready to initialize
            # Step 2: Initialize reference position
            status = aocs.initialize_reference_position()
            print(f"Initialization Status: {aocs.get_status_message()}")
            
            if status == 1:  # Initialization successful
                aocs.save_initialization_data()
                print("AOCS Stage 1 completed successfully")
            else:
                print("AOCS Stage 1 failed during initialization")
        else:
            print("AOCS Stage 1 failed during health check")
        
        return status
        
    except KeyboardInterrupt:
        print("\nOperation interrupted by user")
        return -1
    except Exception as e:
        print(f"Unexpected error: {e}")
        return -1
    finally:
        aocs.cleanup()

if __name__ == "__main__":
    exit_code = main()
    print(f"Stage 1 completed with status: {exit_code}")

