import time
import smbus
import math

# Constants
MPU9250_ADDR = 0x68
AK8963_ADDR = 0x0C
I2C_BUS = 1

# Registers
PWR_MGMT_1 = 0x6B
INT_PIN_CFG = 0x37
AK8963_CNTL = 0x0A
AK8963_ST1 = 0x02
AK8963_XOUT_L = 0x03

def read_mag_data(bus):
    # Wait for data ready
    while True:
        status = bus.read_byte_data(AK8963_ADDR, AK8963_ST1)
        if status & 0x01:
            break
        time.sleep(0.01)

    data = bus.read_i2c_block_data(AK8963_ADDR, AK8963_XOUT_L, 7)
    x = (data[1] << 8) | data[0]
    y = (data[3] << 8) | data[2]
    z = (data[5] << 8) | data[4]

    # Convert to signed 16-bit
    x = x - 65536 if x > 32767 else x
    y = y - 65536 if y > 32767 else y
    z = z - 65536 if z > 32767 else z

    return x, y, z

def get_heading(mx, my):
    heading_rad = math.atan2(my, mx)
    heading_deg = math.degrees(heading_rad)
    if heading_deg < 0:
        heading_deg += 360
    return heading_deg

def main():
    try:
        bus = smbus.SMBus(I2C_BUS)

        # Wake up MPU9250
        bus.write_byte_data(MPU9250_ADDR, PWR_MGMT_1, 0x00)
        time.sleep(0.1)

        # Enable bypass to access magnetometer
        bus.write_byte_data(MPU9250_ADDR, INT_PIN_CFG, 0x02)
        time.sleep(0.01)

        # Power down magnetometer, then set to continuous measurement mode
        bus.write_byte_data(AK8963_ADDR, AK8963_CNTL, 0x00)
        time.sleep(0.01)
        bus.write_byte_data(AK8963_ADDR, AK8963_CNTL, 0x16)  # 100Hz, 16-bit
        time.sleep(0.01)

        print("Magnetometer initialised. Reading heading...")

        while True:
            mx, my, mz = read_mag_data(bus)
            heading = get_heading(mx, my)
            print(f"Heading: {heading:.2f}°")
            time.sleep(0.5)

    except KeyboardInterrupt:
        print("\nStopped by user.")
    except Exception as e:
        print(f"Error: {e}")

if __name__ == "__main__":
    main()
