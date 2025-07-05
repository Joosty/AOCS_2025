import smbus
import time

bus = smbus.SMBus(1)

MPU9250_ADDR = 0x68
AK8963_ADDR = 0x0C
PWR_MGMT_1 = 0x6B
INT_PIN_CFG = 0x37

try:
    # Wake up MPU9250
    bus.write_byte_data(MPU9250_ADDR, PWR_MGMT_1, 0x00)
    print("MPU9250 found at 0x68")

    # Enable bypass mode
    bus.write_byte_data(MPU9250_ADDR, INT_PIN_CFG, 0x02)
    time.sleep(0.1)

    # Try reading WHO_AM_I from magnetometer
    who_am_i = bus.read_byte_data(AK8963_ADDR, 0x00)
    print(f"AK8963 WHO_AM_I = 0x{who_am_i:02X}")
    
    if who_am_i == 0x48:
        print("Magnetometer detected successfully.")
    else:
        print("Unexpected response from magnetometer.")

except Exception as e:
    print(f"Error: {e}")
