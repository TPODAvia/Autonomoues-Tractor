import smbus
import time

bus = smbus.SMBus(1)  # Create a new I2C bus
address = 0x68  # I2C address of the device

while True:
    data = bus.read_byte(address)  # Read a single byte from the device
    # gyro_data = bus.read_i2c_block_data(0x5c, 0, 7)
    # print(gyro_data)
    print(data)
    time.sleep(0.1)