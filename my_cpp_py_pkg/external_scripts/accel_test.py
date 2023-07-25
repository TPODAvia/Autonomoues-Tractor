from smbus2 import SMBus

# MPU9250 Registers
MPU9250_ADDR = 0x68
ACCEL_XOUT_H = 0x3B
ACCEL_XOUT_L = 0x3C

# Function to read the accelerometer data
def read_accel(bus):
    high = bus.read_byte_data(MPU9250_ADDR, ACCEL_XOUT_H)
    low = bus.read_byte_data(MPU9250_ADDR, ACCEL_XOUT_L)

    # Combine high and low bytes
    value = (high << 8) | low

    # Convert to signed value
    if value > 32767:
        value -= 65536

    return value

# Main function
def main():
    bus = SMBus(1)  # Create I2C bus object

    while True:
        accel_x = read_accel(bus)
        print("Accelerometer X:", accel_x)

if __name__ == "__main__":
    main()
