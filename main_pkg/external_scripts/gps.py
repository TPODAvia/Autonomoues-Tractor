import serial

ser = serial.Serial('/dev/ttyUSB0')
while True:
    ser_bytes = ser.readline()
    if not ser_bytes:
        break
    decoded_bytes = ser_bytes.decode('utf-8')
    print(decoded_bytes)