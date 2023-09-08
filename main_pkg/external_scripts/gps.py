import serial

ser = serial.Serial('/dev/ttyACM0')
while True:
    ser_bytes = ser.readline()
    if not ser_bytes:
        break
    decoded_bytes = ser_bytes.decode('utf-8')
    # print(decoded_bytes)

    if "GNGGA" in decoded_bytes: #GPGGA
        # print(decoded_bytes)
        if int(decoded_bytes.split(",")[7]) > 1:
            print("Time: ", float(decoded_bytes.split(",")[1]))
            print("Latitude: ", float(decoded_bytes.split(",")[2])*0.01)
            print("Longitude: ", float(decoded_bytes.split(",")[4])*0.01)
            print("Altitude: ", float(decoded_bytes.split(",")[9]))
            print("geoid above WGS84: ", float(decoded_bytes.split(",")[11]))
            print("Number of Satellites: ", int(decoded_bytes.split(",")[7]))
            break

        else:
            print("Not enought sattelite: ", decoded_bytes.split(",")[7])
            break
