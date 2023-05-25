import pickle
import numpy as np
import struct
import matplotlib.pyplot as plt
import sys

if __name__ == "__main__":
    args = sys.argv
    if len(args) != 2:
        print('No filename given')
        exit()

    filename = args[1]
    with open(filename, 'rb') as f:
        text = f.read()
    data = pickle.loads(text)

    time = []
    X = []
    Y = []
    lat = []
    lng = []
    motor1 = []
    motor2 = []
    for point in data:
        time.append(point[0])
        telemetry = point[1]
        i = 0
        gpsX = struct.unpack('<d', telemetry[i: i+8])
        i += 8
        gpsY = struct.unpack('<d', telemetry[i: i+8])
        i += 8
        gpsLat = struct.unpack('<d', telemetry[i: i+8])
        i += 8
        gpsLng = struct.unpack('<d', telemetry[i: i+8])
        i += 8
        power1 = struct.unpack('<l', telemetry[i: i+4])
        print(i)
        i += 4
        power2 = struct.unpack('<l', telemetry[i: i+4])
        print(i)
        i += 4
        X.append(gpsX)
        Y.append(gpsY)
        lat.append(gpsLat)
        lng.append(gpsLng)
        motor1.append(power1)
        motor2.append(power2)

    plt.figure(1)
    plt.title('path')
    plt.plot(X, Y)

    plt.figure(2)
    plt.title('globe')
    plt.plot(lat, lng)

    plt.figure(3)
    plt.title('motors')
    plt.plot(time, motor1)
    plt.plot(time, motor2)

    plt.show()
