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
        X.append(gpsX)
        Y.append(gpsY)
        lat.append(gpsLat)
        lng.append(gpsLng)

    plt.figure(1)
    plt.title('X')
    plt.plot(time, X)

    plt.figure(2)
    plt.title('Y')
    plt.plot(time, Y)

    plt.figure(3)
    plt.title('Lat')
    plt.plot(time, lat)

    plt.figure(4)
    plt.title('Lng')
    plt.plot(time, lng)

    plt.figure(5)
    plt.title('path')
    plt.plot(X, Y)

    plt.figure(6)
    plt.title('globe')
    plt.plot(lat, lng)
    plt.show()
