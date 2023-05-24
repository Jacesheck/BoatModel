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
    for point in data:
        time.append(point[0])
        telemetry = point[1]
        gpsX = struct.unpack('<d', telemetry[0:8])
        gpsY = struct.unpack('<d', telemetry[8:16])
        X.append(gpsX)
        Y.append(gpsY)

    plt.plot(time, X)
    plt.plot(time, Y)
    plt.show()
