import pickle
import struct
import matplotlib.pyplot as plt
import sys
import numpy as np
from Simulation import Simulation

if __name__ == "__main__":
    args = sys.argv
    if len(args) != 2:
        print('No filename given')
        exit()

    filename = args[1]
    with open(filename, 'rb') as f:
        text = f.read()
    data = pickle.loads(text)

    timestamps = []
    X = []
    Y = []
    lat = []
    lng = []
    motor1 = []
    motor2 = []
    gyro = []
    sim = Simulation()
    decodedData = {}
    for point in data:
        decodedData['timestamp'] = point[0]
        timestamps.append(decodedData['timestamp'])
        telemetry = point[1]
        i = 0
        decodedData['gpsX'] = struct.unpack('<d', telemetry[i: i+8])[0]
        i += 8
        decodedData['gpsY'] = struct.unpack('<d', telemetry[i: i+8])[0]
        i += 8
        decodedData['gpsLat'] = struct.unpack('<d', telemetry[i: i+8])[0]
        i += 8
        decodedData['gpsLng'] = struct.unpack('<d', telemetry[i: i+8])[0]
        i += 8
        decodedData['power1'] = struct.unpack('<l', telemetry[i: i+4])[0]
        i += 4
        decodedData['power2'] = struct.unpack('<l', telemetry[i: i+4])[0]
        i += 4
        decodedData['rz'] = struct.unpack('<f', telemetry[i: i+4])[0]
        i += 4

        decodedData['power1'] = -1500 + decodedData['power1']
        decodedData['power2'] = +1500 - decodedData['power2']

        X.append(decodedData['gpsX'])
        Y.append(decodedData['gpsY'])
        lat.append(decodedData['gpsLat'])
        lng.append(decodedData['gpsLng'])
        motor1.append(decodedData['power1'])
        motor2.append(decodedData['power2'])
        gyro.append(decodedData['rz'])

        sim.step(decodedData)

    plot = False
    if plot:
        timestamps = np.array(timestamps) - timestamps[0]
        plt.figure(1)
        plt.subplot(2, 2, 4)
        plt.title('path')
        plt.plot(X, Y)
        plt.grid()
        plt.xlabel('x (metres)')
        plt.ylabel('y (metres)')
        plt.subplot(2, 2, 2)
        plt.title('x coordinate over time')
        plt.plot(timestamps, X)
        plt.grid()
        plt.xlabel('Time (s)')
        plt.ylabel('X Coordinate (m)')
        plt.subplot(2, 2, 3)
        plt.title('y coordinate over time')
        plt.plot(timestamps, Y)
        plt.grid()
        plt.xlabel('Time (s)')
        plt.ylabel('Y Coordinate (m)')

        plt.figure(2)
        plt.title('globe')
        plt.plot(lat, lng)

        plt.figure(3)
        plt.subplot(1, 2, 1)
        plt.title('motors')
        plt.plot(timestamps, np.array(motor1)/4, label='Left motor')
        plt.plot(timestamps, np.array(motor2)/4, label='Right motor')
        plt.xlabel('Time (s)')
        plt.ylabel('Motor power (%)')
        plt.legend()
        plt.grid()
        plt.subplot(1, 2, 2)
        plt.title('Gyro')
        plt.xlabel('Time (s)')
        plt.ylabel('Rotational accel (deg/s)')
        plt.grid()
        plt.plot(timestamps, gyro)

        plt.show()
    else:
        sim.show()
