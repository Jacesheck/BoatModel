import pickle
import struct
import matplotlib.pyplot as plt
import sys
import numpy as np

from Simulation import Simulation

IDLE_POWER: int = 1500

def decode_data(bytes) -> dict[str, list[int] | list[float]]:
    # Create emptry dict
    decoded_data = {
        'timestamp': [],
        'gpsX': [],
        'gpsY': [],
        'gpsLat': [],
        'gpsLng': [],
        'power1': [],
        'power2': [],
        'rz': [],

    } 

    for point in bytes:
        decoded_data['timestamp'].append(point[0])
        telemetry = point[1]
        i = 0
        decoded_data['gpsX'].append(struct.unpack('<d', telemetry[i: i+8])[0])
        i += 8
        decoded_data['gpsY'].append(struct.unpack('<d', telemetry[i: i+8])[0])
        i += 8
        decoded_data['gpsLat'].append(struct.unpack('<d', telemetry[i: i+8])[0])
        i += 8
        decoded_data['gpsLng'].append(struct.unpack('<d', telemetry[i: i+8])[0])
        i += 8
        decoded_data['power1'].append(struct.unpack('<l', telemetry[i: i+4])[0])
        i += 4
        decoded_data['power2'].append(struct.unpack('<l', telemetry[i: i+4])[0])
        i += 4
        decoded_data['rz'].append(struct.unpack('<f', telemetry[i: i+4])[0])
        i += 4

        # Normalise motor powers
        # TODO: Debug here
        decoded_data['power1'][-1] = -IDLE_POWER + decoded_data['power1'][-1]
        decoded_data['power2'][-1] = +IDLE_POWER - decoded_data['power2'][-1]
    return decoded_data
    
def plot_raw(X, Y, lat, lng, motor1, motor2, gyro, timestamps):
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

if __name__ == "__main__":
    args = sys.argv
    if len(args) != 2:
        print('No filename given')
        exit()

    filename = args[1]
    with open(filename, 'rb') as f:
        text = f.read()
    data = pickle.loads(text)
    decoded_data = decode_data(data)

    timestamps = decoded_data['timestamp']
    X = decoded_data['gpsX']
    Y = decoded_data['gpsY']
    lat = decoded_data['gpsLat']
    lng = decoded_data['gpsLng']
    motor1 = decoded_data['power1']
    motor2 = decoded_data['power2']
    gyro = decoded_data['rz']

    #sim = Simulation()
    #for decoded_point in decoded_data:
    #    sim.step(decoded_point)

    plot_raw(X, Y, lat, lng, motor1, motor2, gyro, timestamps)
    
    #sim.show()
