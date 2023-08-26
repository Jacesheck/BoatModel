import pickle
import matplotlib.pyplot as plt
import sys
import numpy as np

from Simulation import Simulation
from DataObject import DataObject
    
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
        #exit()
    args.append("Telemetry/Path_FriMay2612.48.312023")

    filename = args[1]
    with open(filename, 'rb') as f:
        text = f.read()
    data = pickle.loads(text)
    decoded_data = DataObject(data)

    timestamps = decoded_data['timestamp']
    X = decoded_data['gpsX']
    Y = decoded_data['gpsY']
    lat = decoded_data['gpsLat']
    lng = decoded_data['gpsLng']
    motor1 = decoded_data['power1']
    motor2 = decoded_data['power2']
    gyro = decoded_data['rz']
    # TODO: Debug above

    sim = Simulation(decoded_data)
    sim.run()
    sim.showStatic()
    while(1):
        print("\n(tune) (show) (exit)")
        command = input("Command:")
        if command == "show":
            sim.run()
            sim.show()
        elif command == "tune":
            sim.tune()
        elif command == "exit":
            exit()
        else:
            print(f"Invalid command: {command}")

    # Plot raw values (useful for debugging)
    #plot_raw(X, Y, lat, lng, motor1, motor2, gyro, timestamps)