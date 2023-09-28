import matplotlib.pyplot as plt
import json
import sys
import numpy as np

from libs.Simulation import Simulation
from libs.DataObject import DataObject

def main():
    args = sys.argv
    if len(args) != 2:
        print('No filename given')
        #exit()
        args.append("Telemetry/Path_FriMay2612.48.312023")

    filename = args[1]
    with open(filename, 'r') as f:
        data = json.load(f)
    decoded_data = DataObject(data)

    sim = Simulation(decoded_data)
    sim.createScaleFromGPS(1000, 600)
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

    
def plot_raw(X, Y, lat, lng, motor1, motor2, gyro, timestamps):
    "Plot raw deserialised data (sanity check)"
    # TODO: Make more generic
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
    main()
