import matplotlib.pyplot as plt
import numpy as np
import json
import sys

from libs.Simulation import Simulation
from libs.DataObject import DataObject

class Grapher(Simulation):
    def run(self):
        for i in range(len(self.data)):
            self.step(self.data.at(i))
        self.stateHistory = np.array(self.tempStateHistory)
        self.motorHistory = np.array(self.tempMotorHistory)

    def graph_states(self):
        self.reset()
        self.run()

        time = np.arange(0, len(self.stateHistory), 1) / 10
        print(len(time))
        print(time)
        print(len(self.stateHistory))
        assert(len(time) == len(self.stateHistory))
        plt.subplot(3, 1, 1)
        plt.title('Heading')
        plt.ylabel("Heading (deg)")
        plt.xlabel("Time (s)")
        plt.plot(time, self.stateHistory[:, 4, 0], label="Heading")
        plt.legend()
        plt.grid()

        plt.subplot(3, 1, 2)
        plt.title("Displacment")
        plt.ylabel("Displacment (m)")
        plt.xlabel("Time (s)")
        plt.plot(time, self.stateHistory[:, 0, 0], label="x")
        plt.plot(time, self.stateHistory[:, 1, 0], label="y")
        plt.legend()
        plt.grid()

        plt.subplot(3, 1, 3)
        plt.title("Velocity")
        plt.ylabel("Velocity (m/)")
        plt.xlabel("Time (s)")
        plt.plot(time, self.stateHistory[:, 2, 0], label="Vx")
        plt.plot(time, self.stateHistory[:, 3, 0], label="Vy")
        plt.legend()
        plt.grid()
        plt.show()

def main():
    args = sys.argv
    if len(args) != 2:
        print("Must give filename")
        exit()

    with open(args[1],"r") as f:
        data = DataObject(json.load(f))
    grapher = Grapher(data)
    grapher.graph_states()

if __name__ == "__main__":
    main()
