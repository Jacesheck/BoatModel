import matplotlib.pyplot as plt
import json
import numpy as np

from libs.DataObject import DataObject

def main():
    with open("telemetry/telemTue_Sep_26_15.40.53_2023.json") as f:
        telem = DataObject(json.load(f))
    with open("telemetry/kalmanTue_Sep_26_15.40.53_2023.json") as f:
        kalman = DataObject(json.load(f))

    plt.subplot(2, 1, 1)
    plt.title("Heading")
    plt.plot(kalman["heading"])
    plt.subplot(2, 1, 2)
    plt.title("x y")
    plt.plot(telem["gpsX"], telem["gpsY"])
    plt.show()

if __name__ == "__main__":
    main()
