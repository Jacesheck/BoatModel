import numpy as np
import matplotlib.pyplot as plt

from libs.PID import PID
from libs.KalmanFilter import KalmanFilter

def wrap360(num: float) -> float:
    if num < 0:
        return num + 360
    elif num > 360:
        return num - 360
    else:
        return num

def getError(setpoint: float, observation: float) -> float:
    error = setpoint - observation
    if error < -180:
        return error + 360
    elif error > 180:
        return error - 360
    else:
        return error

def constrain(val: float, max: float) -> float:
    if val < -max:
        return -max
    elif val > max:
        return max
    else:
        return val

def tuneAngle():
    plt.ion()
    p, d = (float(num) for num in input("PID: ").split(" "))
    boat = KalmanFilter()
    dt = 0.1
    time = np.arange(0, 10, dt)
    pid  = PID(p, 0, d)
    angles = []
    outputs = []
    for _ in time:
        theta = boat.x[4, 0]
        val = pid.run(90, theta, dt=dt)
        val = constrain(val, 1)
        boat.predict(np.array([[val], [-val]]), dt)
        outputs.append(val)
        angles.append(theta)

    plt.figure(0)
    plt.title("Angle")
    plt.plot(time, angles)

    plt.figure(1)
    plt.title('Output')
    plt.plot(time, outputs)
    plt.pause(0.1)

def tuneDist():
    plt.ion()
    p, d = (float(num) for num in input("PID: ").split(" "))
    boat = KalmanFilter()
    dt = 0.1
    time = np.arange(0, 10, dt)
    pid  = PID(p, 0, d)
    distances = []
    outputs = []
    boat.x[4,0] = 0
    for _ in time:
        dist = boat.x[1, 0]
        val = pid.run(5, dist, dt=dt)
        val = constrain(val, 1)
        boat.predict(np.array([[val], [val]]), dt)
        outputs.append(val)
        distances.append(dist)

    plt.figure(0)
    plt.title("Distances")
    plt.plot(time, distances)

    plt.figure(1)
    plt.title('Output')
    plt.plot(time, outputs)
    plt.pause(0.1)


def tuneMovingDist():
    plt.ion()
    p, d = (float(num) for num in input("PID: ").split(" "))
    boat = KalmanFilter()
    dt = 0.1
    time = np.arange(0, 10, dt)
    pid  = PID(p, 0, d)
    speeds = []
    outputs = []
    boat.x[4,0] = 0
    goal = 0
    for _ in time:
        goal += 0.1 # 1 m/s
        dist = boat.x[1, 0]
        val = pid.run(goal, dist, dt=dt)
        val = constrain(val, 1)
        boat.predict(np.array([[val], [val]]), dt)
        outputs.append(val)
        dx = boat.x[2, 0]
        dy = boat.x[3, 0]
        speed = np.sqrt(np.power(dx, 2) + np.power(dy, 2))
        speeds.append(speed)

    plt.figure(0)
    plt.title("Speeds")
    plt.plot(time, speeds)

    plt.figure(1)
    plt.title('Output')
    plt.plot(time, outputs)
    plt.pause(0.1)

def main():
    pass
    tuneAngle()
    #tuneDist()
    #tuneMovingDist()
    pass


if __name__ == "__main__":
    while True:
        main()
