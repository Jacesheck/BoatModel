import pickle
import numpy as np
import struct
import matplotlib.pyplot as plt
import sys
import pygame

class KalmanFilter():
    def __init__(self):
        self.width = 0.6
        self.x = np.array([[0.], # theta
                           [0.]])# theta_dot

        self.F = np.array([[1, 0.1],
                           [0, 1]])
        self.P = np.array([[90., 0.],
                           [0., 0.]])
        self.Q = np.array([[20., 0.],
                           [0., 1.]])

        self.I = np.array([[1., 0.],
                           [0., 1.]])

    def predict(self):
        self.x = self.F@self.x
        self.P
    def update_gps(self, theta, dist, rz):
        H = np.array([[1., 0.],
                           [0., 1.]])
        R = np.array([[3./dist, 0.],
                      [0., 0.1]])

        z = np.array([[theta], [rz]])
        y = z - H@self.x
        S = H@self.P@H.T + R
        K = self.P@H.T@np.linalg.inv(S)
        self.x = self.x + K@y
        self.P = (np.eye(2) - K@H)@self.P

    def update(self, rz):
        H = np.array([[0., 1.]])
        R = np.array([[0.1]])

        z = np.array([[rz]])
        y = z - H@self.x
        S = H@self.P@H.T + R
        K = self.P@H.T@np.linalg.inv(S)
        self.x = self.x + K@y
        self.P = (np.eye(2) - K@H)@self.P

class Simulation:
    def __init__(self):
        self.filter = KalmanFilter()
        self.lastPos = [0., 0.]
    def step(self, x, y, rz):
        x, y = y, x # Swap because trig was wrong
        self.filter.predict()
        if [x, y] == self.lastPos:
            self.filter.update(rz)
        else:
            dx = x - self.lastPos[0]
            dy = y - self.lastPos[1]
            self.filter.update_gps(
                np.arctan2(dx, dy),
                np.sqrt(dx**2 + dy**2),
                rz
            )
            self.lastPos = [x, y]
    def show(self):
        pygame.init()
        screen = pygame.display.set_mode((1280, 720))
        clock = pygame.time.Clock()
        running = True

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
    gyro = []
    sim = Simulation()
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
        i += 4
        power2 = struct.unpack('<l', telemetry[i: i+4])
        i += 4
        rz = struct.unpack('<f', telemetry[i: i+4])
        i += 4
        X.append(-gpsX[0])
        Y.append(gpsY[0])
        lat.append(gpsLat[0])
        lng.append(gpsLng[0])
        motor1.append(power1[0])
        motor2.append(power2[0])
        gyro.append(rz[0])

        sim.step(gpsX, gpsY, rz)

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

    plt.figure(4)
    plt.title('Gyro')
    plt.plot(time, gyro)

    plt.show()
