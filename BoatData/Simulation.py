from KalmanFilter import KalmanFilter
import numpy as np
import time
import pygame

class Simulation:
    def __init__(self):
        self.filter = KalmanFilter()
        self.lastPos = [0., 0.]
        self.lastTime = -1
        self.tempStateHistory = [] # Eventually becomes stateHistory np.array
        self.tempMotorHistory = [] # Eventually becomes motorHistory np.array


    def step(self, data: dict):
        dt = 0
        if self.lastTime > 0: # Check initialised
            dt = data['timestamp'] - self.lastTime
        self.lastTime = data['timestamp']

        u = np.array([[data['power1'], data['power2']]]).T
        self.filter.predict(u, dt)
        self.filter.update(data)
        self.tempStateHistory.append(self.filter.x)
        self.tempMotorHistory.append(u)


    def scaleToMinMax(self, history: np.ndarray, limits) -> None:
        lower = limits[0]
        upper = limits[1]
        limitRange = upper - lower

        mini = np.amin(history)
        maxi = np.amax(history)
        history_range = maxi - mini

        history[:] = (history[:] - mini)*limitRange/history_range + lower

    def drawBoat(self, screen):
        length = 10
        pos = (self.stateHistory[self.i, 0, 0], self.stateHistory[self.i, 1, 0])
        theta = np.deg2rad(self.stateHistory[self.i, 4, 0])
        endpos = (pos[0] + length*np.sin(theta),
                  pos[1] + length*np.cos(theta))

        pygame.draw.circle(screen, "black", pos, 3)
        pygame.draw.line(screen, "black", pos, endpos)

        time.sleep(0.1)

    def drawMotors(self, screen):
        motor1 = self.motorHistory[self.i, 0, 0]/2
        motor2 = self.motorHistory[self.i, 1, 0]/2
        leftMotor   = pygame.Rect(1000, min(300-motor1, 300), 20, abs(motor1))
        rightMotor  = pygame.Rect(1080, min(300-motor2, 300), 20, abs(motor2))
        pygame.draw.rect(screen, "red", leftMotor)
        pygame.draw.rect(screen, "green", rightMotor)

    def drawVel(self, screen):
        centre = (1160, 300)
        velx = self.stateHistory[self.i, 2, 0] * 100
        vely = self.stateHistory[self.i, 3, 0] * 100
        endpoint = (centre[0] + velx, centre[1] + vely)
        pygame.draw.circle(screen, 'black', centre, 2)
        pygame.draw.line(screen, 'black', centre, endpoint)

    def drawPath(self, screen):
        for i in range(10, len(self.stateHistory)-1):
            thisPoint = (self.stateHistory[i][0, 0], self.stateHistory[i][1, 0])
            nextPoint = (self.stateHistory[i+1][0, 0], self.stateHistory[i+1][1, 0])
            pygame.draw.line(screen, "blue", thisPoint, nextPoint)

    def drawGPS(self, screen):
        centre = (1100, 500)
        if self.filter.gpsUpdated[self.i]:
            print("drawing circular")
            pygame.draw.circle(screen, 'red', centre, 50)

    def show(self):
        pygame.init()
        self.stateHistory = np.array(self.tempStateHistory)
        self.motorHistory = np.array(self.tempMotorHistory)
        self.scaleToMinMax(self.stateHistory[10:, 0, 0], (0, 1000, 0, 1000))
        self.scaleToMinMax(self.stateHistory[10:, 1, 0], (0, 600, 0, 600))

        running = True
        screen = pygame.display.set_mode((1280, 720))
        while running:
            for event in pygame.event.get():
                # Check quit
                if event.type == pygame.QUIT:
                    running = False
                    break
            if not hasattr(self, 'i'):
                self.i = 0
            self.i += 1

            screen.fill("lightblue")

            # Start drawing
            self.drawPath(screen)
            self.drawBoat(screen)
            self.drawMotors(screen)
            self.drawVel(screen)
            self.drawGPS(screen)

            pygame.display.flip()
        pygame.quit()
