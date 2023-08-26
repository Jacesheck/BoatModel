from KalmanFilter import KalmanFilter
from DataObject import DataObject

import numpy as np
import time
import pygame

class Simulation:
    def __init__(self, data: DataObject):
        self.filter = KalmanFilter()
        self.lastPos = [0., 0.]
        self.data: DataObject = data
        self.reset()

        pygame.init()
        self.screen = pygame.display.set_mode((1280, 720))

    def reset(self):
        self.lastTime = -1
        self.tempStateHistory = [] # Eventually becomes stateHistory np.array
        self.tempMotorHistory = [] # Eventually becomes motorHistory np.array
        self.dtHistory = []

    def tuneParam(self, param: str):
        "Returns after exit"
        while True:
            value: float;
            str_value = input("Value: ")
            try:
                if 'exit' in str_value:
                    return
                else:
                    value = float(str_value)
            except Exception:
                print("Not a string value")
                return

            self.filter.reset()
            setattr(self.filter, param, value)
            self.run()
            self.showStatic()

    def tune(self):
        "Returns after exit"
        while True:
            self.filter.showVars()
            param = input("Param: ")
            if 'exit' in param:
                return
            elif hasattr(self.filter, param):
                self.tuneParam(param)
            else:
                print(f"Invalid param: {param}")

    
    def run(self):
        self.reset()
        for i in range(len(self.data)):
            self.step(self.data.at(i))
        self.clipAndScale(10, 400)


    def step(self, data: dict[str, float | int]):
        dt = 0
        if self.lastTime > 0: # Check initialised
            dt = data['timestamp'] - self.lastTime
        self.lastTime = data['timestamp']

        u = np.array([[data['power1'], data['power2']]]).T
        self.filter.predict(u, dt)

        assert(len(self.dtHistory) == len(self.tempStateHistory))
        assert(len(self.tempMotorHistory) == len(self.tempStateHistory))
        self.filter.update(data)
        self.tempStateHistory.append(self.filter.x)
        self.tempMotorHistory.append(u)
        self.dtHistory.append(dt)


    def scaleArrayToMinMax(self, history: np.ndarray, width: int, height: int) -> None:
        x_min = np.min(history[:, 0])
        x_max = np.max(history[:, 0])
        y_min = np.min(history[:, 1])
        y_max = np.max(history[:, 1])

        x_old_range = x_max - x_min
        y_old_range = y_max - y_min

        x_scale_factor = width / x_old_range
        y_scale_factor = height / y_old_range

        scale_factor = np.min([x_scale_factor, y_scale_factor])

        history[:, 0] = (history[:, 0] - x_min)*scale_factor
        history[:, 1] = (history[:, 1] - y_min)*scale_factor

    def drawBoat(self, screen):
        length = 10
        pos = (self.stateHistory[self.i, 0, 0], self.stateHistory[self.i, 1, 0])
        theta = np.deg2rad(self.stateHistory[self.i, 4, 0])
        endpos = (pos[0] + length*np.sin(theta),
                  pos[1] + length*np.cos(theta))

        pygame.draw.circle(screen, "black", pos, 3)
        pygame.draw.line(screen, "black", pos, endpos)

    def drawMotors(self, screen):
        motor1 = self.motorHistory[self.i, 0, 0]/2
        motor2 = self.motorHistory[self.i, 1, 0]/2
        left_motor   = pygame.Rect(1000, min(300-motor1, 300), 20, abs(motor1))
        right_motor  = pygame.Rect(1080, min(300-motor2, 300), 20, abs(motor2))
        pygame.draw.rect(screen, "red", left_motor)
        pygame.draw.rect(screen, "green", right_motor)

    def drawVel(self, screen):
        centre = (1160, 300)
        vel_x = self.stateHistory[self.i, 2, 0] * 100
        vel_y = self.stateHistory[self.i, 3, 0] * 100
        endpoint = (centre[0] + vel_x, centre[1] + vel_y)
        pygame.draw.circle(screen, 'black', centre, 2)
        pygame.draw.line(screen, 'black', centre, endpoint)

    def drawPath(self, screen):
        for i in range(len(self.stateHistory)-1):
            thisPoint = (self.stateHistory[i][0, 0], self.stateHistory[i][1, 0])
            nextPoint = (self.stateHistory[i+1][0, 0], self.stateHistory[i+1][1, 0])
            pygame.draw.line(screen, "blue", thisPoint, nextPoint)

    def drawGPSPoints(self, screen):
        for i in range(len(self.data)):
            centre = (self.data.at(i)["gpsX"], self.data.at(i)["gpsY"])
            #centre[0] = self.scaleArrayToMinMax(0, 1000)
            pygame.draw.circle(screen, 'red', centre, 1)

    def drawGPS(self, screen):
        centre = (1100, 500)
        if self.filter.gpsUpdated[self.i]:
            pygame.draw.circle(screen, 'red', centre, 50)

    def clipAndScale(self, start: int, end:int):
        # Clip
        self.stateHistory = np.array(self.tempStateHistory[start:-end])
        self.motorHistory = np.array(self.tempMotorHistory[start:-end])
        self.dtHistory = self.dtHistory[start:-end]

        # Scale
        self.scaleArrayToMinMax(self.stateHistory[:, :, 0], 1000, 600)

    def showStatic(self):
        "Only shows static image"

        self.screen.fill("lightblue")
        self.drawGPSPoints(self.screen)
        self.drawPath(self.screen)

        pygame.display.flip()

    def show(self):
        "Returns after animation"

        running = True
        self.i = 0
        while running and self.i < (len(self.stateHistory) - 1):
            for event in pygame.event.get():
                # Check quit
                if event.type == pygame.QUIT:
                    running = False
                    break
            self.i += 1

            self.screen.fill("lightblue")

            # Start drawing
            self.drawGPSPoints(self.screen)
            self.drawPath(self.screen)
            self.drawBoat(self.screen)
            self.drawMotors(self.screen)
            self.drawVel(self.screen)
            self.drawGPS(self.screen)
            time.sleep(self.dtHistory[self.i])

            pygame.display.flip()
