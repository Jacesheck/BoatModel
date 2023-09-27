from libs.KalmanFilter import KalmanFilter
from libs.DataObject import DataObject

import numpy as np
import time
import pygame
import matplotlib.pyplot as plt

# Spacing for window
DISPLAY_SPACING: int = 20

class Simulation:
    def __init__(self, data: DataObject):
        """Simulation window object
        -------
        Parameters
        data : DataObject
            The deserialised telemetry of the boat journey
        """
        self.filter = KalmanFilter()
        self.lastPos = [0., 0.]
        self.data: DataObject = data
        self.reset()

        pygame.init()
        self.screen = pygame.display.set_mode((1280, 720))

    def reset(self):
        "Reset history arrays"
        self.lastTime: float = -1
        self.tempStateHistory = [] # Eventually becomes stateHistory np.array
        self.tempMotorHistory = [] # Eventually becomes motorHistory np.array
        self.dtHistory = []

    def tuneParam(self, param: str):
        """ Start the tuning process with a parameter eg(b1)

        ----------
        Parameters
        param : str
            The parameter to start tuning

        NOTE: Returns after "exit" command
        """
        while True:
            plt.ion()
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
            plt.figure(0)
            plt.clf();
            plt.subplot(3, 1, 1)
            plt.title("Angle")
            plt.plot(self.stateHistory[:, 4, 0], label="before")
            plt.subplot(3, 1, 2)
            plt.title("Displacement")
            plt.plot(self.stateHistory[:, 0, 0], label="x before")
            plt.plot(self.stateHistory[:, 1, 0], label="y before")
            plt.subplot(3, 1, 3)
            plt.title("Velocity")
            plt.plot(self.stateHistory[:, 2, 0], label="dx before")
            plt.plot(self.stateHistory[:, 3, 0], label="dy before")
            self.run()
            self.showStatic()
            plt.subplot(3, 1, 1)
            plt.plot(self.stateHistory[:, 4, 0], label="after")
            plt.legend()
            plt.subplot(3, 1, 2)
            plt.plot(self.stateHistory[:, 0, 0], label="x after")
            plt.plot(self.stateHistory[:, 1, 0], label="y after")
            plt.legend()
            plt.subplot(3, 1, 3)
            plt.plot(self.stateHistory[:, 2, 0], label="dx before")
            plt.plot(self.stateHistory[:, 3, 0], label="dy before")
            plt.legend()
            plt.pause(0.1)

    def tune(self):
        """Enter tuning mode
        Allows selection of parameter to tune
        -----------
        NOTE: Returns on "exit" command
        """
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
        """Run the simulation with the passed data object
        """
        self.reset()
        for i in range(len(self.data)):
            self.step(self.data.at(i))
        self.stateHistory = np.array(self.tempStateHistory)
        self.motorHistory = np.array(self.tempMotorHistory)
        self.scaleStateHistory()

    def step(self, data: dict[str, float | int]):
        """Run the kalman filter predict and update cycles"""
        dt = 0
        if self.lastTime > 0: # Check initialised
            dt = data['timestamp'] - self.lastTime
        self.lastTime = data['timestamp']
        dt = 0.1

        # Make sure arrays are same sizes
        assert(len(self.dtHistory) == len(self.tempStateHistory))
        assert(len(self.tempMotorHistory) == len(self.tempStateHistory))

        u = np.array([[data['powerLeft'], data['powerRight']]]).T
        self.filter.predict(u, dt)
        self.filter.update(data)
        self.tempStateHistory.append(self.filter.x)
        self.tempMotorHistory.append(u)
        self.dtHistory.append(dt)


    def createScaleFromGPS(self, width: int, height: int):
        """Create scale data from raw gps points
        ---------
        Parameters
        states : DataObject
            Deserialised raw telemetry
        width : int
            Expected showing width of path display
        height : int
            Expected showing height of path display
        """
        x_min = min(self.data["gpsX"])
        x_max = max(self.data["gpsX"])
        y_min = min(self.data["gpsY"])
        y_max = max(self.data["gpsY"])

        x_old_range = x_max - x_min
        y_old_range = y_max - y_min

        print(f"gps X range : {x_old_range}")
        print(f"gps Y range : {y_old_range}")

        x_scale_factor = width / x_old_range
        y_scale_factor = height / y_old_range

        scale_factor = np.min([x_scale_factor, y_scale_factor])
        self.scale_m = np.array([[scale_factor, 0],
                                 [0, -scale_factor]])
        self.scale_c = np.array([[DISPLAY_SPACING],
                                 [height + DISPLAY_SPACING]])
        self.gps_min = np.array([[x_min],
                                 [y_min]])

    def scaleStateHistory(self):
        """Apply scaling to states
        """
        for i in range(len(self.stateHistory)):
            self.stateHistory[i, :2] = self.scale_m@(self.stateHistory[i, :2] - self.gps_min) + self.scale_c

    def scalePoint(self, point: np.ndarray):
        point = self.scale_m@(point - self.gps_min) + self.scale_c

    def drawBoat(self, screen):
        "Draw boat"
        length = 10
        pos = (self.stateHistory[self.i, 0, 0], self.stateHistory[self.i, 1, 0])
        theta = np.deg2rad(self.stateHistory[self.i, 4, 0])
        endpos = (pos[0] + length*np.sin(theta),
                  pos[1] - length*np.cos(theta)) # negative because origin TL

        pygame.draw.circle(screen, "black", pos, 3)
        pygame.draw.line(screen, "black", pos, endpos)

    def drawMotors(self, screen):
        "Draw Motor display bars"
        motor1 = self.motorHistory[self.i, 0, 0]*200
        motor2 = self.motorHistory[self.i, 1, 0]*200
        left_motor  = pygame.Rect(1000, min(300-motor1, 300), 20, abs(motor1))
        right_motor = pygame.Rect(1080, min(300-motor2, 300), 20, abs(motor2))
        pygame.draw.rect(screen, "red", left_motor)
        pygame.draw.rect(screen, "green", right_motor)

    def drawVel(self, screen):
        "Draw velocity vector bars"
        centre = (1160, 300)
        vel_x = self.stateHistory[self.i, 2, 0] * 100
        vel_y = -self.stateHistory[self.i, 3, 0] * 100 # Negative because origin at TL
        endpoint = (centre[0] + vel_x, centre[1] + vel_y)
        pygame.draw.circle(screen, 'black', centre, 2)
        pygame.draw.line(screen, 'black', centre, endpoint)

    def drawPath(self, screen):
        "Draw a line path"
        for i in range(len(self.stateHistory)-1):
            thisPoint = (self.stateHistory[i][0, 0], self.stateHistory[i][1, 0])
            nextPoint = (self.stateHistory[i+1][0, 0], self.stateHistory[i+1][1, 0])
            pygame.draw.line(screen, "blue", thisPoint, nextPoint)

    def drawGPSPoints(self, screen):
        "Draw GPS points"
        for i in range(len(self.data)):
            centre = np.array([[self.data.at(i)["gpsX"]], [self.data.at(i)["gpsY"]]])
            centre = self.scale_m @ (centre - self.gps_min) + self.scale_c
            pygame.draw.circle(screen, 'red', (centre[0,0], centre[1,0]), 3)

    def drawGPS(self, screen):
        "Draw GPS dot, shows on GPS update"
        centre = (1100, 500)
        if self.filter.gpsUpdated[self.i]:
            pygame.draw.circle(screen, 'red', centre, 50)

    def clip(self, start: int, end:int):
        """Clip state history
        --------
        Paramters
        start : int
            Start index to clip x state history, allows for ignoring x = 0, y = 0
        end : int
            End index to clip x state history
            Length from end of array
        """
        # Clip
        self.stateHistory = self.stateHistory[start:-end]
        self.motorHistory = self.motorHistory[start:-end]
        self.dtHistory = self.dtHistory[start:-end]

    def showStatic(self):
        "Only shows static path"

        self.screen.fill("lightblue")
        self.drawGPSPoints(self.screen)
        self.drawPath(self.screen)

        pygame.display.flip()

    def show(self):
        """"Shows the animation and returns when finished
        ----------
        Can be interrupted/paused by quitting window
        Will return to menu when interrupted/paused
        """

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
            self.drawPath(self.screen)
            self.drawBoat(self.screen)
            self.drawMotors(self.screen)
            self.drawVel(self.screen)
            self.drawGPS(self.screen)
            self.drawGPSPoints(self.screen)
            time.sleep(self.dtHistory[self.i])

            pygame.display.flip()
