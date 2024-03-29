import pygame
import numpy as np
import time

from libs.KalmanFilter import KalmanFilter
from libs.PID import PID

SCALE_FACTOR: float = 21.

def normalize(input1: float, input2: float, max: float) -> tuple[float, float]:
    """Sandbox normlise function
    --------
    Parameters
    input1 : float
        Motor 1 input
    input2 : float
        Motor 2 input
    max : float
        Max output power
    -------
    Return normalised input1 and inpu2 : tuple[float, float]
    """
    total = abs(input1) + abs(input2)
    if total > max:
        input1 /= total
        input2 /= total
    return input1, input2

class Waypoint:
    def __init__(self, pos : tuple[float, float]):
        """Waypoint created with mouse press
        -------
        Parameters
        pos : tupe[int, int]
            XY values of mouse position
        """
        self.pos = pos

    def draw(self, screen):
        """Draw waypoint on screen"""
        pygame.draw.circle(screen, "red", self.pos, 3)

class Boat(KalmanFilter) :
    def __init__(self, x: float, y: float):
        """Boat object with integrated kalman filter
        -------
        Parameters
        x : float
            Start x position
        y : float
            Start y position
        w : float
            Width of boat (for kalman filter)
        """
        KalmanFilter.__init__(self)
        self.boatImg = pygame.image.load('boat.png')
        self.time = time.time()
        self.u = np.array([[0., 0.]]).transpose()
        self.x[0, 0] = x
        self.x[1, 0] = y

        self.turnPid = PID(1, 0, 0.5)

    def pos(self) -> list[float]:
        """Return position of boat as tuple
        -------
        Return position of boat : tuple[float, float]"""

        return [self.x[0,0], self.x[1,0]]

    def auto(self, waypoint: Waypoint) -> bool:
        """ Runs the auto waypoint mode
        --------
        Parameters
        waypoint : Waypoint
            Current waypoing goal
        ---------
        Returns true if auto and false if manual : bool
        """
        new = True
        if hasattr(self, "waypoint_prev"):
            if waypoint == self.waypoint_prev:
                new = False
        self.waypoint_prev = waypoint

        # Press "a" key
        if pygame.key.get_pressed()[pygame.K_a]:
            # Control rotation pid
            dx = waypoint.pos[0] - self.x[0,0]
            dy = waypoint.pos[1] - self.x[1,0]
            theta_goal = np.arctan2(dy, dx)
            theta_cur = self.x[4,0]
            e_theta = theta_goal - theta_cur
            if e_theta > np.pi:
                theta_cur += 2*np.pi
            elif e_theta < -np.pi:
                theta_cur -= 2*np.pi
            turnPower = self.turnPid.run(theta_cur, theta_goal, new)
            print(f'{new = }')

            distPower = np.cos(e_theta)**21
            print(f'{distPower = }')
            print(f'{turnPower = }')
            print('')
            distPower, turnPower = normalize(distPower, turnPower, 1)
            self.u[0,0] = distPower + turnPower
            self.u[1,0] = distPower - turnPower

            return True
        return False

    def getInput(self):
        """Update self.u input matrix with user input"""
        if pygame.key.get_pressed()[pygame.K_u]:
            self.u[0, 0] = 1
        elif pygame.key.get_pressed()[pygame.K_j]:
            self.u[0, 0] = -1
        else:
            self.u[0, 0] = 0

        if pygame.key.get_pressed()[pygame.K_i]:
            self.u[1, 0] = 1
        elif pygame.key.get_pressed()[pygame.K_k]:
            self.u[1, 0] = -1
        else:
            self.u[1, 0] = 0

    def predict(self):
        "Predict cycle of kalman filter"
        newTime = time.time()
        dt = newTime - self.time
        self.time = newTime

        super().predict(self.u, dt)
        print(self.x)

    def draw(self, screen, waypoint: Waypoint | None = None):
        """Draw boat
        --------
        Parameters
        waypoint : Waypoint | None
            Next active waypoint (default None)
        """
        scaledBoat = pygame.transform.scale_by(self.boatImg, 1)
        scaledBoat = pygame.transform.rotate(scaledBoat, -90-self.x[4,0]*180/np.pi)
        width, height = scaledBoat.get_width(), scaledBoat.get_height()
        screen.blit(scaledBoat, (self.x[0,0] - width/2, self.x[1,0] - height/2))

        self.drawGoal(screen, waypoint)

    def drawGoal(self, screen, waypoint: Waypoint | None):
        """Draw goal with line to waypoint
        ---------
        Parameters
        waypoint : Waypoint | None
            Active waypoint (default None)
        """
        if waypoint != None:
            pygame.draw.circle(screen, "red", waypoint.pos, 22)
            pygame.draw.circle(screen, "white", waypoint.pos, 20)
            pygame.draw.line(screen, "red", waypoint.pos, self.pos())

    def checkAtWaypoint(self, w: Waypoint, thresh=20) -> bool:
        """Check if close enough to waypoint
        --------
        Parameters
        w : Waypoint
            Active waypoint
        thresh : float
            Threshold of waypoint
        -------
        Return reached waypoint : bool
        """
        dx = w.pos[0] - self.x[0,0]
        dy = w.pos[1] - self.x[1,0]
        dist = np.sqrt(dx**2 + dy**2)
        return dist < thresh 


if __name__ == "__main__":
    pygame.init()
    screen = pygame.display.set_mode((1280, 720))
    clock = pygame.time.Clock()
    running = True

    boat = Boat(300, 300)
    waypoints = []
    while running:

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
                break
            if event.type == pygame.MOUSEBUTTONUP:
                newWaypoint = Waypoint(pygame.mouse.get_pos())
                waypoints.append(newWaypoint)

        screen.fill("lightblue")
        # Render everything

        # Auto if auto else manual
        if len(waypoints) == 0 or not boat.auto(waypoints[0]):
            boat.getInput()

        boat.predict()
        # Draw waypoints
        if len(waypoints):
            boat.draw(screen, waypoints[0])
        else:
            boat.draw(screen)

        for i, w in enumerate(waypoints):
            if i == 0 and boat.checkAtWaypoint(w):
                del waypoints[i] 
            else:
                w.draw(screen)

        # Code done

        pygame.display.flip()
    pygame.quit()
