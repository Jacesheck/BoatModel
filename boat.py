import pygame
import numpy as np
import time

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

class PID:
    def __init__(self, p: float, i: float, d: float):
        """Pid controller
        -------
        Parameters
        p : float
            Proportional value
        i : float
            Integral value
        d : float
            Derivative value
        """
        self.p = p
        self.i = i
        self.d = d
        self.time = time.time()
        self.accum = 0
        self.e_prev = 0
        self.goal = 0

    def run(self, observation: float, setpoint: float, new: bool=False) -> float:
        """Run pid controller
        ---------
        Parameters
        observation : float
            Observation of value
        setpoint : float
            Setpoint goal
        new : bool
            Is new setpoint
        --------
        Return output value : float
        """
        newTime = time.time()
        dt = newTime - self.time
        self.time = newTime
        if dt == 0:
            time.sleep(0.001)
            return self.run(observation, setpoint)

        e = setpoint - observation
        if new:
            self.goal = setpoint
            self.accum = 0
            de = 0.
        else:
            de: float = e - self.e_prev
        sum = self.accum
        self.accum += e * dt
        self.e_prev = e

        return self.p * e + self.d * de/dt + self.i * sum * dt

class Waypoint:
    def __init__(self, pos : tuple[int, int]):
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

class Boat:
    def __init__(self, x: float, y: float, w: float):
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
        self.boatImg = pygame.image.load('boat.png')

        self.w = w
        self.x = np.array([[x], # x
                           [y], # y
                           [0.], # v_x
                           [0.], # v_y
                           [0.], # theta
                           [0.]])# theta_dot

        self.F = np.array([[1, 0, 0, 0, 0, 0],
                           [0, 1, 0, 0, 0, 0],
                           [0, 0, 1, 0, 0, 0],
                           [0, 0, 0, 1, 0, 0],
                           [0, 0, 0, 0, 1, 0],
                           [0, 0, 0, 0, 0, 1]])

        self.B = np.array([[0., 0.],
                           [0., 0.],
                           [1., 1.],
                           [1., 1.],
                           [0., 0.],
                           [1., -1.]])

        self.u = np.array([[0., 0.]]).transpose()
        self.b1 = 1
        self.b2 = 1
        self.time = time.time()

        self.turnPid = PID(1, 0, 0.5)

    def pos(self) -> tuple[float, float]:
        """Return position of boat as tuple
        -------
        Return position of boat : tuple[float, float]"""

        return (self.x[0,0], self.x[1,0])

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

        theta = self.x[4,0]
        delta_th = 1-dt*self.b2

        self.F = np.array([[1., 0., dt, 0., 0., 0.],
                           [0., 1., 0., dt, 0., 0.],
                           [0., 0., np.max([0, 1.-dt*self.b1]), 0., 0., 0.],
                           [0., 0., 0., np.max([0, 1.-dt*self.b1]), 0., 0.],
                           [0., 0., 0., 0., 1., dt],
                           [0., 0., 0., 0., 0., delta_th]])

        self.B = np.array([[0, 0],
                           [0, 0],
                           [300*dt*np.cos(theta), 300*dt*np.cos(theta)],
                           [300*dt*np.sin(theta), 300*dt*np.sin(theta)],
                           [0, 0],
                           [0.5*dt*self.w/2, -0.5*dt*self.w/2]])

        self.x = self.F@self.x + self.B@self.u
        # Stops boat angle getting above 360 degrees
        self.x[4,0] = self.x[4,0] % (2*np.pi)
    

    def draw(self, screen, waypoint: Waypoint | None =None):
        """Draw entire game
        --------
        Parameters
        waypoint : Waypoint | None
            Next active waypoint (default None)
        """
        scaledBoat = pygame.transform.scale_by(self.boatImg, 10)
        scaledBoat = pygame.transform.rotate(scaledBoat, -90-self.x[4,0]*180/np.pi)
        width, height = scaledBoat.get_width(), scaledBoat.get_height()
        fullBoat = self.boatImg.copy()
        fullBoat = pygame.transform.scale_by(self.boatImg, 10)
        fullBoat = pygame.transform.rotate(fullBoat, -90-self.x[4,0]*180/np.pi)
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

    boat = Boat(300, 300, 20)
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
