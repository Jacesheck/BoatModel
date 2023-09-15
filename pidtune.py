import numpy as np
import matplotlib.pyplot as plt

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
        self.w = w
        self.x = np.array([[x], # x
                           [y], # y
                           [0.], # v_x
                           [0.], # v_y
                           [315.], # theta
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
        self.b1 = 2
        self.b2 = 3
        self.motorForce = 0.001

    def predict(self, left: float, right: float, dt: float):
        "Predict cycle of kalman filter"
        theta_r = self.x[4,0] * np.pi / 180.
        delta_th = 1-dt*self.b2
        self.u = np.array([[left],
                           [right]])

        self.F = np.array([[1., 0., dt, 0., 0., 0.],
                           [0., 1., 0., dt, 0., 0.],
                           [0., 0., np.max([0, 1.-dt*self.b1]), 0., 0., 0.],
                           [0., 0., 0., np.max([0, 1.-dt*self.b1]), 0., 0.],
                           [0., 0., 0., 0., 1., dt],
                           [0., 0., 0., 0., 0., delta_th]])

        self.B = np.array([[0, 0],
                           [0, 0],
                           [self.motorForce*dt*np.cos(theta_r), self.motorForce*dt*np.cos(theta_r)],
                           [self.motorForce*dt*np.sin(theta_r), self.motorForce*dt*np.sin(theta_r)],
                           [0, 0],
                           [self.motorForce*dt*self.w/2, -self.motorForce*dt*self.w/2]])

        self.x = self.F@self.x + self.B@self.u
        self.x[4,0] = self.x[4,0]

        self.x[4,0] = wrap360(self.x[4,0])

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
        self.accum = 0
        self.e_prev = 0
        self.goal = 0

    def run(self, setpoint: float, observation: float, dt: float, new: bool=False) -> float:
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
        e = getError(setpoint, observation)
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

if __name__ == "__main__":
    boat = Boat(0, 0, 0.8)
    dt = 0.1
    time = np.arange(0, 50, 0.1)
    pid  = PID(1000., 0., 0.)
    angles = []
    outputs = []
    for t in time:
        theta = boat.x[4, 0]
        val = pid.run(90, theta, dt)
        val = constrain(val, 400)
        boat.predict(val, -val, dt)
        outputs.append(val)
        angles.append(theta)

    plt.figure(0)
    plt.title("Angle")
    plt.plot(time, angles)

    plt.figure(1)
    plt.title('Output')
    plt.plot(time, outputs)
    plt.show()
