import numpy as np
from libs.PID import PID
from libs.Point import Point

NEW_BOAT: bool = True


# Notes
# m states
# n inputs
# y : n x 1
# B : m x n
# u : n x 1
# z : n x 1
# H : n x m
# F : m x m
# x : m x 1
# S : n x n
# P : m x m
# R : n x n
# K : m x n
# Q : m x m


class KalmanFilter():
    def __init__(self):
        """6 parameter kalman filter for boat with drift"""
        self.reset()

        if (NEW_BOAT):
            self.b1         = 0.8 # drag on water
            self.b2         = 3. # Rotational drag
            self.gpsNoise   = 4.
            self.gpsAngleNoise = 10.
            self.gyroNoise  = 0.1
            self.motorForce = 0.6
            self.motorTorque = 100.
        else:
            self.b1         = 2. # drag on water
            self.b2         = 3. # Rotational drag
            self.gpsNoise   = 4.
            self.gpsAngleNoise = 10.
            self.gyroNoise  = 0.1
            self.motorForce = 0.4
            self.motorTorque = 50.

    def reset(self):
        """Set all states to initial defaults"""

        self.w = 0.8 # Width of boat
        "State space"
        self.x = np.array([[0.], # x
                           [0.], # y
                           [0.], # v_x
                           [0.], # v_y
                           [0.], # theta (heading)
                           [0.]])# theta_dot

        "Initial uncertainty"
        self.P = np.array([[30., 0., 0., 0., 0., 0.],
                           [0., 30., 0., 0., 0., 0.],
                           [0., 0., 5., 0., 0., 0.],
                           [0., 0., 0., 5., 0., 0.],
                           [0., 0., 0., 0., 180., 0.],
                           [0., 0., 0., 0., 0., 0.]])

        "Process uncertainty"
        self.Q = np.array([[.05, 0., 0., 0., 0., 0.],
                           [0., .05, 0., 0., 0., 0.],
                           [0., 0., .5, 0., 0., 0.],
                           [0., 0., 0., .5, 0., 0.],
                           [0., 0., 0., 0., .01, 0.],
                           [0., 0., 0., 0., 0., .01]])

        "Input matrix"
        self.B = np.array([[0., 0.],
                           [0., 0.],
                           [1., 1.],
                           [1., 1.],
                           [0., 0.],
                           [1., -1.]])

        self.turnPid = PID(1, 0, 0.5)
        self.gpsUpdated = [] # for visual


    def showVars(self):
        """Print variables and current values to screen"""
        print(f"""
        b1 (water drag)     : {self.b1}
        b2 (rotational drag): {self.b2}
        gpsNoise            : {self.gpsNoise}
        gpsAngleNoise       : {self.gpsAngleNoise}
        gyroNoise           : {self.gyroNoise}
        motorForce          : {self.motorForce}
        motorTorque         : {self.motorTorque}
        """)


    def predict(self, u: np.ndarray, dt: float):
        """ Kalman filter predit cycle
        Parameters
        --------
        u : np.ndarry
            Input (motor) matrix
        dt : float
            Delta time
        """
        F = np.array([[1., 0., dt, 0., 0., 0.],
                      [0., 1., 0., dt, 0., 0.],
                      [0., 0., 1.-dt*self.b1, 0., 0., 0.],
                      [0., 0., 0., 1.-dt*self.b1, 0., 0.],
                      [0., 0., 0., 0., 1., dt],
                      [0., 0., 0., 0., 0., 1-dt*self.b2]], dtype=np.float64)
        #F = np.eye(6)

        theta_r  = np.deg2rad(self.x[4,0]) # Heading
        self.B = np.array([[0., 0.],
                           [0., 0.],
                           [self.motorForce*dt*np.sin(theta_r), self.motorForce*dt*np.sin(theta_r)],
                           [self.motorForce*dt*np.cos(theta_r), self.motorForce*dt*np.cos(theta_r)],
                           [0., 0.],
                           [self.motorTorque*dt, -self.motorTorque*dt]])

        self.x = F@self.x + self.B@u
        self.P = F@self.P@F.T + self.Q
        self.wrapTheta()

    def wrap180(self, angle: float) -> float:
        """Wrap angle to 180
        -----
        Parameters
        angle : float
            Input angle (deg [0, 360))
        ------
        Return Output angle (deg (-180, 180]) : float
        """
        if angle > 180:
            angle -= 360
        return angle


    def wrap360(self, angle: float) -> float:
        """Wrap angle to 360
        -------
        Parameters
        angle : float
            Input angle (deg)
        ------
        Return Output angle (deg [0, 360))
        """
        if angle < 0:
            angle += 360
        elif angle >= 360:
            angle -= 360
        return angle


    def wrapTheta(self):
        """Wrap heading angle"""
        self.x[4, 0] = self.wrap360(self.x[4, 0]) # Wrap theta


    def update(self, data: dict):
        """Kalman filter update cycle
        --------
        Parameters
        data : dict
            Dict containing gpsX, gpsY
        """
        gpsLoc = Point(data['gpsX'], data['gpsY'])
        if not hasattr(self, 'lastGPS') or self.lastGPS == gpsLoc:
            self.update_nogps(data)
        else:
            data['dist_gps']   = self.lastGPS.distanceTo(gpsLoc)
            data['course_gps'] = self.lastGPS.courseTo(gpsLoc)
            self.update_gps(data)
        self.wrapTheta()
        self.lastGPS = gpsLoc


    def update_gps(self, data: dict):
        """Kalman filter update with gps data (and gyro)
        --------
        Parameters
        data : dict
            Dict containing updated gps data
        """
        z = np.array([[data['gpsX']],
                      [data['gpsY']],
                      [data['course_gps']],
                      [data['rz']]])
        H = np.array([[1., 0., 0., 0., 0., 0.],
                      [0., 1., 0., 0., 0., 0.],
                      [0., 0., 0., 0., 1., 0.],
                      [0., 0., 0., 0., 0., 1.]])
        #H[2, 4] = 1 if 1 > self.gpsNoise else 0

        R = np.array([[self.gpsNoise, 0., 0., 0.], # Tune for gps
                      [0., self.gpsNoise, 0., 0.],
                      [0., 0., 1e-9, 0.],
                      [0., 0., 0., self.gyroNoise]])
        R[2, 2] = max([0., self.gpsAngleNoise*(0.5 - data['dist_gps'])]);

        y      = z - H@self.x
        y[2,0] = self.wrap180(y[2,0])
        S      = H@self.P@H.T + R
        K      = self.P@H.T@np.linalg.inv(S)
        self.x = self.x + K@y
        self.P = (np.eye(6) - K@H)@self.P
        self.gpsUpdated.append(True)


    def update_nogps(self, data: dict):
        """Kalman filter update without gps data (just gyro)
        --------
        Parameters
        data : dict
            Dict containing gyro data
        """
        rz = -data['rz']
        H  = np.array([[0., 0., 0., 0., 0., 1.]])
        R  = np.array([[self.gyroNoise]]) # Tune for no gps

        z  = np.array([[rz]])
        y  = z - H@self.x
        S  = H@self.P@H.T + R
        K  = self.P@H.T@np.linalg.inv(S)
        self.x = self.x + K@y
        self.P = (np.eye(6) - K@H)@self.P
        self.gpsUpdated.append(False)
