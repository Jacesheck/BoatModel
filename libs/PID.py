import time

def wrap180(angle: float) -> float:
    if angle < -180:
        return angle + 360
    elif angle > 180:
        return angle - 360
    else:
        return angle

def wrap360(angle: float) -> float:
    if angle < 0:
        return angle + 360
    elif angle > 360:
        return angle - 360
    else:
        return angle

class PID:
    def __init__(self, p, i, d):
        self.p = p
        self.i = i
        self.d = d
        self.time = time.time()
        self.accum = 0
        self.e_prev = 0
        self.goal = 0
    def run(self, goal, val, new=False, dt: None | float = None):
        if dt == None:
            print("dt is none")
            newTime = time.time()
            dt = newTime - self.time
            self.time = newTime
            if dt == 0:
                time.sleep(0.001)
                return self.run(val, goal)

        e = wrap180(goal - val)
        if new:
            self.goal = goal
            self.accum = 0.
            de = 0.
        else:
            de: float = wrap180(e - self.e_prev)
        #sum = self.accum
        self.accum += e * dt
        self.e_prev = e

        return self.p * e + self.d * de/dt# + self.i * sum * dt
