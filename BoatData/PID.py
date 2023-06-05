import time

class PID:
    def __init__(self, p, i, d):
        self.p = p
        self.i = i
        self.d = d
        self.time = time.time()
        self.accum = 0
        self.e_prev = 0
        self.goal = 0
    def run(self, val, goal, new=False):
        newTime = time.time()
        dt = newTime - self.time
        self.time = newTime
        if dt == 0:
            time.sleep(0.001)
            return self.run(val, goal)

        e = goal - val
        if new:
            self.goal = goal
            self.accum = 0
            de = 0.
        else:
            de: float = e - self.e_prev
        sum = self.accum
        self.accum += e * dt
        self.e_prev = e

        return self.p * e + self.d * de/dt + self.i * sum * dt
