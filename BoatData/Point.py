from __future__ import annotations
import numpy as np

class Point:
    def __init__(self, x, y):
        self.x = x
        self.y = y


    def __eq__(self, __value: Point) -> bool:
        return self.x == __value.x and self.y == __value.y


    def __str__(self):
        return f'Point({self.x}, {self.y})'


    def distanceTo(self, pt: Point):
        dx = pt.x - self.x
        dy = pt.y - self.y
        return np.sqrt(dx**2 + dy**2)
    

    def courseTo(self, pt: Point) -> float:
        'Returns angle in degrees [0, 360)'
        dx = pt.x - self.x
        dy = pt.y - self.y
        angle = np.angle(complex(dy, dx), deg=True)
        if angle < 0:
            angle += 360
        return float(angle)


