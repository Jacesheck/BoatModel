from __future__ import annotations
import numpy as np

class Point:
    def __init__(self, x, y):
        """Point object
        Parameters
        ---------
        x : int
            x coord
        y : int
            y coord
        """
        self.x = x
        self.y = y


    def __eq__(self, __value: Point) -> bool:
        return self.x == __value.x and self.y == __value.y


    def __str__(self):
        return f'Point({self.x}, {self.y})'


    def distanceTo(self, pt: Point) -> float:
        """Get absolute distance to point
        ---------
        Parameters
        pt : Point
            Point to find distance between
        -------
        Return distance (m) : float
        """
        dx = pt.x - self.x
        dy = pt.y - self.y
        return np.sqrt(dx**2 + dy**2)
    

    def courseTo(self, pt: Point) -> float:
        """Return distance in degrees to point
        -------
        Parameters
        pt : Point
            Point to find heading to
        --------
        Return heading to point (deg [0, 360)): float
        """

        dx = pt.x - self.x
        dy = pt.y - self.y
        angle = np.angle(complex(dy, dx), deg=True)
        if angle < 0:
            angle += 360
        return float(angle)


