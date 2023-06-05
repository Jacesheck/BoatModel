import unittest
import numpy as np
from DataAnalayser import Point, KalmanFilter, Simulation

class TestPoint(unittest.TestCase):
    def test_distanceTo(self):
        p1 = Point(0, 0)
        p2 = Point(1, 1)
        self.assertEqual(p1.distanceTo(p2), np.sqrt(2))
        p1 = Point(0, 0)
        p2 = Point(1, -1)
        self.assertEqual(p1.distanceTo(p2), np.sqrt(2))

    def test_courseTo(self):
        p1 = Point(0,0)
        p2 = Point(2,-2)
        self.assertEqual(p1.courseTo(p2), 90+45)

        p1 = Point(0,0) # ^
        p2 = Point(0,2)
        self.assertEqual(p1.courseTo(p2), 0)

        p1 = Point(0,0) # >
        p2 = Point(2,0)
        self.assertEqual(p1.courseTo(p2), 90)

        p1 = Point(0,0) # v
        p2 = Point(0,-2)
        self.assertEqual(p1.courseTo(p2), 180)

        p1 = Point(0,0) # <
        p2 = Point(-2,0)
        self.assertEqual(p1.courseTo(p2), 270)

class TestKalmanFilter(unittest.TestCase):
    def test_wrap180(self):
        kf = KalmanFilter()
        self.assertEqual(kf.wrap180(0), 0)
        self.assertEqual(kf.wrap180(90), 90)
        self.assertEqual(kf.wrap180(180), 180)
        self.assertEqual(kf.wrap180(270), -90)
        self.assertEqual(kf.wrap180(355), -5)
    
    def test_wrap360(self):
        kf = KalmanFilter()
        self.assertEqual(kf.wrap360(0), 0)
        self.assertEqual(kf.wrap360(90), 90)
        self.assertEqual(kf.wrap360(180), 180)
        self.assertEqual(kf.wrap360(-90), 270)
        self.assertEqual(kf.wrap360(-5), 355)

    def test_kf(self):
        kf = KalmanFilter()
        u = np.array([[2., 2.,]]).T
        kf.predict(u, 1.)
        self.assertIsNot(kf.x[3, 0],  0)
        self.assertEqual(kf.x[4, 0],  0)
        u = np.array([[2, -2]]).T
        self.assertIsNot(kf.x[4, 0],  0)
        self.assertIsNot(kf.x[5, 0],  0)

class TestSimulation(unittest.TestCase):
    def test_scaleToMinMax(self):
        sim = Simulation()
        history = np.array([1, 2, 3, 4, 5])
        sim.scaleToMinMax(history, (0, 100, 0, 100))
        self.assertEqual(history[0], 0)
        self.assertEqual(history[1], 25)
        self.assertEqual(history[4], 100)

        sim = Simulation()
        history = np.array([-1, -2, -3, -4, -5])
        sim.scaleToMinMax(history, (0, 100, 0, 100))
        self.assertEqual(history[0], 100)
        self.assertEqual(history[1], 75)
        self.assertEqual(history[4], 0)
