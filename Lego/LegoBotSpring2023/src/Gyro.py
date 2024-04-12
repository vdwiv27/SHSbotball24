import kipr
from constants import *

class Gyro:
    def __init__(self):
        kipr.gyro_calibrate()
        self.calibrate_gyro()

    def calibrate_gyro(self):
        num_readings = 50
        avg = 0
        kipr.mav(0, 0)
        kipr.mav(1, 0)
        for _ in range(num_readings):
            avg += kipr.gyro_z()
            kipr.mav(0, 0)
            kipr.mav(1, 0)
            kipr.msleep(5)
        self.bias = avg / num_readings
        print("Bias: {}".format(self.bias))
