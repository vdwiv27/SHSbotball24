#!/usr/bin/python
import os, sys
sys.path.append("/usr/lib")
import kipr
import threading
import multiprocessing

from Servo import *
from Motor import *
from Sensors import *
from Lego_Utilities import *
from constants import *
from Controller import *
from Camera import *

def initialize_objects():
    Servo0 = Servo(CLAW_PORT)
    Servo1 = Servo(FRONT_PORT)
    Servo2 = Servo(PING_PORT)

    Motor0 = Motor(RIGHT_MOTOR_PORT, float(1420) / 1500)
    Motor1 = Motor(LEFT_MOTOR_PORT, float(1500)/1500) 

    ReflectanceSensor = Analog(REFLECTANCE_PORT)
    SideReflectanceSensor = Analog(SIDE_REFLECTANCE_PORT)
    ReflectanceBackwardsSensor = Analog(REFLECTANCE_BACKWARDS_PORT)
    DistanceSensor = Analog(RANGEFINDER_PORT)  
    LeverSensor0 = Digital(LEVER_FRONT_PORT)
    
    sensors = {REFLECTANCE: ReflectanceSensor, SIDE_REFLECTANCE: SideReflectanceSensor, REFLECTANCE_BACKWARDS: ReflectanceBackwardsSensor, RANGEFINDER: DistanceSensor, LEVER_FRONT: LeverSensor0}

    servos = ServoHandler(dict(map(lambda x: (x.port, x), [Servo0, Servo1, Servo2])))
    motors = MotorHandler(dict(map(lambda x: (x.port, x), [Motor0, Motor1])))
    motorServos = None #MotorServoHandler(dict(map(lambda x: (x.port, x), [Motor2])))

    utilities = Utilities(motors, servos, sensors)
        
    camera = CameraHandler(motors)

    controller = Controller(motors, servos, motorServos, sensors, utilities, camera)

    return motors, servos, motorServos, sensors, utilities, controller, camera

def main():
    motors, servos, motorServos, sensors, utilities, controller, camera  = initialize_objects()
    print('hi')
    print("warned up")
    controller.practice()
            
if __name__ == "__main__":
    sys.stdout = os.fdopen(sys.stdout.fileno(), "w", 0)
    try:
        main()
    except Exception as e:
        print(e)