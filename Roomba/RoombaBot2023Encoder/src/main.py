#!/usr/bin/python
import os, sys
sys.path.append("/usr/lib")
import kipr
import time
import random

from Servo import *
from drive2 import *
from Sensors import *
from Create_Utilities import *
from Create_constants import *
from Controller_Create_Alt_Refactor import *
from Camera import *
from Motor import *

def initialize_objects():
    Servo1 = Servo(ARM_PORT)
    Servo2 = Servo(WRIST_PORT)
    Servo3 = Servo(CLAW_PORT)
        
    MotorServo1 = Motor(DROPPER_PORT)
    
    sensors = {}

    #driver = Drive()
    driver = Drive2()
    
    camera = None #CameraHandler(driver)

    servos = ServoHandler(dict(map(lambda x: (x.port, x), [Servo1, Servo2, Servo3])))

    motorServos = MotorServoHandler(dict(map(lambda x: (x.port, x), [MotorServo1])))

    utilities = Utilities(driver, camera, sensors)

    controller = Controller(driver, servos, motorServos, camera, utilities, sensors)

    return driver, camera, servos, motorServos, utilities, controller, sensors


def main2():
    driver, camera, servos, motorServos, utilities, controller, sensors = initialize_objects()
    print("warned")
    controller.execute_game()
        
def main3():
    driver = Drive2()
    for _ in range(4):
        driver.turn2(-90, 250, debug=True)
    return
    driver.drive_distance(50, -250)
    driver.drive_distance(50, 250)
    #driver.drive_distance2(100, 250)
    return
    time.sleep(1)
    print(kp_vals, ki_vals, kd_vals)
    write_values("kp_vals", kp_vals)
    write_values("ki_vals", ki_vals)
    write_values("kd_vals", kd_vals)
    return
    pid_errors = []
    pid_raw_enc = []
    pid_initial = []
    direction = 1
    for _ in range(10):
        driver = Drive2()
        pid_initial.append((driver.left_enc_initial, driver.right_enc_initial))
        l, r = driver.drive_straight(100, 450 * direction)
        pid_errors.append(r - l)
        pid_raw_enc.append((l, r))
        print l, r
        direction *= -1
    print()      
    no_pid_initial = []
    no_pid_errors = []
    no_pid_raw_enc = []
    for _ in range(10):
        driver = Drive2()
        no_pid_initial.append((driver.left_enc_initial, driver.right_enc_initial))
        l, r = driver.drive_distance(100, 450 * direction)
        no_pid_errors.append(r - l)
        no_pid_raw_enc.append((l, r))
        print l, r
        direction *= -1
    print("Pid", pid_initial)
    print("No Pid", no_pid_initial)
    '''
    print("Pid:", pid_errors)
    print("No Pid:", no_pid_errors)
    print()
    print("Pid:", pid_raw_enc)
    print("No Pid:", no_pid_raw_enc)
    '''
    
def write_values(filename, data):
    with open("{}.txt".format(filename), "w") as txt_file:
        for line in data:
            for val in line:
                txt_file.write(str(val) + " ")
            txt_file.write("\n")
        
if __name__ == "__main__":
    sys.stdout = os.fdopen(sys.stdout.fileno(), "w", 0)
    kipr.create_connect()
    kipr.create_full()
    #print(kipr.get_create_rcliff_amt(), kipr.get_create_lcliff_amt())
    #time.sleep(10)
    try:
        main2()
    except Exception as e:
        print(e)
    kipr.create_stop()
    kipr.create_disconnect()
            