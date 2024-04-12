import kipr
import time
import random
import threading
import multiprocessing

from Motor import *
from Servo import *
from Sensors import *
from constants import *
from Lego_Utilities import *
from TimeTracker import *
from Camera import *

class Controller:
    def __init__(self, motors, servos, motorServos, sensors, utilities, camera):
        self.motors = motors
        self.servos = servos
        self.motorServos = motorServos
        self.sensors = sensors
        self.utilities = utilities
        self.camera = camera
	
    def practice(self):
        print("warned up")
 
        #self.motors.calc_wheel_offset(3)
        #move to airlock
        self.camera.warm_up()
        #self.servos.change_servo_positions(SERVO_INCREMENT, SERVO_SLEEP_TIME, {0: 1400})
        print("Driving to black line...")
        self.drive_to_object(REFLECTANCE, FRONT_FLOOR_REFLECTANCE_TARGET, speed=1250, pass_target=True)
        print("turning")
        self.motors.turn(-1000,24)
        # I am going to use the drive_to_black line function as a base to code our movement for the lego for the whole round
        print("using camera")
        time.sleep(2)
        #kipr.camera_open()
        
        print("test 1")
        X = self.camera.get_objects_x_center(0,1)
        print("test 2")
        Y = self.camera.get_objects_y_center(0,1)
      
            
        print(X)
        print(Y)
            
        self.camera.find_object(0)
        print("temu")
        self.camera.align_with_object_x(0, 1, X, max_deviation=5, turn_speed=1, timer=True)
        print("monkey")
        self.camera.drive_towards_object(0,1,X,Y, area_threshold=None, max_deviation=5, turn_speed=1, direction=1, timer=True)
        #self.camera.find_object(0)
        #self.camera.align_with_object_x(0, 1, X, max_deviation, turn_speed, timer=T)
        
        print("balls")
        #Close claw
        #self.servos.change_servo_positions(SERVO_INCREMENT, SERVO_SLEEP_TIME, {0: 150})
        #Move Backward and Forwards
        
        #time.sleep(3)
        #self.motors.drive_distance(1000,50)
        #time.sleep(1)            
            
        #self.servos.change_servo_positions(SERVO_INCREMENT, SERVO_SLEEP_TIME, {0: 1300})
        #self.motors.drive_distance(-1000,20)
        #self.servos.change_servo_positions(SERVO_INCREMENT, SERVO_SLEEP_TIME, {0: 550})
        #self.motors.drive_distance(1000,10)
        #self.servos.change_servo_positions(SERVO_INCREMENT, SERVO_SLEEP_TIME, {0: 750})
        #self.motors.drive_distance(-1000,20)
        #time.sleep(3)
        #self.motors.drive_distance(1000,30)
        #self.servos.change_servo_positions(SERVO_INCREMENT, SERVO_SLEEP_TIME, {0: 950})
        #self.servos.change_servo_positions(SERVO_INCREMENT, SERVO_SLEEP_TIME, {0: 700})
            
            
        #Unlock Claw
        #self.motors.change_servo_positions(-SERVO_INCREMENT, SERVO_SLEEP_TIME, {0 : 150})
#        while self.sensors[REFLECTANCE].analog() < 240:
#            print("looking for black")
#            self.motors.drive_speed({0: 1000, 1: -1000})
#        print("found black")
    def execute_game(self):
            
        self.start_on_light(LIGHT_SENSOR)

        debug = False
        game_timer = TimeTracker()
        self.servos.change_servo_positions(SERVO_INCREMENT, SERVO_SLEEP_TIME, {CLAW_PORT: CLAW_DOWN_POSITION, PING_PORT: PING_UP_POSITION, FRONT_PORT: FRONT_DOWN_POSITION})
        if debug:            
            t = multiprocessing.Process(target=self.debug_analog_sensor_values, args=([SIDE_REFLECTANCE],))
            t.start()
            
        kipr.shut_down_in(118)
            

        self.drive_to_material_transport()
            
        self.drive_to_ping_pong()
        self.shake_balls(game_timer)
        self.move_material_transport()
        self.drop_balls()
        self.retreat_to_starting()
            
        if debug:
            t.join()

    #NEEDED
    
    def turn_to_airlock(self):
        print("Driving to black line...")
        self.drive_to_object(REFLECTANCE, FRONT_FLOOR_REFLECTANCE_TARGET, speed=1150, pass_target=True)
        self.motors.turn(1000,-110)
     
    def drive_to_material_transport(self):
        print("Driving to black line...")
        self.drive_to_object(REFLECTANCE, FRONT_FLOOR_REFLECTANCE_TARGET, speed=1250, pass_target=True)
        self.motors.drive_distance(1250, 10)
        print("turning in mt function")
        self.motors.turn(1000, -110)
        self.servos.change_servo_positions(SERVO_INCREMENT, SERVO_SLEEP_TIME, {CLAW_PORT: CLAW_DOWN_POSITION, PING_PORT: PING_UP_POSITION, FRONT_PORT: FRONT_DOWN_POSITION})
        print("claw down")
        self.utilities.follow_line_PID(KP, KI, KD, LINE_FOLLOW_BASE_SPEED, side="left", location="digital", target=FLOOR_REFLECTANCE_TARGET, time_to_travel=5, direction=-1)
        self.motors.turn(1000, 20)
        self.utilities.follow_line_PID(KP, KI, KD, LINE_FOLLOW_BASE_SPEED, side="right", location="digital", target=FLOOR_REFLECTANCE_TARGET, time_to_travel=2, direction=-1)
        self.servos.change_servo_positions(SERVO_INCREMENT, SERVO_SLEEP_TIME, {CLAW_PORT: CLAW_UP_POSITION, PING_PORT: PING_UP_POSITION})
        print("claw up")
        self.utilities.follow_line_PID(KP, KI, KD, LINE_FOLLOW_BASE_SPEED, side="right", location="rangefinder", target=RANGEFINDER_THRESHOLD, time_to_travel=3, direction=-1)
        self.utilities.follow_line_PID(KP, KI, KD, LINE_FOLLOW_BASE_SPEED, side="right", location="digital", target=FLOOR_REFLECTANCE_TARGET, time_to_travel=2, direction=-1)

        self.motors.turn(1000, -4)
        self.motors.drive_distance(-1000, 8)

        self.servos.change_servo_positions(SERVO_INCREMENT, SERVO_SLEEP_TIME, {CLAW_PORT: CLAW_DOWN_POSITION, PING_PORT: PING_UP_POSITION, FRONT_PORT: FRONT_DOWN_POSITION})
        #time.sleep(3)
        #self.motors.turn(1000, 4)

    
    #NEEDED
    def drive_to_ping_pong(self):
        #Drive to ping pong drop
        self.motors.drive_distance(1000, 5)
        self.motors.turn(1000,10)
        line_count = 0
        while line_count < 2:
            self.utilities.follow_line_PID(KP, KI, KD, LINE_FOLLOW_BASE_SPEED, side="left", location="black_line", sensor=SIDE_REFLECTANCE, target=FLOOR_REFLECTANCE_TARGET, time_to_travel=10, direction=1)
            print("Reached black line {}".format(line_count))
            self.drive_to_object(SIDE_REFLECTANCE, SIDE_FLOOR_REFLECTANCE_TARGET, speed=1500, pass_target=True)
            print("Passed black line {}".format(line_count))
            line_count += 1
        #self.motors.drive_distance(1500, 10)
        self.utilities.follow_line_PID(KP, KI, KD, LINE_FOLLOW_BASE_SPEED, side="left", location="floor", sensor=None, target=None, time_to_travel=2, direction=1)
        print("Waiting for last line")
        self.utilities.follow_line_PID(KP, KI, KD, LINE_FOLLOW_BASE_SPEED, side="left", location="black_line", sensor=SIDE_REFLECTANCE, target=FLOOR_REFLECTANCE_TARGET, time_to_travel=10, direction=1)
        print("Reached last black line")
        self.drive_to_object(SIDE_REFLECTANCE, SIDE_FLOOR_REFLECTANCE_TARGET, speed=1500, pass_target=True)
        print("Passed last black line")
        self.motors.drive_distance(1000, 30.25)
        self.motors.turn(500, 3)
        self.motors.drive_distance(1000, 2)
                
    def shake_balls(self, timer):
        print("Waiting to shake")
        while timer.get_current_time() < 60 + 5:
            continue
        #shake_direction = 1
        #for _ in range(6):
        #    self.motors.turn(1500, 10 * shake_direction)
        #    self.motors.drive_distance(1500 * shake_direction, 2)
        #    shake_direction *= - 1
        #time.sleep(2)
        #print("Shaking balls")
        self.motors.drive_distance(-1500, 6)
    
    def move_material_transport(self):
        self.utilities.follow_line_PID(KP, KI, KD, LINE_FOLLOW_BASE_SPEED, side="left", location="black_line", sensor=SIDE_REFLECTANCE, target=FLOOR_REFLECTANCE_TARGET, time_to_travel=3, direction=-1)
        print("Reached black line")
        self.motors.turn(1000, 90)
        self.utilities.follow_line_PID(KP, KI, KD, LINE_FOLLOW_BASE_SPEED, side="left", location="floor_backwards", sensor=None, target=None, time_to_travel=4, direction=-1)
        self.servos.change_servo_positions(SERVO_INCREMENT, SERVO_SLEEP_TIME, {CLAW_PORT: CLAW_UP_POSITION})
    
    #NEEDED
    def drop_balls(self):
        self.drive_to_object(SIDE_REFLECTANCE, SIDE_FLOOR_REFLECTANCE_TARGET, speed=1500, pass_target=False)
        self.motors.turn(1000, -180)
        self.utilities.follow_line_PID(KP, KI, KD, LINE_FOLLOW_BASE_SPEED, side="right", location="move_mt_floor", sensor=LEVER_FRONT, target=None, time_to_travel=6.5, direction=1)
        self.motors.turn(1000, 10)
        self.servos.change_servo_positions(SERVO_INCREMENT, SERVO_SLEEP_TIME, {PING_PORT: PING_DOWN_POSITION})
        self.servos.change_servo_positions(SERVO_INCREMENT, SERVO_SLEEP_TIME, {FRONT_PORT: FRONT_UP_POSITION})
        time.sleep(0.5)
        self.servos.change_servo_positions(SERVO_INCREMENT, SERVO_SLEEP_TIME, {FRONT_PORT: FRONT_DOWN_POSITION})
        time.sleep(0.5)
        self.servos.change_servo_positions(SERVO_INCREMENT, SERVO_SLEEP_TIME, {FRONT_PORT: FRONT_UP_POSITION})
        time.sleep(0.5)
        self.servos.change_servo_positions(SERVO_INCREMENT, SERVO_SLEEP_TIME, {FRONT_PORT: FRONT_DOWN_POSITION})



        self.drive_to_object(SIDE_REFLECTANCE, SIDE_FLOOR_REFLECTANCE_TARGET, speed=-1500, pass_target=False)
        self.motors.turn(1000, 180)
        self.utilities.follow_line_PID(KP, KI, KD, LINE_FOLLOW_BASE_SPEED, side="left", location="floor_backwards", sensor=None, target=None, time_to_travel=2, direction=-1)
        self.servos.change_servo_positions(SERVO_INCREMENT, SERVO_SLEEP_TIME, {CLAW_PORT: CLAW_DOWN_POSITION})
        self.motors.turn(1000, -90)
            
    #NEEDED
    def drive_to_dropoff(self):
 		#Drive to material transport drop off
        for i in range(3):
            self.utilities.follow_line_PID(KP, KI, KD, LINE_FOLLOW_BASE_SPEED, side="left", location="black_line", sensor=SIDE_REFLECTANCE, target=SIDE_REFLECTANCE_TARGET, time_to_travel=10, direction=1)
            print("Reached black line {}".format(i))
            self.drive_to_object(SIDE_REFLECTANCE, SIDE_REFLECTANCE_TARGET, speed=1500, pass_target=True)
            
        return
        self.motors.drive_distance(-1250, 5)
        self.motors.turn(1000, -45)
        self.servos.change_servo_positions(SERVO_INCREMENT, SERVO_SLEEP_TIME, {CLAW_PORT: CLAW_UP_POSITION})
        self.motors.turn(1000, 180)
        self.servos.change_servo_positions(SERVO_INCREMENT, SERVO_SLEEP_TIME, {PING_PORT: BALL_DROP_POSITION})

    def drive_to_black_line(self):
        print("Driving to black line...")
        self.drive_to_object(REFLECTANCE, FRONT_FLOOR_REFLECTANCE_TARGET, speed=1250, pass_target=False)
        #self.motors.drive_distance(1250, 10)
        print("turning")
        #self.motors.turn(1000, -20)
        self.camera.find_object(1,0)
        self.utilities.follow_line_PID(KP, KI, KD, LINE_FOLLOW_BASE_SPEED, side="left", location="black_line", sensor=REFLECTANCE_BACKWARDS, target=FLOOR_REFLECTANCE_TARGET, time_to_travel=10, direction=1)
        self.servos.change_servo_positions(SERVO_INCREMENT, SERVO_SLEEP_TIME, {CLAW_PORT: CLAW_UP_POSITION})
        self.utilities.squareUp("edge", TOUCH_BACK_BOTTOM_LEFT, TOUCH_BACK_BOTTOM_RIGHT, speed=1250, time_to_squareUp=10)
        print("Hit wall moving forward...")
        self.motors.drive_distance(1250, 15)
        print("Facing black line...")
        self.motors.turn(1000,-90)
        self.utilities.squareUp("edge", TOUCH_BACK_BOTTOM_LEFT, TOUCH_BACK_BOTTOM_RIGHT, speed=1250, time_to_squareUp=5)
        print("Driving to main black line...")
        self.drive_to_object(REFLECTANCE, FLOOR_REFLECTANCE_TARGET, speed=1250, pass_target=False)
        self.motors.turn(1000, 90)
        self.utilities.squareUp("edge", TOUCH_BACK_BOTTOM_LEFT, TOUCH_BACK_BOTTOM_RIGHT, speed=1250, time_to_squareUp=2)
        self.servos.change_servo_positions(SERVO_INCREMENT, SERVO_SLEEP_TIME, {CLAW_PORT: CLAW_DOWN_POSITION})
        print("Standing next to black line...")
            
    def retreat_to_starting(self):
        self.servos.change_servo_positions(SERVO_INCREMENT, SERVO_SLEEP_TIME, {CLAW_PORT: CLAW_UP_POSITION})
        #self.motors.turn(1250, -115)
        self.motors.drive_distance(1250, 60)
        return
        
    def start_on_light(self, port):
        print("Press button to begin recording LIGHT ON VALUE...")
        while kipr.side_button() == 0:
            pass
        print("Recording value...")
        off_value = 0
        on_value = 0
        start_time = time.time()
        calibration_time = 3
        increment_time = 0.1
        increments = 0
        while time.time() - start_time < calibration_time:
            on_value += kipr.analog(port)
            time.sleep(increment_time)
            increments += 1
            if increments % 10 == 0:
                print "{}s...".format(int(increment_time * increments)),
        
        on_value /= (calibration_time / increment_time)
        print("LIGHT OFF VALUE: {}\n".format(on_value))
        
        print("Press button to begin recording LIGHT OFF VALUE...")
        while kipr.side_button() == 0:
            pass
        print("Recording value...")
        increments = 0
        start_time = time.time()
        while time.time() - start_time < calibration_time:
            off_value += kipr.analog(port)
            time.sleep(increment_time)
            increments += 1
            if increments % 10 == 0:
                print "{}s...".format(int(increment_time * increments)),
        off_value /= (calibration_time / increment_time)
        print("LIGHT ON VALUE: {}\n".format(off_value))
        
        THRESHOLD_VALUE = (on_value + off_value) / 2# - abs(on_value - off_value) / 4
        print("LIGHT THRESHOLD VALUE: {}".format(THRESHOLD_VALUE))
        print("READY TO RUN...")
        print(kipr.analog(port))
        while kipr.analog(port) > THRESHOLD_VALUE:
            pass
        
    def drive_to_object(self, sensor, target, speed=MAX_MOTOR_SPEED, pass_target=False):
        while self.sensors[sensor].analog() < target:
            # print(self.sensors[sensor].analog())
            self.motors.drive_speed({RIGHT_MOTOR_PORT: speed, LEFT_MOTOR_PORT: speed})
        if pass_target:
            while self.sensors[sensor].analog() > target - 200:
                self.motors.drive_speed({RIGHT_MOTOR_PORT: speed, LEFT_MOTOR_PORT: speed})
        self.motors.lock_wheels()

    def find_object(self, target):
        degree = 0
        while self.sensors[RANGEFINDER_FRONT].analog() < target:
            self.motors.turn(1000, 1)
            degree += 1

        start = time.time()

        while time.time() - start < 0.25:
            self.motors.turn(1000, 1)
            degree += 1
        self.motors.lock_wheels()

        return degree

    def start(self, port):
        while kipr.analog(port) > 3000:
            pass
   
    def debug_analog_sensor_values(self, sensors):
        start = time.time()
        poll = time.time()
        while True:
            if time.time() - poll > 0.2:
                for key in sensors:
                    print("{}: {}".format(key, self.sensors[key].analog()))
                poll = time.time()
            if  time.time() - start > 118:
                break