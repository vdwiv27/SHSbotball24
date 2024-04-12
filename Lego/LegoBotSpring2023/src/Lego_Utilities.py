import kipr

from Motor import *
from Servo import *
from Sensors import *
from constants import *
import time

class Utilities:
    
    '''Class used for legobot utilities'''

    def __init__(self, motors, servos, sensors):
        self.motors = motors
        self.servos = servos
        self.sensors = sensors

    def follow_line_PID(self, Kp, Ki, Kd, base_speed, sensor=None, target=None, location=None, side="left", time_to_travel=10, direction=1):
        total_error = 0
        last_error = 0
        start = time.time()
        if location == "floor":
           line_follow_start = time.time()
           while time.time() - line_follow_start < time_to_travel:
                total_error, last_error, start = self.line_follow_algorithm(Kp, Ki, Kd, base_speed, total_error, last_error, start, side, direction=direction)
        elif location == "digital":
           line_follow_start = time.time()
           while time.time() - line_follow_start < time_to_travel and self.sensors[LEVER_FRONT].digital() == 0:
                total_error, last_error, start = self.line_follow_algorithm(Kp, Ki, Kd, base_speed, total_error, last_error, start, side, direction=direction)
        elif location == "floor_backwards_to_switch":
           while self.sensors[TOUCH_BACK_BOTTOM_LEFT].digital() == 0 or self.sensors[TOUCH_BACK_BOTTOM_RIGHT].digital() == 0:
                total_error, last_error, start = self.line_follow_algorithm(Kp, Ki, Kd, base_speed, total_error, last_error, start, side="right", direction=-1)
        elif location == "floor_backwards":
           line_follow_start = time.time()
           while time.time() - line_follow_start < time_to_travel:
                total_error, last_error, start = self.line_follow_algorithm(Kp, Ki, Kd, base_speed, total_error, last_error, start, side="right", direction=-1)
        elif location == "black_line":
            while self.sensors[sensor].analog() < target:
                total_error, last_error, start = self.line_follow_algorithm(Kp, Ki, Kd, base_speed, total_error, last_error, start, side, direction)
            #print(self.sensors[sensor].analog())
        elif location == "mine":
            while self.sensors[sensor].analog() > target:
                total_error, last_error, start = self.line_follow_algorithm(Kp, Ki, Kd, base_speed, total_error, last_error, start, side, direction)
        elif location == "rangefinder":
            line_follow_start = time.time()
        elif location == "move_mt_floor":
            line_follow_start = time.time()
            while time.time() - line_follow_start < time_to_travel and self.sensors[sensor].digital() == 0:
                total_error, last_error, start = self.line_follow_algorithm(Kp, Ki, Kd, base_speed, total_error, last_error, start, side="left", direction=direction)
            

            print("rangefinding")
            print(self.sensors[RANGEFINDER].analog())
            while time.time() - line_follow_start < time_to_travel and self.sensors[RANGEFINDER].analog() < target:
                #print(self.sensors[RANGEFINDER].analog())
                total_error, last_error, start = self.line_follow_algorithm(Kp, Ki, Kd, base_speed, total_error, last_error, start, side, direction)

        self.motors.lock_wheels()
    def drive_till_black(self, speed, sensor):
        #self.drive_distance(1000,2000)
		print(self.sensors[sensor])
		while(self.sensors[sensor] < 4000):
			print("Shashwath is super handsome")
			self.motors.drive_speed({RIGHT_MOTOR_PORT: speed, LEFT_MOTOR_PORT:speed})

                    
    def line_follow_algorithm(self, Kp, Ki, Kd, base_speed, total_error, last_error, start, side, direction=1):
        
        '''Function: Line follow algorithm
           Parameters: Kp Constant (int), Kp Constant (int), Kd Constant (int), Total Error (int), Last Error (int), Start (float), Side (String), Direction (int)
           Example Call: line_follow_algorithm(Kp, Ki, Kd, 1250, 0, 0, 20.0, "right", -1)
           Return: Total Error (int), Last Error (int), Start (float)
        '''
        
        sensor = REFLECTANCE_BACKWARDS if direction == -1 else REFLECTANCE
        side_to_drive = -1 if side == "right" else 1
        error = self.sensors[sensor].analog() - LINE_FOLLOW_TARGET
        if time.time() - start > 0.1:
            total_error += error
            error_difference = error - last_error
            speed_update = error * Kp + total_error * Ki + error_difference * Kd
            # print("Error: "+str(error)+"\nTotal Error: "+str(total_error)+"\nError Difference: "+str(error_difference)+"\nSpeed Update: "+str(speed_update))
            start = time.time()
            last_error = error
        else:
            speed_update = error * Kp
        # If you want to turn on right/sharp angles, uncomment the code below
        # if abs(speed_update) > 375 and location == "floor":
        #    speed_update = int(speed_update * 2.5)
        new_speed = {RIGHT_MOTOR_PORT: direction * (base_speed + side_to_drive * speed_update), LEFT_MOTOR_PORT: direction * (base_speed - side_to_drive * speed_update)}
        self.motors.drive_speed(new_speed)
        
        return total_error, last_error, start
        
    def squareUp(self, location, left_sensor, right_sensor, speed=MAX_MOTOR_SPEED, time_to_squareUp=None): 
        # Direction is "edge" or "corner" to indicate whether the robot is squaring up at a corner or at an edge
    
        '''Function: squares up with an edge or a corner
           Parameters: Location (String), Left Sensor (string), Right Sensor (string), speed (int), Time until square up ends (float)
           Example Call: squareUp("edge", "touch0", "touch1", 1500, 3)
           Return: None
        '''
               
        if location == "corner":
            self.motors.turn(1000, 90)
                
        if time_to_squareUp == None:
           timer = False
        else:
           timer = True
           
        start_time = time.time()
               
        while self.sensors[left_sensor].digital() == 0 or self.sensors[right_sensor].digital() == 0:
            if self.sensors[left_sensor].digital() == 0 and self.sensors[right_sensor].digital() == 1:
                self.motors.drive_speed({RIGHT_MOTOR_PORT: 0.2 * speed, LEFT_MOTOR_PORT: -0.9 * speed})

            elif self.sensors[left_sensor].digital() == 1 and self.sensors[right_sensor].digital() == 0:
                self.motors.drive_speed({RIGHT_MOTOR_PORT: -0.9 * speed, LEFT_MOTOR_PORT: 0.2 * speed})

            else:
                self.motors.drive_speed({RIGHT_MOTOR_PORT: -speed, LEFT_MOTOR_PORT: -speed})
            
            if timer and time.time() - start_time > time_to_squareUp:
                print(time.time() - start_time)
                print("stopping square up early")
                break

        self.motors.lock_wheels()
