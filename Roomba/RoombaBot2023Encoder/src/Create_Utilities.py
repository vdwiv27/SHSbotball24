import kipr
from Create_constants import *
import time
from Drive import *
from Camera import *

class Utilities:
    def __init__(self, driver, camera, sensors):
        self.motors = driver
        self.camera = camera
        self.sensors = sensors

    def follow_line_PID(self, Kp, Ki, Kd, base_speed, location=None, side="left", time_to_travel=10, sensor=None):
        total_error = 0
        last_error = 0
        start = time.time()
        start_time = time.time()
        if location == "floor":
            while kipr.get_create_lbump() == 0 and kipr.get_create_rbump() == 0:
                total_error, last_error, start = self.line_follow_algorithm(Kp, Ki, Kd, base_speed, total_error,
                                                                            last_error, start, side)
                if time.time() - start_time > time_to_travel:
                    break
                        
        if location == "classifier":
            while self.sensors[sensor].digital() != 1:
                total_error, last_error, start = self.line_follow_algorithm(Kp, Ki, Kd, base_speed, total_error,
                                                                            last_error, start, side)
                if time.time() - start_time > time_to_travel:
                    break
            
        if location == "material_transport":
            print("RFCLIFF_VALUE: ", kipr.get_create_rfcliff_amt())
            while kipr.get_create_rfcliff_amt() > LINE_FOLLOW_TARGET:
                total_error, last_error, start = self.line_follow_algorithm(Kp, Ki, Kd, base_speed, total_error,
                                                                            last_error, start, side)
                if time.time() - start_time > time_to_travel:
                    break
        self.motors.lock_wheels()

    def line_follow_algorithm(self, Kp, Ki, Kd, base_speed, total_error, last_error, start, side):
        side_to_drive = 1 if side == "right" else -1
        error = kipr.get_create_lfcliff_amt() - LINE_FOLLOW_TARGET
        #print(error)
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
        #print(-(base_speed + int(speed_update * side_to_drive)), -(base_speed - int(speed_update * side_to_drive)))
        self.motors.drive_speed(-(base_speed + int(speed_update * side_to_drive)), -(base_speed - int(speed_update * side_to_drive)))
        return total_error, last_error, start

    def squareUp(self, location, speed=250, time_to_squareUp=None, reduce_end_speed=False):  # Direction is "edge" or "corner" to indicate whether the robot is squaring up at a corner or at an edge
        if location == "corner":
            self.motors.turn(-90, speed)

        if time_to_squareUp == None:
            timer = False
        else:
            timer = True

        start_time = time.time()

        while kipr.get_create_lbump() == 0 or kipr.get_create_rbump() == 0:
            if kipr.get_create_lbump() == 0 and kipr.get_create_rbump() == 1:
                self.motors.drive_speed(0, speed)
                    
            elif kipr.get_create_lbump() == 1 and kipr.get_create_rbump() == 0:
                self.motors.drive_speed(speed, 0)

            else:
                self.motors.drive_speed(-speed, -speed)
            
            if reduce_end_speed and timer and time.time() - start_time > 0.75 * time_to_squareUp:
                speed = 100

            if timer and time.time() - start_time > time_to_squareUp:
                print("Stopping square up early")
                break

        self.motors.lock_wheels()
                    
    # Requires strict calibration to determine line_follow_target value in order to work
    def align_with_line(self, direction=1, front_sensors=False, cross_line=False, speed=200, time_to_align=None):
        
        def update_left_value():
            if front_sensors:
                return kipr.get_create_lfcliff_amt()
            return kipr.get_create_lcliff_amt()
                    
        def update_right_value():
            if front_sensors:
                return kipr.get_create_rfcliff_amt()
            return kipr.get_create_rcliff_amt()
                    
        lpos = update_left_value()
        rpos = update_right_value()
        start = time.time()
        refresh_timer = time.time()
                    
        while lpos > LINE_FOLLOW_TARGET or rpos > LINE_FOLLOW_TARGET:
            if lpos < LINE_FOLLOW_TARGET and rpos > LINE_FOLLOW_TARGET:
                self.motors.drive_speed(direction * -50, direction * 50)
            elif lpos > LINE_FOLLOW_TARGET and rpos < LINE_FOLLOW_TARGET:
                self.motors.drive_speed(direction * 50, direction * -50)
            else:
                self.motors.drive_speed(direction * speed, direction * speed)
            
            lpos = update_left_value()
            rpos = update_right_value()
                    
            if time.time() - refresh_timer > 0.015:
                self.motors.update_encoder_values()
                refresh_timer = time.time()
            
            if time_to_align > 0 and time.time() - start > time_to_align:
                break
                    
        if cross_line:
            while lpos < LINE_FOLLOW_TARGET or rpos < LINE_FOLLOW_TARGET:
                self.motors.drive_speed(direction * speed, direction * speed)
                
                lpos = update_left_value()
                rpos = update_right_value()
                    
                if time.time() - refresh_timer > 0.015:
                    self.motors.update_encoder_values()
                    refresh_timer = time.time()
                    
        #print(lpos, rpos)
        self.motors.lock_wheels()
        self.motors.update_encoder_values()
        print("aligned with line")