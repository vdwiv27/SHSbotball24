import kipr
import time
import multiprocessing as mp
from Drive import *
from Camera import *
from Servo import *
from Create_Utilities import *
from Create_constants import *
from Motor import *


class Controller:
    def __init__(self, drive, servo, motorServos, camera, utilities, sensors):
        self.sensors = sensors
        self.start_time = None
        self.drive = drive
        self.servos = servo
        self.motorServos = motorServos
        self.camera = camera
        self.utilities = utilities
    def practice(self):
		self.drive.drive_distance(-5,5)
		self.drive.turn(45,150)
		self.drive_until_collision(-250)
		self.drive.drive_distance(5,5)
        
    def execute_game(self):
        #self.servos.change_servo_positions(SERVO_INCREMENT, SERVO_SLEEP_TIME, {ARM_PORT: ARM_STARTING_POSITION, WRIST_PORT: WRIST_STARTING_POSITION, CLAW_PORT: CLAW_STARTING_POSITION})
        self.practice()
		#self.start_on_light(port=0)
        #kipr.shut_down_in(118)

        #self.drive_to_botgal()
        #self.grab_and_drop_botgal()
        #self.recenter_create()

        #self.drive_to_yellow_cube_left_alt()
        #self.grab_cube()
        #self.drive_back_to_lab(side="left", height="tall", first=False)
        #self.stack_cube(backup_distance=BACKUP_DISTANCE_1, arm_position=ARM_CUBE_POSITION_DROP_1, wrist_position=WRIST_CUBE_POSITION_DROP_1)

        #self.drive_to_cube(vertical_distance_alignment=RIGHT_TALL_TOWER_ALIGNMENT_DISTANCE, 
                          # horizontal_distance_alignment=RIGHT_TALL_TOWER_HORIZONTAL_DISTANCE,
                          # vertical_distance_final=RIGHT_TALL_TOWER_VERTICAL_DISTANCE,
                          # side="right",
                          # height="tall")
        #self.grab_cube()
        #self.drive_back_to_lab(side="right", height="tall")
        #self.stack_cube(backup_distance=BACKUP_DISTANCE_2, arm_position=ARM_CUBE_POSITION_DROP_2, wrist_position=WRIST_CUBE_POSITION_DROP_2)

        #self.drive_to_cube(vertical_distance_alignment=RIGHT_SHORT_TOWER_ALIGNMENT_DISTANCE, 
                           #horizontal_distance_alignment=RIGHT_SHORT_TOWER_HORIZONTAL_DISTANCE,
                           #vertical_distance_final=RIGHT_SHORT_TOWER_VERTICAL_DISTANCE,
                           #side="right",
                           #height="short")
        #self.grab_cube()
        #self.drive_back_to_lab(side="right", height="short")
        #self.stack_cube(backup_distance=BACKUP_DISTANCE_3, arm_position=ARM_CUBE_POSITION_DROP_3, wrist_position=WRIST_CUBE_POSITION_DROP_3)

        #self.drive_to_cube(vertical_distance_alignment=LEFT_SHORT_TOWER_ALIGNMENT_DISTANCE, 
                           #horizontal_distance_alignment=LEFT_SHORT_TOWER_HORIZONTAL_DISTANCE,
                           #vertical_distance_final=LEFT_SHORT_TOWER_VERTICAL_DISTANCE,
                           #side="left",
                           #height="short")
        #self.grab_cube()
        #self.drive_back_to_lab(side="left", height="short")
        #self.stack_cube(backup_distance=BACKUP_DISTANCE_4, arm_position=ARM_CUBE_POSITION_DROP_4, wrist_position=WRIST_CUBE_POSITION_DROP_4, last=True)
    def drive_to_square(self):
		self.drive.drive_distance(5,5)
		self.drive.turn(45,150)
		#self.drive_until_collision(-250)
		#self.drive.drive_distance(5,5)
    def drive_to_botgal(self):
        # Turn to starting position
        self.drive.turn(-45, 150)
            
        # Turn and get out of starting box
        # Change servo positions to grab
        t = mp.Process(target=self.servos.change_servo_positions, args=(SERVO_INCREMENT, SERVO_SLEEP_TIME, {ARM_PORT: ARM_GRAB_POSITION_TOP, WRIST_PORT: WRIST_GRAB_POSITION_TOP, CLAW_PORT: CLAW_OPEN_POSITION}))
        t.start()

        # Align on black starting box line
        self.drive.turn(-45, 150)
        self.utilities.align_with_line(direction=-1,cross_line=True)
            
        # Align on center line
        print("ALIGNED TO START BOX BLACK LINE")
        self.drive.drive_distance(50, -450)
        self.utilities.align_with_line(direction=-1,cross_line=False, speed=250)
        self.drive.drive_distance(HORIZONTAL_BOTGAL_ALIGNMENT_DISTANCE, -250)
        self.drive.turn(90, 150)

        self.utilities.align_with_line(direction=-1,cross_line=False)

        print("DRIVING TO BOTGAL")
        t.join()
        self.drive.drive_distance(43, -250)

        print("AT BOTGAL")
    
    def drive_to_botgal_alt(self):
        self.drive.turn(-45, 150)
        self.utilities.align_with_line(direction=-1,cross_line=True)
        self.drive.turn(45, 150)
        # Turn and get out of starting box

        # Change servo positions to grab
        t = mp.Process(target=self.servos.change_servo_positions, args=(SERVO_INCREMENT_FAST, SERVO_SLEEP_TIME, {ARM_PORT: ARM_GRAB_POSITION_TOP, WRIST_PORT: WRIST_GRAB_POSITION_TOP, CLAW_PORT: CLAW_OPEN_POSITION}))
        t.start()
        self.drive.drive_distance(75, -450)
        self.drive.drive_distance(25, -250)
    
    def grab_and_drop_botgal_alt(self):
        # Raise botgal
        print("GRABBING BOTGAL")
        self.servos.change_servo_positions(SERVO_INCREMENT, SERVO_SLEEP_TIME, {CLAW_PORT: CLAW_CLOSED_POSITION})
        time.sleep(0.5)
        self.servos.change_servo_positions(SERVO_INCREMENT, SERVO_SLEEP_TIME, {ARM_PORT: ARM_BOTGAL_RAISE_POSITION, WRIST_PORT: WRIST_BOTGAL_RAISE_POSITION})
        time.sleep(0.5)
        self.drive.turn(45, 250)

        # Align on middle black line
        print("ALIGNING TO MIDDLE BLACK LINE")
        self.utilities.align_with_line(direction=1,cross_line=False)
        # Lower botgal for stability purposes
        self.servos.change_servo_positions(SERVO_INCREMENT_FAST, SERVO_SLEEP_TIME, {ARM_PORT: ARM_BOTGAL_LOWER_POSITION, WRIST_PORT: WRIST_BOTGAL_LOWER_POSITION})
        
        self.drive.drive_distance(15, -450)
        self.drive.turn(90, 250)
        self.utilities.align_with_line(direction=1,cross_line=False)
        self.drive.turn(90, 250)
        self.utilities.align_with_line(direction=-1,cross_line=False)
        
        # Turn 120 degrees to face material analysis box
        self.drive.turn(-60, 250)

        # Drive into material analysis box
        self.drive.drive_distance(BOTGAL_FORWARD_DROP_DISTANCE, -450)

        # Drop botgal
        self.servos.change_servo_positions(SERVO_INCREMENT_FAST, SERVO_SLEEP_TIME, {ARM_PORT: ARM_BOTGAL_DROP_POSITION, WRIST_PORT: WRIST_BOTGAL_DROP_POSITION})
        t = mp.Process(target=self.servos.change_servo_positions, args=(SERVO_INCREMENT, SERVO_SLEEP_TIME, {CLAW_PORT: CLAW_OPEN_POSITION}))
        t.start()
        self.drive.drive_distance(BOTGAL_FORWARD_DROP_DISTANCE, -450)
        t.join()
                 
    def grab_and_drop_botgal(self):
        # Raise botgal
        print("GRABBING BOTGAL")
        self.servos.change_servo_positions(SERVO_INCREMENT, SERVO_SLEEP_TIME, {CLAW_PORT: CLAW_CLOSED_POSITION})
        time.sleep(0.5)
        self.servos.change_servo_positions(SERVO_INCREMENT, SERVO_SLEEP_TIME, {ARM_PORT: ARM_BOTGAL_RAISE_POSITION, WRIST_PORT: WRIST_BOTGAL_RAISE_POSITION})
        time.sleep(0.5)

        # Align on middle black line
        print("ALIGNING TO MIDDLE BLACK LINE")
        self.utilities.align_with_line(direction=1,cross_line=False)
        # Lower botgal for stability purposes
        t = mp.Process(target=self.servos.change_servo_positions, args=(SERVO_INCREMENT_FAST, SERVO_SLEEP_TIME, {ARM_PORT: ARM_BOTGAL_LOWER_POSITION, WRIST_PORT: WRIST_BOTGAL_LOWER_POSITION}))
        t.start()
        
        # Turn 120 degrees to face material analysis box
        self.drive.turn(120, 250)

        # Drive into material analysis box
        self.drive.drive_distance(BOTGAL_FORWARD_DROP_DISTANCE, -250)
        t.join()

        # Drop botgal
        self.servos.change_servo_positions(SERVO_INCREMENT_FAST, SERVO_SLEEP_TIME, {ARM_PORT: ARM_BOTGAL_DROP_POSITION, WRIST_PORT: WRIST_BOTGAL_DROP_POSITION})
        self.servos.change_servo_positions(SERVO_INCREMENT_FAST, SERVO_SLEEP_TIME, {CLAW_PORT: CLAW_OPEN_POSITION})
        self.drive.drive_distance(BOTGAL_FORWARD_DROP_DISTANCE, -250)
        
        # may not be needed
        # self.servos.change_servo_positions(SERVO_INCREMENT, SERVO_SLEEP_TIME, {ARM_PORT: 650})

    def recenter_create(self):
        # Change servo positions to grab
        t = mp.Process(target=self.servos.change_servo_positions, args=(SERVO_INCREMENT_FAST, SERVO_SLEEP_TIME, {ARM_PORT: ARM_DRIVE_POSITION, WRIST_PORT: WRIST_DRIVE_POSITION, CLAW_PORT: CLAW_STARTING_POSITION}))
        t.start()

        # Drive back out of material analysis box (question: why 30 and not 40 cm?)        
        self.drive.drive_distance(30, 450)
        
        # Turn 120 degrees clockwise to face center tower
        self.drive.turn(-120, 250)
        
        # Alignment steps
        self.drive.drive_distance(10, -450)
        self.utilities.align_with_line(direction=1,cross_line=False, speed=150)
        
        t.join()

    def drive_to_yellow_cube_left_alt(self):
        # Turn 90 degrees counter-clockwise to face right side of board
        self.drive.turn(90, 250)

        # Drive to right tall tower
        print("DRIVING TO LEFT TALL TOWER HORIZONTALLY")
        self.drive.drive_distance(LEFT_TALL_TOWER_HORIZONTAL_DISTANCE, -450)

        # Turn 90 degrees clockwise to face tower
        self.drive.turn(-90, 250)

        # alignment steps
        self.drive.drive_distance(5, -450)
        self.utilities.align_with_line(direction=1,cross_line=False)

        # setup claw
        t = mp.Process(target=self.servos.change_servo_positions, args=(SERVO_INCREMENT_FAST, SERVO_SLEEP_TIME, {ARM_PORT: ARM_GRAB_CUBE_POSITION_TOP, WRIST_PORT: WRIST_GRAB_CUBE_POSITION_TOP}))
        t.start()
            
        # drive to yellow cube right
        print("DRIVING TO TALL LEFT TOWER")
        self.drive.drive_distance(LEFT_TALL_TOWER_VERTICAL_DISTANCE, -450)
        t.join()

    def drive_to_yellow_cube_right(self):
        # drive out from botgal
        self.drive.drive_distance(25, 150)
        self.drive.turn(90, 250)
        # alignment steps
        self.drive.drive_distance(10, -150)
        self.utilities.align_with_line(direction=1,cross_line=False)
        # drive to yellow cube horizontally
        self.drive.drive_distance(RIGHT_TALL_TOWER_HORIZONTAL_DISTANCE, 250)
        # face yellow cube
        self.drive.turn(-90, 250)
        # alignment steps
        self.utilities.align_with_line(direction=1,cross_line=False)
        # setup claw
        t = mp.Process(target=self.servos.change_servo_positions, args=(SERVO_INCREMENT_FAST, SERVO_SLEEP_TIME, {ARM_PORT: ARM_GRAB_CUBE_POSITION_TOP, WRIST_PORT: WRIST_GRAB_CUBE_POSITION_TOP}))
        t.start()
        # drive to yellow cube right
        self.drive.drive_distance(RIGHT_TALL_TOWER_VERTICAL_DISTANCE, -250)
        t.join()

    def grab_cube(self):
        self.servos.change_servo_positions(SERVO_INCREMENT, SERVO_SLEEP_TIME, {CLAW_PORT: CLAW_CLOSED_POSITION})

    def drive_back_to_lab(self, side, height, first=False):
        if side == "left":
            direction = 1
        else:
            direction = -1

        # Lift cube up
        self.servos.change_servo_positions(SERVO_INCREMENT, SERVO_SLEEP_TIME, {ARM_PORT: ARM_CUBE_RAISE_POSITION, WRIST_PORT: WRIST_CUBE_RAISE_POSITION})
        # Move out of tower
        if height == "tall":
            self.drive.drive_distance(5, -450)
        else:
            self.drive.drive_distance(10, -450)
        # Rotate and align to center black line
        print("ALIGNING BACK TO CENTER LINE")
        self.drive.turn(90, 250)
        if height == "short":
            self.drive.drive_distance(50, 450 * direction)
        else:
            self.drive.drive_distance(15, 450 * direction)
        self.utilities.align_with_line(direction=direction, cross_line=False)
        # Drive out of black line to cube stacking location horizontally
        self.drive.drive_distance(30, -450)
        # Rotate and align to parallel black line while dropping cube low to push anything below the cube
        self.drive.turn(90, 250)

        if first:
            t = mp.Process(target=self.servos.change_servo_positions, args=(SERVO_INCREMENT, SERVO_SLEEP_TIME, {ARM_PORT: ARM_CUBE_LOWER_POSITION, WRIST_PORT: WRIST_CUBE_LOWER_POSITION}))
            t.start()

        self.utilities.align_with_line(direction=-1, cross_line=False)
    
        if first:
            t.join()
            time.sleep(0.25)

    def drive_to_cube(self, vertical_distance_alignment, horizontal_distance_alignment, vertical_distance_final, side, height):
        if side == "left":
            direction = -1
        else:
            direction = 1

        if height == "tall":
            arm_position = ARM_GRAB_CUBE_POSITION_TOP
            wrist_position = WRIST_GRAB_CUBE_POSITION_TOP
        else:
            arm_position = ARM_GRAB_CUBE_POSITION_BOTTOM
            wrist_position = WRIST_GRAB_CUBE_POSITION_BOTTOM       

        # drive out from line
        if height == "tall":
            t = mp.Process(target=self.servos.change_servo_positions, args=(SERVO_INCREMENT_FAST, SERVO_SLEEP_TIME, {ARM_PORT: ARM_DRIVE_POSITION, WRIST_PORT: WRIST_DRIVE_POSITION, CLAW_PORT: CLAW_STARTING_POSITION}))
        else:
            t = mp.Process(target=self.servos.change_servo_positions, args=(SERVO_INCREMENT_FAST, SERVO_SLEEP_TIME, {ARM_PORT: ARM_DRIVE_POSITION, WRIST_PORT: WRIST_DRIVE_POSITION, CLAW_PORT: CLAW_STARTING_POSITION}))
        t.start()
        print("DRIVING OUT FROM BLACK HORIZONTAL LINE TOWARDS TOWERS", side, height)
        self.drive.drive_distance(vertical_distance_alignment, 450)
        t.join()

        self.drive.turn(90 * direction, 250)
        # alignment steps
        print("ALIGNING WITH MIDDLE BLACK LINE")
        self.utilities.align_with_line(direction = -direction, cross_line=False)
        # drive to yellow cube horizontally
        print("DRIVING HORIZONTALLY TO CUBE")
        self.drive.drive_distance(horizontal_distance_alignment, -450)
        # face yellow cube
        self.drive.turn(90 * direction, 250)
        # alignment steps
        #self.drive.drive_distance(5, -250)
        self.utilities.align_with_line(direction=1, cross_line=False)
        # setup claw
        t=mp.Process(target=self.servos.change_servo_positions, args=(SERVO_INCREMENT, SERVO_SLEEP_TIME, {ARM_PORT: arm_position, WRIST_PORT: wrist_position}))
        t.start()
        # drive to yellow cube right
        print("DRIVING TO CUBE", side, height)
        if side == "left" and height == "short":
            t.join()
            self.drive.drive_distance(vertical_distance_final, -450)
        else:
            self.drive.drive_distance(vertical_distance_final, -450)
            t.join()
        time.sleep(0.25)
   
    def stack_cube(self, backup_distance, arm_position, wrist_position, last=False):
        print("BACKING UP", backup_distance)
        self.drive.drive_distance(backup_distance, 450)
        self.servos.change_servo_positions(SERVO_INCREMENT, SERVO_SLEEP_TIME, {ARM_PORT: arm_position, WRIST_PORT: wrist_position})
        if not last:
            time.sleep(0.25)
            self.servos.change_servo_positions(SERVO_INCREMENT_FAST, SERVO_SLEEP_TIME, {CLAW_PORT: CLAW_STARTING_POSITION})

    def drive_until_collision(self, speed):
        val = 0
        while val < 10:
            if kipr.get_create_lbump() == 1 or kipr.get_create_rbump() == 1:
                val += 1
            if kipr.get_create_lbump() == 1 and kipr.get_create_rbump() == 1:
                print("MAJOR COLLISION")
            self.drive.drive_speed(speed, speed)
        self.drive.drive_speed(0, 0)
   
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
                print("{}s...".format(int(increment_time * increments))),
        
        on_value /= (calibration_time / increment_time)
        print("LIGHT ON VALUE: {}\n".format(on_value))
        
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
                print("{}s...".format(int(increment_time * increments))),
        off_value /= (calibration_time / increment_time)
        print("LIGHT OFF VALUE: {}\n".format(off_value))
        
        THRESHOLD_VALUE = (on_value + off_value) / 2# - abs(on_value - off_value) / 4
        print("LIGHT THRESHOLD VALUE: {}".format(THRESHOLD_VALUE))
        print("READY TO RUN...")
        
        while kipr.analog(port) > THRESHOLD_VALUE:
            pass