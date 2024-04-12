import kipr
from Create_constants import *
import time

class Motor:
    def __init__(self, port, ratio=1):
        self.port = port
        self.ratio = ratio

    def mav(self, velocity): # Set speed of motor by velocity in ticks
        kipr.mav(self.port, velocity)

    def mrp(self, velocity, relative_pos): # Sets speed of motor with a goal position
        kipr.mrp(self.port, velocity, relative_pos)
    
    def mtp(self, velocity, pos): # Sets speed of motor with a goal position
        kipr.mtp(self.port, velocity, pos)

    def motor(self, power): # Set speed of motor by percent
        kipr.motor(self.port, power)

    def gmpc(self): # Get motor position count
        return kipr.gmpc(self.port)

    def cmpc(self): # Clear motor position count
        return kipr.cmpc(self.port)

class MotorServoHandler:
    def __init__(self, motorServos):
        self.motorServos = motorServos
        self.cmpc_all(self.motorServos.keys())

    def set_speed(self, speed):  # Speed is a dictionary of speeds - {0: 1500, 1: -1000}
        for port in speed.keys():
            self.motorServos[port].mav(int(speed[port]))

    def change_motorServo_positions(self, speed, pos): # Pos is a dictionary of ports: positions
        #for port in pos.keys():
        #    self.motorServos[port].mrp(speed, pos[port])
        range = {port: pos[port] - self.motorServos[port].gmpc() for port in pos.keys()}
        deviation = {port: abs(pos[port] - self.motorServos[port].gmpc()) for port in pos.keys()}
        time_vals = {port: abs(pos[port] - self.motorServos[port].gmpc()) / float(speed[port]) for port in pos.keys()}
        print(time_vals)
        
        def determine_direction(motorServo, pos):
            if motorServo.gmpc() > pos:
                return -1
            else:
                return 1
                    
        start_time = time.time()            
        while max(deviation.values()) > 10:
            for port in pos.keys():
                motorServo = self.motorServos[port]
                if deviation[port] > 10:
                    direction = determine_direction(motorServo, pos[port])
                    if range[port] > 0:
                        motorServo.mav(int(speed[port] * direction))
                        #motorServo.mav(max(speed/2 * direction, int(speed * direction * deviation[port] / float(abs(range[port])))))
                    #motorServo.mav(int(speed * direction * deviation[port] / float(range[port])))
                    else:
                        motorServo.mav(int(speed[port] * direction))
                else:
                    #print("{} Finished".format(port))
                    motorServo.mav(0)
                    kipr.msleep(10)
                
            if time.time() - start_time > max(time_vals.values()) + 1.5:
                print("STOPPING EARLY")
                break
                    
            deviation = {port: abs(pos[port] - self.motorServos[port].gmpc()) for port in pos.keys()}
            #print(deviation)
        
        #print(max(map(abs, change_pos.values())) / float(abs(speed)))
        #time.sleep(max(map(abs, change_pos.values())) / float(abs(speed)))
        for port in pos.keys():
            #self.motorServos[port].mav(-100)
            #kipr.msleep(10)
            self.motorServos[port].mav(0)
            kipr.msleep(10)
            #self.motorServos[port].cmpc()
         
        print("Finished changing motorServos")
    
    def change_motorServo_positions_relative2(self, speed, pos):
        time_vals = {port: abs(pos[port]) / float(speed[port]) for port in pos.keys()}
        direction = {port: pos[port] / abs(pos[port]) for port in pos.keys()}
        for port in pos.keys():
            print(abs(speed[port]) * direction[port])
            self.motorServos[port].mrp(speed[port] * direction[port], pos[port])
        time.sleep(max(time_vals.values()))
                
        for port in pos.keys():
            self.motorServos[port].mav(0)
            kipr.msleep(10)
         
    def change_motorServo_positions_relative(self, speed, pos): # Pos is a dictionary of ports: positions
        #for port in pos.keys():
        #    self.motorServos[port].mrp(speed, pos[port])
        new_pos = {port: self.motorServos[port].gmpc() + pos[port] for port in pos.keys()}

        deviation = {port: abs(new_pos[port] - self.motorServos[port].gmpc()) for port in pos.keys()}
        print(deviation)
        time_vals = {port: abs(pos[port]) / float(speed[port]) for port in pos.keys()}
        #print(time_vals)
        
        def determine_direction(motorServo, pos):
            if motorServo.gmpc() > pos:
                return -1
            else:
                return 1
                    
        start_time = time.time()            
        while max(deviation.values()) > 10:
            for port in pos.keys():
                motorServo = self.motorServos[port]
                if deviation[port] > 10:
                    direction = determine_direction(motorServo, new_pos[port])
                    motorServo.mav(int(speed[port] * direction))
                        #motorServo.mav(max(speed/2 * direction, int(speed * direction * deviation[port] / float(abs(range[port])))))
                    #motorServo.mav(int(speed * direction * deviation[port] / float(range[port])))
                else:
                    #print("{} Finished".format(port))
                    motorServo.mav(0)
                    kipr.msleep(10)
                
            if time.time() - start_time > max(time_vals.values()) + 1.5:
                print("STOPPING EARLY")
                break
                    
            deviation = {port: abs(new_pos[port] - self.motorServos[port].gmpc()) for port in pos.keys()}
            #print(deviation)
        
        #print(max(map(abs, change_pos.values())) / float(abs(speed)))
        #time.sleep(max(map(abs, change_pos.values())) / float(abs(speed)))
        for port in pos.keys():
            #self.motorServos[port].mav(-100)
            #kipr.msleep(10)
            self.motorServos[port].mav(0)
            kipr.msleep(10)
            self.motorServos[port].cmpc()
         
        print("Finished changing motorServos")
    # def change_motorServo_positions(self, speed, pos):
    #
    #     threads = []
    #
    #     def change_motorServo_position(speed_indiv, pos_indiv):
    #         self.motorServos[port].mrp(speed_indiv, pos_indiv)
    #         time.sleep(abs(pos_indiv) / float(speed_indiv))
    #
    #     for port in pos.keys():
    #         t = threading.Thread(target=change_motorServo_position, args=(speed[port], pos[port]))
    #         t.start()
    #         threads.append(t)
    #
    #     for thread in threads:
    #         thread.join()

    def lock_motors(self):
        self.set_speed({RIGHT_MOTOR_PORT: 0, LEFT_MOTOR_PORT: 0})
        # kipr.msleep(50)

    def cmpc_all(self, ports): # Ports is a list of ports to set motor position count to zero
        for port in ports:
            self.motorServos[port].cmpc()
                
class MotorHandler:
    def __init__(self, motors):
        self.motors = motors  # Motors is dictionary of motors - {0: Motor_0, 1: Motor_1}
        #self.gyro = Gyro()

    def drive_speed(self, speed):  # Speed is a dictionary of speeds - {0: 1500, 1: -1000}
        for port in speed.keys():
            self.motors[port].mav(int(speed[port]))

    def drive_power(self, power):  # Power is a dictionary of speeds - {0: 50, 1: -100}
        for port in power.keys():
            self.motors[port].motor(int(power[port]))

    # Drives robot straight for specified distance in cm
    # Speed is an integer because the robot drives straight
    def drive_distance(self, speed, distance):
        distance_in_ticks = abs(distance) * TICKS_PER_CM
        time_to_wait = self.calc_time(speed, distance_in_ticks)  # Calculated driving time in milliseconds
        self.drive_time({RIGHT_MOTOR_PORT: speed, LEFT_MOTOR_PORT: speed}, time_to_wait)

    def drive_time(self, speed, time_to_wait):  # Speed is a dictionary of speeds - {0: 1500, 1: -1000}
        self.drive_speed(speed)
        kipr.msleep(time_to_wait)
        self.lock_wheels()

    # Speed is an integer specifying turning speed
    def turn(self, speed, degrees):
        left_motor_dir = -1 if degrees >= 0 else 1
        right_motor_dir = -1 if degrees < 0 else 1

        ticks_to_turn = abs(TICKS_PER_REVOLUTION * (float(degrees) / 360))  # Calculate ticks to turn

        time_to_wait = self.calc_time(speed, ticks_to_turn)  # Calculated turning time in milliseconds
        self.drive_time({RIGHT_MOTOR_PORT: speed * right_motor_dir, LEFT_MOTOR_PORT: speed * left_motor_dir}, time_to_wait)

    def calc_time(self, speed, distance):
        return int((float(distance) / abs(speed)) * 1000)  # Need time to be an integer value in milliseconds

    def cmpc_all(self):
        for motor in self.motors.values():
            motor.cmpc()

    def lock_wheels(self):
        self.drive_speed({RIGHT_MOTOR_PORT: 0, LEFT_MOTOR_PORT: 0})
        kipr.msleep(10)

    # These functions probably won't be used, as they aren't necessary

    # def drive_straight(self, speed, time_to_wait):
    #      start = time.time()
    #      theta = 0
    #      error = 0
    #      target = 0
    #      current_gyro = kipr.gyro_z()
    #      while time.time() - start < time_to_wait:
    #          old_gyro = current_gyro
    #          #self.drive_speed({RIGHT_MOTOR_PORT: speed + error, LEFT_MOTOR_PORT: speed - error})
    #          if theta < target:
    #              self.drive_speed({RIGHT_MOTOR_PORT: speed - 100, LEFT_MOTOR_PORT: speed + 100})
    #          else:
    #              self.drive_speed({RIGHT_MOTOR_PORT: speed + 100, LEFT_MOTOR_PORT: speed - 100})
    #          kipr.msleep(10)
    #          current_gyro = kipr.gyro_z()
    #
    #          theta += 0.5 * (current_gyro - old_gyro) * float(10)/10000 + old_gyro * float(10)/10000
    #          print(kipr.gyro_z(), theta)
    #          error = target - theta
    #
    #
    # def gyro_z_average(self, loops):
    #     avg = 0
    #     for _ in range(loops):
    #         avg += kipr.gyro_z()
    #         kipr.msleep(3)
    #     return avg / loops
    #
    # def drive_straight3(self, Kp, Ki, Kd, base_speed, direction=1):
    #     total_error = 0
    #     last_error = 0
    #     error = 0
    #     start_time = time.time()
    #     start = time.time()
    #     while time.time() - start_time < 10:
    #         if time.time() - start > 0.1:
    #             error = kipr.gyro_z() - self.gyro.bias
    #             total_error += error
    #             error_difference = error - last_error
    #             speed_update = error * Kp + total_error * Ki + error_difference * Kd
    #             print(error, speed_update)
    #             # print("Error: "+str(error)+"\nTotal Error: "+str(total_error)+"\nError Difference: "+str(error_difference)+"\nSpeed Update: "+str(speed_update))
    #             start = time.time()
    #             last_error = error
    #         else:
    #             speed_update = error * Kp
    #         # If you want to turn on right/sharp angles, uncomment the code below
    #         # if abs(speed_update) > 375 and location == "floor":
    #         #    speed_update = int(speed_update * 2.5)
    #         new_speed = {RIGHT_MOTOR_PORT: direction * (base_speed + direction * speed_update),
    #                      LEFT_MOTOR_PORT: direction * (base_speed - direction * speed_update)}
    #         self.drive_speed(new_speed)
    #
    #
    # def drive_straight2(self, speed):  # Speed is an integer value at which to drive straight
    #     # self.cmpc_all() # When function is used with other functions, cmpc should be placed outside of any loops
    #
    #     if self.motors[0].gmpc() < self.motors[1].gmpc():
    #         self.drive_speed({RIGHT_MOTOR_PORT: speed, LEFT_MOTOR_PORT: speed * 0.95})
    #     elif self.motors[0].gmpc() > self.motors[1].gmpc():
    #         self.drive_speed({RIGHT_MOTOR_PORT: speed * 0.95, LEFT_MOTOR_PORT: speed})
    #     else:
    #         self.drive_speed({RIGHT_MOTOR_PORT: speed, LEFT_MOTOR_PORT: speed})

        # With wheel offsets/ratios defined, do this instead:
        # right_offset = self.motors[0].ratio
        # left_offset  = self.motors[1].ratio
        # self.drive_speed(0: speed * right_offset, 1: speed * left_offset)

    # Testing function meant to be used with calibration of motors

    def calc_wheel_offset(self, loops):
        motor_0_offset_forward = 0
        motor_1_offset_forward = 0

        motor_0_offset_backward = 0
        motor_1_offset_backward = 0

        for loop in range(loops):
            self.cmpc_all()
            self.drive_distance(1000, 25)
            if loop != loops - 1:
                motor_0_offset_forward += abs(float(self.motors[1].gmpc()) / self.motors[0].gmpc())
                motor_1_offset_forward += abs(float(self.motors[0].gmpc()) / self.motors[1].gmpc())
            kipr.ao()

            self.cmpc_all()
            self.drive_distance(-1000, 25)
            if loop != loops - 1:
                motor_0_offset_backward += abs(float(self.motors[1].gmpc()) / self.motors[0].gmpc())
                motor_1_offset_backward += abs(float(self.motors[0].gmpc()) / self.motors[1].gmpc())
            kipr.ao()

        motor_0_offset_forward /= (loops - 1)
        motor_1_offset_forward /= (loops - 1)

        motor_0_offset_backward /= (loops - 1)
        motor_1_offset_backward /= (loops - 1)

        motor_0_offset = float(motor_0_offset_forward + motor_0_offset_backward) / 2
        motor_1_offset = float(motor_1_offset_forward + motor_1_offset_backward) / 2

        print("Motor_0_Offset_Forward: {} \n Motor_1_Offset_Forward: {}"
              " \n Motor_0_Offset_Backward: {} \n Motor_1_Offset_Backward: {} \n "
              "Motor_0_Offset: {} \n Motor_1_Offset: {} \n ".format(motor_0_offset_forward, motor_1_offset_forward,
                                                                    motor_0_offset_backward, motor_1_offset_backward,
                                                                    motor_0_offset, motor_1_offset))