import kipr
from Create_constants import *

class Servo:
    def __init__(self, port):
        self.port = port

    def set_servo_position(self, pos):
        return kipr.set_servo_position(self.port, pos)

    def get_servo_position(self):
        return kipr.get_servo_position(self.port)

    def increment_servo_position(self, direction, increment):
        return self.set_servo_position(self.get_servo_position() + increment * direction)


class ServoHandler:
    def __init__(self, servos):
        # Assume servos is a dictionary of servos being used
        self.servos = servos
        kipr.enable_servos()

    def disable_servos(self, servos_to_disable):
        # servos_to_disable is a list of servo ports to disable
        for servo in servos_to_disable:
            kipr.disable_servo(self.servos[servo])
        return
                
    def enable_servos(self, servos_to_enable):
        # servos_to_disable is a list of servo ports to disable
        for servo in servos_to_enable:
            kipr.enable_servo(self.servos[servo])
        return
            
    def change_servo_positions(self, increment, time, pos):
        for position in pos.values():
            if position < 0:
                print("NO NEGATIVE VALUES")
                return
        # pos is a dictionary mapping of {port : new_position}

        # Calculate deviation from actual servo positions and desired servo positions
        deviation = {port: abs(self.servos[port].get_servo_position() - pos[port]) for port in pos.keys()}
        
        while max(deviation.values()) > max(increment.values()):
            for port in pos.keys():
                # Increment servo position if desired and current servo deviation is not within the desired range
                if deviation[port] > increment[port]:
                    servo = self.servos[port] # Grab the servo from the servos list
                    direction = self.determine_direction(servo, pos[port]) # Determine the direction to turn the servo
                    servo.increment_servo_position(direction, increment[port]) # Increment the servo position

            # Recalculate deviation from actual servo positions and desired servo positions
            deviation = {port: abs(self.servos[port].get_servo_position() - pos[port]) for port in pos.keys()}

            kipr.msleep(time) # Sleep for specified time between servo position changes

                
    def determine_direction(self, servo, pos):
        if servo.get_servo_position() > pos:
            return -1
        else:
            return 1
