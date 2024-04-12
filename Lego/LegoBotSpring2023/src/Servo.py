import kipr
from constants import *


class Servo:

    '''Wrapper class for KIPR Servo Functions'''

    def __init__(self, port):
        self.port = port

    def set_servo_position(self, pos):
        '''Function: Uses KIPR functions to set the servos position
        Parameters: pos (int)
        Example Call: set_servo_position(500)
        Return: None
        '''
        return kipr.set_servo_position(self.port, pos)

    def get_servo_position(self):
        '''Function: Uses KIPR functions to get the servos position
        Parameters: None 
        Example Call: get_servo_position()
        Return: Position (int)
        '''

        return kipr.get_servo_position(self.port)

    def increment_servo_position(self, direction, increment):
        '''Function: Increments the servo by increment in specific direction
        Parameters: direction (int), increment (int)
        Example Call: increment_servo_position(1, 50)
        Return: None
        '''

        return self.set_servo_position(self.get_servo_position() + increment * direction)


class ServoHandler:

    ''' A class used to control multiple servos'''

    def __init__(self, servos):
        # Assume servos is a dictionary of servos being used
        self.servos = servos
        kipr.enable_servos()

    def change_servo_positions(self, increment, time, pos):
        # pos is a dictionary mapping of {port : new_position}
        '''Function: Changes the servo position by slowly incrementing to a position in a desired time
        Parameters: increment (int), time (float), pos (int)
        Example Call: change_servo_positions(50, 0.5, 600)
        Return: None
        '''

        # Calculate deviation from actual servo positions and desired servo positions
        deviation = {port: abs(
            self.servos[port].get_servo_position() - pos[port]) for port in pos.keys()}

        while max(deviation.values()) > max(increment.values()):
            for port in pos.keys():
                # Increment servo position if desired and current servo deviation is not within the desired range
                if deviation[port] > increment[port]:
                    # Grab the servo from the servos list
                    servo = self.servos[port]
                    # Determine the direction to turn the servo
                    direction = self.determine_direction(servo, pos[port])
                    # Increment the servo position
                    servo.increment_servo_position(direction, increment[port])

            # Recalculate deviation from actual servo positions and desired servo positions
            deviation = {port: abs(
                self.servos[port].get_servo_position() - pos[port]) for port in pos.keys()}

            # Sleep for specified time between servo position changes
            kipr.msleep(time)

    def determine_direction(self, servo, pos):
        '''Function: Determines the direction of the servo
        Parameters: servo (Servo), pos (int) 
        Example Call: determine_direction(1, 500)
        Return: None
        '''

        if servo.get_servo_position() > pos:
            return -1
        else:
            return 1
