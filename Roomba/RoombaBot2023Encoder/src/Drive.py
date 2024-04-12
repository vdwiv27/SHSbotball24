import kipr
import math
import time
from Create_constants import *

class Drive:
  def __init__(self):
    pass

  def drive_speed(self, speed_left, speed_right):
    kipr.create_drive_direct(-int(speed_left), -int(speed_right))

  def drive_time(self,speed_left, speed_right, time_to_wait, reduce_endspeed=False):
    start = time.time()
    while time.time() - start < time_to_wait:
        if reduce_endspeed and time.time() - start > 0.8 * time_to_wait:

            self.drive_speed(int(0.5 * speed_left), int(0.5 * speed_right))
        else:
            self.drive_speed(speed_left, speed_right)
    #self.drive_speed(speed_left, speed_right)
    #time.sleep(time_to_wait)
    #kipr.msleep(time_to_wait)
    self.lock_wheels()
    
  # Drive straight with specified speed for certain distance
  def drive_distance(self, distance, speed, reduce_endspeed=False):
    distance_in_mm = distance * 10
    time_to_wait = self.calc_time(speed, distance_in_mm)
    self.drive_time(speed, speed, time_to_wait, reduce_endspeed)

  # Turn specified degrees
  def turn(self, degrees, speed):
    left_motor_dir = -1 if degrees >= 0 else 1
    right_motor_dir = -1 if degrees < 0 else 1
    # Constant for speed 100
    distance = abs((float(degrees) / 360) * (195 * math.pi))
    time_to_wait = self.calc_time(speed, distance)
    
    # Speed 150
    if speed ==  100:
        time_to_wait *= 1.07
    # Speed 150
    if speed ==  150:
        time_to_wait *= 1.26
    # Speed 200
    if speed ==  200:
        time_to_wait *= 1.26
    # Speed 250
    if speed ==  250:
        time_to_wait *= 1.28
    # Speed 300
    if speed ==  300:
        time_to_wait *= 1.24
    # Speed 350
    if speed ==  350:
        time_to_wait *= 1.40
            
    self.drive_time(-left_motor_dir * speed, -right_motor_dir * speed, time_to_wait)

  def lock_wheels(self):
    self.drive_speed(0, 0)
    kipr.msleep(50)

  def calc_angle_speed_ramp(self, final_angle, current_angle):
    if final_angle < 20:
       accel_angle = min(5, final_angle / 2)
    else:
       accel_angle = 10
    
    #print(current_angle, accel_angle)

    if (current_angle < accel_angle):
      norm_x = current_angle / accel_angle
      tween = math.sin(norm_x * math.pi / 2)
      if (tween < 0.2):
        tween = 0.2
    
    elif (current_angle > final_angle - accel_angle):
      norm_x = (final_angle - current_angle - accel_angle) / accel_angle
      tween = math.cos(norm_x * math.pi / 2)
      if (tween < 0.2):
        tween = 0.2
    
    else:
       tween = 1
    
    return tween
  
  def turn_ramp(self, degrees, speed):
    left_motor_dir = -1 if degrees >= 0 else 1
    right_motor_dir = -1 if degrees < 0 else 1

    kipr.set_create_total_angle(0)
    angle = kipr.get_create_total_angle()
    print(angle)
    while (abs(angle) < abs(degrees)):
      mult = self.calc_angle_speed_ramp(abs(degrees), angle)
      print(type(mult))
      self.drive_speed(-left_motor_dir * mult * speed, -right_motor_dir * mult * speed)
      angle = kipr.get_create_total_angle()
      time.sleep(0.01)
      print(angle, left_motor_dir * mult * speed)
    self.lock_wheels()

  def calc_speed_ramp(self, final_dist, current_dist):
    if final_dist < 20:
       accel_distance = min(5, final_dist / 2)
    else:
       accel_distance = 10

    if (current_dist < accel_distance):
      norm_x = current_dist / accel_distance
      tween = math.sin(norm_x * math.pi / 2)
      if (tween < 0.2):
        tween = 0.2
    
    elif (current_dist > final_dist - accel_distance):
      norm_x = (final_dist - current_dist - accel_distance) / accel_distance
      tween = math.cos(norm_x * math.pi / 2)
      if (tween < 0.2):
        tween = 0.2
    
    else:
       tween = 1

    return tween

  def drive_distance_ramp(self, distance, speed):
    kipr.set_create_distance(0)
    dist = kipr.get_create_distance() / 10
    while (dist < distance):
      mult = self.calc_speed_ramp(distance, dist)
      self.drive_speed(mult * speed, mult * speed)
      dist = kipr.get_create_distance() / 10
      time.sleep(0.01)
    self.lock_wheels()

  def calc_time(self, speed, distance):
    #return int((float(distance) / abs(speed)) * 1000) # Need time to be an integer value in milliseconds
    return (float(distance) / abs(speed))
  
  def read_sensor_val(self, sensor_byte, size):
    LP_c_char = ctypes.create_string_buffer(0)
    print(type(LP_c_char))
    kipr.create_write_byte(chr(142))
    kipr.create_write_byte(chr(sensor_byte))
    val = LP_c_char
    print(kipr.create_read_block(LP_c_char, 0))
    print(type(val), val)
    return val
  
  def update_encoder_values(self):
    left_enc = self.read_sensor_val(43, 2)
    right_enc = self.read_sensor_val(44, 2)

    left_enc = left_enc * (math.pi * 72.0 / 508.8) / 10 # cm
    right_enc = right_enc * (math.pi * 72.0 / 508.8) / 10 # cm
    
    return left_enc, right_enc

  def drive_straight(self, base_speed, distance):
    # kipr.set_create_distance(0)
    # dist = kipr.get_create_distance()

    left_enc, right_enc = self.update_encoder_values()
    left_enc_start, right_enc_start = left_enc, right_enc
    dist_traveled = abs(right_enc - right_enc_start)

    total_error = 0
    last_error = 0

    start_time_pid = time.time()
    # start_time_dist = time.time()
    start_timeout = time.time()
    
    # Try replacing dist with encoder distances
    while (dist_traveled < distance):
      mult = self.calc_speed_ramp(distance, dist_traveled)
      error = right_enc - left_enc

      if time.time() - start_time_pid > 0.1:
         total_error += error
         error_difference = error - last_error
         speed_update = (KP_drive * error) + (KI_drive * total_error) + (KD_drive * error_difference)
         
         start_time_pid = time.time()
         last_error = error
      else:
         speed_update = error * KP_drive
      
      # if time.time() - start_time_dist > 0.005:
      #   dist = kipr.get_create_distance()
      #   start_time_dist = time.time()

      left_enc, right_enc = self.update_encoder_values()
      dist_traveled = abs(right_enc - right_enc_start)

      self.drive_speed(mult * (base_speed + speed_update), mult * (base_speed - speed_update))

      # 10 second timeout
      if time.time() - start_timeout > 10:
         break
      
    self.lock_wheels()