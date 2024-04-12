import kipr
import math
import time
from Create_constants import *
import struct

class Drive2:
  def __init__(self):
    self.xPose =   0.0
    self.yPose =   0.0
    self.thrPose = 0.0
    self.leftEncoder = -1
    self.rightEncoder = -1
    self.leftEncoder_old = -1
    self.rightEncoder_old = -1
    self.angle_target = 0

    time.sleep(0.25)
    self.setPose(0,0,0)

  def getPose(self, dist='cm', angle='deg'):
    """ getPose returns the current estimate of the
    robot's global pose
    dist may be 'cm' or 'mm'
    angle may be 'deg' or 'rad'
    """
    x = 0; y = 0; th = 0
    if dist == 'cm':
        x = self.xPose/10.0; y = self.yPose/10.0
    else:
        x = self.xPose; y = self.yPose
        
    if angle == 'deg':
        th = math.degrees(self.thrPose)
    else:
        th = self.thrPose
        
    return (x,y,th)
    
    
  def setPose(self, x, y, th, dist='cm', angle='deg'):
    """ setPose sets the internal odometry to the input values
    x: global x in mm
    y: global y in mm
    th: global th in radians
    dist: 'cm' or 'mm' for x and y
    angle: 'deg' or 'rad' for th
    """
    if dist == 'cm':
        self.xPose = x*10.0; self.yPose = y*10.0
    else:
        self.xPose = x; self.yPose = y
        
    if angle == 'deg':
        self.thrPose = math.radians(th)
    else:
        self.thrPose = th
    
    
  def resetPose(self):
    """ resetPose simply sets the internal odometry to 0,0,0
    """
    self.setPose(0.0,0.0,0.0)

  def _getEncoderDelta(self, oldEnc, newEnc):
    #encoder wrap around at 2^16
    #check if the step is bigger than half the 
    #possible range and treat this as wraparound
    delta = newEnc-oldEnc
    if delta < -65536/2:
        delta = (newEnc+65536)-oldEnc
        print("OVERFLOW NEGATIVE", oldEnc, newEnc, delta)
    if delta > 65536/2:
        delta = newEnc-(oldEnc+65536)
        print("OVERFLOW POSITIVE", oldEnc, newEnc, delta)
        
    return delta

  def _integrateNextEncoderStep(self):
    if self.leftEncoder_old == -1:
        self.leftEncoder_old = self.leftEncoder
        self.rightEncoder_old = self.rightEncoder
        return
    left_diff  = self._getEncoderDelta(self.leftEncoder_old,self.leftEncoder)
    right_diff = self._getEncoderDelta(self.rightEncoder_old,self.rightEncoder)
            
    while abs(left_diff) > 500:
        print("INVALID PACKET LEFT... PAUSING")
        print(left_diff, self.leftEncoder_old, self.leftEncoder)
        #self.leftEncoder_old = self.leftEncoder
        self.drive_speed(0, 0)
        time.sleep(0.015)
        self.leftEncoder = self.read_sensor_val(43, 2)
        left_diff  = self._getEncoderDelta(self.leftEncoder_old, self.leftEncoder)
        print(left_diff, self.leftEncoder_old, self.leftEncoder)
    
    while abs(right_diff) > 500:
        print("INVALID PACKET RIGHT... PAUSING")
        print(right_diff, self.rightEncoder_old, self.rightEncoder)
        #self.rightEncoder_old = self.rightEncoder
        self.drive_speed(0, 0)
        time.sleep(0.015)
        self.rightEncoder = self.read_sensor_val(44, 2)
        right_diff = self._getEncoderDelta(self.rightEncoder_old,self.rightEncoder)
        print(right_diff, self.rightEncoder_old, self.rightEncoder)

    left_mm = left_diff / TICK_PER_MM
    right_mm = right_diff / TICK_PER_MM

    distance = (left_mm + right_mm) / 2.0;        
    dAngle = (right_mm - left_mm) / WHEEL_SPAN
    # dAngle *= ANGULAR_ERROR     
        
    deltaX = math.cos(self.thrPose)
    deltaY = math.sin(self.thrPose)
    #print(dAngle, math.degrees(self.thrPose), deltaY, self.yPose)
            
    self.xPose += distance * deltaX
    self.yPose += distance * deltaY
    self.thrPose += dAngle

    self.leftEncoder_old = self.leftEncoder
    self.rightEncoder_old = self.rightEncoder

  def drive_speed(self, speed_left, speed_right):
    if (abs(speed_left) < 10 or abs(speed_right) < 10) and (speed_left != 0 or speed_right!=0):
        speed_left = 12.5 * speed_left/abs(speed_left)
        speed_right = 12.5 * speed_right/abs(speed_right)
    kipr.create_drive_direct(-int(speed_left), -int(speed_right))

  def calc_angle_speed_ramp(self, final_angle, current_angle):
    final_angle, current_angle = abs(final_angle), abs(current_angle)
    if final_angle < 20:
       accel_angle = min(5, final_angle / 2)
    else:
       accel_angle = 30
    
    #print(current_angle, accel_angle)

    if (current_angle < accel_angle):
      norm_x = current_angle / accel_angle
      tween = abs(math.sin(norm_x * math.pi / 2))
      if (tween < 0.2):
        tween = 0.2
    
    elif (current_angle > final_angle - accel_angle):
      norm_x = (final_angle - current_angle - accel_angle) / accel_angle
      tween = abs(math.cos(norm_x * math.pi / 2))
      #print("DECLERATING", final_angle, current_angle, accel_angle, norm_x, tween)
      if (tween < 0.2):
        tween = 0.05
    
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
    kipr.create_clear_serial_buffer()
    kipr.create_write_byte(chr(142))
    kipr.create_write_byte(chr(sensor_byte))
    val = b'00'
    kipr.create_read_block(val, size)
    val = struct.unpack('>h', val)[0]
    #print(val)
    # ADD 5 MS timer (page 21 of create documentation)
    return val
  
  def update_encoder_values(self):
    self.leftEncoder = self.read_sensor_val(43, 2)
    self.rightEncoder = self.read_sensor_val(44, 2)
    
    self._integrateNextEncoderStep()

  def drive_distance_y(self, distance, base_speed, debug=False):
    # kipr.set_create_distance(0)
    # dist = kipr.get_create_distance()

    if debug:
      kp_vals, ki_vals, kd_vals = [], [], []
    
    self.update_encoder_values()
    start_x, start_y, start_th = self.getPose("mm")
    #print("initial pose", start_x, self.xPose, start_y, start_th, self.leftEncoder, self.rightEncoder)
    dist_traveled = abs(self.xPose - start_x) / 10
    
    total_error = 0
    last_error = 0
    speed_update = 0

    start_time_pid = time.time()
    # start_time_dist = time.time()
    start_timeout = time.time()
    
    # Try replacing dist with encoder distances
    while (dist_traveled < distance):
      mult = self.calc_speed_ramp(distance, dist_traveled)
      error = self.yPose - start_y

      if time.time() - start_time_pid > 0.015:
         total_error += error
         error_difference = error - last_error
         speed_update = (KP_drive * error) + (KI_drive * total_error) + (KD_drive * error_difference)
         print "Encoder:", self.leftEncoder, self.rightEncoder, "| Error: ", error, "| Update:", speed_update, "| Pose: ", self.xPose, self.yPose, math.degrees(self.thrPose)
         print "KP", KP_drive * error, "KI", KI_drive * total_error, "KD", KD_drive * error_difference, "| Dist Traveled", dist_traveled
         if debug:
             kp_vals.append((KP_drive * error, time.time() - start_timeout))
             ki_vals.append((KI_drive * total_error, time.time() - start_timeout))
             kd_vals.append((KD_drive * error_difference, time.time() - start_timeout))
         start_time_pid = time.time()
         last_error = error
         self.update_encoder_values()

      # if time.time() - start_time_dist > 0.005:
      #   dist = kipr.get_create_distance()
      #   start_time_dist = time.time()

      dist_traveled = abs(self.xPose - start_x) / 10
                 
      if base_speed < 0:
          self.drive_speed(mult * (base_speed - speed_update), mult * (base_speed + speed_update))
      else:
          self.drive_speed(mult * (base_speed + speed_update), mult * (base_speed - speed_update))

      # 10 second timeout
      #if time.time() - start_timeout > 10:
      #   break
   
    self.lock_wheels()
    self.update_encoder_values()
    #print "Encoder:", self.leftEncoder, self.rightEncoder, "| Error: ", error, "| Update:", speed_update, "| Pose: ", self.xPose, self.yPose, math.degrees(self.thrPose), "| Dist Traveled", dist_traveled
    if debug:
      return kp_vals, ki_vals, kd_vals
    else:
      return self.xPose, self.yPose, self.thrPose
      
  def drive_distance(self, distance, base_speed, debug=False, smooth=True):
    # kipr.set_create_distance(0)
    # dist = kipr.get_create_distance()

    if debug:
      kp_vals, ki_vals, kd_vals = [], [], []
    
    self.update_encoder_values()
    start_x, start_y, start_th = self.getPose("mm")
    print("initial pose", start_x, self.xPose, start_y, start_th, self.leftEncoder, self.rightEncoder)
    dist_traveled = math.sqrt(abs(self.xPose - start_x)**2 + abs(self.yPose - start_y)**2) / 10
    
    total_error = 0
    last_error = 0
    speed_update = 0

    start_time_pid = time.time()
    # start_time_dist = time.time()
    start_timeout = time.time()
    
    # Try replacing dist with encoder distances
    while (dist_traveled < distance):
      if smooth:
          mult = self.calc_speed_ramp(distance, dist_traveled)
      else:
          mult = 1
      error = self.thrPose - math.radians(start_th)

      if time.time() - start_time_pid > 0.015:
         total_error += error
         error_difference = error - last_error
         speed_update = (KP_drive_angle * error) + (KI_drive_angle * total_error) + (KD_drive_angle * error_difference)
         #print "Encoder:", self.leftEncoder, self.rightEncoder, "| Error: ", error, "| Update:", speed_update, "| Pose: ", self.xPose, self.yPose, math.degrees(self.thrPose)
         #print "KP", KP_drive * error, "KI", KI_drive * total_error, "KD", KD_drive * error_difference, "| Dist Traveled", dist_traveled
         if debug:
             kp_vals.append((KP_drive * error, time.time() - start_timeout))
             ki_vals.append((KI_drive * total_error, time.time() - start_timeout))
             kd_vals.append((KD_drive * error_difference, time.time() - start_timeout))
         start_time_pid = time.time()
         last_error = error
         self.update_encoder_values()

      # if time.time() - start_time_dist > 0.005:
      #   dist = kipr.get_create_distance()
      #   start_time_dist = time.time()

      dist_traveled = math.sqrt(abs(self.xPose - start_x)**2 + abs(self.yPose - start_y)**2) / 10
                 
      if base_speed < 0:
          self.drive_speed(mult * (base_speed - speed_update), mult * (base_speed + speed_update))
      else:
          self.drive_speed(mult * (base_speed + speed_update), mult * (base_speed - speed_update))

      # 10 second timeout
      #if time.time() - start_timeout > 10:
      #   break
   
    self.lock_wheels()
    self.update_encoder_values()
    #print "Encoder:", self.leftEncoder, self.rightEncoder, "| Error: ", error, "| Update:", speed_update, "| Pose: ", self.xPose, self.yPose, math.degrees(self.thrPose), "| Dist Traveled", dist_traveled
    if debug:
      return kp_vals, ki_vals, kd_vals
    else:
      return self.xPose, self.yPose, self.thrPose
      
  def drive_distance_x(self, distance, base_speed, debug=False):
    # kipr.set_create_distance(0)
    # dist = kipr.get_create_distance()

    if debug:
      kp_vals, ki_vals, kd_vals = [], [], []
    
    self.update_encoder_values()
    start_x, start_y, start_th = self.getPose("mm")
    #print("initial pose", start_x, self.xPose, start_y, start_th, self.leftEncoder, self.rightEncoder)
    dist_traveled = abs(self.yPose - start_y) / 10
    
    total_error = 0
    last_error = 0
    speed_update = 0

    start_time_pid = time.time()
    # start_time_dist = time.time()
    start_timeout = time.time()
    
    # Try replacing dist with encoder distances
    while (dist_traveled < distance):
      mult = self.calc_speed_ramp(distance, dist_traveled)
      error = self.xPose - start_x

      if time.time() - start_time_pid > 0.015:
         total_error += error
         error_difference = error - last_error
         speed_update = (KP_drive * error) + (KI_drive * total_error) + (KD_drive * error_difference)
         print "Encoder:", self.leftEncoder, self.rightEncoder, "| Error: ", error, "| Update:", speed_update, "| Pose: ", self.xPose, self.yPose, math.degrees(self.thrPose)
         print "KP", KP_drive * error, "KI", KI_drive * total_error, "KD", KD_drive * error_difference, "| Dist Traveled", dist_traveled
         if debug:
             kp_vals.append((KP_drive * error, time.time() - start_timeout))
             ki_vals.append((KI_drive * total_error, time.time() - start_timeout))
             kd_vals.append((KD_drive * error_difference, time.time() - start_timeout))
         start_time_pid = time.time()
         last_error = error
         self.update_encoder_values()

      # if time.time() - start_time_dist > 0.005:
      #   dist = kipr.get_create_distance()
      #   start_time_dist = time.time()

      dist_traveled = abs(self.yPose - start_y) / 10
                 
      if base_speed < 0:
          self.drive_speed(mult * (base_speed - speed_update), mult * (base_speed + speed_update))
      else:
          self.drive_speed(mult * (base_speed + speed_update), mult * (base_speed - speed_update))

      # 10 second timeout
      #if time.time() - start_timeout > 10:
      #   break
   
    self.lock_wheels()
    self.update_encoder_values()
    #print "Encoder:", self.leftEncoder, self.rightEncoder, "| Error: ", error, "| Update:", speed_update, "| Pose: ", self.xPose, self.yPose, math.degrees(self.thrPose), "| Dist Traveled", dist_traveled
    if debug:
      return kp_vals, ki_vals, kd_vals
    else:
      return self.xPose, self.yPose, self.thrPose
      
  def turn(self, degrees, speed, debug=False, smooth=True):

    if debug:
        kp_vals, ki_vals, kd_vals = [], [], []
    
    self.angle_target += degrees

    left_motor_dir = -1 if degrees >= 0 else 1
    right_motor_dir = -1 if degrees < 0 else 1

    self.update_encoder_values()
    start_x, start_y, start_th = self.getPose() # degrees
    print("initial pose", start_x, start_y, start_th)
    angle_traveled = math.degrees(self.thrPose) - start_th
    print(angle_traveled)
    start_time = time.time()
    timeout = time.time()

    while abs(math.degrees(self.thrPose) - self.angle_target) > 0.5:
       if smooth:
          mult = self.calc_angle_speed_ramp(degrees, angle_traveled)
       else:
          mult = 1
       error = math.degrees(self.thrPose) - self.angle_target
           
       if time.time() - start_time > 0.015:
          self.update_encoder_values()
          angle_traveled = math.degrees(self.thrPose) - start_th
          print(mult, error, angle_traveled, self.leftEncoder, self.rightEncoder, -left_motor_dir * mult * speed * error/abs(error) * -degrees/abs(degrees))
          start_time = time.time()
       
       if time.time() - timeout > 5:
           break
       
       self.drive_speed(-left_motor_dir * mult * speed * error/abs(error) * -degrees/abs(degrees), -right_motor_dir * mult * speed * error/abs(error) * -degrees/abs(degrees))
       
    self.lock_wheels()

  def turn2(self, degrees, speed, debug=False):

    if debug:
        kp_vals, ki_vals, kd_vals = [], [], []

    left_motor_dir = -1 if degrees >= 0 else 1
    right_motor_dir = -1 if degrees < 0 else 1

    self.update_encoder_values()
    start_x, start_y, start_th = self.getPose() # degrees
    #print("initial pose", start_x, start_y, start_th)
    angle_traveled = math.degrees(self.thrPose) - start_th
    print(angle_traveled)
    start_time = time.time()
    timeout = time.time()
        
    while abs(angle_traveled - degrees) > 0.5:
       mult = self.calc_angle_speed_ramp(degrees, angle_traveled)
       error = angle_traveled - degrees
           
       if time.time() - start_time > 0.015:
          self.update_encoder_values()
          angle_traveled = math.degrees(self.thrPose) - start_th
          #print(mult, error, angle_traveled, self.leftEncoder, self.rightEncoder)
          start_time = time.time()
       
       if time.time() - timeout > 5:
           break
       
       self.drive_speed(-left_motor_dir * mult * speed * error/abs(error) * -degrees/abs(degrees), -right_motor_dir * mult * speed * error/abs(error) * -degrees/abs(degrees))
       
    self.lock_wheels()
               
  def align_with_target(self):     
      #target_heading = self.determine_norm_target()
      amount_to_turn = self.angle_target - math.degrees(self.thrPose) #target_heading - math.degrees(self.thrPose)
      print("Ideal angle", self.angle_target, "Current Angle", math.degrees(self.thrPose))
      self.turn(amount_to_turn, 12.5, debug=True, correction=True)

  def determine_norm_target():
      possible_vals = [x for x in range(-360, 360, 90)][1:]
      error = 99999
      ideal_val = 360
      for val in possible_vals:
        temp = abs(math.fmod(math.degrees(self.thrPose), 360) - val)
        if temp < error:
          error = temp
          ideal_val = val
      return ideal_val



    
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
  def drive_distance2(self, distance, speed, reduce_endspeed=False):
    distance_in_mm = distance * 10
    time_to_wait = self.calc_time(speed, distance_in_mm)
    self.drive_time(speed, speed, time_to_wait, reduce_endspeed)
    left_enc, right_enc = self.update_encoder_values()
    return left_enc, right_enc

  # Turn specified degrees
  def turn3(self, degrees, speed):
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