import kipr
import time
from Create_constants import *
from Drive import *

class CameraHandler:
    def __init__(self, drive):
      kipr.camera_open()
      self.drive = drive
    
    def warm_up(self):
      start = time.time()
      while time.time() - start < 3:
        kipr.camera_update()
            
    def find_object(self, channel, obj=0, area_threshold=None, direction=1, timer=True):
      start = time.time()
      kipr.camera_update()
      angle = -3 * direction
      total_angle = 0
      # Increased confidence threshold from 0.5
      if area_threshold != None:
          while kipr.get_object_confidence(channel, obj) < 0.25 or kipr.get_object_area(channel, obj) < area_threshold:
              #print("Area:", kipr.get_object_area(channel, obj))
              self.drive.turn(angle, 50)
              if abs(total_angle) >= 30:
                  angle *= -1
              total_angle += angle
              kipr.camera_update()
              # 10 Second timeout
              if timer and time.time() - start > 3:
                  print("stopped early")
                  return -1
      else: 
          while kipr.get_object_confidence(channel, obj) < 0.25:
              self.drive.turn(angle, 50)
              if abs(total_angle) >= 30:
                  angle *= -1
              total_angle += angle
              kipr.camera_update()
              # 10 Second timeout
              if timer and time.time() - start > 3:
                  print("stopped finding early")
                  return -1
      print("Area:", kipr.get_object_area(channel, obj))
      print("found object")
      return 0
                      
    def align_with_object_x(self, channel, obj, center, max_deviation=5, turn_speed=1, timer=True):
      start = time.time()
      obj_x = kipr.get_object_center_x(channel, obj)
      while abs(obj_x - center) > max_deviation:
        speed = 25
        if obj_x < center:
          self.drive.drive_speed(speed, -speed)
        else:
          self.drive.drive_speed(-speed, speed)
        
        # self.drive.turn(-(obj_x - center) * 0.15, 15)
	
        kipr.camera_update()
        obj_x = kipr.get_object_center_x(channel, obj)
        #print("Obj_x", obj_x)
        #print("Obj_y", kipr.get_object_center_y(channel, obj))
        # 10 Second timeout
        if timer and time.time() - start > 5:
            print("stopped aligning early")
            return -1

      print("Final Obj_x", obj_x)
      print("aligned")

    def drive_towards_object(self, channel, obj, dest_x, dest_y, area_threshold=None, max_deviation=5, turn_speed=1, direction=1, timer=True):
      found_object = self.find_object(channel, obj, area_threshold, direction=direction, timer=timer)
      if found_object == -1:
          return found_object
      aligned = self.align_with_object_x(channel, obj, dest_x, max_deviation, turn_speed, timer=timer)
      if aligned == -1:
          return aligned
      kipr.camera_update()
      obj_y =  kipr.get_object_center_y(channel, obj)
      prev_update = obj_y
      start = time.time()
      while abs(obj_y - dest_y) > max_deviation:
        #print("Obj_y1 {}".format(obj_y))
        #print("Deviation:", abs(obj_y - dest_y))
        if obj_y < dest_y:
          self.drive.drive_speed(250, 250)
        else:
          self.drive.drive_speed(-250, -250)
        kipr.camera_update()
        obj_y =  kipr.get_object_center_y(channel, obj)
        # print(abs(kipr.get_object_center_x(channel, obj) - dest_x) > max_deviation, obj_y - prev_update, obj_y < dest_y - 20)
        if abs(kipr.get_object_center_x(channel, obj) - dest_x) > max_deviation and obj_y - prev_update > 15 and obj_y < dest_y - 20:
          #center_x += 5
          #print("x update")
          self.align_with_object_x(channel, obj, dest_x, max_deviation, turn_speed)
          #print("Obj_y", obj_y)
          prev_update = obj_y
        # 10 Second timeout
        if timer and time.time() - start > 5:
            #print("stopped driving forward early")
            return -1
        kipr.camera_update()
        #print("Obj_y {}".format(obj_y))
      self.drive.lock_wheels()
      kipr.camera_update()
      obj_y =  kipr.get_object_center_y(channel, obj)
      print("Final Obj_y {}".format(obj_y))
      return obj_y
      
    def drive_towards_object_pom(self, channel, obj, dest_x, dest_y, area_threshold=None, max_deviation=5, turn_speed=1, direction=1, timer=True):
      found_object = self.find_object(channel, obj, area_threshold, direction=direction, timer=timer)
      if found_object == -1:
          return found_object
      aligned = self.align_with_object_x(channel, obj, dest_x, max_deviation, turn_speed, timer=timer)
      if aligned == -1:
          return aligned
      kipr.camera_update()
      obj_y =  kipr.get_object_center_y(channel, obj)
      prev_update = obj_y
      start = time.time()
      while abs(obj_y - dest_y) > max_deviation:
        #print("Obj_y1 {}".format(obj_y))
        #print("Deviation:", abs(obj_y - dest_y))
        deviation = abs(obj_y - dest_y)
        if deviation < 0.4 * dest_y and deviation > 0.25 * dest_y:
            #speed = 250 * max(deviation / float(dest_y), 0.4)
            #if obj_y < dest_y:
            #    self.drive.drive_speed(speed, speed)
            #else:
            #    self.drive.drive_speed(-speed, -speed)
            if obj_y < dest_y:
                self.drive.drive_distance(4, 150)
            else:
                self.drive.drive_distance(4, -150)
        elif deviation < 0.25 * dest_y:
            if obj_y < dest_y:
                self.drive.drive_distance(2, 150)
            else:
                self.drive.drive_distance(2, -150)
        elif obj_y < dest_y:
          self.drive.drive_speed(250, 250)
        else:
          self.drive.drive_speed(-250, -250)
        kipr.camera_update()
        obj_y =  kipr.get_object_center_y(channel, obj)
        # print(abs(kipr.get_object_center_x(channel, obj) - dest_x) > max_deviation, obj_y - prev_update, obj_y < dest_y - 20)
        if abs(kipr.get_object_center_x(channel, obj) - dest_x) > max_deviation and obj_y - prev_update > 15 and obj_y < dest_y - 20:
          #center_x += 5
          #print("x update")
          self.align_with_object_x(channel, obj, dest_x, max_deviation, turn_speed)
          #print("Obj_y", obj_y)
          prev_update = obj_y
        # 10 Second timeout
        if time.time() - start > 5:
            #print("stopped driving forward early")
            return -1
        kipr.camera_update()
        #print("Obj_y {}".format(obj_y))
      self.drive.lock_wheels()
      time.sleep(0.5)
      kipr.camera_update()
      obj_y =  kipr.get_object_center_y(channel, obj)
      print("Final Obj_y {}".format(obj_y))
      return obj_y

    def drive_towards_object_with_area(self, channel, obj, dest_x, dest_y, area_threshold=None, max_deviation=25, turn_speed=1, direction=1):
      self.find_object(channel, obj, area_threshold, direction=direction)
      self.align_with_object_x(channel, obj, dest_x, max_deviation, turn_speed)
      obj_y = kipr.get_object_area(channel, obj)
      prev_update = obj_y
      start = time.time()
      while abs(obj_y - dest_y) > max_deviation:
        if obj_y < dest_y:
          self.drive.drive_speed(250, 250)
        else:
          self.drive.drive_speed(-250, -250)
        kipr.camera_update()
        obj_y =  kipr.get_object_area(channel, obj)
        print(abs(kipr.get_object_center_x(channel, obj) - dest_x) > max_deviation, obj_y - prev_update, obj_y < dest_y - 20)
        if abs(kipr.get_object_center_x(channel, obj) - dest_x) > max_deviation and obj_y - prev_update > 0.25 * dest_y and obj_y < dest_y - 100:
          #center_x += 5
          print("x update")
          self.align_with_object_x(channel, obj, dest_x, max_deviation, turn_speed)
          #print("Obj_y", obj_y)
          prev_update = obj_y
        # 10 Second timeout
        if time.time() - start > 5:
            print("stopped driving forward early")
            break
        print("Final Obj_y {}".format(obj_y))
      self.drive.lock_wheels()

###################################### Code for multiple objects ########################################

    def find_objects(self, channel, num_objects):
      kipr.camera_update()
      angle = 5
      total_angle = 0
      while self.get_objects_confidence(channel, num_objects):
        self.drive.turn(angle, 50)
        if abs(total_angle) >= 30:
          angle *= -1
        total_angle += angle
        kipr.camera_update()
      print("found objects")

    def align_with_objects_x(self, channel, num_objects, center):
      obj_x = self.get_objects_x_center(channel, num_objects)
      while abs(obj_x - center) > 3:
        if obj_x < center:
          self.drive.turn(1, 15)
        else:
          self.drive.turn(-1, 15)

        kipr.camera_update()
        obj_x = self.get_objects_x_center(channel, num_objects)
        print("Obj_x", obj_x)
        print("Area:", kipr.get_object_area(channel, obj))

      print("Final Obj_x", obj_x)
      print("aligned")

    def drive_towards_objects(self, channel, num_objects, center_x, center_y):
      print("106")
      self.find_objects(channel, num_objects)
      self.align_with_objects_x(channel, num_objects, center_x)

      obj_y =  self.get_objects_y_center(channel, num_objects)
      prev_update = obj_y
      print("112")
      start = time.time()
      while abs(obj_y - center_y) > 5:
        print("Area:", kipr.get_object_area(0, 0))
        if obj_y < center_y:
          self.drive.drive_speed(250, 250)
        else:
          self.drive.drive_speed(-250, -250)

        kipr.camera_update()
        obj_y = self.get_objects_y_center(channel, num_objects)

        if abs(self.get_objects_x_center(channel, num_objects) - center_x) > 5 and obj_y - prev_update > 15 and obj_y < center_y - 20:
          #center_x += 5
          print("x update")
          self.align_with_objects_x(channel, num_objects, center_x)
          #print("Obj_y", obj_y)
          prev_update = obj_y
        print("Obj_y {}".format(obj_y))
        
        # 10 Second timeout
        if time.time() - start > 10:
            break
                
      self.drive.lock_wheels()

    def get_objects_confidence(self, channel, num_objects):
      for obj in range(num_objects):
        if kipr.get_object_confidence(channel, obj) < 0.5:
          return True
      return False

    def get_objects_x_center(self, channel, num_objects):
      average = 0
      for obj in range(num_objects):
        average += kipr.get_object_center_x(channel, obj)
      average /= float(num_objects)
      return average

    def get_objects_y_center(self, channel, num_objects):
      average = 0
      for obj in range(num_objects):
        average += kipr.get_object_center_y(channel, obj)
      average /= float(num_objects)
      return average