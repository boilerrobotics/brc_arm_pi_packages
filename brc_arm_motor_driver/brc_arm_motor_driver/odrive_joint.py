import time
from odrive.enums import *
import odrive

class OdriveJoint:
  def __init__(
    self,
    name,
    odr: odrive,
    trajectory_limits
  ):
    self.name = name
    self.odr = odr
    self.axis = getattr(odr, "axis1")
    self.is_homed = False
    self.axis.controller.config.control_mode = CONTROL_MODE_POSITION_CONTROL
    self.axis.controller.config.input_mode = INPUT_MODE_TRAP_TRAJ
    self.configure_trajectory_control(trajectory_limits[0], 
                                      trajectory_limits[1], 
                                      trajectory_limits[2], 
                                      trajectory_limits[3],
                                      trajectory_limits[4])

  def go_to_position(self, position):
    try: 
      self.axis.controller.config.control_mode = CONTROL_MODE_POSITION_CONTROL
      self.axis.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
      self.axis.controller.config.input_mode = INPUT_MODE_TRAP_TRAJ
      self.axis.controller.input_pos = position
      print(
          f"Joint {self.name}:\t command sent, position = {position}"
      )
    except Exception as e: 
      self.stop()
      print(f"Joint {self.name}: \t function: go_to_position | error: {e}")
      print(e)

  def configure_trajectory_control(self, bandwidth, vel_limit, accel_limit, decel_limit, inertia):
    self.axis.controller.config.input_filter_bandwidth = bandwidth
    self.axis.controller.config.input_mode = INPUT_MODE_TRAP_TRAJ
    self.axis.controller.config.control_mode = CONTROL_MODE_POSITION_CONTROL
    self.axis.trap_traj.config.vel_limit = vel_limit
    self.axis.trap_traj.config.accel_limit = accel_limit
    self.axis.trap_traj.config.decel_limit = decel_limit
    self.axis.controller.config.inertia = inertia
  
  def home_joint(self):
    start_time = time.monotonic()
    timeout = 10
    rate = 10
    print(f"Homing joint {self.name}")
    try:
      while True:
        self.odr = odrive.find_any()
        if self.odr != None:
          break
        time.sleep(0.5)
      self.axis = getattr(self.odr, "axis1")
      while self.axis.current_state != AXIS_STATE_HOMING:
        self.axis.requested_state = AXIS_STATE_HOMING
        if start_time - time.monotonic() > timeout:
          print(
            f"Homing joint {self.name} failed, took longer than {timeout}s"
          )
          return self.is_homed
        time.sleep(1 / rate)      
    except Exception as e:
      self.stop()
      print(f"Joint {self.name}: \t function: home_joint | error: {e}")
    self.is_homed = self.axis.is_homed
    return self.is_homed
  
  def set_enc(self): 
    return True
    
  def stop(self): 
    self.axis.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL
    self.axis.controller.config.input_mode = INPUT_MODE_VEL_RAMP
    self.axis.requested_state = AXIS_STATE_IDLE
    self.axis.controller.input_vel = 0
    print(f"Joint {self.name}:\t command sent, stop")
    
def main():
  print('Starting...')
  try:
    while True:
      odr = odrive.find_any()
      if odr != None:
        break
      time.sleep(0.5) 
    odr_joint = OdriveJoint("odrv_arm", odr, [1, 1, 1, 1, 1])
    while True:
      x = int(input("\nEnter a function:\n (0) quit\n (1) go_to_position(position)\n (2) home_joint()\n (3) stop()\n"))
      if x == 1:
        y = float(input("Enter a position\n"))
        odr_joint.go_to_position(y)
      elif x == 2:
        odr_joint.home_joint()
      elif x == 3: 
        odr_joint.stop()
      elif x == 4:
        print(f'Current Position: {odr_joint.axis.encoder.pos_estimate}')
      elif x == 0:
        break
    print('Goodbye!')
  except KeyboardInterrupt as e: 
    odr_joint.stop()
    print(f"Main had a keyboard interupt: {e}") 
  except Exception as e: 
    print(f"Main threw an exception: {e}")
    

if __name__ == "__main__":
    main()