import time
import logging
import threading
import odrive

class OdriveJoint:
  def __init__(
    self,
    name,
    odr: odrive,
    trajectory_limits,
  ):
    self.name = name
    self.odr = odr
    self.axis = getattr(odr, "axis1")
    self.configure_trajectory_control(trajectory_limits[0], 
                                      trajectory_limits[1], 
                                      trajectory_limits[2], 
                                      trajectory_limits[3],
                                      trajectory_limits[4])
    self.is_homed = False
    self.desired_pos = 0
    # self.odr.controller.config.control_mode = VELOCITY_CONTROL
    self.thread = threading.Thread(target=self.thread_pos_controller, args=(1,), daemon=True)
    self.thread.start()
    
  def threaded_pos_controller(self): 
    while True:
      k = 1 # tune PID value
      self.axis.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
      self.axis.controller.input_vel = k * (self.desired_pos - self.axis.encoder.pos_estimate)
      time.sleep(0.01)

  def go_to_position(self, position):
    try: 
      self.desired_pos = position
      print(
          f"Joint {self.name}:\t command sent, position = {position}"
      )
    except Exception as e: 
      self.stop()
      print(f"Joint {self.name}: \t function: go_to_position | error: {e}")
      print(e)
    
  def configure_trajectory_control(
    self, bandwidth, vel_limit, accel_limit, decel_limit, inertia):
    self.logger.info("configuring axis")
    try:
      self.axis.controller.config.input_filter_bandwidth = bandwidth
      self.axis.controller.config.input_mode = InputMode.TRAP_TRAJ
      self.odr.controller.config.control_mode = ControlMode.POSITION_CONTROL
      self.axis.trap_traj.config.vel_limit = vel_limit
      self.axis.trap_traj.config.accel_limit = accel_limit
      self.axis.trap_traj.config.decel_limit = decel_limit
      self.axis.controller.config.inertia = inertia
    except Exception as e: 
      self.stop()
      self.logger.warn(f"Joint {self.name}: \t function: configure_trajectory_control | error: {e}")
      self.logger.warn(e)
  
  def home_joint(self):
    start_time = time.monotonic()
    timeout = 10
    rate = 10
    print(f"Homing joint {self.name}")
    try:
      # configure variables for min (pin 1 on the odrive)
      self.odr.config.gpio1_mode = GPIO_MODE_DIGITAL
      self.axis.min_endstop.config.gpio_num = 1
      self.axis.min_endstop.config.is_active_high = False
      self.axis.min_endstop.config.offset = 0
      self.axis.min_endstop.config.enabled = True
      self.odr.config.gpio1_mode = GPIO_MODE_DIGITAL_PULL_UP
      #configure variables for max (pin 2 on the odrive)
      self.odr.config.gpio2_mode = GPIO_MODE_DIGITAL
      self.axis.max_endstop.config.gpio_num = 2
      self.axis.max_endstop.config.is_active_high = False
      self.axis.max_endstop.config.offset = 1
      self.axis.max_endstop.config.enabled = True
      self.odr.config.gpio2_mode = GPIO_MODE_DIGITAL_PULL_UP
      self.odr.save_configuration()
      while self.axis.current_state != HOMING:
        self.axis.requested_state = HOMING
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
    self.axis.requested_state = AXIS_STATE_IDLE
    self.odr.controller.config.control_mode = VELOCITY_CONTROL
    self.axis.controller.input_vel = 0
    print(f"Joint {self.name}:\t command sent, stop")
    
def main():
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
    elif x == 0:
      break

if __name__ == "__main__":
    main()