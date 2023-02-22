import time
import odrive
from rclpy.impl.rcutils_logger import RcutilsLogger

class OdriveJoint:
  def __init__(
    self,
    name,
    odr: odrive,
    logger: RcutilsLogger,
    trajectory_limits
  ):
    self.name = name
    self.odr = odr
    self.logger = logger 
    self.axis = getattr(odr, "axis1")
    self.configure_trajectory_control(trajectory_limits[0], 
                                      trajectory_limits[1], 
                                      trajectory_limits[2], 
                                      trajectory_limits[3],
                                      trajectory_limits[4])
    self.is_homed = False
  
  def go_to_position(self, position):
    try: 
      self.axis.controller.input_pos = position
      self.logger.info(
          f"Joint {self.name}:\t command sent, position = {position}"
      )
    except Exception as e: 
      self.stop()
      self.logger.warn(f"Joint {self.name}: \t function: go_to_position | error: {e}")
      self.logger.debug(e)
    
  def configure_trajectory_control(self, bandwidth, vel_limit, accel_limit, decel_limit, inertia):
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
      self.logger.debug(e)
  
  def home_joint(self):
    start_time = time.monotonic()
    timeout = 10
    rate = 10
    self.logger.info(f"Homing joint {self.name}")
    try:
      self.axis.controller.config.homing_speed = 0.25
      while self.axis.current_state != HOMING:
        self.axis.requested_state = HOMING
        if start_time - time.monotonic() > timeout:
          self.logger.warn(
            f"Homing joint {self.name} failed, took longer than {timeout}s"
          )
          return self.is_homed
        time.sleep(1 / rate)      
    except Exception as e:
      self.stop()
      self.logger.warn(f"Joint {self.name}: \t function: home_joint | error: {e}")
    self.is_homed = self.axis.is_homed
    return self.is_homed
    
  def stop(self): 
    self.axis.requested_state = AXIS_STATE_IDLE
    self.axis.controller.input_vel = 0
    self.logger.info(f"Joint {self.name}:\t command sent, stop")