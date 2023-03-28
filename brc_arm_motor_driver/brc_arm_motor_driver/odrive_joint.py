import time
from odrive.enums import *
import odrive
from rclpy.impl.rcutils_logger import RcutilsLogger

class OdriveJoint:
  def __init__(
    self,
    name,
    odr: odrive,
    trajectory_limits,
    logger
  ):
    self.name = name
    self.odr = odr
    self.axis = getattr(odr, "axis1")
    self.is_homed = False
    self.axis.controller.config.control_mode = CONTROL_MODE_POSITION_CONTROL
    self.axis.controller.config.input_mode = INPUT_MODE_TRAP_TRAJ
    self.axis.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
    self.configure_trajectory_control(trajectory_limits[0], 
                                      trajectory_limits[1], 
                                      trajectory_limits[2], 
                                      trajectory_limits[3],
                                      trajectory_limits[4])
    
  def set_to_closed_loop(self):
    self.odr.clear_errors()
    self.axis.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL

  def go_to_position(self, position):
    if self.axis.current_state == AXIS_STATE_IDLE: 
      self.logger.info('An error was thrown that set the axis state to IDLE')
      return
    try: 
      self.axis.controller.config.control_mode = CONTROL_MODE_POSITION_CONTROL
      self.axis.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
      self.axis.controller.config.input_mode = INPUT_MODE_TRAP_TRAJ
      self.axis.controller.input_pos = position
      self.logger.info(
          f"Joint {self.name}:\t command sent, position = {position}"
      )
    except Exception as e: 
      self.stop()
      self.logger.warn(f"Joint {self.name}: \t function: go_to_position | error: {e}")
      self.logger.debug(e)

  def configure_trajectory_control(self, bandwidth, vel_limit, accel_limit, decel_limit, inertia): # bandwidth = 10 | vel_limit = 10 | accel_limit = 5 | decel_limit = 5 | inertia = 0
    self.axis.controller.config.input_filter_bandwidth = bandwidth
    self.axis.controller.config.input_mode = INPUT_MODE_TRAP_TRAJ
    self.axis.controller.config.control_mode = CONTROL_MODE_POSITION_CONTROL
    self.axis.trap_traj.config.vel_limit = vel_limit
    self.axis.trap_traj.config.accel_limit = accel_limit
    self.axis.trap_traj.config.decel_limit = decel_limit
    self.axis.controller.config.inertia = inertia
  
  # odrv0.axis1.min_endstop.config.gpio_num: 1
  # odrv0.axis1.max_endstop.config.gpio_num: 2
  def home_joint(self):
    self.logger.info(f"Homing joint {self.name}")
    try:
      self.axis.requested_state = AXIS_STATE_HOMING
      while not self.axis.current_state == AXIS_STATE_IDLE:
        time.sleep(0.1)
    except Exception as e:
      self.stop()
      self.logger.warn(f"Joint {self.name}: \t function: home_joint | error: {e}")
      self.logger.debug(e)
    self.is_homed = self.axis.is_homed
    self.logger.info("Homing complete\n")
    self.odr.clear_errors()
    self.axis.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
    return self.is_homed
  
  def read_enc(self):
    return self.axis.encoder.pos_estimate
  
  def set_enc(self): 
    return True
    
  def stop(self): 
    self.axis.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL
    self.axis.controller.config.input_mode = INPUT_MODE_VEL_RAMP
    self.axis.requested_state = AXIS_STATE_IDLE
    self.axis.controller.input_vel = 0
    self.logger.info(f"Joint {self.name}:\t command sent, stop")