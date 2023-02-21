import time
import odrive
from rclpy.impl.rcutils_logger import RcutilsLogger

class OdriveJoint:
  def __init__(
    self,
    name,
    odr: Odrive,
    logger: RcutilsLogger,
    trajectory_limits
  ):
    self.name = name
    self.odr = odr
    self.logger = logger 
    self.axis = getattr(odr, "axis1")
    # todo: manually find trajectory limits and set them as 
    # constants in brc_arm_motor_driver
    self.configure_trajectory_control(trajectory_limits[0], 
                                      trajectory_limits[1], 
                                      trajectory_limits[2], 
                                      trajectory_limits[3],
                                      trajectory_limits[4])
  
  def go_to_position(self, position):
    self.axis.controller.input_pos = position
    self.logger.info(
        f"Joint {self.name}:\t command sent, position = {position}"
    )
    
  def configure_trajectory_control(self, bandwidth, vel_limit, accel_limit, decel_limit, inertia):
    self.axis.controller.config.input_filter_bandwidth = bandwidth
    self.axis.controller.config.input_mode = InputMode.TRAP_TRAJ
    self.odr.controller.config.control_mode = ControlMode.POSITION_CONTROL
    self.axis.trap_traj.config.vel_limit = vel_limit
    self.axis.trap_traj.config.accel_limit = accel_limit
    self.axis.trap_traj.config.decel_limit = decel_limit
    self.axis.controller.config.inertia = inertia
    
  def stop(self): 
    self.axis.controller.input_vel = 0
    self.logger.info(f"Joint {self.name}:\t command sent, stop")