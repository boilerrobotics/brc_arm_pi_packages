import time
from odrive.enums import *
import odrive
from odrive.utils import * 

from rclpy.impl.rcutils_logger import RcutilsLogger

class OdriveJoint:
    def __init__(self, name, odr: odrive, trajectory_limits, logger):
        self.name = name
        self.odr = odr
        self.axis = getattr(odr, "axis0")
        self.logger = logger
        self.is_homed = False
        # self.axis.controller.config.control_mode = CONTROL_MODE_POSITION_CONTROL
        # self.axis.controller.config.input_mode = INPUT_MODE_PASSTHROUGH  # TRAP_TRAJ
        self.set_to_closed_loop()
        self.configure_trajectory_control(trajectory_limits[0],
                                          trajectory_limits[1],
                                          trajectory_limits[2],
                                          trajectory_limits[3],
                                          trajectory_limits[4])

    def set_to_closed_loop(self):
        self.odr.clear_errors()
        self.axis.requested_state = AxisState.CLOSED_LOOP_CONTROL

    def get_axis_state(self):
        print(f"IDLE: {AxisState.IDLE}")
        print(f"CLOSED_LOOP: {AxisState.CLOSED_LOOP_CONTROL}")
        print(f"TRAP_TRAJ: {InputMode.TRAP_TRAJ}")
        print(f"axis state: {self.axis.current_state}")        

    def go_to_position(self, position):
        # while count <= 5:
        #     odr = odrive.find_any()
        #     if odr != None:
        #         break
        #     time.sleep(0.5)
        #     count += 1
        # print(f"odr.error = {self.odr.error}")
        # print(f"current: {self.odr.ibus} voltage: {self.odr.vbus_voltage}")
        # print(
        #     f"pos setpoint: {self.axis.controller.pos_setpoint} vel setpoint: {self.aaxis_numxis.controller.vel_setpoint}"
        # )
        # print(f"encoder est: {self.axis.encoder.pos_estimate}")
        if self.axis.current_state == AxisState.IDLE:
            print("An error was thrown that set the axis state to IDLE")
            return
        try:
            if position != round(self.axis.controller.pos_setpoint):
                # self.axis.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
                # self.axis.controller.config.control_mode = CONTROL_MODE_POSITION_CONTROL
                # self.axis.controller.config.input_mode = INPUT_MODE_TRAP_TRAJ
                self.axis.controller.input_pos = position
            print(f"Joint {self.name}:\t command sent, position = {position}")
        except Exception as e:
            self.stop()
            print(f"Joint {self.name}: \t function: go_to_position | error: {e}")
            print(e)

    def configure_trajectory_control(
        self, bandwidth, vel_limit, accel_limit, decel_limit, inertia
    ):  # bandwidth = 10 | vel_limit = 10 | accel_limit = 5 | decel_limit = 5 | inertia = 0
        self.axis.controller.config.control_mode = ControlMode.POSITION_CONTROL
        self.axis.controller.config.input_mode = InputMode.TRAP_TRAJ
        self.axis.controller.config.input_filter_bandwidth = bandwidth
        self.axis.trap_traj.config.vel_limit = vel_limit
        self.axis.trap_traj.config.accel_limit = accel_limit
        self.axis.trap_traj.config.decel_limit = decel_limit
        self.axis.controller.config.inertia = inertia

    # odrv0.axis0.min_endstop.config.gpio_num: 1
    # odrv0.axis0.max_endstop.config.gpio_num: 2
    # FYI: can home on startup if we need (look documentation https://docs.odriverobotics.com/v/0.5.4/endstops.html)
    def home_joint(self):
        print(f"Homing joint {self.name}")
        try:
            # the commented commands below should already be configured on the odrive  
            
            # self.axis0.min_endstop.config.enabled = True
            # self.axis0.max_endstop.config.enabled = True
            # self.axis0.min_endstop.config.offset = 0 # TENTATIVE -> figure out how many rotations away from the lim switch we want 0 to be 
            # self.axis0.controller.config.vel_ramp_rate = 0.5 # TENTATIVE -> figure out what this should be 
            
            
            self.axis.requested_state = AxisState.HOMING # starts the homing process
            # while not self.axis.current_state == AXIS_STATE_IDLE:
            #     time.sleep(0.1)
        except Exception as e:
            self.stop()
            print(f"Joint {self.name}: \t function: home_joint | error: {e}")
        self.is_homed = self.axis.is_homed
        print("Homing complete\n")
        self.odr.clear_errors()
        self.axis.requested_state = AxisState.CLOSED_LOOP_CONTROL
        return self.is_homed

    def read_enc(self):
        print(self.axis.encoder.pos_estimate)
        return self.axis.encoder.pos_estimate

    def set_enc(self):
        return True

    def stop(self):
        self.axis.controller.config.control_mode = ControlMode.VELOCITY_CONTROL
        self.axis.controller.config.input_mode = InputMode.VEL_RAMP
        self.axis.requested_state = AxisState.IDLE
        self.axis.controller.input_vel = 0
        print(f"Joint {self.name}:\t command sent, stop")


def main():
    print("Starting...")
    try:
        odr = odrive.find_any()
        odr_joint = OdriveJoint("odrv_arm", odr, [10, 10, 5, 5, 0], RcutilsLogger(name='Gantry'))
        while True:
            x = int(
                input(
                    "\n------------------------------------------------------------------------------------------------" +
                    "Enter a function:\n (0) quit\n (1) go_to_position(position)\n (2) home_joint()\n"+
                    " (3) stop()\n (4) current position\n (5) set state to closed_loop_control\n (6) current axis state\n" + 
                    " (7) Dump and Clear Errors\n"
                )
            )
            if x == 1:
                y = float(input("Enter a position\n"))
                odr_joint.go_to_position(y)
            elif x == 2:
                odr_joint.home_joint()
            elif x == 3:
                odr_joint.stop()
            elif x == 4:
                print(f"Current Position: {odr_joint.axis.encoder.pos_estimate}")
            elif x == 5:
                odr_joint.set_to_closed_loop()
            elif x == 6:
                odr_joint.get_axis_state()
            elif x == 7:
                dump_errors(odr)
                odr.clear_errors()
            elif x == 0:
                break
        print("Goodbye!")
    except KeyboardInterrupt as e:
        odr_joint.stop()
        print(f"Main had a keyboard interupt: {e}")
    except Exception as e:
        print(f"Main threw an exception: {e}")


if __name__ == "__main__":
    main()
