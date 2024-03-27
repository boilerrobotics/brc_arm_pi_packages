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
        self.set_closed_loop_pos_control()
        # self.configure_trajectory_control(trajectory_limits[0],
        #                                   trajectory_limits[1],
        #                                   trajectory_limits[2],
        #                                   trajectory_limits[3],
        #                                   trajectory_limits[4])

    def set_closed_loop_pos_control(self):
        self.odr.clear_errors()
        self.axis.requested_state = AxisState.CLOSED_LOOP_CONTROL
        self.axis.controller.config.input_mode = InputMode.PASSTHROUGH
        self.axis.controller.config.control_mode = ControlMode.POSITION_CONTROL

    def get_states(self):
        print("{:<26}".format("\naxis state:") + AxisState(self.axis.current_state).name.ljust(15))  
        print("{:<25}".format("input mode:") + InputMode(self.axis.controller.config.input_mode).name.ljust(15))      
        print("{:<25}".format("control mode:") + ControlMode(self.axis.controller.config.control_mode).name.ljust(15))

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
                self.axis.controller.input_pos = position
                try:
                    print("\'ctrl + c\' to stop printing")
                    max_current = 0
                    while True:
                        curr = self.axis.motor.I_bus
                        print("{:<20}".format("axis.motor.I_bus:") + "{:<23}".format(str(curr)) + " | " +
                              "{:<25}".format("axis.encoder.vel_estimate:") + "{:<10}".format(str(self.axis.encoder.vel_estimate)))
                        if (max_current < curr):
                            max_current = curr
                        time.sleep(0.5)
                        if abs(self.axis.encoder.pos_estimate - self.axis.controller.input_pos) < 0.6:
                            break 
                except KeyboardInterrupt:
                    None
            print(f"\nJoint {self.name}:\t command sent, input_pos = {position}")
            print("{:<35}".format("max_current:") + str(max_current))
            print("{:<35}".format("axis.encoder.pos_estimate:") + str(self.axis.encoder.pos_estimate))
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

    def pos_gain(self):
        print("{:<30}".format("controller.config.pos_gain:") + str(self.axis.controller.config.pos_gain))
        x = input("Do you want to change pos_gain? (y/n)")
        if x.lower() == "y":
            x = float(input("Input the new pos_gain: "))
            min = 0
            max = 1000
            if x >= min and x <= max:
                self.axis.controller.config.pos_gain = x
            else:
                print(f"FAILED: new pos_gain must be between {min} and {max} inclusive")

    def vel_gain(self):
        print("{:<30}".format("controller.config.vel_gain:") + str(self.axis.controller.config.vel_gain))
        x = input("Do you want to change vel_gain? (y/n)")
        if x.lower() == "y":
            x = float(input("Input the new vel_gain: "))
            min = 0
            if x >= min:
                self.axis.controller.config.vel_gain = x
            else:
                print(f"FAILURE: new vel_gain must be greater than or equal to {min}")
    
    def vel_int_gain(self):
        print("{:<30}".format("controller.config.vel_integrator_gain:") + str(self.axis.controller.config.vel_integrator_gain))
        x = input("Do you want to change vel_integrator_gain? (y/n)")
        if x.lower() == "y":
            x = float(input("Input the new vel_integrator_gain: "))
            self.axis.controller.config.vel_integrator_gain = x

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
                    "\n------------------------------------------------------------------------------------------------\n" +
                    "Enter a function:\n (0) quit\n (1) go_to_position(position)\n (2) home_joint()\n"+
                    " (3) stop()\n (4) current position\n (5) set state to closed loop position control\n (6) current states\n" + 
                    " (7) Dump and Clear Errors\n (8) read and/or set pos_gain\n (9) read and/or set vel_gain\n" +
                    " (10) read and/or set vel_integrator_gain\n"
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
                odr_joint.set_closed_loop_pos_control()
            elif x == 6:
                odr_joint.get_states()
            elif x == 7:
                dump_errors(odr)
                odr.clear_errors()
            elif x == 8:
                odr_joint.pos_gain()
            elif x == 9:
                odr_joint.vel_gain()
            elif x == 10:
                odr_joint.vel_int_gain()
            elif x == 0:
                odr_joint.odr.save_configuration()
                break
        print("Configuration Saved. Goodbye!")
    except KeyboardInterrupt as e:
        odr_joint.stop()
        print(f"Main had a keyboard interupt: {e}")
    except Exception as e:
        print(f"Main threw an exception: {e}")


if __name__ == "__main__":
    main()
