import rclpy
import yaml
import os
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rclpy.duration import Duration

from brc_arm_msg_srv.msg import Encoders, Positions
from brc_arm_msg_srv.srv import Homing

import odrive
import serial.tools.list_ports
from .roboclaw_3 import Roboclaw
from .roboclaw_arm_joint import RoboclawArmJoint
from .roboclaw_ee_joint import RoboclawEEJoint
from .odrive_joint import OdriveJoint

ROBOCLAW_ADDRESS = 128


class BrcArmMotorDriver(Node):
    def __init__(self, simulate):
        super().__init__(
            "brc_arm_motor_driver",
        )
        self.simulate = simulate
        self.configure_joints()

        # Pause to intialize everything
        self.get_clock().sleep_for(Duration(seconds=2))

        self.enc_pub = self.create_publisher(Encoders, "brc_arm/encoders", 1)
        self.pos_sub = self.create_subscription(
            Positions, "brc_arm/positions", self.pos_callback, 1
        )
        self.home_srv = self.create_service(Homing, "homing", self.homing)

        # Run motor controllers at 10 Hz
        self.create_timer(0.1, self.run)

    def configure_joints(self):
        # Load parameters
        config = os.path.join(
            get_package_share_directory("brc_arm_motor_driver"),
            "config",
            "motor_config.yaml",
        )
        f = open(config, "r")
        params = yaml.full_load(f)["/brc_arm_motor_driver"]["ros__parameters"]
        f.close()

        # Save list of joints
        joint_num = 0  # Change to 0 when including GL
        self.joints = [None] * len(params["joints"])
        self.controllers = [None]

        if not self.simulate:
            # Configer motor drivers
            self.find_controllers(params)
            for joint in params["joints"]:
                if "roboclaw" in params["joints"][joint]["controller"]:
                    jointType = params["joints"][joint]["type"]
                    self.get_logger().info(f"Joint {joint_num}: {jointType}: {joint}")
                    roboclaw_idx = next(
                        (
                            i
                            for i, v in enumerate(self.controllers)
                            if v[1] == params["joints"][joint]["identifier"]
                        ),
                        None,
                    )
                    if jointType == "arm":
                        self.joints[joint_num] = RoboclawArmJoint(
                            address=self.controllers[roboclaw_idx][1],
                            name=joint,
                            roboclaw=self.controllers[roboclaw_idx][0],
                            motorNum=params["joints"][joint]["motor_num"],
                            encoderMode=params["joints"][joint]["encoder_mode"],
                            pid_vel=params["joints"][joint]["PID_VEL"],
                            pid_pos=params["joints"][joint]["PID_POS"],
                            speed=params["joints"][joint]["speed"],
                            logger=self.get_logger(),
                        )
                    elif jointType == "EE":
                        print("TODO EE JOINT")
                        self.joints[joint_num] = RoboclawEEJoint(
                            address=self.controllers[roboclaw_idx][1],
                            name=joint,
                            roboclaw=self.controllers[roboclaw_idx][0],
                            motorNum=params["joints"][joint]["motor_num"],
                            currentLimit=params["joints"][joint]["currentLimit"],
                            logger=self.get_logger(),
                        )
                    joint_num += 1
                elif "odrive" in params["joints"][joint]["controller"]:
                    jointType = params["joints"][joint]["type"]
                    self.get_logger().info(f"Joint {joint_num}: {jointType}: {joint}")
                    if jointType == "arm":
                        odrive = self.controllers[0][0]
                        self.joints[joint_num] = OdriveJoint(
                            name=joint, 
                            odr=odrive, 
                            trajectory_limits=params["joints"][joint]["trap_traj_const"],
                            logger=self.get_logger())
                    joint_num += 1

        # For simulating/testing
        else:
            for joint in params["joints"]:
                jointType = params["joints"][joint]["type"]
                self.get_logger().info(f"Joint {joint_num}: {jointType}: {joint}")
                self.joints[joint_num] = None
                joint_num += 1

    def find_controllers(self, params):
        # Detect odrives
        try:
            # TODO: fix odrive detection stuff
            odr = odrive.find_any(timeout=5)
            self.get_logger().info(
                # f"Connected to odrive: {odrive.get_serial_number_str(odr)}"
                f"Connected to odrive: {odr}"
            )
            self.controllers[0] = (odr, -1)
        except TimeoutError as e:
            self.get_logger().error(type(e).__name__)
            self.get_logger().warn("Failed to detect odrive")
            self.get_logger().debug(e)
        # Detect roboclaws
        ports = list(serial.tools.list_ports.comports())
        if len(ports) < 2:  # UPDATE!!!!
            self.get_logger().fatal("Not enough COM devices detected")
            self.destroy_node()
            return
        for p in ports:
            if "Roboclaw" in p.description:
                self.get_logger().info(f"Roboclaw detected at port {p.device}")
                roboclaw = Roboclaw(
                    p.device, params["motor_controllers"]["roboclaw"]["baud"]
                )
                try:
                    self.get_logger().info(f"Connecting to Roboclaw at {p.device}")
                    roboclaw.Open()
                except Exception as e:
                    self.get_logger().fatal(
                        f"Could not connect to Roboclaw at {p.device}"
                    )
                    self.get_logger().debug(e)
                    self.destroy_node()
                    return

                # Get roboclaw address
                # Should have more robust duplicate checking and stuff but works good enough
                address = 0
                for adr in params["motor_controllers"]["roboclaw"]["identifiers"]:
                    try:
                        version = roboclaw.ReadVersion(adr)
                    except AttributeError as e:
                        self.get_logger().fatal(
                            f"Could not connect to Roboclaw at {adr}"
                        )
                        self.get_logger().debug(e)
                        self.destroy_node()
                        return
                    except Exception as e:
                        self.get_logger().fatal(type(e).__name__)
                        self.get_logger().fatal(
                            f"Problem getting roboclaw version at {adr}"
                        )
                        self.get_logger().debug(e)
                        pass
                    if not version[0]:
                        address = -1
                    else:
                        address = adr
                        break
                if address == -1:
                    self.destroy_node()
                    return
                else:
                    self.get_logger().info(f"Connected to Roboclaw {address}")
                    roboclaw.SetPinFunctions(address, 0x00, 0x62, 0x62)

            self.controllers.append((roboclaw, address))

    def run(self):
        self.pub_encoders()

    def pos_callback(self, msg: Positions):
        for idx, goal in enumerate(msg.encoder_goal):
            # self.get_logger().info(f"Joint {idx}: = {int(goal)}")
            if self.simulate and self.joints[idx].is_homed:
                self.joints[idx] = goal
            elif self.joints[idx].is_homed or True: # DANGEROUS!!!
                self.joints[idx].go_to_position(int(goal))
            else:
                self.get_logger().info(f"Joint {idx} is not homed")

    def pub_encoders(self):
        encoder_msg = Encoders()
        encoder_msg.encoder_count = [0.0] * len(self.joints)
        for idx, joint in enumerate(self.joints):
            if joint is not None:
                if self.simulate:
                    encoder_msg.encoder_count[idx] = joint
                else:
                    encoder_msg.encoder_count[idx] = self.joints[idx].read_enc()
        self.get_logger().info(f"Encoder message: {encoder_msg.encoder_count}")
        self.enc_pub.publish(encoder_msg)

    def homing(self, request: Homing.Request, response: Homing.Response):
        bits = [
            (request.home_goal >> bit) & 1
            for bit in range(len(self.joints) - 1, -1, -1)
        ]
        self.get_logger().info(f"Homing request: {list(reversed(bits))}")
        if not self.simulate:
            for idx, bit in enumerate(list(reversed(bits))):
                if bit:
                    bit = self.joints[idx].home_joint()
            # TODO FIX:
            # response.home_status = sum(v << i for i, v in enumerate(bit[::-1]))
            response.home_status = request.home_goal
        else:
            response.home_status = request.home_goal
        return response


def main(args=None):
    rclpy.init(args=args)
    brc_arm_motor_driver = BrcArmMotorDriver(simulate=False)
    rclpy.spin(brc_arm_motor_driver)
    brc_arm_motor_driver.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
