import rclpy
import yaml
import os
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rclpy.duration import Duration

from brc_arm_msg_srv.msg import Encoders, Positions
from brc_arm_msg_srv.srv import Homing

import odrive
from .roboclaw_3 import Roboclaw
from .roboclaw_joint import RoboclawJoint


class BrcArmMotorDriver(Node):
    def __init__(self, simulate):
        super().__init__(
            "brc_arm_motor_driver",
        )
        self.simulate = simulate
        self.configure_motors()

        # Pause to intialize everything
        self.get_clock().sleep_for(Duration(seconds=2))

        self.enc_pub = self.create_publisher(Encoders, "brc_arm/encoders", 1)
        self.pos_sub = self.create_subscription(
            Positions, "brc_arm/positions", self.pos_callback, 1
        )
        self.home_srv = self.create_service(Homing, "homing", self.homing)

        # Run motor controllers at 10 Hz
        self.create_timer(0.1, self.run)

    def find_controllers(self):
        print("TODO")
        # odr = odrive.find_any()

    def configure_motors(self):
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
        self.controllers = self.find_controllers()

        if not self.simulate:
            # Configer motor drivers
            for motor_controller in params["motor_controllers"]:
                dev_name = params["motor_controllers"][motor_controller]["dev"]
                baud_rate = params["motor_controllers"][motor_controller]["baud"]
                address = params["motor_controllers"][motor_controller]["address"]
                # Configure roboclaws
                if "roboclaw" in motor_controller:
                    print(f"Roboclaw: {dev_name}@{address}")
                    if address > 0x87 or address < 0x80:
                        self.get_logger().fatal("Address out of range")
                        self.destroy_node()
                        return
                    roboclaw = Roboclaw(dev_name, baud_rate)
                    try:
                        self.get_logger().info("Connecting to Roboclaw at %d", address)
                        roboclaw.Open()
                    except Exception as e:
                        self.get_logger().fatal(
                            "Could not connect to Roboclaw at %d", address
                        )
                        self.get_logger().debug(e)
                        self.destroy_node()
                        return

                    try:
                        version = roboclaw.ReadVersion(address)
                    except AttributeError as e:
                        self.get_logger().fatal(
                            "Could not connect to Roboclaw at %d", address
                        )
                        self.get_logger().debug(e)
                        self.destroy_node()
                        return
                    except Exception as e:
                        self.get_logger().error(type(e).__name__)
                        self.get_logger().warn("Problem getting roboclaw version")
                        self.get_logger().debug(e)
                        pass

                    if not version[0]:
                        self.get_logger().warn("Could not get version from roboclaw")
                    else:
                        self.get_logger().debug(repr(version[1]))

                    self.get_logger().info("Connected to Roboclaw at %d", address)
                    roboclaw.SetPinFunctions(address, 0x00, 0x62, 0x62)

                    for joint in params["motor_controllers"][motor_controller][
                        "joints"
                    ]:
                        jointType = params["joints"][joint]["type"]
                        print(f"Joint {joint_num}: {jointType}: {joint}")
                        if jointType == "arm":
                            self.joints[joint_num] = RoboclawJoint(
                                joint,
                                roboclaw,
                                address,
                                joint["motor_num"],
                                joint["PIDMM"],
                                joint["speed"],
                                self.get_logger(),
                            )
                        elif jointType == "EE":
                            print("TODO EE")
                        joint_num += 1

                elif "odrive" in motor_controller:
                    print("TODO ODRIVE")
                    for joint in params["motor_controllers"][motor_controller][
                        "joints"
                    ]:
                        jointType = params["joints"][joint]["type"]
                        print(f"Joint {joint_num}: {jointType}: {joint}")
                        if jointType == "arm":
                            self.joints[joint_num] = None
                        elif jointType == "EE":
                            print("TODO EE")
                        joint_num += 1

        # For simulating/testing
        else:
            for joint in params["joints"]:
                jointType = params["joints"][joint]["type"]
                print(f"Joint {joint_num}: {jointType}: {joint}")
                self.joints[joint_num] = None
                joint_num += 1

    def run(self):
        self.pub_encoders()

    def pos_callback(self, msg: Positions):
        for idx, goal in enumerate(msg.encoder_goal):
            print(f"Joint {idx}: = {goal}")
            if self.simulate:
                self.joints[idx] = goal
            else:
                self.joints[idx].go_to_position(goal)

    def pub_encoders(self):
        encoder_msg = Encoders()
        encoder_msg.encoder_count = [0.0] * len(self.joints)
        for idx, joint in enumerate(self.joints):
            if joint is not None:
                if self.simulate:
                    encoder_msg.encoder_count[idx] = joint
                else:
                    encoder_msg.encoder_count[idx] = self.joints[idx].read_enc()
        self.enc_pub.publish(encoder_msg)

    def homing(self, request: Homing.Request, response: Homing.Response):
        bits = [
            (request.home_goal >> bit) & 1 for bit in range(self.joint_num - 1, -1, -1)
        ]
        self.get_logger().info(f"Homing request: {bits}")
        if not self.simulate:
            for idx, bit in bits:
                if bit:
                    bit = self.joints[idx].home()
            response.home_status = sum(v << i for i, v in enumerate(bit[::-1]))
        else:
            response.home_status = request.home_goal
        return response


def main(args=None):
    rclpy.init(args=args)
    brc_arm_motor_driver = BrcArmMotorDriver(simulate=True)
    rclpy.spin(brc_arm_motor_driver)
    brc_arm_motor_driver.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
