import time
from rclpy.impl.rcutils_logger import RcutilsLogger

from .roboclaw_3 import Roboclaw

class RoboclawEEJoint:
    def __init__(
        self,
        address,
        name,
        roboclaw: Roboclaw,
        motorNum,
        currentLimit,
        logger: RcutilsLogger,
    ):
        self.address = address
        self.name = name
        self.roboclaw = roboclaw
        self.logger = logger
        self.motorNum = motorNum
        self.currentLimit = currentLimit
        self.is_homed = False
        # 0 = closed, 1 = open
        self.position = 0

        # TODO: implement current limit stuff

    def read_enc(self):
        return self.position

    def set_enc(self, count):
        try:
            if self.motorNum == 1:
                self.roboclaw.SetEncM1(self.address, count)
            elif self.motorNum == 2:
                self.roboclaw.SetEncM2(self.address, count)
            self.logger.info(f"Joint {self.name} encoder set to {count}")
        except OSError as e:
            self.logger.warn(
                f"Joint {self.name}:\t error: SetEncM{self.motorNum} OSError: {e.errno}"
            )
            self.logger.debug(e)

    def home_joint(self):
        # return self.is_homed
        return True

    def go_to_position(self, position):
        self.logger.info(f"EE joint to position: {position}")

    def stop(self):
        if self.motorNum == 1:
            self.roboclaw.ForwardM1(self.address, 0)
        elif self.motorNum == 2:
            self.roboclaw.ForwardM2(self.address, 0)
