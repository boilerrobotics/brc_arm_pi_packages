import time
from rclpy.impl.rcutils_logger import RcutilsLogger
from brc_arm_msg_srv.msg import Encoders, Positions
from brc_arm_msg_srv.srv import Homing

from .roboclaw_3 import Roboclaw


class RoboclawJoint:
    def __init__(
        self,
        name,
        roboclaw: Roboclaw,
        address,
        motorNum,
        pid,
        speed,
        logger: RcutilsLogger,
    ):
        self.name = name
        self.roboclaw = roboclaw
        self.address = address
        self.logger = logger
        self.motorNum = motorNum
        self.speed = speed
        self.is_homed = False

        try:
            if self.motorNum == 1:
                self.roboclaw.SetM1PositionPID(
                    self.address, pid[1], pid[2], pid[3], pid[4], 0, 0, 100000
                )
            elif self.motorNum == 2:
                self.roboclaw.SetM2PositionPID(
                    self.address, pid[1], pid[2], pid[3], pid[4], 0, 0, 100000
                )
        except OSError as e:
            self.logger.warn(f"Joint {self.name}:\t PID Error: OSError: {e.errno}")
            self.logger.debug(e)
        self.set_enc(0)

    def read_enc(self):
        status, enc, crc = None, None, None

        try:
            if self.motorNum == 1:
                status, enc, crc = self.roboclaw.ReadEncM1(self.address)
            elif self.motorNum == 2:
                status, enc, crc = self.roboclaw.ReadEncM2(self.address)
            self.logger.info(f"Joint {self.name}:\t EncM{self.motorNum} Reading: {enc}")
        except ValueError:
            self.logger.warn(f"Joint {self.name}:\t ReadEncM{self.motorNum} ValueError")
            return
        except OSError as e:
            self.logger.warn(
                f"Joint {self.name}:\t error: ReadEncM{self.motorNum} OSError: {e.errno}"
            )
            self.logger.debug(e)
            return

        if enc != None:
            return float(enc)

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
        start_time = time.monotonic()
        timeout = 10
        error_code = 0
        rate = 10  # Hz

        self.logger.info(f"Homing joint {self.name}")
        try:
            if self.motorNum == 1:
                self.roboclaw.BackwardM1(self.address, 20)
                error_code = 0x400000
            elif self.motorNum == 2:
                self.roboclaw.BackwardM2(self.address, 20)
                error_code = 0x800000
            while self.roboclaw.ReadError(self.address)[1] != error_code:
                if start_time - time.monotonic() > timeout:
                    self.logger.warn(
                        f"Homing joint {self.name} failed, took longer than {timeout}s"
                    )
                    return self.is_homed
                time.sleep(1 / rate)
        except OSError as e:
            self.logger.warn(f"Homing joint {self.name} error: {e.errno}")
            self.logger.debug(e)
            return False
        self.is_homed = True
        return self.is_homed

    def go_to_position(self, position):
        try:
            if self.motorNum == 1:
                self.roboclaw.SpeedAccelDeccelPositionM1(
                    self.address,
                    self.speed[0],
                    self.speed[1],
                    self.speed[2],
                    position,
                    1,
                )
            elif self.motorNum == 2:
                self.roboclaw.SpeedAccelDeccelPositionM2(
                    self.address,
                    self.speed[0],
                    self.speed[1],
                    self.speed[2],
                    position,
                    1,
                )
            self.logger.info(
                f"Joint {self.name}:\t command sent, position = {position}"
            )
        except OSError as e:
            self.logger.warn(
                f"Joint {self.name}:\t error: SpeedAccelDeccelPositionM{self.motorNum} OSError: {e.errno}"
            )
            self.logger.debug(e)

    def stop(self):
        if self.motorNum == 1:
            self.roboclaw.ForwardM1(self.address, 0)
        elif self.motorNum == 2:
            self.roboclaw.ForwardM2(self.address, 0)
