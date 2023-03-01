import time
from rclpy.impl.rcutils_logger import RcutilsLogger

from .roboclaw_3 import Roboclaw

class RoboclawArmJoint:
    def __init__(
        self,
        address,
        name,
        roboclaw: Roboclaw,
        motorNum,
        encoderMode,
        pid,
        speed,
        logger: RcutilsLogger,
    ):
        self.address = address
        self.name = name
        self.roboclaw = roboclaw
        self.logger = logger
        self.motorNum = motorNum
        self.speed = speed
        self.is_homed = False

        try:
            self.roboclaw.SetPinFunctions(self.address, 0x00, 0x62, 0x62) # DANGEROUS
            if self.motorNum == 1:
                self.roboclaw.SetM1PositionPID(
                    self.address, pid[0], pid[1], pid[2], pid[3], 0, -100, 100000
                )
                self.roboclaw.SetM1EncoderMode(self.address, encoderMode)
            elif self.motorNum == 2:
                self.roboclaw.SetM2PositionPID(
                    self.address, pid[0], pid[1], pid[2], pid[3], 0, -100, 100000
                )
                self.roboclaw.SetM2EncoderMode(self.address, encoderMode)
        except OSError as e:
            self.logger.warn(f"Joint {self.name}:\t Setup Error: OSError: {e.errno}")
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
            return int(enc)

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
        timeout = 60
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
            while self.roboclaw.ReadError(self.address)[1] not in (error_code, 0xC00000):
                self.logger.info(f"{self.roboclaw.ReadError(self.address)[1]}")
                if time.monotonic() - start_time > timeout:
                    self.logger.warn(
                        f"Homing joint {self.name} failed, took longer than {timeout}s"
                    )
                    return self.is_homed
            self.set_enc(0)
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
