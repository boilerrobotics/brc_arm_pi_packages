import odrive
from odrive.enums import *
import time

odr = odrive.find_any()
axis = getattr(odr, "axis1")
axis.requested_state = AXIS_STATE_IDLE
axis.controller.input_vel = 0