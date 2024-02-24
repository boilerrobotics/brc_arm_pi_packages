import sys
import time
import odrive
from odrive.enums import *
# intentional error to stop you from running this....
class HBMotorConfig:
    """
    Class for configuring an Odrive axis for a Flipsky motor. 
    """
    
    # Motor Kv
    MOTOR_KV = 270.0
        
    # Min/Max phase inductance of motor
    MIN_PHASE_INDUCTANCE = 0
    MAX_PHASE_INDUCTANCE = 0.017
    
    # Min/Max phase resistance of motor
    MIN_PHASE_RESISTANCE = 0
    MAX_PHASE_RESISTANCE = 0.5
    
    # Tolerance for encoder offset float
    ENCODER_OFFSET_FLOAT_TOLERANCE = 0.05

    def __init__(self, axis_num):
        """
        Initalizes HBMotorConfig class by finding odrive, erase its 
        configuration, and grabbing specified axis object.
        
        :param axis_num: Which channel/motor on the odrive your referring to.
        :type axis_num: int (0 or 1)
        """
        
        self.axis_num = axis_num
    
        # Connect to Odrive
        print("Looking for ODrive...")
        self._find_odrive()
        print("Found ODrive.")
        
    def _find_odrive(self):
        # connect to Odrive
        try:
            self.odrv = odrive.find_any()
            self.odrv_axis = getattr(self.odrv, "axis{}".format(self.axis_num))
        except KeyboardInterrupt:
            print("Keyboard Interrupt Detected. Exiting")
    
    def configure(self):
        """
        Configures the odrive device for a  motor.
        """
        
        
        self._find_odrive()
        input("ODrive Found, press enter to start configuration")
        
        # Erase pre-exsisting configuration
        print("Erasing pre-exsisting configuration...")
        
        try:
            self.odrv.erase_configuration()
        except:
            pass
        self._find_odrive()
        
        # Set Pole Pairs
        self.odrv_axis.motor.config.pole_pairs = 7

        # Hoverboard hub motors are quite high resistance compared to the hobby 
        # aircraft motors, so we want to use a bit higher voltage for the motor 
        # calibration, and set up the current sense gain to be more sensitive. 
        # The motors are also fairly high inductance, so we need to reduce the 
        # bandwidth of the current controller from the default to keep it 
        # stable.
        self.odrv_axis.motor.config.resistance_calib_max_voltage = 4
        self.odrv_axis.motor.config.requested_current_range      = 20
        self.odrv_axis.motor.config.current_control_bandwidth    = 100
        self.odrv_axis.motor.config.calibration_current = 10
        self.odrv.config.enable_brake_resistor = True
        

        self.odrv_axis.motor.config.torque_constant = 8.27 / self.MOTOR_KV

        # Flipsky motors contain hall effect sensors instead of incremental 
        # encorders
        self.odrv_axis.encoder.config.mode = EncoderMode.HALL

        # The hall feedback has 6 states for every pole pair in the motor. Since
        # we have 7 pole pairs, we set the cpr to 7*6 = 42.
        self.odrv_axis.encoder.config.cpr = 42

        # Since hall sensors are low resolution feedback, we also bump up the 
        #offset calibration displacement to get better calibration accuracy.
        self.odrv_axis.encoder.config.calib_scan_distance = 150

        # Since the hall feedback only has 90 counts per revolution, we want to 
        # reduce the velocity tracking bandwidth to get smoother velocity 
        # estimates. We can also set these fairly modest gains that will be a
        # bit sloppy but shouldn’t shake your rig apart if it’s built poorly. 
        # Make sure to tune the gains up when you have everything else working 
        # to a stiffness that is applicable to your application.
        self.odrv_axis.encoder.config.bandwidth = 100
        self.odrv_axis.controller.config.pos_gain = 1
        self.odrv_axis.controller.config.vel_gain = 0.02 * self.odrv_axis.motor.config.torque_constant * self.odrv_axis.encoder.config.cpr
        self.odrv_axis.controller.config.vel_integrator_gain = 0.1 * self.odrv_axis.motor.config.torque_constant * self.odrv_axis.encoder.config.cpr
        self.odrv_axis.controller.config.vel_limit = 10

        # Set in position control mode so we can control the position of the 
        # motor
        self.odrv_axis.controller.config.control_mode = ControlMode.POSITION_CONTROL

        # In the next step we are going to start powering the motor and so we 
        # want to make sure that some of the above settings that require a 
        # reboot are applied first.
        print("Saving manual configuration and rebooting...")
        try:
            self.odrv.save_configuration()
        except:
            pass
        self._find_odrive()
        print("Manual configuration saved.")
            
        try:
            self.odrv.reboot()
        except:
            pass
        self._find_odrive()
            

        input("Make sure the motor is free to move, then press enter...")
        
        print("Calibrating Odrive for  motor (you should hear a "
        "beep)...")
        
        self.odrv_axis.requested_state = AxisState.MOTOR_CALIBRATION
        
        # Wait for calibration to take place
        time.sleep(10)

        if self.odrv_axis.motor.error != 0:
            print("Error: Odrive reported an error of {} while in the state " 
            "AXIS_STATE_MOTOR_CALIBRATION. Printing out Odrive motor data for "
            "debug:\n{}".format(self.odrv_axis.motor.error, 
                                self.odrv_axis.motor))
            
            sys.exit(1)

        if self.odrv_axis.motor.config.phase_inductance <= self.MIN_PHASE_INDUCTANCE or \
        self.odrv_axis.motor.config.phase_inductance >= self.MAX_PHASE_INDUCTANCE:
            print("Error: After odrive motor calibration, the phase inductance "
            "is at {}, which is outside of the expected range. Either widen the "
            "boundaries of MIN_PHASE_INDUCTANCE and MAX_PHASE_INDUCTANCE (which "
            "is between {} and {} respectively) or debug/fix your setup. Printing "
            "out Odrive motor data for debug:\n{}".format(self.odrv_axis.motor.config.phase_inductance, 
                                                          self.MIN_PHASE_INDUCTANCE,
                                                          self.MAX_PHASE_INDUCTANCE, 
                                                          self.odrv_axis.motor))
            
            sys.exit(1)

        if self.odrv_axis.motor.config.phase_resistance <= self.MIN_PHASE_RESISTANCE or \
        self.odrv_axis.motor.config.phase_resistance >= self.MAX_PHASE_RESISTANCE:
            print("Error: After odrive motor calibration, the phase resistance "
            "is at {}, which is outside of the expected range. Either raise the "
            "MAX_PHASE_RESISTANCE (which is between {} and {} respectively) or "
            "debug/fix your setup. Printing out Odrive motor data for " 
            "debug:\n{}".format(self.odrv_axis.motor.config.phase_resistance, 
                                self.MIN_PHASE_RESISTANCE,
                                self.MAX_PHASE_RESISTANCE, 
                                self.odrv_axis.motor))
            
            sys.exit(1)

        # If all looks good, then lets tell ODrive that saving this calibration 
        # to persistent memory is OK
        self.odrv_axis.motor.config.pre_calibrated = True

        # Check the alignment between the motor and the hall sensor. Because of 
        # this step you are allowed to plug the motor phases in random order and
        # also the hall signals can be random. Just don’t change it after 
        # calibration.
        print("Calibrating Odrive for encoder...")
       # self.odrv_axis.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
        
        # Wait for calibration to take place
        #time.sleep(40)
            
        #if self.odrv_axis.encoder.error != 0:
            #print("Error: Odrive reported an error of {} while in the state "
            #"AXIS_STATE_ENCODER_OFFSET_CALIBRATION. Printing out Odrive encoder "
            #"data for debug:\n{}".format(self.odrv_axis.encoder.error, 
                                         #self.odrv_axis.encoder))
            
            #sys.exit(1)
        
        # If offset_float isn't 0.5 within some tolerance, or its not 1.5 within
        # some tolerance, raise an error
        #if not ((self.odrv_axis.encoder.config.offset_float > 0.5 - self.ENCODER_OFFSET_FLOAT_TOLERANCE and \
        #self.odrv_axis.encoder.config.offset_float < 0.5 + self.ENCODER_OFFSET_FLOAT_TOLERANCE) or \
        #(self.odrv_axis.encoder.config.offset_float > 1.5 - self.ENCODER_OFFSET_FLOAT_TOLERANCE and \
        #self.odrv_axis.encoder.config.offset_float < 1.5 + self.ENCODER_OFFSET_FLOAT_TOLERANCE)):
            #print("Error: After odrive encoder calibration, the 'offset_float' "
            #"is at {}, which is outside of the expected range. 'offset_float' "
            #"should be close to 0.5 or 1.5 with a tolerance of {}. Either "
            #"increase the tolerance or debug/fix your setup. Printing out "
            #"Odrive encoder data for debug:\n{}".format(self.odrv_axis.encoder.config.offset_float, 
                                                       # self.ENCODER_OFFSET_FLOAT_TOLERANCE, 
                                                       # self.odrv_axis.encoder))
                       
            #sys.exit(1)
        
        # If all looks good, then lets tell ODrive that saving this calibration 
        # to persistent memory is OK
        self.odrv_axis.encoder.config.pre_calibrated = True
        
        print("Saving calibration configuration and rebooting...")
        try:
            self.odrv.save_configuration()
        except:
            pass
        self._find_odrive()
        print("Calibration configuration saved.")
        
        try:
            self.odrv.reboot()
        except:
            pass
        self._find_odrive()
        
        print("Odrive configuration finished.")
    
    def mode_idle(self):
        """
        Puts the motor in idle (i.e. can move freely).
        """
        
        self.odrv_axis.requested_state = AxisState.IDLE
    
    def mode_close_loop_control(self):
        """
        Puts the motor in closed loop control.
        """
        
        self.odrv_axis.requested_state = AxisState.CLOSED_LOOP_CONTROL
        
    def move_input_pos(self, angle):
        """
        Puts the motor at a certain angle.
        
        :param angle: Angle you want the motor to move.
        :type angle: int or float
        """
        
        self.odrv_axis.controller.input_pos = angle/360.0

if __name__ == "__main__":
    print("DO NOT RUN THIS FILE")
    exit

    # hb_motor_config = HBMotorConfig(axis_num = 0)
    # hb_motor_config.configure()
    
    # input("CONDUCTING MOTOR TEST, press enter to start")
    # print("Placing motor in close loop control. If you move motor, motor will "
    #       "resist you.")
    # hb_motor_config.mode_close_loop_control()
    
    # Go from 0 to 360 degrees in increments of 30 degrees
    #for angle in range(0, 390, 30):
    #    print("Setting motor to {} degrees.".format(angle))
    #    hb_motor_config.move_input_pos(angle)
    #    time.sleep(5)
    
    #print("Placing motor in idle. If you move motor, motor will "
    #      "move freely")
    #hb_motor_config.mode_idle()
