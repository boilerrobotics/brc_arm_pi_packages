import time
import odrive
from odrive.enums import *
from odrive.utils import *
import matplotlib.pyplot as plt
import keyboard

# Function to plot
def liveplot(odrv_axis):
    plt.scatter(time.time(), odrv_axis.encoder.pos_estimate, c='k')
    plt.scatter(time.time(), odrv.vbus_voltage, c='r')
    plt.scatter(time.time(), odrv.ibus, c='b')
    plt.pause(0.05)
    start_liveplotter(lambda: [
    odrv_axis.controller.vel_setpoint,
    odrv_axis.encoder.vel_estimate
    odrv_axis.motor.current_control.Iq_setpoint,
    odrv_axis.motor.current_control.Iq_measured
    ])
def find_odrive():
    odrv = odrive.find_any()
    odrv_axis = getattr(odrv, "axis0")
    return odrv, odrv_axis
def disp_pid(odrv_axis):
    print(odrv_axis.controller.config.pos_gain)
    print(odrv_axis.controller.config.vel_gain)
    print(odrv_axis.controller.config.vel_integrator_gain)
    return
def set_pid(odrv, odrv_axis, pos_gain, vel_gain, vel_integrator_gain):
    odrv_axis.controller.config.pos_gain = pos_gain
    odrv_axis.controller.config.vel_gain = vel_gain
    odrv_axis.controller.config.vel_integrator_gain = vel_integrator_gain
    try:
        odrv.save_configuration()
    except:
        pass
    
    return

odrv, odrv_axis = find_odrive()
print("Found ODrive");
try:
    dump_errors(odrv)
    if(input("Ckear errors (y/n):") == "y"):
        odrv.clear_errors()
    print("Current Values")
    disp_pid(odrv_axis)
    if(input("Set gains (y/n):") == "y"):
        pos_gain = input("Pos Gain = ")
        vel_gain = input("Vel Gain = ")
        vel_integrator_gain = input("Vel Integrator Gain = ")
        set_pid(odrv, odrv_axis, pos_gain,   vel_gain, vel_integrator_gain)
    odrv, odrv_axis = find_odrive()
    if(input("Trap control (y/n):") == "y"):
        odrv_axis.controller.config.input_mode = INPUT_MODE_TRAP_TRAJ
        odrv_axis.trap_traj.config.vel_limit = 15.0
        odrv_axis.trap_traj.config.accel_limit = 3.0
        odrv_axis.trap_traj.config.decel_limit = 3.0
        odrv_axis.controller.config.inertia = 0.0
    else:
        odrv_axis.controller.config.input_mode = INPUT_MODE_PASSTHROUGH
    print("Set Values.")
    input("Press Enter to Start Motor")
    odrv_axis.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
    position = 0
    increment = 0
    position = float(input("Go To: "))
    while(True):
        if(keyboard.is_pressed("g")):
            print("Stopping...")
            odrv_axis.requested_state = AXIS_STATE_IDLE
            break
        if(keyboard.is_pressed("p")):
            position = float(input("Go To: "))
        liveplot(odrv_axis)
        position += increment
        #odrv_axis.controller.input_vel = 2
        odrv_axis.controller.input_pos = position
        print(str(odrv_axis.encoder.pos_estimate) + ", " + str(odrv_axis.encoder.vel_estimate) + ", " + str(odrv_axis.controller.pos_setpoint))
        #if (increment != 0):
        #    while (1):
        #        print(str(odrv_axis.encoder.pos_estimate))
        #        odrv_axis.controller.input_pos = position
        #        time.sleep(.01)
        #increment = float(input("Go By: "))
        time.sleep(.1)
except:
    odrv_axis.requested_state = AXIS_STATE_IDLE
    #odrv_axis.controller.input_vel = 0

