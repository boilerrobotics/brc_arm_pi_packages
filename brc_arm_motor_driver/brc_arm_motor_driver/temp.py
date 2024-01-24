import odrive_joint
import odrive

odrv = odrive.find_any()
print(str(odrv.vbus_voltage))