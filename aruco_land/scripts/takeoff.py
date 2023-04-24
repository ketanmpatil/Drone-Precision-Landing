# Import mavutil
from pymavlink import mavutil
import time

# Create the connection
master = mavutil.mavlink_connection('udp:127.0.0.1:14550')

# Wait a heartbeat before sending commands
master.wait_heartbeat()

mavutil.mavfile.set_mode(master,'GUIDED',0,0) # Setting mode to Guided


master.mav.command_long_send( # Command Message Long
    master.target_system, 
    master.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, # Command for Arming
    0,
    1, 0, 0, 0, 0, 0, 0)

altitude = 8 # Altitude : 8 Meters


master.mav.command_long_send(0, 0, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, # Command for Takeoff
                                               0, 0, 0, 0, 0, 0, 0, 
                                               altitude)


time.sleep(8)


master.mav.command_long_send(0, 0, mavutil.mavlink.MAV_CMD_NAV_LAND, # Command for Takeoff
                                               0, 0, 0, 0, 0, 0, 0, 
                                               0)
