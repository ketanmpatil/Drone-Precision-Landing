# Import mavutil
from pymavlink import mavutil

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
