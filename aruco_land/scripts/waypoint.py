from pymavlink import mavutil
import time

# Start a connection listening to a UDP port
master = mavutil.mavlink_connection("udp:127.0.0.1:14550")

# Wait for the first heartbeat
#   This sets the system and component ID of remote system for the link
master.wait_heartbeat()
print("Heartbeat from system (system %u component %u)" %
      
      (master.target_system, master.target_component))

# Setting the mode to GUIDED
mavutil.mavfile.set_mode(master,'GUIDED',0,0)


master.mav.command_long_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, # Command
    0,
    1, 0, 0, 0, 0, 0, 0)

altitude = 8 # Meters

master.mav.command_long_send(0, 0, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, # Command
                                               0, 0, 0, 0, 0, 0, 0, altitude)

time.sleep(3)

# Uncomment following lines to use NED (North East Down) Coordinate Frame
# master.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(0, master.target_system,
#                         master.target_component, mavutil.mavlink.MAV_FRAME_LOCAL_NED, # Command
#                         int(0b010111111000), # Type
#                         40, # X in meters
#                         0, # Y in meters
#                         -10, # Z in meters ( Down )
#                         0, 0, 0, 0, 0, 0, 
#                         1.57, # Fixed Yaw
#                         0.5)) # Yaw


# Uncomment following lines to use Global (GPS) Coordinate Frame
master.mav.send(mavutil.mavlink.MAVLink_set_position_target_global_int_message(0, master.target_system,
                        master.target_component, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,  # Frame
                        int(0b110111111000), # Type
                        int(-35.3929849 * 1E7), # Latitude * 1E7
                        int(149.1949185 * 1E7),  # Longitude * 1E7
                        10,  # Altitude Meters
                        0, 0, 0, 0, 0, 0, 
                        1.57, # Fixed Yaw
                        0.5)) # Yaw Rate

# time.sleep(2)

while 1:
    msg = master.recv_match( # Recieve Message of type LOCAL_POSITION_NED, block untill received
        type='LOCAL_POSITION_NED', blocking=True)
    print(msg)
