from pymavlink import mavutil
import time
import math

RADIUS = 10 # Radius of the circle

# Arming the Drone
def arm(master):
    print("Arming the Drone")
    master.mav.command_long_send(
        master.target_system, 
        master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, # Command
        0,
        1, 0, 0, 0, 0, 0, 0)
    
    callback(master, "HEARTBEAT")

# Taking OFF
def takeoff(master):
    print("Preparing Take OFF!!")
    altitude = 8 # Meters
    master.mav.command_long_send(0, 0, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, # Command
                                                0, 0, 0, 0, 0, 0, 0, altitude)
    callback(master, "COMMAND_ACK")


def waypoint(master, x, y, z):
    print("Waypoint Received")
    # Local NED Frame (NORTH, EAST, DOWN)
    master.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(10, # Boot Time, Optional
                            master.target_system,
                            master.target_component, 
                            mavutil.mavlink.MAV_FRAME_LOCAL_NED, # Frame
                            int(0b100111111000), # Accepting Position and Fixed Yaw
                            x, # NORTH
                            y, # EAST
                            -z, # -DOWN = UP
                            0, 0, 0, 0, 0, 0, 
                            1.57, # Fixed Yaw
                            0.5)) # Yaw Rate

    # Check wheater the waypoint is reached
    reached = False
    while not reached:
        msg = master.recv_match( # Recieve Message of type LOCAL_POSITION_NED, will keep blocking untill received.
            type='LOCAL_POSITION_NED', blocking=True)
        
        if abs(msg.x - x ) <= 0.1 and abs(msg.y - y ) <= 0.1 and abs((-msg.z) - z ) <= 0.05:
            reached = True
            print("Waypoint Reached")

# Acknowledgement 
def callback(master, keyword):
    print(f"Message Read: {master.recv_match(type=keyword, blocking=True)}")


if __name__ == "__main__":
    # Start a connection listening to a UDP port
    master = mavutil.mavlink_connection("udp:127.0.0.1:14550")

    # Wait for the first heartbeat
    #   This sets the system and component ID of remote system for the link
    master.wait_heartbeat()
    print("Heartbeat from system (system %u component %u)" %
        
        (master.target_system, master.target_component))
    
    # Setting mode to Guided
    mavutil.mavfile.set_mode(master,'GUIDED',0,0)

    arm(master)

    takeoff(master)

    time.sleep(3) # Let the drone Gain some Height


    boot = time.time()

    i = 0
    waypoints = []
    degrees = 0
    while True:
        # X and Y, Coordinates of a circle with center 0+dx, 
        X = RADIUS * math.cos(degrees)
        Y = RADIUS * math.sin(degrees)

        degrees += math.pi/5

        waypoint(master, X, Y, 4)

        RADIUS += 1