#!/usr/bin/env python3
import rospy
from cv_bridge import CvBridge
import cv2
from cv2 import aruco
from pymap3d import geodetic2ecef,geodetic2enu
import numpy as np
from pymavlink import mavutil
import time
from sensor_msgs.msg import Image, CameraInfo

import math 


bridge = CvBridge()
image = None
camera_matrix = None
distortion_coef = None
marker_size = 1 # 1 meter
corners = None

# Arming thr drone
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
    altitude = 5 # Meters
    master.mav.command_long_send(0, 0, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, # Command
                                                0, 0, 0, 0, 0, 0, 0, altitude)
    callback(master, "COMMAND_ACK")


def waypoint(master, x, y, z):
    print("Waypoint Received")
    # Local NED Frame (NORTH, EAST, DOWN)
    master.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(10, # Boot Time, Optional
                            master.target_system,
                            master.target_component, 
                            mavutil.mavlink.MAV_FRAME_LOCAL_OFFSET_NED, # Frame
                            int(0b100111111000), # Accepting Position and Fixed Yaw
                            x, # NORTH
                            y, # EAST
                            -z, # -DOWN = UP
                            0, 0, 0, 0, 0, 0, 
                            0, # Fixed Yaw
                            0.5)) # Yaw Rate

    # Check wheater the waypoint is reached
    reached = False
    while not reached:
        msg = master.recv_match( # Recieve Message of type LOCAL_POSITION_NED, will keep blocking untill received.
            type='LOCAL_POSITION_NED', blocking=True)
        
        # If the velocity is less the 0.1, Consider the target to be reached
        if msg.vx < 0.1:
            reached = True
            print("Reached")

# Acknowledgement 
def callback(master, keyword):
    print(f"Message Read: {master.recv_match(type=keyword, blocking=True)}")


def img_callback(msg):
    global image, corners
    image = bridge.imgmsg_to_cv2(msg) # Convert ROS Image message to CV2 usable form
    aruco_dict = aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_1000) 
    parameters = aruco.DetectorParameters()
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    corners, ids, rejectedImgPoints = aruco.detectMarkers( # Get Pixel Coordinates for Arcuo Cordner Coordinates
        gray, aruco_dict, parameters=parameters)
    
    aruco.drawDetectedMarkers(image, corners, ids) # Draw Bounding Box on return Image window
    


def getArucoPose(marker_corners):
    rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(marker_corners, marker_size, np.array(
        camera_matrix).reshape(3, 3), np.array(distortion_coef).reshape(5, 1))
    return rvecs, tvecs


def camera_callback(msg):
    global camera_matrix, distortion_coef
    camera_matrix = msg.K

    distortion_coef = msg.D


# Get Local NED coordinates from Global GPS Coordinates
def globalToLocal(lat, long, alt):

    # Work in Progress......

    lat0 = 35.36326210 
    lon0 = 149.16525118
    alt0 = 4

    e, n, u = geodetic2enu(lat, long, alt, lat0, lon0, alt0)

    rotation = np.array([[1,0,0],[0,1,0],[0,0,1]])
    translation = np.array([-n, -e, -u])

    return rotation*translation, rotation, translation


def main():
    rospy.init_node("Aruco_Marker", anonymous=True)

    rospy.Subscriber("image_raw", Image, img_callback)
    rospy.Subscriber("camera_info", CameraInfo, camera_callback)
    pub = rospy.Publisher("aruco_detected", Image)

    master = mavutil.mavlink_connection("udp:127.0.0.1:14550")

    # Wait for the first heartbeat
    #   This sets the system and component ID of remote system for the link
    master.wait_heartbeat()
    print("Heartbeat from system (system %u component %u)" %
        
        (master.target_system, master.target_component))
    
    # Setting mode to Guided
    mavutil.mavfile.set_mode(master,'GUIDED',0,0)

    arm(master)
    
    time.sleep(1) # Let the drone Gain some Height

    takeoff(master)

    time.sleep(5) # Let the drone Gain some Height
    doOnce = True # For passing the waypoint just once

    while (not rospy.is_shutdown()):
        if doOnce:
        
            waypoint(master,10,10,5)
            doOnce = False
        
        try:
            cv2.imshow("Image", image)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

            if np.array(corners).size > 1:

                # Get Rotation and Translation Matrix of the Aruco with respect to the Camera Frame
                rvecs, tvecs = getArucoPose(np.array(corners).reshape(1, 4, 2))
                # rotation = np.array([[1,0,0],[0,1,0],[0,0,1]])

                # Translation matrix from Camera WRT Drone.
                translation = np.array([0, 0, -0.216]).reshape(3,1)

                # Translation of Aruco with respect to Drone Frame
                P = translation + tvecs.reshape(3,1)

                # Pushing Waypoints for getting the drone to the aruco center
                waypoint(master, -P[1]/3 , P[0]/3, 0)
                if (abs(P[0] < 0.1) and (abs(P[1]) < 0.1)):
                    # Command for Landing Sequence
                    print("Initiating Landing Sequence")
                    master.mav.command_long_send(0, 0, mavutil.mavlink.MAV_CMD_NAV_LAND, 
                                               0, 0, 0, 0, 0, 0, 0, 
                                               0)
                    
                    break

        except Exception as e:
            print(e)


if __name__ == "__main__":
    main()
