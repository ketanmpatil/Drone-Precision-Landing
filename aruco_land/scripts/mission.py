from pymavlink import mavutil, mavwp
import time

class MissionItem:
    def __init__(self, i, current, x, y, z):
        self.seq = i
        self.frame = mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT
        self.command = mavutil.mavlink.MAV_CMD_NAV_WAYPOINT
        self.current = current
        self.auto = 1
        self.param1 = 0
        self.param2 = 2.0
        self.param3 = 20.0
        self.param4 = 0
        self.param5 = x
        self.param6 = y
        self.param7 = z
        self.mission_type = 0

def ack(master, keyword):
    print(f"Message Read: {str(master.recv_match(type=keyword, blocking=True))}")

def setMode(mode="GUIDED"):
    mavutil.mavfile.set_mode(master,mode,0,0)

    ack(master, "HEARTBEAT")


def upload_mission(master, waypoints):
    n = len(waypoints)
    print("Sending Message Out")

    master.mav.mission_count_send(master.target_system,
    master.target_component,n, 0)

    ack(master, "MISSION_REQUEST")

    for point in waypoints:
        print("Creating Waypoint")

        master.mav.mission_item_send(master.target_system, master.target_component,
                                     point.seq,
                                     point.frame,
                                     point.command,
                                     point.current,
                                     point.auto,
                                     point.param1,
                                     point.param2,
                                     point.param3,
                                     point.param4,
                                     point.param5,
                                     point.param6,
                                     point.param7,
                                     point.mission_type)
        
    if waypoints != mission_waypoints[n-1]:
        ack(master, "MISSION_REQUEST")

    ack(master,"MISSION_ACK")

def arm(master):
    master.mav.command_long_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    0,
    1, 0, 0, 0, 0, 0, 0)

    ack(master, "COMMAND_ACK")

def takeoff(master):
    master.mav.command_long_send(0, 0, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
                                               0, 0, 0, 0, 0, 0, 0, 5)
    
    ack(master, "COMMAND_ACK")

def start_mission(master):
    print("Mission Start")

    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_MISSION_START,
        0,
        0, 0, 0, 0, 0, 0, 0)

    ack(master, "COMMAND_ACK")




def set_return(master):

    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH,
        0,
        0, 0, 0, 0, 0, 0, 0)

    ack(master, "COMMAND_ACK")



if __name__ == "__main__":
    
    master = mavutil.mavlink_connection("udp:127.0.0.1:14550")

    master.wait_heartbeat()
    print("Heartbeat from system (system %u component %u)" %
        
        (master.target_system, master.target_component))
    

    mission_waypoints = []
    
    mission_waypoints.append(MissionItem(0, 0, -35.36131955,  149.16892747 , 3))
    mission_waypoints.append(MissionItem(1, 0, -35.36131955,  149.16892747 , 3))
    # mission_waypoints.append(MissionItem(2, 0, -35.35948407, 149.16795170 , 3))
    
    print(len(mission_waypoints))
    upload_mission(master, mission_waypoints)

    setMode()

    time.sleep(1)

    arm(master)

    time.sleep(1)

    takeoff(master)

    time.sleep(3)

    start_mission(master)

    

    # for item in mission_waypoints:
        # print(item.seq)condition='MISSION_ITEM_REACHED.seq=={item.seq}'

    m = master.recv_match(type="MISSION_ITEM_REACHED",blocking=True)
    print(m.seq)
    # continue
        # ack(master, "MISSION_ITEM_REACHED")
    set_return(master)

