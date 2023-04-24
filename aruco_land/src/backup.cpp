
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/Waypoint.h>
#include <mavros_msgs/WaypointPush.h>
#include <mavros_msgs/CommandCode.h>
#include <list>
#include <mavros_msgs/CommandTOL.h>

mavros_msgs::State current_state;

void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
	bool connected = current_state.connected;
	bool armed = current_state.armed;
	ROS_INFO("%s", armed ? "" : "DisArmed");
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "auto_set_mode");
    ros::NodeHandle nh;
    
    mavros_msgs::SetMode auto_set_mode;
    auto_set_mode.request.custom_mode = "GUIDED";
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient takeoff_client = nh.serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/takeoff");
    ros::ServiceClient wp_client = nh.serviceClient<mavros_msgs::WaypointPush>
            ("mavros/mission/push");
    mavros_msgs::WaypointPush wp_push_srv; // List of Waypoints
    mavros_msgs::Waypoint wp;
    /*
        uint8 FRAME_GLOBAL=0
        uint8 FRAME_LOCAL_NED=1
        uint8 FRAME_MISSION=2
        uint8 FRAME_GLOBAL_REL_ALT=3
        uint8 FRAME_LOCAL_ENU=4
        uint8 frame
        uint16 command
        bool is_current
        bool autocontinue
        float32 param1
        float32 param2
        float32 param3
        float32 param4
        float64 x_lat
        float64 y_long
        float64 z_alt
    */
    // WP 0
    wp.frame          = mavros_msgs::Waypoint::FRAME_GLOBAL;
    wp.command        = mavros_msgs::CommandCode::NAV_TAKEOFF;
    wp.is_current     = true;
    wp.autocontinue   = true;
    wp.x_lat          = -35.36322841 ;
    wp.y_long         = 149.16519611;
    wp.z_alt          = 10;
    wp_push_srv.request.waypoints.push_back(wp);
    // WP 1
    // wp.frame          = mavros_msgs::Waypoint::FRAME_GLOBAL;
    // wp.command        = mavros_msgs::CommandCode::NAV_WAYPOINT;
    // wp.is_current     = false;
    // wp.autocontinue   = true;
    // wp.x_lat          = 47.3962527;
    // wp.y_long         = 8.5467917;
    // wp.z_alt          = 20;
	// wp.param1			= 10;
	// wp.param3			= 2;
	// wp.param4			= 1;
    // wp_push_srv.request.waypoints.push_back(wp);
    
    // // WP 2
    // wp.frame          = mavros_msgs::Waypoint::FRAME_GLOBAL;
    // wp.command        = mavros_msgs::CommandCode::NAV_WAYPOINT;
    // wp.is_current     = false;
    // wp.autocontinue   = true;
    // wp.x_lat          = 47.3977783;
    // wp.y_long         = 8.547906;
    // wp.z_alt          = 20;
    // wp_push_srv.request.waypoints.push_back(wp);

    // WP FRAME_GLOBAL
    wp.frame          = mavros_msgs::Waypoint::FRAME_GLOBAL;
    wp.command        = mavros_msgs::CommandCode::NAV_RETURN_TO_LAUNCH;
    wp.is_current     = false;
    wp.autocontinue   = true;
    wp.x_lat          = 0;
    wp.y_long         = 0;
    wp.z_alt          = 0;
    wp_push_srv.request.waypoints.push_back(wp);

    ros::Rate rate(20.0);


    // wait for FCU connection
    while(ros::ok() && current_state.connected){
        // ros::spinOnce();
        // rate.sleep();
    
        ROS_INFO("Connected to PX4!");
    // ARM
        mavros_msgs::SetMode offb_set_mode;
        offb_set_mode.request.custom_mode = "GUIDED";
        set_mode_client.call(offb_set_mode);

        // Arm the UAV
        mavros_msgs::CommandBool arm_cmd;
        arm_cmd.request.value = true;
        arming_client.call(arm_cmd);
         // Wait for the arming to complete
        while (!arm_cmd.response.success)
        {
            // ros::spinOnce();
            ros::Duration(0.01).sleep();
            arming_client.call(arm_cmd);
        }
        
        ROS_INFO("Armed");
        // Send takeoff command
        mavros_msgs::CommandTOL srv_takeoff;
        srv_takeoff.request.altitude = 3.0;
        takeoff_client.call(srv_takeoff);

        // Wait for the takeoff to complete
        while (!srv_takeoff.response.success)
        {
            // ros::spinOnce();
            ros::Duration(0.01).sleep();
            takeoff_client.call(srv_takeoff);
        }

        ROS_INFO("Took Off");

    // return 0;

    // Send WPs to Vehicle
        if (wp_client.call(wp_push_srv)) {
            ROS_INFO("Send waypoints ok: %d", wp_push_srv.response.success);
            if (current_state.mode != "GUIDED") {
                if( set_mode_client.call(auto_set_mode) &&
                    auto_set_mode.response.mode_sent){
                    ROS_INFO("GUIDED enabled");
                }
            }
        }
        else
            ROS_ERROR("Send waypoints FAILED.");


        break;
    // return 0;
    }
    while(ros::ok()){
        ros::spin();
    }
}
