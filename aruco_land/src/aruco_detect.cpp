#include <opencv2/aruco.hpp>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <vector>


 cv::Mat img;
 cv_bridge::CvImage bridge;

void img_callback(const sensor_msgs::ImageConstPtr &msg){
    
    

    try{
        bridge = cv_bridge::CvImage(msg->header, sensor_msgs::image_encodings::RGB8, img);
    }
    catch(...){
        ROS_ERROR("No Image Found");
    }
}

int main(int argc, char** argv){
    ros::init(argc, argv,"Aruco_detection");

     cv::Mat img_drawn;

    ros::NodeHandle nh;
    cv::Ptr<cv::aruco::Dictionary> dictionary;

    ros::Subscriber image = nh.subscribe<sensor_msgs::Image>("image_raw",10, img_callback);

    ros::Publisher img_pub = nh.advertise<sensor_msgs::Image>("aruco_detected", 1000);
                
    // Declare the vectors that would contain the detected marker corners and the rejected marker candidates
    std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;

    // The ids of the detected markers are stored in a vector
    std::vector<int> markerIds;
                
    // Detect the markers in the image

    while (ros::ok()){
        cv::aruco::detectMarkers(img, dictionary,markerCorners, markerIds);
        cv::aruco::drawDetectedMarkers(img, markerCorners);

        sensor_msgs::Image img_;

        
        
    }

    

}