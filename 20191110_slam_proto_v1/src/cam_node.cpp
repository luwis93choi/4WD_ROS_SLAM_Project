#include <ros/ros.h>

#include <usb_cam/usb_cam.h>
#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>
#include <sstream>
#include <std_srvs/Empty.h>
#include <string.h>

#include <cv_brige/cv_brige.h>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>

int image_RX_count;

void imageCallback(const sensor_msgs::ImageConstPtr& msg){

    cv_bridge::CvImagePtr cv_ptr;

    image_RX_count++;

    ROS_INFO("imageCallback call count : %d", image_RX_count);
    ROS_INFO("[Image height : %d] [Image width : %d] [Image encoding : %s] \n", msg->height, msg->widht, msg->ending());

    // Use 'cv_bridge' and its 'toCvCopy' in order to convert ROS msgs Image into OpenCV Image
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

    cv::imshow("USB_CAM image", cv_ptr->image); // Actual image data is stored in member variable 'image' of CvImagePtr
    cv::waitKey(3);
}

int main(int argc, char** argv){

    // Initialize the node with argc and argv, and Name it as "cam_node"
    ros::init(argc, argv, "cam_node");

    // Declare node handler
    ros::NodeHandle nh;

    // Declare loop rate 10Hz 
    // --> This Rate class will do the best to keep the loop at 10Hz by accounting for the time used by the work done during the loop
    ros::Rate loop_rate(10);

    // In order to transport image data between nodes, we utilize "ImageTransport".
    // ImageTransport class will deliver images between nodes as image sensor messages.
    image_transport::ImageTransport it(nh);

    // Through ImageTransport, image data is delivered as messages.
    // As a result, in order for the node to receive the image data, it has to subscribe the image just as it does for receiving standard messages from other nodes.
    image_transport::Subscriber sub = it.subscribe("/usb_cam/image_raw", 1000, imageCallback);

    
    ros::spinOnce();

    while(ros::ok()){

        ros::spinOnce();

        loop_rate.sleep();            
    }

    cv::destroyWindow("USB CAM image");

    return 0;
}