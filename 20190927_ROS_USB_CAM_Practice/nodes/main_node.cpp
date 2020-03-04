/*
image_transport reference : http://wiki.ros.org/image_transport
                            https://docs.ros.org/api/image_transport/html/classimage__transport_1_1CameraSubscriber.html

sensor_msgs/Image reference : http://docs.ros.org/melodic/api/sensor_msgs/html/msg/Image.html

Supported data types in ROS message : http://wiki.ros.org/msg

Multiple publishers and subscribers in one node : https://gist.github.com/PrieureDeSion/77c109a074573ce9d13da244e5f82c4d

How to build only one package from catkin_make : https://answers.ros.org/question/54178/how-to-build-just-one-package-using-catkin_make/

How to setup git for Linux : https://yokang90.tistory.com/47
*/

#include <ros/ros.h>
#include <ros_usb_cam_practice/main_node_msg.h>

// usb_cam related libraries /////////////////////////
#include <usb_cam/usb_cam.h>
#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>
#include <sstream>
#include <std_srvs/Empty.h>
//////////////////////////////////////////////////////

// OpenCV related libraries /////////////////////////
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <opencv2/highgui.hpp>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
/////////////////////////////////////////////////////

int image_RX_count = 0;

void imageCallback(const sensor_msgs::ImageConstPtr& msg){

    cv_bridge::CvImagePtr cv_ptr;

    image_RX_count++;
    ROS_INFO("imageCallback call count : %d", image_RX_count);
    ROS_INFO("[Image height : %d] [Image width : %d] [Image encoding : %s] \n", msg->height, msg->width, msg->encoding());

    // Use 'cv_bridge' and its 'toCvCopy' in order to convert ROS msgs Image into OpenCV image
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

    cv::imshow("USB CAM image", cv_ptr->image); // Actual image data is stored in 'image' member
    cv::waitKey(3);
}

int main(int argc, char **argv){

    ros::init(argc, argv, "main_node");

    ros::NodeHandle nh;
    
    ros::Publisher main_node_pub = nh.advertise<ros_usb_cam_practice::main_node_msg>("image_RX_count", 10);
    ros_usb_cam_practice::main_node_msg node_msg;
    ros::Rate loop_rate(10);

    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe("/usb_cam/image_raw", 1000, imageCallback);

    ros::spinOnce();

    while(ros::ok()){

        node_msg.image_rx_count = image_RX_count;

        main_node_pub.publish(node_msg);

        ros::spinOnce();

        loop_rate.sleep();
    }

    cv::destroyWindow("USB CAM image");

    return 0;
}