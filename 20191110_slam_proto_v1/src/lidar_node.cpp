/*
 * RPLidar HW datasheet/reference : http://bucket.download.slamtec.com/b42b54878a603e13c76a0a0500b53595846614c6/LR001_SLAMTEC_rplidar_protocol_v1.1_en.pdf 
 *                                  http://www.slamtec.com/en/Lidar/A1
 * 
 * RPLidar SW reference : http://wiki.ros.org/rplidar 
 *                        https://github.com/Slamtec/rplidar_ros 
 *                        https://github.com/robopeak/rplidar_ros 
 * 
 * 2D Laser Scan data reference : http://docs.ros.org/melodic/api/sensor_msgs/html/msg/LaserScan.html 
 * 
 * Applying RPLiar for SLAM : http://www.geduino.org/site/archives/35
 *                            https://www.seeedstudio.com/blog/2018/11/09/rplidar-and-ros-the-best-way-to-learn-robot-and-slam/  
 *                            https://answers.ros.org/question/205594/localization-problem-with-gmapping-and-rplidar/ 
 *                            https://answers.ros.org/question/206185/why-rplidar-gmapping-has-a-bad-result/ 
 * 
 * RPLidar Orientation-related issues/references : https://github.com/robopeak/rplidar_ros/issues/77 
 *                                                 https://github.com/robopeak/rplidar_ros/blob/master/rplidar_A1.png 
 * 
 * RPLidar Installation : "sudo apt-get install ros-<ros-distro>-rplidar-ros" (ex : sudo apt-get install ros-kinetic-rplidar-ros (for ROS Kinetic))
 */

/*
 * This node is for receiving 2D Laser Scan data from "rplidarNode" of "rplidar_ros package"
 * 
 * Therefore, in order to use this node properly, use the following command in order to activate "rplidarNode"
 * 
 * Installation : "sudo apt-get install ros-<ros-distro>-rplidar-ros" (ex : sudo apt-get install ros-kinetic-rplidar-ros (for ROS Kinetic))
 * 
 * Running usb_cam_node : "rosrun rplidar_ros rpliarNode" 
 */

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

#define RAD2DEG(x) ((x) * 180./M_PI)

void lidar_RX_Callback(const sensor_msgs::LaserScan::ConstPtr& scan){

    int count = scan->scan_time / scan->time_increment;

    ROS_INFO("Current Scan Time : [%f] / Current Sample Frequency : [%f]", scan->scan_time, 1/scan->scan_time);
    ROS_INFO("Current angle_min : %f / Current angle_increment : %f", scan->angle_min, scan->angle_increment);
}

int main(int argc, char** argv){

    ros::init(argc, argv, "lidar_node");

    ros::NodeHandle nh;

    ros::Subscriber lidar_subscriber = nh.subscribe("/scan", 1000, lidar_RX_Callback);

    while(ros::ok()){

        ros::spinOnce();
    }

    return 0;
}