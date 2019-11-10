/*
 * RPLidar HW datasheet/reference : http://bucket.download.slamtec.com/b42b54878a603e13c76a0a0500b53595846614c6/LR001_SLAMTEC_rplidar_protocol_v1.1_en.pdf 
 *                                  http://www.slamtec.com/en/Lidar/A1
 * 
 * RPLidar SW reference : http://wiki.ros.org/rplidar 
 *                        https://github.com/Slamtec/rplidar_ros 
 *                        https://github.com/robopeak/rplidar_ros 
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

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

#define RAD2DEG(x) ((x) * 180./M_PI)

void rplidar_RX_Callback(const sensor_msgs::LaserScan::ConstPtr& scan){

    int count = scan->scan_time / scan->time_increment;

    ROS_INFO("Current Scan Time : [%f] / Current Sample Frequency : [%f]", scan->scan_time, 1/scan->scan_time);
    ROS_INFO("Current angle_min : %f / Current angle_increment : %f", scan->angle_min, scan->angle_increment);
    for (int i = 0; i < count; i++){

        float degree = RAD2DEG(scan->angle_min + (scan->angle_increment * i));
        ROS_INFO("[%f : %f m]", degree, scan->ranges[i]);
    }
    ROS_INFO("------------------------------------------------------------");
}

int main(int argc, char** argv){

    ros::init(argc, argv, "process_node");

    ros::NodeHandle nh;

    ros::Subscriber rplidar_subscriber = nh.subscribe("/scan", 1000, rplidar_RX_Callback);

    while(ros::ok()){

        ros::spinOnce();
    }

    return 0;
}