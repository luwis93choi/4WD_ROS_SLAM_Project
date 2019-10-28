#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

#define RAD2DEG(x) ((x) * 180./M_PI)

void rplidar_RX_Callback(const sensor_msgs::LaserScan::ConstPtr& scan){

    int count = scan->scan_time / scan->time_increment;

    ROS_INFO("Current Scan Time : [%f] / Current Sample Frequency : [%f]", scan->scan_time, 1/scan->scan_time);
    ROS_INFO("Current angle_min : %f / Current angle_increment : %f", scan->angle_min, scan->angle_increment);
    for (int i = 0; i < count/2; i++){

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