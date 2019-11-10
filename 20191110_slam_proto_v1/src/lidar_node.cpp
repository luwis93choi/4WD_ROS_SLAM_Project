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
#include <sensor_msgs/LaserScan.h>  // Since rplidarNode publishes 2D Laser Scan sensor messages, this node has to become able to handle 2D Laser Scan sensor messages.

#define RAD2DEG(x) ((x) * 180./M_PI)    // Pre-defined function for converting radian to degree

// This callback function will be invoked if the subscriber receives 2D Laser Scan sensor messages.
// This function receives 2D Laser Scan data as sensor messages
void lidar_RX_Callback(const sensor_msgs::LaserScan::ConstPtr& scan){

    /*
     * [sensor_msgs::LaserScan] : Single scan from a planar laser range-finder / Laser-specific application
     * 
     * Header header : timestamp in the header is the acquisition time of the first ray in the scan.
                       in frame frame_id, angles are measured around the positive Z axis (counterclockwise, if Z is up)
                       with zero angle being forward along the x axis
                                
     * float32 angle_min        : start angle of the scan [rad]
     * float32 angle_max        : end angle of the scan [rad]
     * float32 angle_increment  : angular distance between measurements [rad]

     * float32 time_increment   : time between measurements [seconds]
                                  if your scanner is moving, this will be used in interpolating position of 3d points
     * float32 scan_time        : time between scans [seconds]

     *** Total number of data collected in 1 scan = 360 = scan_time / time_increment
         (1) By dividing scan_time with time_increment, we will get 360.
         (2) Sampling Frquency = 1 / scan_time
         (3) This means that RPLidar acquires 360 data for each scan. This also shows that RPLidar collects Laser Scan data on every 1 degree.
         (4) Since total number of collected data is a static value that must not be changed, 
             by adjusting scan_time parameter, we can adjust sampling frequency, time increment, and angle increment. Furthermore, we can adjust Resolution through scan_time pararmeter.
             ==> Lower scan_time = Higher Sampling Frequency --> Lower angle_increment --> Higher Resolution
                                 = Lower time_increment

     * float32 range_min        : minimum range value [m]
     * float32 range_max        : maximum range value [m]

     * float32[] ranges         : range / Lidar Scan data [m] (Note: values < range_min or > range_max should be discarded)
     * float32[] intensities    : intensity data [device-specific units].  If your device does not provide intensities, please leave the array empty.

     */

    int count = scan->scan_time / scan->time_increment; // 360 : Total number of collected data in 1 scan given scan_time and time_increment

    ROS_INFO("Current Scan Time : [%f] / Current Sample Frequency : [%f]", scan->scan_time, 1/scan->scan_time);
    ROS_INFO("Current angle_min : %f / Current angle_increment : %f", scan->angle_min, scan->angle_increment);

    // From 0 to 360 degree...
    for (int i = 0; i < count; i++){

        float degree = RAD2DEG(scan->angle_min + (scan->angle_increment * i));  // Calculate the current degree by adding from the minimum Lidar angle
        ROS_INFO("[%f : %f m]", degree, scan->ranges[i]);   // Access 2D Lidar data through scan->ranges member variable
    }
    ROS_INFO("-----------------------------------------------------------------");
}

int main(int argc, char** argv){

    // Initialize the node with argc and argv, and Name it as "lidar_node"
    ros::init(argc, argv, "lidar_node");

    // Declare node handler
    ros::NodeHandle nh;

    // Declare the subscriber that receives 2D Laser Scan sensor messages
    // When this node receives 2D Laser Scan data from rpliarNode, it will invoke lidar_RX_Callback function.
    ros::Subscriber lidar_subscriber = nh.subscribe("/scan", 1000, lidar_RX_Callback);

    // While roscore is operational, loop will continue on...
    while(ros::ok()){

        ros::spinOnce();    // Handle all the message callbacks
    }

    return 0;
}