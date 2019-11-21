/*
 * ROS Transformation reference : http://wiki.ros.org/tf/Tutorials 
 *                                http://wiki.ros.org/tf 
 *                                http://wiki.ros.org/navigation/Tutorials/RobotSetup/TF
 *                                https://web.ics.purdue.edu/~rvoyles/Classes/ROSprogramming/Lectures/TF%20(transform)%20in%20ROS.pdf
 *                                https://subinlab.github.io/ros/2019/04/04/ROS-tf.html  
 *
 * RPLidar - Gmapping : http://wiki.ros.org/gmapping 
 *                      https://www.seeedstudio.com/blog/2018/11/09/rplidar-and-ros-the-best-way-to-learn-robot-and-slam/ 
 *
 * Setting up Gmapping results for Rviz : https://answers.ros.org/question/306948/gmapping-error-no-map-received/ 
 *                                        https://answers.ros.org/question/209276/rviz-slam-rplidar-problem/ 
 *                                        https://answers.ros.org/question/225048/slam-gmapping-real-time-on-rviz/ 
 *                                        https://answers.ros.org/question/117396/how-to-use-gmapping-to-build-a-map/ 
 */

/*
 * ROS-based systems need following development requirements fulfilled.
 * 
 * 1. Define nodes of various functions and devices (sensors and motors) for core SW operations
 * 2. Define TF tree in order to define how the system is built and designed in axis-based grids
 * 3. Bind nodes of various devices to TF tree in order to finalize the model design of ROS-based systems
 *
 * In order to properly build ROS-based system and run gmapping SLAM, we have to define TF frames of the vehicle and sensors. (* frame : a node in TF tree)
 * We have to build TF tree that includes base_link (basic TF frame of vehicle itself), odom (odometry TF frame), map (map TF frame), and base_laser (basic TF frame of Lidar).
 * Also, we have to organize TF tree so that it is compatible with gmapping TF requirements and the requirements of other 3rd party packages.
 *
 * To define and build TF tree, we construct transform_broadcaster that defines/publishes/broadcasts how TF frames are organized in the tree. 
 * 
 * Since transform_broadcaster is originated from 'tf' library, we have to add this in the code, CMake, and package.xml.
 */

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>   // Adding 'transform_broadcaster' header from 'tf' library
                                        // Since 'tf' library is needed, tf is also added on CMake and package.xml

int main(int argc, char** argv){

    ros::init(argc, argv, "robot_tf_publisher");    // Initialize the node as 'robot_tf_publisher'
    ros::NodeHandle nh;

    ros::Rate rate(100);

    tf::TransformBroadcaster broadcaster;   // Define TransformBroadcaster 
                                            // --> This broadcaster will define/publish TF tree to ROS system
                                            // --> By broadcasting TF tree, we can announce how HW and ROS SW cores are bound in the system.
    while(nh.ok()){

        /* send Transform requires following arguments
         * 
         * [tf::StampedTransform]
         * 1. Rotation between frames : Define how 2 frames are located in angluar perspective
         *                              Utilize Quaternion to express angular relationship between frames
         *
         * 2. Translation between frames : Define how 2 frames are apart from each other in x, y, z axis
         *                                 Utilize tf::Vector3 in order to express xyz distance between frames
         *
         * [Time]
         * 3. Time of transformation : Define time of transformation data reception
         *                             *** Since transformation requires extensive calculation, 
         *                                 it is impossible to immediately receive transformation results.
         *                             *** ROS often holds the operation until transformation data is available for use. 
         *
         * [Target Frames]
         * 4, 5. Child Frame ID, Parent Frame ID of transformation : Parent Frame ID --> Child Frame ID  
         */ 
        broadcaster.sendTransform(
            tf::StampedTransform(
                tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.1, 0.0, 0.2)),
                ros::Time::now(), "base_link", "base_laser"));  
        // Linking base_link and base_laser frames for transformation (base_laser --> base_link)

        // *** Since Lidar frame id is declared as "base_laser", it is essential that rplidar frame_id or Lidar frame_id is declared as the same "base_laser" at the launch file. ***
        // *** By matching frame id of Lidar scan data, we can bind Lidar node to actual Lidar HW. ***
        
        broadcaster.sendTransform(
            tf::StampedTransform(
                tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.0, 0.0, 0.0)), 
                ros::Time::now(), "odom", "base_link"));
        // Linking odom and base_link frames for transformation (base_link --> odom)

        rate.sleep();
    }
}