#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Int32.h>
#include <sensor_msgs/Imu.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include "slam_proto_v1/DC_velocity.h"

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>

using namespace message_filters;

int VEHICLE_STATUS = 0;

// Odometry Variables //////////////////////////////////////////////
ros::Time current_time;     // [sec] : Epoch Linux Time
ros::Time last_time;        // [sec] : Epoch Linux Time

double x = 0;               // [m]
double y = 0;               // [m]
double theta = 0;           // [radian]

double delta_x = 0;         // [m]
double delta_y = 0;         // [m]

double delta_time = 0;      // [sec]

double wheel_velocity = 0;  // [m/s]

geometry_msgs::TransformStamped odom_trans;
// *** Change this value according to the body lenght of car-like robot *** //
double body_length = 0.25;  // 0.25m [m]

int encoder_target_val = 2600;
int one_revolution_encoder_count = 2600;
double wheel_radius = 0.0325;        // 0.0325[m]
double wheel_circumference = 2 * wheel_radius * M_PI * ((double)encoder_target_val / (double)one_revolution_encoder_count); // [m]
////////////////////////////////////////////////////////////////////////////////

// Heading Angle Calculation Variables ///////////////////////////////////////
/*
 * In SLAM RC Car Proto V1, IMU is installed at the front center point of the vehicle.
 * Traditionally, the heading of 4WD vehicle with Ackermann steering is originated from the vehicle's steering angle.
 * However, the heading of 4WD with Ackermann steering is measured at the front center point of the vehicle.
 * In order to acquire the heading of 4WD vehicle, instead of trying to gain steering angle, 
 * this project directly calculates the vehicle heading from IMU insatlled at the front center point
 */

double gravitional_accel = 9.728;   // [m/s^2]   Gravitional Accleration

geometry_msgs::Quaternion odom_quat;    // Quaterninon that will be applied to odometry TF

tf2::Quaternion ref_quat_tf;    // Reference Axis Quaternion created by initial IMU values

int ref_flag = 0;   // Reference flag value that is used to notify whether IMU is ready and Reference Axis Quaterninon is formed.
//////////////////////////////////////////////////////

// Path Dispaly //////////////////////////////////////
nav_msgs::Path path;    // Navigation Message used for Rviz Path Visualization

geometry_msgs::PoseStamped pose;    // Timestamped pose used for Rviz Path Visualization
/////////////////////////////////////////////////////////
/*
 * MessageFilter has been implemented in order to synchronize wheel velocity values and IMU sensor values.
 * It is necessary to synchronize wheel velocity values and IMU sensor values, because odometry has to produce the pose of the robot from sensor values of same timestamps.
 * ROS MessageFilter API is used to fuse various sensor messages and process them in sychronized time according to sychronization policy defined by the developer.
 * In future works, it is imperative to produe timestamped-ROS messages in order to fuse the data for complicated operations.
 */
void odom_calcCB(const slam_proto_v1::DC_velocityConstPtr& enc_msg, const sensor_msgs::ImuConstPtr& imu_msg){

    // Timestamped Wheel Velocity value from MCU //////////////////////////////////////////////////////
    wheel_velocity = enc_msg->enc_count;    // Directly acquire timestamped velocity value from serial_RX_vel_stamp node
    ///////////////////////////////////////////////////////////////////////////////////////////////////

    /// Vehicle heading calcaultion through Rotational Difference from IMU at front center point /////
    /* 
     * Since there is Realsense IMU installed at the front center point, instead of steering sensor,
     * there is no need to acquire steering angle in order to calculate the heading of the vehicle.
     * Heading of the vehicle can be calculated from rotational difference between Reference Axis Quaternion created by IMU and current IMU Quaternion 
     */ 

    tf2::Quaternion current_quat_tf;        // Current IMU Quaternion ---> This can be regarded as current orientation of the vehicle based from gravitional axis
    tf2::Quaternion quat_relative_rotation; // Rotational Difference Quaterninon between reference axis and current IMU

    if(ref_flag == 0){

        tf2::convert(imu_msg->orientation, ref_quat_tf);    // Convert current IMU sensor value (linear velocity & angular velocity) into Quaternion
    }

    tf2::convert(imu_msg->orientation, current_quat_tf);    // Convert current IMU sensor value (linear velocity & angular velocity) into Quaternion

    // Heading angle calculation using Rotational Difference between Quaternions and Realsense IMU
    quat_relative_rotation = current_quat_tf * (ref_quat_tf.inverse()); // Multiply inverse matrix of Reference Axis Quaterninon with current IMU Quaternion
                                                                        // in order to acquire rotational difference between each IMU message

    quat_relative_rotation.normalize(); // Normalize rotational difference quaterninon in order to make them eligible for Euler Angle Conversion

    odom_quat = tf2::toMsg(quat_relative_rotation); // Convert Rotational Difference Quaterninon into geomtery_msgs::Quaternino for odomtery TF - geometry_msgs::TransformStamped

    current_time = ros::Time::now();    // Acquire current time of Callback in Epoch Linux time
    /////////////////////////////////////////////////////////////////////////////////////////////////


    /// Odometry Calculation for 4WD Vehicle with Ackermann Steering /////

    // delta_time calculation
    delta_time = (current_time - last_time).toSec();

    // Ackermann Steering Odometry Model
    /*
     * Traditionally, Ackermann Steering Odometry Model has following formulas
     * - theta = (wheel_velocity / body_length) * tan(steering_angle) + prev_theta
     * - x = wheel_velocity * cos(theta) * delta_time + prev_x
     * - y = wheel_velocity * sin(theta) * delta_time + prev_y
     *
     * However, since there is no mechanical steering sensor on RC Car and IMU is installed at the front center point,
     * Yaw value from Rotational Difference Quaternion can be used as the heading of the vehicle.
     *
     * From ROS Quaternion (x, y, z, w), Roll, Pitch, and Yaw can be acquired through following formulas.
     * (*** ROS Quaternion (x, y, z, w) is different from original Quaterninon form, which is (w, x, y, z) ***)
     * (*** In actual C++ implementation, atan2 is used in order to express the entire range of angular radian values ***)
     * - Roll = arctan (2 * (w * x + y * z) / (1 - 2 * (x^2 + y^2))) =  atan2 (2 * (w * x + y * z) / (1 - 2 * (x^2 + y^2)))
     * - Pitch = asin(2 * (w * y - z * x))
     * - Yaw = arctan (2 * (w * z + x * y) / (1 - 2 * (y ^ 2 + z ^ 3))) = atan2 (2 * (w * z + x * y) / (1 - 2 * (y ^ 2 + z ^ 3)))
     */

    theta = atan2(2 * (odom_quat.w * odom_quat.z + odom_quat.x * odom_quat.y), 1 - 2 * (odom_quat.y * odom_quat.y + odom_quat.z * odom_quat.z));
    // Calcaulte heading of the vehicle from Yaw value drawn from Rotaional Difference Quaternion

    x = x + (wheel_velocity * cos(theta) * delta_time); // Translation on X axis over time ---> Discrete Integration of Wheel Velocity over time
    y = y + (wheel_velocity * sin(theta) * delta_time); // Translation on Y axis over time ---> Discrete Integration of Wheel Velocity over time

    ROS_INFO("Velocity : %f / x : %f / y : %f / theta : %f", wheel_velocity, x, y, theta);
    
    last_time = current_time;  // Update the latest time

    ref_flag = 1;   // If Reference Axis Quaternion is formed from initial IMU values, all Odometry sensors are ready.
}

int main(int argc, char** argv){

    ros::init(argc, argv, "ackermann_odom");
    ros::NodeHandle nh;

    // Odometry TF Publisher //////////////////////////////////////////
    tf::TransformBroadcaster odom_broadcaster;          // TF broadcaster for odom -> base_link
    tf::TransformBroadcaster odom_broadcaster_base;     // TF broadcaster for base_link -> base_laser
    ///////////////////////////////////////////////////////////////////
    
    // Message Filter for fusing Timestamped Velocity data and IMU data
    message_filters::Subscriber<slam_proto_v1::DC_velocity> stamped_vel_sub(nh, "/stamped_vel", 1); // Add '/stamped_vel' message from 'serial_RX_vel_stamp' for fusing
    message_filters::Subscriber<sensor_msgs::Imu> imu_sub(nh, "/imu/data", 1);                      // Add '/imu/data' message from 'madgwick filter' for fusing
  
    // Declare synchronization policy of message filter as "Apporixmate Time" synchronization policy
    typedef sync_policies::ApproximateTime<slam_proto_v1::DC_velocity, sensor_msgs::Imu> odom_sync_policy;

    // Synchronization Object that synchronize subscribers with Approximate Time synchronization policty
    Synchronizer<odom_sync_policy> sync(odom_sync_policy(10), stamped_vel_sub, imu_sub);

    // Register Callbacks to synchronization object
    sync.registerCallback(boost::bind(&odom_calcCB, _1, _2));
    ///////////////////////////////////////////////////////////////////
    
    current_time = ros::Time::now();    // Initial declaration for current time 
    last_time = ros::Time::now();       // Initial declaration for last_time

    ref_quat_tf.setRPY(0, 0, 0);    // Initial declaration for Reference Axis Quaternion
    ref_quat_tf.normalize();        // Initial normalization for Reference Axis Quaternion

    // *******************************************************************************//

    // Path Update Publisher ///////////////////////////////////////////
    ros::Publisher path_pub = nh.advertise<nav_msgs::Path>("odom_path", 10);
    ///////////////////////////////////////////////////////////////////

    nh.setParam("body_length", body_length);

    ros::Rate loop_rate(40);   // This node will run at 40Hz
    // In order to conduct high-speed calculation for Odometry, we have to set loop rate at high Hz

    int reday_log = 0;

    while(ros::ok()){

        nh.getParam("body_length", body_length);

        if(VEHICLE_STATUS == 0){

            //ROS_INFO("Vehicle Stopped / Initializing velocity variables");
        }
        else if(VEHICLE_STATUS == 1){

            //ROS_INFO("Moving / Current Speed : %f m/s", wheel_velocity);
        }
        else if(VEHICLE_STATUS == 2){

            //ROS_INFO("Vehicle Stopped during motion / Initializing velocity variables");
        }
        
        // TF update for Odometry and Path Visualization //////////////////////////////////
        // If all Odometry sensors are ready, update Odometry status
        if(ref_flag == 1){

            // Odometry Update ////////////////////////////
            odom_trans.header.stamp = ros::Time::now(); // Timestamp update with latest time
            odom_trans.header.frame_id = "odom";        // Parent TF Node
            odom_trans.child_frame_id = "base_link";    // Child TF Node

            odom_trans.transform.translation.x = x;     // Update X axis translation value
            odom_trans.transform.translation.y = y;     // Update Y axis translation value
            odom_trans.transform.translation.z = 0.0;   // Set Z as 0 since RC Car does not move Z axis
            odom_trans.transform.rotation = odom_quat;  // Update Orientataion of RC Car with heading value acquired from Rotational Difference Quaternion
            ///////////////////////////////////////////////

            // Path Update from Odometry TF ///////////////
            pose.header.stamp = odom_trans.header.stamp;    
            pose.header.frame_id = "odom_path";
            pose.pose.position.x = odom_trans.transform.translation.x;
            pose.pose.position.y = odom_trans.transform.translation.y;
            pose.pose.position.z = odom_trans.transform.translation.z;
            pose.pose.orientation = odom_trans.transform.rotation;
            ///////////////////////////////////////////////
        }
        // If Odometry sensors are not ready, set Odometry value as default values
        else{

            odom_trans.header.stamp = ros::Time::now();
            odom_trans.header.frame_id = "odom";
            odom_trans.child_frame_id = "base_link";

            odom_trans.transform.translation.x = x;
            odom_trans.transform.translation.y = y;
            odom_trans.transform.translation.z = 0.0;
            odom_trans.transform.rotation.x = 0;
            odom_trans.transform.rotation.y = 0;
            odom_trans.transform.rotation.z = 0;
            odom_trans.transform.rotation.w = 1;

            // Path Update from Odometry TF ///////////////
            pose.header.stamp = odom_trans.header.stamp;
            pose.header.frame_id = "odom_path";
            pose.pose.position.x = odom_trans.transform.translation.x;
            pose.pose.position.y = odom_trans.transform.translation.y;
            pose.pose.position.z = odom_trans.transform.translation.z;
            pose.pose.orientation = odom_trans.transform.rotation;
            ///////////////////////////////////////////////
        }
        //////////////////////////////////////////////////////////////////////////////////////

        // Send odom - base_link transformation /////////////////////////////////////////////
        odom_broadcaster.sendTransform(odom_trans); // Publish odoemtry TF values

        // LiDAR - base_link transformation declaration /////////////////////////////////////
        odom_broadcaster_base.sendTransform(
            tf::StampedTransform(
                tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.1, 0.0, 0.2)),
                ros::Time::now(), "base_link", "base_laser"));
        //////////////////////////////////////////////////////////////////////////////////////


        // Path Update //////////////////////////////////////////////////////////////////////
        path.header.stamp = odom_trans.header.stamp;
        path.header.frame_id = odom_trans.header.frame_id;
        path.poses.push_back(pose); // Add latest pose values to dynamic array of Path

	    path_pub.publish(path);
        //////////////////////////////////////////////////////////////////////////////////////

        ros::spinOnce();    // Wait for all Callbacks to return

        loop_rate.sleep();  // Make sure that this node runs at a specific Hz
    }
}
