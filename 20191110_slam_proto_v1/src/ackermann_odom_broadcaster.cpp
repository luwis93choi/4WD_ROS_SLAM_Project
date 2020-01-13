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

#define TEST 0
int VEHICLE_STATUS = 0;

// Odometry Variables //////////////////////////////////////////////

ros::Time current_time;
ros::Time last_time;

double x = 0;               // [m]
double y = 0;               // [m]
double theta = 0;           // [radian]

double delta_x = 0;         // [m]
double delta_y = 0;         // [m]
double delta_theta = 0;     // [sec]

double delta_time = 0;

double wheel_velocity = 0;  // [m/s]
double steering_angle = 0;  // [radian]

geometry_msgs::TransformStamped odom_trans;
// *** Change this value according to the body lenght of car-like robot *** //
double body_length = 0.25;  // 0.25m [m]

int encoder_target_val = 2600;
int one_revolution_encoder_count = 2600;
double wheel_radius = 0.0325;        // 0.0325[m]
double wheel_circumference = 2 * wheel_radius * M_PI * ((double)encoder_target_val / (double)one_revolution_encoder_count); // [m]
////////////////////////////////////////////////////////////////////////////////

// Velocity Calculation Variables //////////////////////////////////////////////
ros::Time vel_current_time;     // [sec]
ros::Time vel_target_time;      // [sec]
double vel_delta_time = 0.0;    // [sec]
int log_flag = 0;

int calc_flag = 0;

int prev_encoder_count = 0;
////////////////////////////////////////////////////////////////////////////////


// Steering Angle Calculation Variables //////////////////////////////////////////////
double gravitional_accel = 9.728;   // [m/s^2]
double IMU_inclination_angle = 0;   // [radian]

double tilt_angle_on_gravity_axis = 0;

geometry_msgs::Quaternion odom_quat;

tf2::Quaternion prev_quat_tf;

int ref_flag = 0;
////////////////////////////////////////////////////////////////////////////////
/*
void wheel_velocity_calcCB(const std_msgs::Int32& msg){

    if(calc_flag == 1){

        // wheel_velocity calculation based on encoder count value
        if(prev_encoder_count == msg.data){

            VEHICLE_STATUS = 2;

            log_flag = 0;
            vel_current_time = ros::Time::now();
            vel_target_time = vel_target_time;
            vel_delta_time = 0;
            wheel_velocity = 0;
        }
        else if((msg.data >= encoder_target_val) && (log_flag == 0)){
        
            VEHICLE_STATUS = 1;

            vel_current_time = ros::Time::now();
            log_flag = 1;
        }
        else if((msg.data >= encoder_target_val) && (log_flag == 1)){

            VEHICLE_STATUS = 1;

            vel_target_time = ros::Time::now();

            vel_delta_time = (vel_target_time - vel_current_time).toSec();

            wheel_velocity = (wheel_circumference / vel_delta_time);

            log_flag = 0;
        }

        prev_encoder_count = msg.data;
    }
    else if(calc_flag == 0){

        VEHICLE_STATUS = 0;

        log_flag = 0;
        vel_current_time = ros::Time::now();
        vel_target_time = vel_target_time;
        vel_delta_time = 0;
        wheel_velocity = 0;
    }
}

void steering_angle_calcCB(const sensor_msgs::Imu msg){

    tf2::Quaternion current_quat_tf;
    tf2::Quaternion quat_relative_rotation;

    tf2::convert(msg.orientation, current_quat_tf);

    if(ref_flag == 0){

        tf2::convert(msg.orientation, prev_quat_tf);

        ref_flag = 1;
    }

    // steering_angle calculation based on Realsense IMU
    quat_relative_rotation = current_quat_tf * (prev_quat_tf.inverse());

    quat_relative_rotation.normalize();

    odom_quat = tf2::toMsg(quat_relative_rotation);
}
*/

using namespace message_filters;

void odom_calcCB(const slam_proto_v1::DC_velocityConstPtr& enc_msg, const sensor_msgs::ImuConstPtr& imu_msg){

    wheel_velocity = enc_msg->enc_count;

    //////////////////////////////////////////////////////////////////////////////////

    tf2::Quaternion current_quat_tf;
    tf2::Quaternion quat_relative_rotation;

    tf2::convert(imu_msg->orientation, current_quat_tf);

    if(ref_flag == 0){
        tf2::convert(imu_msg->orientation, prev_quat_tf);
    }
    // steering_angle calculation based on Realsense IMU
    quat_relative_rotation = current_quat_tf * (prev_quat_tf.inverse());

    quat_relative_rotation.normalize();

    odom_quat = tf2::toMsg(quat_relative_rotation);

    current_time = ros::Time::now();
    
    //////////////////////////////////////////////////////////////////////////////////

    // delta_time calculation
    delta_time = (current_time - last_time).toSec();

    // Ackermann Steering Odometry Model
    //theta = theta + ((wheel_velocity/body_length) * tan(steering_angle));
    //theta = odom_quat.z / (sqrt(1 - pow(odom_quat.w, 2)));
    theta = atan2(2 * (odom_quat.w * odom_quat.z + odom_quat.x * odom_quat.y), 1 - 2 * (odom_quat.y * odom_quat.y + odom_quat.z * odom_quat.z));
    x = x + (wheel_velocity * cos(theta) * delta_time);
    y = y + (wheel_velocity * sin(theta) * delta_time);

    ROS_INFO("Velocity : %f / x : %f / y : %f / theta : %f", wheel_velocity, x, y, theta);

    // *******************************************************************************//
    // 6DOF Odometry (*** Change this according to Ackermann Model ***)
    //geometry_msgs::Quaternion odom_quater = tf::createQuaternionMsgFromYaw(theta);

    // odom - base_link transformation delcaration
    // geometry_msgs::TransformStamped odom_trans;
    
    // Update the latest time
    last_time = current_time;

    ref_flag = 1;
}

int main(int argc, char** argv){

    ros::init(argc, argv, "ackermann_odom");
    ros::NodeHandle nh;

    // *******************************************************************************//
    // Odometry TF Publisher 
    ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 50);
    tf::TransformBroadcaster odom_broadcaster;
    tf::TransformBroadcaster odom_broadcaster_1;

    nh.setParam("body_length", body_length);
    // *******************************************************************************//

    // *******************************************************************************//
    // Wheel Velocity Calculation
    
    message_filters::Subscriber<slam_proto_v1::DC_velocity> encoder_sub(nh, "/stamped_enc", 1);
    message_filters::Subscriber<sensor_msgs::Imu> imu_sub(nh, "/imu/data", 1);

    typedef sync_policies::ApproximateTime<slam_proto_v1::DC_velocity, sensor_msgs::Imu> odom_sync_policy;

    Synchronizer<odom_sync_policy> sync(odom_sync_policy(10), encoder_sub, imu_sub);
    sync.registerCallback(boost::bind(&odom_calcCB, _1, _2));
    
    //ros::Subscriber wheel_velocity_calc = nh.subscribe("encoder", 1000, wheel_velocity_calcCB);
    
    current_time = ros::Time::now();
    last_time = ros::Time::now();

    vel_current_time = ros::Time::now();
    vel_target_time = ros::Time::now();
    // ros::Subscriber velocity_dir_command = nh.subscribe("dc_direction_ctrl_value", 100, velocity_ctrlCB); // Used to detect whether the vehicle has stopped
    // ros::Subscriber velocity_speed_command = nh.subscribe("dc_speed_ctrl", 100, velocity_ctrlCB);   // Used to detect whether the vehicle has stopped
    // *******************************************************************************//
    
    // *******************************************************************************//
    // Steering Angle Calculation
    //ros::Subscriber steering_angle_calc = nh.subscribe("/imu/data", 100, steering_angle_calcCB);

    prev_quat_tf.setRPY(0, 0, 0);
    prev_quat_tf.normalize();

    // *******************************************************************************//

    ros::Rate loop_rate(40);   // This node will run at 100Hz
    // In order to conduct high-speed calculation for Odometry, we have to set loop rate at high Hz

    int reday_log = 0;

    while(ros::ok()){

        nh.getParam("body_length", body_length);
        
        nh.getParam("dc_direction_ctrl_value", calc_flag);

        if(VEHICLE_STATUS == 0){

            //ROS_INFO("Vehicle Stopped / Initializing velocity variables");
        }
        else if(VEHICLE_STATUS == 1){

            //ROS_INFO("Moving / Current Speed : %f m/s", wheel_velocity);
        }
        else if(VEHICLE_STATUS == 2){

            //ROS_INFO("Vehicle Stopped during motion / Initializing velocity variables");
        }

        // *******************************************************************************//
        
        if(ref_flag == 1){
            odom_trans.header.stamp = ros::Time::now();
            odom_trans.header.frame_id = "odom";
            odom_trans.child_frame_id = "base_link";

            odom_trans.transform.translation.x = x;
            odom_trans.transform.translation.y = y;
            odom_trans.transform.translation.z = 0.0;
            odom_trans.transform.rotation = odom_quat;
        }
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
        }

        // Send odom - base_link transformation
        odom_broadcaster.sendTransform(odom_trans);

        // *******************************************************************************//
        // LiDAR - base_link transformation declaration
        odom_broadcaster_1.sendTransform(
            tf::StampedTransform(
                tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.1, 0.0, 0.2)),
                ros::Time::now(), "base_link", "base_laser"));

        // *******************************************************************************//


        // Publish odometry message
        //nav_msgs::Odometry odom;
        
        //odom.header.stamp = current_time;
        //odom.header.frame_id = "odom";

        // Set position values of the robot
        //odom.pose.pose.position.x = x;
        //odom.pose.pose.position.y = y;
        //odom.pose.pose.position.z = 0.0;
        //odom.pose.pose.orientation = odom_quat;

        // Set velocity values of the robot
        //odom.child_frame_id = "base_link";
        //odom.twist.twist.linear.x = wheel_velocity * cos(theta);
        //odom.twist.twist.linear.y = wheel_velocity * sin(theta);
        //odom.twist.twist.linear.z = (wheel_velocity/body_length) * tan(steering_angle);

        // Publish odometry message
        //odom_pub.publish(odom);

        ros::spinOnce();

        loop_rate.sleep();
    }
}
