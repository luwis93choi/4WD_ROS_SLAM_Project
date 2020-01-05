#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Int32.h>
#include <sensor_msgs/Imu.h>

// Odometry Variables //////////////////////////////////////////////
double x = 0;               // [m]
double y = 0;               // [m]
double theta = 0;           // [radian]

double delta_x = 0;         // [m]
double delta_y = 0;         // [m]
double delta_theta = 0;     // [sec]

double delta_time = 0;

double wheel_velocity = 0;  // [m/s]
double steering_angle = 0;  // [radian]

// *** Change this value according to the body lenght of car-like robot *** //
double body_length = 0.25;  // 0.25m [m]
double wheel_circumference = 0.2041; // 0.2041m [m]
////////////////////////////////////////////////////////////////////////////////

// Velocity Calculation Variables //////////////////////////////////////////////
ros::Time vel_current_time;     // [sec]
ros::Time vel_target_time;      // [sec]
double vel_delta_time = 0.0;    // [sec]
int log_flag = 0;

bool calc_flag = false;
////////////////////////////////////////////////////////////////////////////////


// Steering Angle Calculation Variables //////////////////////////////////////////////
double gravitional_accel = 9.728;   // [m/s^2]
double IMU_inclination_angle = 0;   // [radian]

double tilt_angle_on_gravity_axis = 0;
////////////////////////////////////////////////////////////////////////////////

void wheel_velocity_calcCB(const std_msgs::Int32& msg){

    if(calc_flag == true){

        // wheel_velocity calculation based on encoder count value
        if((msg.data >= 2600) && (log_flag == 0)){
        
            vel_current_time = ros::Time::now();
            log_flag = 1;
        }
        else if((msg.data >= 2600) && (log_flag == 1)){

            vel_target_time = ros::Time::now();

            vel_delta_time = (vel_target_time - vel_current_time).toSec();

            wheel_velocity = (wheel_circumference / vel_delta_time);

            ROS_INFO("Current Speed : %f m/s", wheel_velocity);

            log_flag = 0;
        }
    }
    else if(calc_flag == false){

        ROS_INFO("Vehicle Stopped / Initializing velocity variables");
        log_flag = 0;
        vel_current_time = ros::Time::now();
        vel_target_time = vel_target_time;
        vel_delta_time = 0;
        wheel_velocity = 0;
    }
}

void velocity_ctrlCB(const std_msgs::Int32& msg){

    if(msg.data == 0){
        calc_flag = false;
    }
    else{
        calc_flag = true;
    }
}

void steering_angle_calcCB(const sensor_msgs::Imu msg){

    // steering_angle calculation based on Realsense IMU
    double quat_x = msg.orientation.x;
    double quat_y = msg.orientation.y;
    double quat_z = msg.orientation.z;
    double quat_w = msg.orientation.w;

    ROS_INFO("Quaternion orientation.x : %f", quat_x);
    ROS_INFO("Quaternion orientation.y : %f", quat_y);
    ROS_INFO("Quaternion orientation.z : %f", quat_z);
    ROS_INFO("Quaternion orientation.w : %f \n", quat_w);
/*
    double linear_accel_x = msg.linear_acceleration.x;
    double linear_accel_y = msg.linear_acceleration.y;
    double linear_accel_z = msg.linear_acceleration.z;

    double incline_calc_input = sqrt( (pow(linear_accel_y, 2) + pow(linear_accel_z, 2)) / pow(gravitional_accel, 2) );

    if(incline_calc_input > 1){

        incline_calc_input = 1; 
        IMU_inclination_angle = acos(incline_calc_input);
    }
    else{

        IMU_inclination_angle = acos(incline_calc_input);
    }

    ROS_INFO("Current IMU inclination angle : %f", IMU_inclination_angle);

    tilt_angle_on_gravity_axis = atan( sqrt( pow(linear_accel_y, 2) + pow(linear_accel_z, 2) ) / linear_accel_z );

    ROS_INFO("Current Tilt Angle on Gravity Axis : %f", tilt_angle_on_gravity_axis);

    double steering_angle_input = ( (-1 * linear_accel_y * linear_accel_z) - (linear_accel_z * ( linear_accel_y * sin(IMU_inclination_angle) - gravitional_accel * cos(IMU_inclination_angle) - linear_accel_x * cos(IMU_inclination_angle) )) / 
                                    linear_accel_y * (linear_accel_y * sin(IMU_inclination_angle) - gravitional_accel * cos(IMU_inclination_angle) - linear_accel_x * cos(IMU_inclination_angle)) - linear_accel_z * linear_accel_z);

    steering_angle = atan(steering_angle_input);

    ROS_INFO("Current steering_angle : %f \n", steering_angle);
*/
}

int main(int argc, char** argv){

    ros::init(argc, argv, "ackermann_odom");
    ros::NodeHandle nh;

    // *******************************************************************************//
    // Odometry TF Publisher 
    ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 50);
    tf::TransformBroadcaster odom_broadcaster;

    nh.setParam("body_length", body_length);
    // *******************************************************************************//

    // *******************************************************************************//
    // Wheel Velocity Calculation
    ros::Subscriber wheel_velocity_calc = nh.subscribe("encoder", 1000, wheel_velocity_calcCB);
    
    ros::Time current_time, last_time;
    current_time = ros::Time::now();
    last_time = ros::Time::now();

    vel_current_time = ros::Time::now();
    vel_target_time = ros::Time::now();

    ros::Subscriber velocity_dir_command = nh.subscribe("dc_direction_ctrl", 100, velocity_ctrlCB); // Used to detect whether the vehicle has stopped
    ros::Subscriber velocity_speed_command = nh.subscribe("dc_speed_ctrl", 100, velocity_ctrlCB);   // Used to detect whether the vehicle has stopped
    // *******************************************************************************//
    
    // *******************************************************************************//
    // Steering Angle Calculation
    ros::Subscriber steering_angle_calc = nh.subscribe("/imu/data", 100, steering_angle_calcCB);

    // *******************************************************************************//

    ros::Rate loop_rate(100);   // This node will run at 100Hz
    // In order to conduct high-speed calculation for Odometry, we have to set loop rate at high Hz


    int test = 0;

    while(nh.ok()){

        if(test == 1){
            ros::spinOnce();
            loop_rate.sleep();
        }
        if(test == 0){
            ros::spinOnce();

            current_time = ros::Time::now();

            nh.getParam("body_length", body_length);

            // delta_time calculation
            delta_time = (current_time - last_time).toSec();

            // *******************************************************************************//
            // Ackermann Steering Odometry Model
            theta = theta + ((wheel_velocity/body_length) * tan(steering_angle));
            x =  x + (wheel_velocity * cos(theta) * delta_time);
            y = y + (wheel_velocity * sin(theta) * delta_time);

            // *******************************************************************************//
            // 6DOF Odometry (*** Change this according to Ackermann Model ***)
            geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(theta);

            // odom - base_link transformation delcaration
            geometry_msgs::TransformStamped odom_trans;
            
            odom_trans.header.stamp = current_time;
            odom_trans.header.frame_id = "odom";
            odom_trans.child_frame_id = "base_link";

            odom_trans.transform.translation.x = x;
            odom_trans.transform.translation.y = y;
            odom_trans.transform.translation.z = 0.0;
            odom_trans.transform.rotation  = odom_quat;

            // Send odom - base_link transformation
            odom_broadcaster.sendTransform(odom_trans);

            // *******************************************************************************//
            // LiDAR - base_link transformation declaration
            odom_broadcaster.sendTransform(
                tf::StampedTransform(
                    tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.1, 0.0, 0.2)),
                    ros::Time::now(), "base_link", "base_laser"));

            // *******************************************************************************//
            // Publish odometry message
            nav_msgs::Odometry odom;
            
            odom.header.stamp = current_time;
            odom.header.frame_id = "odom";

            // Set position values of the robot
            odom.pose.pose.position.x = x;
            odom.pose.pose.position.y = y;
            odom.pose.pose.position.z = 0.0;
            odom.pose.pose.orientation = odom_quat;

            // Set velocity values of the robot
            odom.child_frame_id = "base_link";
            odom.twist.twist.linear.x = wheel_velocity * cos(theta);
            odom.twist.twist.linear.y = wheel_velocity * sin(theta);
            odom.twist.twist.linear.z = (wheel_velocity/body_length) * tan(steering_angle);

            // Publish odometry message
            odom_pub.publish(odom);

            // Update the latest time
            last_time = current_time;
            loop_rate.sleep();
        }
    }
}