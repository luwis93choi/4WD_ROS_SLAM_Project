#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Int32.h>

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


ros::Time vel_current_time;     // [sec]
ros::Time vel_target_time;      // [sec]
double vel_delta_time = 0.0;    // [sec]
int log_flag = 0;


void wheel_velocity_calcCB(const std_msgs::Int32& msg){

    // wheel_velocity calculation based on encoder count value

    if((msg.data >= 2600) && (log_flag == 0)){
    
        vel_current_time = ros::Time::now();
        log_flag = 1;
    }
    else if((msg.data >= 2600) && (log_flag == 1)){

        vel_target_time = ros::Time::now();

        vel_delta_time = (vel_target_time - vel_current_time).toSec();

        wheel_velocity = (wheel_circumference / vel_delta_time);

        /*
        ROS_INFO("Target Time reached");        
        ROS_INFO("target_time : %f sec", vel_target_time.toSec());
        ROS_INFO("current_time : %f sec", vel_current_time.toSec());
        ROS_INFO("delta t : %f sec \n", vel_delta_time);
        */

        ROS_INFO("Current Speed : %f m/s", wheel_velocity);

        log_flag = 0;
    }
}

void steering_angle_calcCB(const std_msgs::Int32& msg){

    // steering_angle calculation based on servo PWM value
    /*
    steering_angle = msg.data * conversion_value;
     */
}

int main(int argc, char** argv){

    ros::init(argc, argv, "ackermann_odom");
    ros::NodeHandle nh;

    ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 50);
    tf::TransformBroadcaster odom_broadcaster;

    ros::Subscriber wheel_velocity_calc = nh.subscribe("encoder", 1000, wheel_velocity_calcCB);
    //ros::Subscriber steering_angle_calc = nh.subscribe("servo_ctrl", 10, steering_angle_calcCB);

    ros::Time current_time, last_time;
    current_time = ros::Time::now();
    last_time = ros::Time::now();

    nh.setParam("body_length", body_length);

    ros::Rate loop_rate(100);   // This node will run at 100Hz
    // In order to conduct high-speed calculation for Odometry, we have to set loop rate at high Hz


    int test = 0;

    vel_current_time = ros::Time::now();
    vel_target_time = ros::Time::now();


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
            x = x + (wheel_velocity * cos(theta) * delta_time);
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