/*
 * roslaunh for python-based node : https://answers.ros.org/question/186319/rosserial_python-wont-start-from-launch-file/
 * 
 * rosserial_arduino reference : http://wiki.ros.org/rosserial_arduino/Tutorials/Arduino%20IDE%20Setup
 *                               http://wiki.ros.org/rosserial_arduino/Tutorials/Hello%20World
 *                               http://wiki.ros.org/rosserial_arduino/Tutorials/Blink
 */

/*
 * This node is for receiving serial data from "serial_node.py" of "rosserial_python package"
 * 
 * Therefore, in order to use this node properly, use the following command in order to activate "rplidarNode"
 * 
 * Installation : "sudo apt-get install ros-<ros-distro>-rosserial-python" (ex : sudo apt-get install ros-kinetic-rosserial-python (for ROS Kinetic))
 * 
 * Running serial_node.py : "rosrun rosserial_python serial_node.py /dev/(serial_port_value)" 
 */

#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>

#include "slam_proto_v1/DC_velocity.h"
#include <std_msgs/Header.h>            // Need message_runtime in package.xml

slam_proto_v1::DC_velocity stamped_enc_msg;
int is_stamped = 0;
unsigned int seq = 0;

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


// This callback function will be invoked if the subscriber receives data from serial_node.py
// This function receives serial data as standard String message
void arduino_RX_Callback(const std_msgs::Float32& msg){

    ROS_INFO("Arduino RX Message : %f", msg.data);
    // String message data is stored in 'data' member variable
    // Use c_str() in order to convert it into string value for CLI print

    stamped_enc_msg.header.seq = seq++;
    stamped_enc_msg.header.stamp = ros::Time::now();
    stamped_enc_msg.header.frame_id = "DC_encoder";
    stamped_enc_msg.enc_count = msg.data;

    //ROS_INFO("Velocity : %f", msg.data);
}

int main(int argc, char** argv){

    // Initialize the node with argc and argv, and Name it as "serial_RX_node"
    ros::init(argc, argv, "serial_RX_node");

    // Declare node handler
    ros::NodeHandle nh;

    // Declare the subscriber that receives serial data from serial_node.py
    // Receiving messages from serial_node.py will invoke arduino_RX_Callback function
    ros::Subscriber arduino_subscriber = nh.subscribe("encoder", 1000, arduino_RX_Callback);
    ros::Publisher stamped_enc_pub = nh.advertise<slam_proto_v1::DC_velocity>("stamped_enc", 10);

    stamped_enc_msg.header.seq = seq++;
    stamped_enc_msg.header.stamp = ros::Time::now();
    stamped_enc_msg.header.frame_id = "DC_encoder";
    stamped_enc_msg.enc_count = wheel_velocity;

    std_msgs::Int32 servo_ctrl_msg; // Declare standard Int32 message for servo control
    // Declare the publisher that will convey servo control value through Int32 message and serial port
    ros::Publisher servo_control = nh.advertise<std_msgs::Int32>("servo_ctrl", 10); 
    servo_ctrl_msg.data = 128;  // Define initial value of servo control value
    nh.setParam("servo_ctrl_value", 128);   // Define servo control value as paramter in order for the user to manually control the servo motor if necessary

    std_msgs::Int32 dc_direction_ctrl_msg;
    std_msgs::Int32 dc_speed_ctrl_msg;
    ros::Publisher dc_direction_control = nh.advertise<std_msgs::Int32>("dc_direction_ctrl",10);
    ros::Publisher dc_speed_control = nh.advertise<std_msgs::Int32>("dc_speed_ctrl",10);
    dc_direction_ctrl_msg.data = 0;
    dc_speed_ctrl_msg.data = 0;
    nh.setParam("dc_direction_ctrl_value", 0);
    nh.setParam("dc_speed_ctrl_value", 0);

    // Declare loop rate 30Hz
    ros::Rate loop_rate(100);

    current_time = ros::Time::now();
    //target_time = ros::Time::now();

    // While roscore is operational, loop will continue on...
    while(ros::ok()){

        nh.getParam("servo_ctrl_value", servo_ctrl_msg.data);   // Bind servo control parameter to servo control message data value
        servo_control.publish(servo_ctrl_msg);  // Publish servo control message through serial port

        nh.getParam("dc_direction_ctrl_value", dc_direction_ctrl_msg.data);
        nh.getParam("dc_speed_ctrl_value", dc_speed_ctrl_msg.data);

        dc_direction_control.publish(dc_direction_ctrl_msg);
        dc_speed_control.publish(dc_speed_ctrl_msg);

        nh.getParam("dc_direction_ctrl_value", calc_flag);
        stamped_enc_pub.publish(stamped_enc_msg);

        ros::spinOnce();    // Handle all the message callbacks

        loop_rate.sleep();  // Use sleep in order to run this loop at 10Hz
    }

    return 0;
}
