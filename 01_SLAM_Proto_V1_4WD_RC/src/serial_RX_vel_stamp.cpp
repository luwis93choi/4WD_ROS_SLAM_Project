/*
 * roslaunh for python-based node : https://answers.ros.org/question/186319/rosserial_python-wont-start-from-launch-file/
 * 
 * rosserial_arduino reference : http://wiki.ros.org/rosserial_arduino/Tutorials/Arduino%20IDE%20Setup
 *                               http://wiki.ros.org/rosserial_arduino/Tutorials/Hello%20World
 *                               http://wiki.ros.org/rosserial_arduino/Tutorials/Blink
 */

/*
 * This node is for receiving serial data from "serial_node.py" of "rosserial_python package" and producing timestamped velocity messages
 * It used serial communication with MCU to acquire velocity data and assign Standard Header to custom message, "DC_velocity.msg"
 * 
 * Therefore, in order to use this node properly, use the following command in order to activate "rosserial_python"
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
#include <std_msgs/Header.h>                // This header is used to create stamped message
                                            // Need message_runtime in package.xml

#include <sensor_msgs/Joy.h>    // This header is used to receive Xbox Joystick commands

slam_proto_v1::DC_velocity stamped_enc_msg;
unsigned int seq = 0;

std_msgs::Int32 servo_ctrl_msg; // Declare standard Int32 message for servo control
std_msgs::Int32 dc_direction_ctrl_msg;
std_msgs::Int32 dc_speed_ctrl_msg;

int calc_flag = 0;

// Manual Control Variables ///////////////////////////////////////////////////
int current_direction_ctrl = 0;
int current_speed_val = 0;
int current_servo_val = 90;

int prev_increase_speed_ctrl = 0;
int prev_decrease_speed_ctrl = 0;
int prev_steer_left_ctrl = 0;
int prev_steer_right_ctrl = 0;
//////////////////////////////////////////////////////////////////////////////


// This callback function will be invoked if the subscriber receives data from serial_node.py
// This function receives serial data as standard Float32 message
void arduino_RX_Callback(const std_msgs::Float32& msg){

    ROS_INFO("Arduino RX Message : %f", msg.data);
    // Float32 message data is stored in 'data' member variable
    
    stamped_enc_msg.header.seq = seq++;
    stamped_enc_msg.header.stamp = ros::Time::now();
    stamped_enc_msg.header.frame_id = "DC_encoder";
    stamped_enc_msg.enc_count = msg.data;
}

void xbox_pad_CB(const sensor_msgs::Joy& xbox_msg){

    if(xbox_msg.buttons[2] == 1){
        
        current_speed_val = 0;
        current_direction_ctrl = 0;
    }
    else if(xbox_msg.buttons[1] == 1){

        current_direction_ctrl = 1;
    }
    else{

        if(prev_increase_speed_ctrl != xbox_msg.buttons[3]){

            current_speed_val = current_speed_val + 1;

            if(current_speed_val > 40){

                current_speed_val = 40;
            }
        }
        if(prev_decrease_speed_ctrl != xbox_msg.buttons[0]){
            
            current_speed_val = current_speed_val - 1;

            if(current_speed_val < 0){
                
                current_speed_val = 0;
            }
        }
        if(xbox_msg.buttons[4] == 1){
            
            current_servo_val = current_servo_val - 3;

            if(current_servo_val > 200){

                current_servo_val = 200;
            }
        }
        if(xbox_msg.buttons[5] == 1){
            
            current_servo_val = current_servo_val + 3;

            if(current_servo_val < 20){

                current_servo_val = 20;
            }
        }
    }
}

int main(int argc, char** argv){

    // Initialize the node with argc and argv, and Name it as "serial_RX_node"
    ros::init(argc, argv, "serial_RX_node");

    // Declare node handler
    ros::NodeHandle nh;

    // Declare the subscriber that receives serial data from serial_node.py
    // Receiving messages from serial_node.py will invoke arduino_RX_Callback function
    ros::Subscriber arduino_subscriber = nh.subscribe("wheel_velocity", 1000, arduino_RX_Callback);
    ros::Publisher stamped_enc_pub = nh.advertise<slam_proto_v1::DC_velocity>("stamped_vel", 10);

    stamped_enc_msg.header.seq = seq++;
    stamped_enc_msg.header.stamp = ros::Time::now();
    stamped_enc_msg.header.frame_id = "DC_encoder";
    stamped_enc_msg.enc_count = 0;

    // Declare the publisher that will convey servo control value through Int32 message and serial port
    ros::Publisher servo_control = nh.advertise<std_msgs::Int32>("servo_ctrl", 10); 
    servo_ctrl_msg.data = 90;  // Define initial value of servo control value
    nh.setParam("servo_ctrl_value", 90);   // Define servo control value as paramter in order for the user to manually control the servo motor if necessary

    ros::Publisher dc_direction_control = nh.advertise<std_msgs::Int32>("dc_direction_ctrl",10);
    ros::Publisher dc_speed_control = nh.advertise<std_msgs::Int32>("dc_speed_ctrl",10);
    dc_direction_ctrl_msg.data = 0;
    dc_speed_ctrl_msg.data = 0;
    nh.setParam("dc_direction_ctrl_value", 0);
    nh.setParam("dc_speed_ctrl_value", 0);


    // *** Manual Control using Xbox 360 Pad *** //
    ros::Subscriber xbox_sub = nh.subscribe("joy", 1000, xbox_pad_CB);
    ///////////////////////////////////////////////

    // Declare loop rate 30Hz
    ros::Rate loop_rate(30);

    // While roscore is operational, loop will continue on...
    while(ros::ok()){

        ros::spinOnce();    // Handle all the message callbacks

        dc_direction_ctrl_msg.data = current_direction_ctrl;
        dc_speed_ctrl_msg.data = current_speed_val;
        servo_ctrl_msg.data = current_servo_val;

        servo_control.publish(servo_ctrl_msg);  // Publish servo control message through serial port
        dc_direction_control.publish(dc_direction_ctrl_msg);
        dc_speed_control.publish(dc_speed_ctrl_msg);

        nh.getParam("dc_direction_ctrl_value", calc_flag);
        stamped_enc_pub.publish(stamped_enc_msg);

        loop_rate.sleep();  // Use sleep in order to run this loop at 10Hz
    }

    return 0;
}
