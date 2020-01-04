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
#include <std_msgs/String.h>

ros::Time current_time;
ros::Time target_time;
double delta_time = 0.0;
int log_flag = 0;

int encoder_count = 0;

// This callback function will be invoked if the subscriber receives data from serial_node.py
// This function receives serial data as standard String message
void arduino_RX_Callback(const std_msgs::Int32& msg){

    ROS_INFO("Arduino RX Message : %d", msg.data);
    // String message data is stored in 'data' member variable
    // Use c_str() in order to convert it into string value for CLI print

    encoder_count = msg.data;

    if((encoder_count >= 1) && (log_flag == 0)){
    
        current_time = ros::Time::now();
        log_flag = 1;
    }
    else if((encoder_count >= 2600) && (log_flag == 1)){

        target_time = ros::Time::now();

        delta_time = (target_time - current_time).toSec();
        ROS_INFO("Target Time reached");

        log_flag = 2;
    }

    if(log_flag == 2){
        ROS_INFO("target_time : %f sec", target_time.toSec());
        ROS_INFO("current_time : %f sec", current_time.toSec());
        ROS_INFO("delta t : %f sec", delta_time);
    }
}

int main(int argc, char** argv){

    // Initialize the node with argc and argv, and Name it as "serial_RX_node"
    ros::init(argc, argv, "serial_RX_node");

    // Declare node handler
    ros::NodeHandle nh;

    // Declare the subscriber that receives serial data from serial_node.py
    // Receiving messages from serial_node.py will invoke arduino_RX_Callback function
    //ros::Subscriber arduino_subscriber = nh.subscribe("encoder", 1000, arduino_RX_Callback);

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

    // Declare loop rate 10Hz
    ros::Rate loop_rate(10);

    ros::spinOnce();


    current_time = ros::Time::now();
    target_time = ros::Time::now();


    // While roscore is operational, loop will continue on...
    while(ros::ok()){

        nh.getParam("servo_ctrl_value", servo_ctrl_msg.data);   // Bind servo control parameter to servo control message data value
        servo_control.publish(servo_ctrl_msg);  // Publish servo control message through serial port

        nh.getParam("dc_direction_ctrl_value", dc_direction_ctrl_msg.data);
        nh.getParam("dc_speed_ctrl_value", dc_speed_ctrl_msg.data);
        dc_direction_control.publish(dc_direction_ctrl_msg);
        dc_speed_control.publish(dc_speed_ctrl_msg);

        ros::spinOnce();    // Handle all the message callbacks

        loop_rate.sleep();  // Use sleep in order to run this loop at 10Hz
    }

    return 0;
}
