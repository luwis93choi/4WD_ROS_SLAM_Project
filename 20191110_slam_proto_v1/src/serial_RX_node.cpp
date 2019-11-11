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

// This callback function will be invoked if the subscriber receives data from serial_node.py
// This function receives serial data as standard String message
void arduino_RX_Callback(const std_msgs::Int32& msg){

    ROS_INFO("Arduino RX Message : %d", msg.data);
    // String message data is stored in 'data' member variable
    // Use c_str() in order to convert it into string value for CLI print
}

int main(int argc, char** argv){

    // Initialize the node with argc and argv, and Name it as "serial_RX_node"
    ros::init(argc, argv, "serial_RX_node");

    // Declare node handler
    ros::NodeHandle nh;

    // Declare the subscriber that receives serial data from serial_node.py
    // Receiving messages from serial_node.py will invoke arduino_RX_Callback function
    ros::Subscriber arduino_subscriber = nh.subscribe("encoder", 1000, arduino_RX_Callback);

    // Declare loop rate 10Hz
    ros::Rate loop_rate(10);

    // While roscore is operational, loop will continue on...
    while(ros::ok()){

        ros::spinOnce();    // Handle all the message callbacks

        loop_rate.sleep();  // Use sleep in order to run this loop at 10Hz
    }

    return 0;
}