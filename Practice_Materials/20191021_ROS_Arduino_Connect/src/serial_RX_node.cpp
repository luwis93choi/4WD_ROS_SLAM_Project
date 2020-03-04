/*
 * catkin_build command : catkin_make --only-pkg-with-deps ros_arduino_connect
 * rosrun command : rosrun ros_arduino_connect serial_RX_node
 * roslaunch command : roslaunch union.launch --screen
 * 
 * roslaunh for python-based node : https://answers.ros.org/question/186319/rosserial_python-wont-start-from-launch-file/
 * 
 * rosserial_arduino reference : http://wiki.ros.org/rosserial_arduino/Tutorials/Arduino%20IDE%20Setup
 *                               http://wiki.ros.org/rosserial_arduino/Tutorials/Hello%20World
 *                               http://wiki.ros.org/rosserial_arduino/Tutorials/Blink
 */

#include <ros/ros.h>
#include <std_msgs/String.h>

void arduino_RX_Callback(const std_msgs::String& msg){

    ROS_INFO("Arduino Message Received : %s", msg.data.c_str());
}

int main(int argc, char **argv){

    ros::init(argc, argv, "serial_RX_node");

    ros::NodeHandle nh;

    ros::Subscriber arduino_subscriber = nh.subscribe("chatter", 1000, arduino_RX_Callback);

    ros::Rate loop_rate(10);

    while(ros::ok()){

        ros::spinOnce();

        loop_rate.sleep();
    }

    return 0;
}