#include <ros.h>
#include <std_msgs/Int32.h>

// ROS publisher /////////////////////////////////
ros::NodeHandle nh;

std_msgs::Int32 str_msg;
ros::Publisher encoder_msg("encoder", &str_msg);
//////////////////////////////////////////////////

// Encoder Interrupt ////////////////////////////
const int encoder_outA = 2; // Phase A output
const int encoder_outB = 4; // Phase B output

String direction = "Unknown";

int encoder_count = 0;

void encoder_counter(){

  direction = (digitalRead(encoder_outB) == HIGH) ? "CW" : "CCW";

  encoder_count++;
}
////////////////////////////////////////////////

void setup() {

  nh.initNode();
  nh.advertise(encoder_msg);

  pinMode(encoder_outB, INPUT);
  attachInterrupt(digitalPinToInterrupt(encoder_outA), encoder_counter, FALLING);
}

void loop() {

  str_msg.data = encoder_count;
  encoder_msg.publish(&str_msg);
  nh.spinOnce();
  delay(10);
}
