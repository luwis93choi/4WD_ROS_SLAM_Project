#include <ros.h>
#include <std_msgs/Int32.h>

// Servo Control Callback ////////////////////////
int servo_pin = 6;

void servo_ctrl_callback(const std_msgs::Int32& servo_ctrl_msg){

  analogWrite(servo_pin, servo_ctrl_msg.data);
}
////////////////////////////////////////////////

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

// ROS publisher & subscriber /////////////////////////////////
ros::NodeHandle nh;

std_msgs::Int32 encoder_sensor_msg;
ros::Publisher encoder_msg("encoder", &encoder_sensor_msg);

ros::Subscriber<std_msgs::Int32> servo_ctrl("servo_ctrl", &servo_ctrl_callback);
//////////////////////////////////////////////////

void setup() {

  nh.initNode();
  nh.advertise(encoder_msg);
  nh.subscribe(servo_ctrl);

  pinMode(encoder_outB, INPUT);
  attachInterrupt(digitalPinToInterrupt(encoder_outA), encoder_counter, FALLING);

  pinMode(servo_pin, OUTPUT);
}

void loop() {

  encoder_sensor_msg.data = encoder_count;
  encoder_msg.publish(&encoder_sensor_msg);
  nh.spinOnce();
  delay(10);
}
