// rosserial_arduino IDE setup reference : http://wiki.ros.org/rosserial_arduino/Tutorials/Arduino%20IDE%20Setup

#include <ros.h>

#include <std_msgs/Int32.h>
#include <std_msgs/String.h>

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

// DC Control Callback ////////////////////////
int dc_directionA = 13;
int dc_directionB = 12;
int dc_enable = 10;
int dc_pin = 11;

int prev_direction = 0;

void dc_direction_ctrl_callback(const std_msgs::Int32& dc_direction_ctrl_msg){

  if(dc_direction_ctrl_msg.data == 0){
    
    digitalWrite(dc_enable, LOW);
    digitalWrite(dc_directionA, LOW);
    digitalWrite(dc_directionB, LOW);
    
    encoder_count = 0;  // Reset encoder count as the vehicle stopped

    prev_direction = dc_direction_ctrl_msg.data;
  }
  else if(dc_direction_ctrl_msg.data == 1){

    digitalWrite(dc_enable, HIGH);
    digitalWrite(dc_directionA, LOW);
    digitalWrite(dc_directionB, HIGH);

    if(prev_direction != dc_direction_ctrl_msg.data){

      encoder_count = 0;  // Reset encoder count as the vehicle changed its wheel direction

      prev_direction = dc_direction_ctrl_msg.data;
    }
  }
  else if(dc_direction_ctrl_msg.data == 2){
    
    digitalWrite(dc_enable, HIGH);
    digitalWrite(dc_directionA, HIGH);
    digitalWrite(dc_directionB, LOW);
    
    if(prev_direction != dc_direction_ctrl_msg.data){

      encoder_count = 0;  // Reset encoder count as the vehicle changed its wheel direction

      prev_direction = dc_direction_ctrl_msg.data;
    }
  }
}

void dc_speed_ctrl_callback(const std_msgs::Int32& dc_speed_ctrl_msg){

  analogWrite(dc_pin, dc_speed_ctrl_msg.data);
}
///////////////////////////////////////////////


int log_flag = 0;

// ROS publisher & subscriber /////////////////////////////////
ros::NodeHandle nh;

std_msgs::Int32 encoder_sensor_msg;
ros::Publisher encoder_msg("encoder", &encoder_sensor_msg);

ros::Subscriber<std_msgs::Int32> servo_ctrl("servo_ctrl", &servo_ctrl_callback);

ros::Subscriber<std_msgs::Int32> dc_direction_ctrl("dc_direction_ctrl", dc_direction_ctrl_callback);
ros::Subscriber<std_msgs::Int32> dc_speed_ctrl("dc_speed_ctrl", dc_speed_ctrl_callback);
//////////////////////////////////////////////////

void setup() {

  nh.initNode();
  nh.advertise(encoder_msg);
  
  nh.subscribe(servo_ctrl);
  nh.subscribe(dc_direction_ctrl);
  nh.subscribe(dc_speed_ctrl);

  pinMode(encoder_outB, INPUT);
  attachInterrupt(digitalPinToInterrupt(encoder_outA), encoder_counter, FALLING);

  pinMode(servo_pin, OUTPUT);

  pinMode(dc_directionA, OUTPUT);
  pinMode(dc_directionB, OUTPUT);
  pinMode(dc_enable, OUTPUT);
  pinMode(dc_pin, OUTPUT);
  digitalWrite(dc_enable, HIGH);
  digitalWrite(dc_directionA, LOW);
  digitalWrite(dc_directionB, LOW);
  analogWrite(dc_pin, 0);
}

void loop() {

  // At every turn of the wheel, notify ROS node.
  // Once ROS node is notified of the turn of the wheel, 
  // it will measure the time taken between each turn and use this in order to calculate the current velocity of the vehicle. 
  if(encoder_count >= 2600){
    encoder_sensor_msg.data = encoder_count;
    encoder_msg.publish(&encoder_sensor_msg);

    encoder_count = 0;  // Reset encoder_count value at every turn of the wheel
  }
  
  nh.spinOnce();
}
