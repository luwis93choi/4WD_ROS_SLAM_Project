// rosserial_arduino IDE setup reference : http://wiki.ros.org/rosserial_arduino/Tutorials/Arduino%20IDE%20Setup

#include <ros.h>
#include <ros/time.h>

#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>

#include <Servo.h>

// Servo Control Callback ////////////////////////
Servo rc_servo;

int servo_pin = 6;
int servo_pwm = 90;

void servo_ctrl_callback(const std_msgs::Int32& servo_ctrl_msg){

  servo_pwm = servo_ctrl_msg.data;
}
////////////////////////////////////////////////

// Encoder Interrupt & Velocity Calculation ////
const int encoder_outA = 2; // Phase A output
const int encoder_outB = 4; // Phase B output

String direction = "Unknown";

int encoder_count = 0;

double prev_time = 0;
double target_time = 0;

int encoder_target_val = 2600;
int one_revolution_encoder_count = 2600;
double wheel_radius = 0.0325;        // 0.0325[m]
double wheel_circumference = 2 * wheel_radius * 3.14; // [m]

double velocity = 0;

int wheel_flag = 0;

int count = 0;

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

// ROS publisher & subscriber /////////////////////////////////
ros::NodeHandle nh;

std_msgs::Float32 encoder_sensor_msg;
ros::Publisher encoder_msg("wheel_velocity", &encoder_sensor_msg);

ros::Subscriber<std_msgs::Int32> servo_ctrl("servo_ctrl", &servo_ctrl_callback);

ros::Subscriber<std_msgs::Int32> dc_direction_ctrl("dc_direction_ctrl", dc_direction_ctrl_callback);
ros::Subscriber<std_msgs::Int32> dc_speed_ctrl("dc_speed_ctrl", dc_speed_ctrl_callback);
//////////////////////////////////////////////////

void setup() {

  nh.initNode();

  nh.getHardware()->setBaud(57600);
  
  nh.advertise(encoder_msg);
  
  nh.subscribe(servo_ctrl);
  nh.subscribe(dc_direction_ctrl);
  nh.subscribe(dc_speed_ctrl);

  pinMode(encoder_outB, INPUT);
  attachInterrupt(digitalPinToInterrupt(encoder_outA), encoder_counter, FALLING);

  rc_servo.attach(servo_pin);
  
  pinMode(dc_directionA, OUTPUT);
  pinMode(dc_directionB, OUTPUT);
  pinMode(dc_enable, OUTPUT);
  pinMode(dc_pin, OUTPUT);
  digitalWrite(dc_enable, HIGH);
  digitalWrite(dc_directionA, LOW);
  digitalWrite(dc_directionB, LOW);
  analogWrite(dc_pin, 0);
  
  prev_time = millis() * 0.001;
}

void loop() {

  if(count >= 15){

    target_time = millis() * 0.001;

    if((target_time - prev_time) == 0){

      velocity = 0;
    }
    else{
      
      velocity = (wheel_circumference * ((double)encoder_count / (double)one_revolution_encoder_count)) / (target_time - prev_time);
    }
    encoder_sensor_msg.data = velocity;
    encoder_msg.publish(&encoder_sensor_msg);

    encoder_count = 0;

    prev_time = target_time;

    count = 0;

    delay(100);
  }
  else{

    count++;
  }

  rc_servo.write(servo_pwm);
  delay(15);
  
  nh.spinOnce();
}
