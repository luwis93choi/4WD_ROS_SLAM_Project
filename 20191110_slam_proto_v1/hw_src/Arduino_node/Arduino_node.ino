/*
 * rosserial Publisher Example
 * Prints "Hello World!"
 * Reference : http://wiki.ros.org/rosserial_arduino/Tutorials/Hello%20World
 * 
 * Arduino Encoder Counter using external interrupts
 * Reference : https://studymake.tistory.com/279
 *             http://autonics.se/wp-content/uploads/2018/02/e30s_en_drw171366aa_20180118_he_20180122.pdf
 *             http://www.hardcopyworld.com/ngine/aduino/index.php/archives/594
 *             https://www.arduino.cc/reference/en/language/functions/external-interrupts/attachinterrupt/
 *             http://ww1.microchip.com/downloads/en/DeviceDoc/Atmel-7810-Automotive-Microcontrollers-ATmega328P_Datasheet.pdf
 * 
 * Arduino I/O
 * Reference : https://kocoafab.cc/tutorial/view/526
 * 
 * Passing pointer to function / How sizeof() function acts differently for pointer-based array and fixed aray
 * Reference : https://boycoding.tistory.com/201
 *             https://vvshinevv.tistory.com/6
 *             https://dojang.io/mod/page/view.php?id=296
 */

#include <ros.h>
#include <std_msgs/String.h>

// ROS publisher //////////////////////////////////////////////
ros::NodeHandle nh;

std_msgs::String str_msg;
ros::Publisher chatter("chatter", &str_msg);

char TX_msg[30] = " ";
//////////////////////////////////////////////////////////////

void int_to_char(int input, char* target_array, int len){

  int i = 0;

  int array_size = len;

  for(i = 0; i < array_size-1; i++){ target_array[i] = 0; }
  
  for(i = array_size-1; i >= 0; i--){

    target_array[i] = (input%10) + 48;

    input = input/10;
  }
}

// Encoder Interrupt /////////////////////////////////////////
const int encoder_outA = 2; // Phase A output
const int encoder_outB = 4; // Phase B output

String direction = "Unknown";

int encoder_count = 0;

void encoder_counter(){

  direction = (digitalRead(encoder_outB) == HIGH) ? "CW" : "CCW";

  encoder_count++;

  int_to_char(encoder_count, TX_msg, sizeof(TX_msg)/sizeof(char));
}
//////////////////////////////////////////////////////////////

void setup() {

  nh.initNode();
  nh.advertise(chatter);

  pinMode(encoder_outB, INPUT);
  attachInterrupt(digitalPinToInterrupt(encoder_outA), encoder_counter, FALLING);
}

void loop() {

  str_msg.data = TX_msg;
  chatter.publish(&str_msg);
  nh.spinOnce();
  delay(10);
}
