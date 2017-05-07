/*
   Rosserial motor writer for the boat

   Gets motor values from ROS and uses PWM to control
   on board motors through the Arduino
*/
#include <Arduino.h>
#include <Servo.h>
#include <ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include "jankboat.h"

#define MOTOR_NUM 4
#define LED_PIN 13
#define IS_ROVER_PIN 19
#define TIMEOUT 1000
#define IS_ROVER_RATE 5
#define PWM_HIGH 2000
#define PWM_LOW 1000
#define ESC_DISCONNECT_WAIT 5

Servo motors[MOTOR_NUM];
bool do_blink = true;
long last_message_millis = -1e5;
long last_is_rover_pub = 0;
bool estopped = false;
bool timedout = true;

ros::NodeHandle  nh;
ros::Subscriber<std_msgs::Float32> prop1_sub("/motor1", motor1_cb);
ros::Subscriber<std_msgs::Float32> prop2_sub("/motor2", motor2_cb);
ros::Subscriber<std_msgs::Float32> prop3_sub("/motor3", motor3_cb);
ros::Subscriber<std_msgs::Float32> prop4_sub("/motor4", motor4_cb);
ros::Subscriber<std_msgs::Bool> estop_sub("/estop", estop_cb);

std_msgs::Bool is_rover;
ros::Publisher is_rover_pub("/is_rover", &is_rover);

//map function modified for float use
float map_float(float val, float inmin, float inmax, float outmin, float outmax){
  return (val - inmin)/(inmax - inmin)*(outmax-outmin) + outmin;
}

void motor1_cb( const std_msgs::Float32& cmd_msg) {
  set_motor_power(0,cmd_msg.data); // -1 to 1
  last_message_millis = millis();
}

void motor2_cb( const std_msgs::Float32& cmd_msg) {
  set_motor_power(1,cmd_msg.data); // -1 to 1
  last_message_millis = millis();
}

void motor3_cb( const std_msgs::Float32& cmd_msg) {
  set_motor_power(2,cmd_msg.data); // -1 to 1
  last_message_millis = millis();
}

void motor4_cb( const std_msgs::Float32& cmd_msg) {
  set_motor_power(3,cmd_msg.data); // -1 to 1
  last_message_millis = millis();
}

void estop_cb( const std_msgs::Bool& cmd_msg){
  if(cmd_msg.data != estopped){
    estopped = cmd_msg.data;
    if(estopped){
      stop();
    }
  }
  last_message_millis = millis();
}

void setup() {
  // Enable the LED pin
  pinMode(LED_PIN, OUTPUT);
  pinMode(IS_ROVER_PIN, INPUT_PULLUP);
  nh.initNode();
  nh.subscribe(prop1_sub);
  nh.subscribe(prop2_sub);
  nh.subscribe(prop3_sub);
  nh.subscribe(prop4_sub);
  nh.subscribe(estop_sub);

  nh.advertise(is_rover_pub);
  attach_motors();

  delay(3000);
}

void attach_motors(){
  stop();

  motors[0].attach(23); //Front left
  motors[1].attach(22); //Front right
  motors[2].attach(21); //Back left
  motors[3].attach(20); //back right
}

void loop() {
  nh.spinOnce();
  handle_failsafes();
  handle_status_LED();
  if(millis() - last_is_rover_pub > 1000/IS_ROVER_RATE){
    is_rover.data = on_ground();
    is_rover_pub.publish(&is_rover);
    last_is_rover_pub = millis();
  }
  delay(1);
}

void set_motor_power(int motor_num, float input_power){
  input_power = constrain( input_power, -1, 1);
  float output_power = map_float(input_power, -1, 1, PWM_LOW, PWM_HIGH); // Convert to usuable microseconds

  if (!timedout && !estopped){
    motors[motor_num].writeMicroseconds(output_power);
  }else{
    motors[motor_num].writeMicroseconds((PWM_LOW + PWM_HIGH)/2);
  }
}

//reads pin 19 to report whether vehicle is a ground vehicle
bool on_ground(){
  if(digitalRead(19) == HIGH){
    return false;
  }else{
    return true;
  }
}

void stop() {
  for(int i=0; i<4; i++){
      set_motor_power(i, (PWM_LOW + PWM_HIGH)/2);
  }
}

void handle_failsafes() {
  long t = millis();
  timedout = (t - last_message_millis) > TIMEOUT;
  do_blink = !timedout;

  if (timedout) {
    stop();
  }
}

void handle_status_LED() {
  long t = millis();
  if (do_blink) {
    digitalWrite(LED_PIN, (t % 400) < 200);
  } else {
    digitalWrite(LED_PIN, HIGH);
  }
}
