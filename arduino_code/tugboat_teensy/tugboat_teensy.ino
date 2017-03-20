/*
 * rosserial Servo Control Example
 *
 * This sketch demonstrates the control of hobby R/C servos
 * using ROS and the arduiono
 * 
 * For the full tutorial write up, visit
 * www.ros.org/wiki/rosserial_arduino_demos
 *
 * For more information on the Arduino Servo Library
 * Checkout :
 * http://www.arduino.cc/en/Reference/Servo
 */

#if (ARDUINO >= 100)
 #include <Arduino.h>
#else
 #include <WProgram.h>
#endif

#include <Servo.h> 
#include <ros.h>
#include <std_msgs/Float32.h>

bool do_blink = true;
long lastMessageMillis = 0;

ros::NodeHandle  nh;

Servo servo;

void servo_cb( const std_msgs::Float32& cmd_msg){
  int angle = cmd_msg.data + 90;
  servo.write(angle); //set servo angle, should be from 0-180
  lastMessageMillis = millis();
}


ros::Subscriber<std_msgs::Float32> sub("/rudder_angle", servo_cb);

void setup(){
  pinMode(13, OUTPUT);

  nh.initNode();
  nh.subscribe(sub);
  
  servo.attach(9); //attach it to pin 9
}

void loop(){
  handleStatusLED();
  nh.spinOnce();
  delay(1);
}

void handleStatusLED(){
  const byte LED_PIN = 13;
  long t = millis();
  bool do_blink = (t - lastMessageMillis) > 200;
  if (do_blink){
    digitalWrite(LED_PIN, (t % 400) < 200);
  }else{
    digitalWrite(LED_PIN, HIGH);
  }
  
}

