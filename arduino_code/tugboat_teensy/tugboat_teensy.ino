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

void handleFailsafes();
void handleStatusLED();

bool do_blink = true;
long lastMessageMillis = -1e5;

ros::NodeHandle  nh;

Servo rudder;

void rudder_cb( const std_msgs::Float32& cmd_msg){
  int angle = cmd_msg.data + 90;
  rudder.write(angle);
  lastMessageMillis = millis();
}
ros::Subscriber<std_msgs::Float32> rudder_sub("/rudder_angle", rudder_cb);

const int PROP_CENTER = 90;
const float PROP_K = 10.0; // How many "degrees" the prop should respond for 1 newton input command

Servo prop;
void prop_cb( const std_msgs::Float32& cmd_msg){
  int power = (cmd_msg.data * PROP_K) + PROP_CENTER;
  prop.write(power); //set servo angle, should be from 0-180
  lastMessageMillis = millis();
}

ros::Subscriber<std_msgs::Float32> prop_sub("/propeller_power", prop_cb);

void setup(){
  // Enable the LED pin
  pinMode(13, OUTPUT);

  nh.initNode();
  nh.subscribe(rudder_sub);
  nh.subscribe(prop_sub);
  
  rudder.attach(9); //Connect to the rudder pin
  prop.attach(10);

}

void loop(){
  nh.spinOnce();
  handleFailsafes();
  handleStatusLED();
  delay(1);
}

void handleFailsafes(){
  long t = millis();
  bool estop = (t - lastMessageMillis) > 1000;
  do_blink = estop;
  if (estop){
    prop.write(PROP_CENTER);
    // TODO: disconnect from servo instead of 
    // just writing a supposedly neutral power
  }
}

void handleStatusLED(){
  long t = millis();
  const byte LED_PIN = 13;
  if (do_blink){
    digitalWrite(LED_PIN, (t % 400) < 200);
  }else{
    digitalWrite(LED_PIN, HIGH);
  }
  
}
