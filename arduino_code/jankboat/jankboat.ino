/*
   Rosserial motor writer for the boat

   Gets motor values from ROS and uses PWM to control
   on board motors through the Arduino
*/
#include <Arduino.h>
#include <Servo.h>
#include <ros.h>
#include <std_msgs/Float32.h>

void handleFailsafes();
void handleStatusLED();

bool do_blink = true;
long lastMessageMillis = -1e5;

ros::NodeHandle  nh;

//map function modified for float use
float mapFloat(float val, float inmin, float inmax, float outmin, float outmax){
  return (val - inmin)/(inmax - inmin)*(outmax-outmin) + outmin;
}

Servo motor1;
void motor1_cb( const std_msgs::Float32& cmd_msg) {
  float inputPower = cmd_msg.data; // -1 to 1
  inputPower = constrain( inputPower, -1, 1);
  float outputPower = mapFloat(inputPower, -1, 1, 1000, 2000); // Convert to usuable microseconds
  motor1.writeMicroseconds(outputPower);
  lastMessageMillis = millis();
}

Servo motor2;
void motor2_cb( const std_msgs::Float32& cmd_msg) {
  float inputPower = cmd_msg.data; // -1 to 1
  inputPower = constrain( inputPower, -1, 1);
  float outputPower = mapFloat(inputPower, -1, 1, 1000, 2000);
  motor2.writeMicroseconds(outputPower);
  lastMessageMillis = millis();
}

Servo motor3;
void motor3_cb( const std_msgs::Float32& cmd_msg) {
  float inputPower = cmd_msg.data; // -1 to 1
  inputPower = constrain( inputPower, -1, 1);
  float outputPower = mapFloat(inputPower, -1, 1, 1000, 2000);
  motor3.writeMicroseconds(outputPower);
  lastMessageMillis = millis();
}

Servo motor4;
void motor4_cb( const std_msgs::Float32& cmd_msg) {
  float inputPower = cmd_msg.data; // -1 to 1
  inputPower = constrain( inputPower, -1, 1);
  float outputPower = mapFloat(inputPower, -1, 1, 1000, 2000);
  motor4.writeMicroseconds(outputPower);
  lastMessageMillis = millis();
}

ros::Subscriber<std_msgs::Float32> prop1_sub("/motor1", motor1_cb);
ros::Subscriber<std_msgs::Float32> prop2_sub("/motor2", motor2_cb);
ros::Subscriber<std_msgs::Float32> prop3_sub("/motor3", motor3_cb);
ros::Subscriber<std_msgs::Float32> prop4_sub("/motor4", motor4_cb);

void setup() {
  // Enable the LED pin
  pinMode(13, OUTPUT);

  nh.initNode();
  nh.subscribe(prop1_sub);
  nh.subscribe(prop2_sub);
  nh.subscribe(prop3_sub);
  nh.subscribe(prop4_sub);

  motor1.attach(23); //Front left
  motor2.attach(22); //Front right
  motor3.attach(21); //Back left
  motor4.attach(20); //back right

}

void loop() {
  nh.spinOnce();
  handleFailsafes();
  handleStatusLED();
  delay(1);
}

void handleFailsafes() {
  long t = millis();
  bool estop = (t - lastMessageMillis) > 1000;
  do_blink = estop;
  if (estop) {
    //Stops writing to motors completely, not just settin "netural" position
    motor1.detach()
    motor2.detach()
    motor3.detach()
    motor4.detach()
  }
}

void handleStatusLED() {
  long t = millis();
  const byte LED_PIN = 13;
  if (do_blink) {
    digitalWrite(LED_PIN, (t % 400) < 200);
  } else {
    digitalWrite(LED_PIN, HIGH);
  }

}
