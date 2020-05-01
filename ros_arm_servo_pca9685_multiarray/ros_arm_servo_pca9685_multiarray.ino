// ROS driver for eezybotarm MK1/MK2

// Servo control
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// ROS
#include <ros.h>
#include <std_msgs/UInt16MultiArray.h>

// For rosserial arduino buffer
#define USE_USBCON

// Total servos in robotic arm
#define SERVO_COUNT 4

// Analog servos run at ~50 Hz updates
#define SERVO_FREQ 50

int servomin[SERVO_COUNT] = {100, 100, 100, 130};
int servomax[SERVO_COUNT] = {470, 470, 470, 520};
int servostart[SERVO_COUNT] = {90, 90, 90, 90};

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

ros::NodeHandle  nh;

//
// ROS Callback functions
//

void servos_cb( const std_msgs::UInt16MultiArray& cmd_msg){
  int i, pulse;  
  for(i = 0; i < SERVO_COUNT; i++) {
    // map degrees to pulse length
    pulse = map(cmd_msg.data[i], 0, 180, servomin[i], servomax[i]);
    pwm.setPWM(i, 0, pulse);
  }
}

ros::Subscriber<std_msgs::UInt16MultiArray> sub_motors("motors", servos_cb);

void setup() {

  nh.initNode();
  nh.subscribe(sub_motors);

  // servo controller setup
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);
  delay(10);
  
  // Move to home position
  int i, pulse;
  for(i = 0; i < SERVO_COUNT; i++) {
    // map start degrees to pulse length
    pulse = map(servostart[i], 0, 180, servomin[i], servomax[i]);
    pwm.setPWM(i, 0, pulse);
  }
}

void loop() {
  nh.spinOnce();
  delay(5);
}
