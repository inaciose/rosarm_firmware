
/* ROS generic servo robotic arm firmware for Arduino and PCA9685
 * Servo motors control with pca9685 module and rosserial
 * v. 1.0
 */
 
// Total servos in robotic arm
#define SERVO_COUNT 4

// Analog servos run at ~50 Hz or 60Hz updates
#define SERVO_FREQ 50

 // ROS includes 
#include <ros.h>
#include <sensor_msgs/JointState.h>

// PCA9685 includes
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

//
// Configuration
//

// servo min pulses (0 degrees)
int servomin[SERVO_COUNT] = {100, 100, 100, 130};

// servo max pulses (180 degrees)
int servomax[SERVO_COUNT] = {470, 470, 470, 520};

// servo start position (in degrees)
int servostart[SERVO_COUNT] = {90, 90, 90, 90};

// servo movement direction ( -1 invert -1.57 rad to 1.57 rad)
int servodir[SERVO_COUNT] = {-1, -1, 1, 1};

//
// Core objects declarations
//

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

ros::NodeHandle  nh;

//
// ROS Callback funtion
//

void messageCb(const sensor_msgs::JointState& msg) {
  uint16_t pulse;
  for(int i = 0; i < SERVO_COUNT; i++){
    // map radians to pulse length
    pulse = map(msg.position[i] * 1000 * servodir[i], -1.57 * 1000, 1.57 * 1000, servomin[i], servomax[i]);
    pwm.setPWM(i, 0, pulse);
  }  
}

ros::Subscriber<sensor_msgs::JointState> sub("/joint_states", &messageCb );

//
// Setup function
//

void setup() {
  // ROS setup  
  nh.initNode();
  nh.subscribe(sub);

  // Servo controller setup
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);
  delay(10);
  
  // Move servos to start position
  int i, pulse;
  for(i = 0; i < SERVO_COUNT; i++) {
    // map start degrees to pulse length
    pulse = map(servostart[i], 0, 180, servomin[i], servomax[i]);
    pwm.setPWM(i, 0, pulse);
  }
}

//
// Idle loop (all work is done in callback)
//

void loop() { 
  nh.spinOnce();
  delay(1);
}
