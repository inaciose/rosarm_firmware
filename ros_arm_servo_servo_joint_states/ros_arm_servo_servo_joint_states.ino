
/* ROS generic servo robotic arm firmware for Arduino and Servo lib
 * Servo motors control with Arduino Servo library and rosserial
 * v. 1.0
 */

#if defined(ARDUINO) && ARDUINO >= 100
  #include "Arduino.h"
#else
  #include <WProgram.h>
#endif

#include <Servo.h> 
#include <ros.h>
#include <sensor_msgs/JointState.h>

//
// Configuration
//

// Change, if required, servo (pwm) pins on setup funtion
// Change, if required, servo start positions on setup funtion
// Add, if required, servo offset in callback function

//
// Core objects declarations
//

ros::NodeHandle  nh;

Servo servo0;
Servo servo1;
Servo servo2;
Servo servo3;
Servo servo4;
Servo servo5;

//
// ROS callback functions
//

void servo_cb(const sensor_msgs::JointState& cmd_msg){
  
  servo0.write(radiansToDegrees(cmd_msg.position[0]));
  servo1.write(radiansToDegrees(cmd_msg.position[1]));
  servo2.write(radiansToDegrees(cmd_msg.position[2]));
  servo3.write(radiansToDegrees(cmd_msg.position[3]));
  servo4.write(radiansToDegrees(cmd_msg.position[4]));
  servo5.write(radiansToDegrees(cmd_msg.position[5]));
  
}

ros::Subscriber<sensor_msgs::JointState> sub("joint_states", servo_cb);

//
// Helper functions
//

double radiansToDegrees(float position_radians) {
  position_radians = position_radians + 1.6;
  return position_radians * 57.2958;
}

//
// Arm setup
//

void setup(){
  // ROS setup 
  //nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.subscribe(sub);

  // Servo setup
  servo0.attach(2); 
  servo1.attach(8); 
  servo2.attach(11);
  servo3.attach(6); 
  servo4.attach(4);
  servo5.attach(3); 
  delay(10);

  // Move servos to start position
  servo0.write(90);
  servo1.write(90);
  servo2.write(90);
  servo3.write(90);
  servo4.write(90);
  servo5.write(90);
}


//
// Idle loop (all work is done in callback)
//
void loop() {
  nh.spinOnce();
  delay(1);
}
