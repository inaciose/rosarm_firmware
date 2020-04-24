//
// ROS robotic arm firmware for A4988 
// multi stepper with endstop based auto home
// and servo on gripper
//
// v.1.0
//

#include <Servo.h> 
#include <ros.h>
#include <sensor_msgs/JointState.h>

#include <AccelStepper.h>

//
// Configuration
//

#define USE_USBCON
#define STEPPER_COUNT 3

// mcu pins for each stepper endstop
int endStopPin[STEPPER_COUNT] = {10, 11, 12};

// steps per rotation for each stepper
int stepsRotation[STEPPER_COUNT] = {200, 200, 200};

// direction of rotation during home set
// positive cw, zero not set, negative ccw
int homeSetStepperDirection[STEPPER_COUNT] = {1, 0, 1};

// offset steps after endstop reach
int homeSetStepperOffset[STEPPER_COUNT] = {-5, 0, -8};

//
// Variables
//

// home set status for each stepper 
boolean homeSetStepper[STEPPER_COUNT] = {false, false, false};

// home set offset status for each stepper 
boolean homeSetOffsetStepper[STEPPER_COUNT] = {false, false, false};

// move status for each stepper
boolean stepperIsMoving[STEPPER_COUNT] = {false, false, false};

// new target available for each stepper
boolean stepperNewTarget[STEPPER_COUNT] = {false, false, false};

// current stepper target for each stepper
int stepperTarget[STEPPER_COUNT] = {0, 0, 0};

// ROS node handle declaration
ros::NodeHandle  nh;

// Servo declarations
Servo servo1;

// Stepper declatations
AccelStepper stepper1(1, 3, 4); // pin 3 = step, pin 4 = direction
AccelStepper stepper2(1, 5, 6); // pin 5 = step, pin 6 = direction
AccelStepper stepper3(1, 7, 8); // pin 7 = step, pin 8 = direction

//
// ROS callback funtion
//

void motors_cb(const sensor_msgs::JointState& cmd_msg) {
  int f;
  double angle;
  for(f=0; f < STEPPER_COUNT; f++) {
    angle = -radiansToDegrees(cmd_msg.position[f]);
    if(angle != stepperTarget[f]) {
      stepperTarget[f] = angle;
      stepperNewTarget[f] = true;    
    }    
  }
  
  servo1.write(radiansToDegrees(cmd_msg.position[3]));
  
}

// Subscriber node declaration, specifies the topic to which subscribe and the callback funtion
ros::Subscriber<sensor_msgs::JointState> sub("joint_states", motors_cb);

//
// helper functions
//

double radiansToDegrees(float position_radians) {
  position_radians = position_radians + 1.6;
  return position_radians * 57.2958;
}

//
// run once on startup
//

void setup() {
  int f;

  // ros setup
  nh.getHardware()->setBaud(115200); 
  nh.initNode();
  nh.subscribe(sub);

  // set start direction for each stepper
  for(f = 0; f < STEPPER_COUNT; f++) {
    setMaxSpeedArray(f, homeSetStepperDirection[f] * 1000);
    setSpeedArray(f, homeSetStepperDirection[f] * 50); 
  }

  // allow stepper driver initialization
  delay(5);

  // set pin mode for each stepper endstop
  pinMode(endStopPin[0], INPUT_PULLUP);
  pinMode(endStopPin[1], INPUT_PULLUP);
  pinMode(endStopPin[2], INPUT_PULLUP);
  
  // servo setup
  servo1.attach(9);

  // servo home position (change for each servo)
  servo1.write(90);
  delay(250);

  // set home position for each stepper
  for(f = 0; f < STEPPER_COUNT; f++) {
    // skip set home if direction is zero
    if(homeSetStepperDirection[f] == 0) continue;
    // set home for selected stepper
    while(!homeSetStepper[f]) {
      setHomeStepper(f);
      nh.spinOnce();
    }  
  }
}

//
// Wrappers to AccelStepper methods for use with arrays
// To add a stepper add one entry for its index in each function 
//

void runSpeedArray(int stepper) {
  switch(stepper) {
    case 0: stepper1.runSpeed(); break;
    case 1: stepper2.runSpeed(); break;
    case 2: stepper3.runSpeed(); break;
  }  
}

void setCurrentPositionArray(int stepper, int position) {
  switch(stepper) {
    case 0: stepper1.setCurrentPosition(position); break;
    case 1: stepper2.setCurrentPosition(position); break;
    case 2: stepper3.setCurrentPosition(position); break;
  }  
}

long getCurrentPositionArray(int stepper) {
  long position = 0;
  switch(stepper) {
    case 0: position = stepper1.currentPosition(); break;
    case 1: position = stepper2.currentPosition(); break;
    case 2: position = stepper3.currentPosition(); break;
  }
  return position; 
}

void moveToArray(int stepper, int target) {
  switch(stepper) {
    case 0: stepper1.moveTo(target); break;
    case 1: stepper2.moveTo(target); break;
    case 2: stepper3.moveTo(target); break;
  }  
}

void setMaxSpeedArray(int stepper, int speed) {
  switch(stepper) {
    case 0: stepper1.setMaxSpeed(speed); break;
    case 1: stepper2.setMaxSpeed(speed); break;
    case 2: stepper3.setMaxSpeed(speed); break;
  }  
}

void setSpeedArray(int stepper, int speed) {
  switch(stepper) {
    case 0: stepper1.setSpeed(speed); break;
    case 1: stepper2.setSpeed(speed); break;
    case 2: stepper3.setSpeed(speed); break;
  }  
}

void runSpeedToPositionArray(int stepper) {
  switch(stepper) {
    case 0: stepper1.runSpeedToPosition(); break;
    case 1: stepper2.runSpeedToPosition(); break;
    case 2: stepper3.runSpeedToPosition(); break;
  }  
}

//
// Home setting procedure to do at startup
// Configurable for each stepper
//

void setHomeStepper(int stepper) {

  if(!homeSetOffsetStepper[stepper]) {

    // do while endstop is not reached
    if(digitalRead(endStopPin[stepper])) {
      runSpeedArray(stepper);
    } else {
      // endstop reached. set home offset for stepper
      setCurrentPositionArray(stepper, 0);
      homeSetOffsetStepper[stepper] = true;
      stepperTarget[stepper] = homeSetStepperOffset[stepper];
      moveToArray(stepper, stepperTarget[stepper]);
      setMaxSpeedArray(stepper, 1000);
      setSpeedArray(stepper, 50);
    }
  } else {
    // move to offset after endstop reached
    runSpeedToPositionArray(stepper);
    if(getCurrentPositionArray(stepper) == stepperTarget[stepper]) {
      // offset reached. end home set for stepper
      homeSetStepper[stepper] = true;
      setCurrentPositionArray(stepper, 0);
    }    
  }
}

//
// Process rotatation and target for each stepper
//

void processStepper(int stepper) {

  // set a new target if it is available
  if(stepperNewTarget[stepper]) {
    moveToArray(stepper, stepperTarget[stepper]);
    setMaxSpeedArray(stepper, 1000);
    setSpeedArray(stepper, 50);
    stepperIsMoving[stepper] = true;
    stepperNewTarget[stepper] = false;    
  }

  // rotate stepper if required
  if(stepperIsMoving[stepper]) {
    // check first if we reached the endstop
    if(digitalRead(endStopPin[stepper])) {
      runSpeedToPositionArray(stepper);
      // check if target is reached
      if(getCurrentPositionArray(stepper) == stepperTarget[stepper]) {
        // target reached disable move flag
        stepperIsMoving[stepper] = false;
      }
    } else {
      // endstop reached disable move flag
      stepperIsMoving[stepper] = false;
    }
  }
}

//
// main loop
//

void loop() {

  int f;

  // process target and stepper rotation
  for(f = 0; f < STEPPER_COUNT; f++) {
    processStepper(f);
  }

  nh.spinOnce();
  delay(1);
}
