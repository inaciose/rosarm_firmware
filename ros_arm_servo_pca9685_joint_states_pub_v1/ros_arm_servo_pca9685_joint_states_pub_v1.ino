// ROS DRIVER FOR EEZYBOTARM MK1/MK2

// Servo control
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// ROS
#include <ros.h>
#include <std_msgs/UInt16.h>
#include <sensor_msgs/JointState.h>

// For rosserial arduino buffer
#define USE_USBCON

// Total servos in robotic arm
#define SERVO_COUNT 4

// Analog servos run at ~50 Hz updates
#define SERVO_FREQ 50

int servomin[SERVO_COUNT] = {100, 100, 100, 130};
int servomax[SERVO_COUNT] = {470, 470, 470, 520};
int servostart[SERVO_COUNT] = {90, 90, 90, 90};
int servopos[SERVO_COUNT] = {90, 90, 90, 90};

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

ros::NodeHandle  nh;
sensor_msgs::JointState robot_state;

char *a[] = {"X", "Y", "Z", "G"}; // Z-Axis, Y-Axis, X-Axis, G-ripper
float pos[4]={0},vel[4]={0},eff[4]={0}; /// stores arduino time

ros::Publisher js_publisher("joint_state", &robot_state);

//
// ROS Callback functions
//

void servo_cbx( const std_msgs::UInt16& cmd_msg) {
  int pulse;
  servopos[1] = cmd_msg.data;
  // map degrees to pulse length
  pulse = map(cmd_msg.data, 0, 180, servomin[1], servomax[1]);
  pwm.setPWM(1, 0, pulse);
}

void servo_cby( const std_msgs::UInt16& cmd_msg) {
  int pulse;
  servopos[0] = cmd_msg.data;
  // map degrees to pulse length
  pulse = map(cmd_msg.data, 0, 180, servomin[0], servomax[0]);
  pwm.setPWM(0, 0, pulse);
}

void servo_cbz( const std_msgs::UInt16& cmd_msg) {
  int pulse;
  servopos[2] = cmd_msg.data;
  // map degrees to pulse length
  pulse = map(cmd_msg.data, 0, 180, servomin[2], servomax[2]);
  pwm.setPWM(2, 0, pulse);  
}

void servo_cbg( const std_msgs::UInt16& cmd_msg) {
  int pulse;
  servopos[3] = cmd_msg.data;
  // map degrees to pulse length
  pulse = map(cmd_msg.data, 0, 180, servomin[3], servomax[3]);
  pwm.setPWM(1, 0, pulse); 
}

ros::Subscriber<std_msgs::UInt16> subx("x", servo_cbx);
ros::Subscriber<std_msgs::UInt16> suby("y", servo_cby);
ros::Subscriber<std_msgs::UInt16> subz("z", servo_cbz);
ros::Subscriber<std_msgs::UInt16> subg("g", servo_cbg);

void setup() {

  nh.initNode();
  nh.advertise(js_publisher);
  nh.subscribe(subx);
  nh.subscribe(suby);
  nh.subscribe(subz);
  nh.subscribe(subg);


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

  // Initialize the joint State message
  robot_state.name_length = 4;
  robot_state.velocity_length = 4;
  robot_state.position_length = 4; /// here used for arduino time
  robot_state.effort_length = 4; /// here used for arduino time
  
  robot_state.name = a;
  robot_state.position = pos;
  robot_state.velocity = vel;
  robot_state.effort = eff;
}

void update_vals() {
  pos[0]= servopos[1];
  pos[1]= servopos[0];
  pos[2]= servopos[2];
  pos[3]= servopos[3];
  robot_state.header.stamp=nh.now();
  robot_state.name = a;
  robot_state.position = pos;
  robot_state.velocity = vel;
  robot_state.effort = eff;
}

void loop() {
  update_vals();
  // Publish robot jointstate
  js_publisher.publish( &robot_state );
  nh.spinOnce();
  delay(100);
}
