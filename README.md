# ros_arm_firmware
ROS generic robotic arm firmware for Arduino with rosserial-arduino

Generic firmware to be used on robotic arms using steppers or servos

- Servos using Arduino Servo Library
- Servos using PCA9685 module and adafruit library
- Steppers using AccelStepper Library, and Servo library for servo in gripper

These sketches require the rosserial on rosmaster, and the rosserial-arduino library installed on Arduino IDE.

Require an robot description URDF file to use the ROS rviz or the ROS moveit. 

For URDF see http://wiki.ros.org/urdf/Tutorials

For rviz see http://wiki.ros.org/rviz

For moveit see http://docs.ros.org/kinetic/api/moveit_tutorials/html/index.html

See also the ntbd core packages repository at: https://github.com/inaciose/ntbd_base
