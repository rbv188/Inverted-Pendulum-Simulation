#include "ros/ros.h"
#include "gazebo_tutorials/inverted_pendulum_states.h"


void chatterCallback(const gazebo_tutorials::inverted_pendulum_states& msg)
{
  float torque = msg.wheel_torques;
  float angle = msg.pendulum_angle;
  float angle_velocity = msg.pendulum_angle_velocity;
  float speed = msg.chasis_speed;
  ROS_INFO("%f,%f,%f,%f\n", torque,angle,angle_velocity,speed);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "listener");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("pendulum_states", 1000, chatterCallback);
  ros::spin();
  return 0;
}
