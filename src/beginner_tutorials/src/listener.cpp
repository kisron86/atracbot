#include "ros/ros.h"
#include "beginner_tutorials/Num.h"

using namespace beginner_tutorials;

void chatterCallback(const Num::ConstPtr &num)
{
  ROS_INFO("I heard: [%d]",  num->age);
}
int main(int argc, char **argv)
{
  ros::init(argc, argv, "listener");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);
  ros::spin();
  return 0;
}
