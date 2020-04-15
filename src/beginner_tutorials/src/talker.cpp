#include "ros/ros.h"
#include "beginner_tutorials/Num.h"

using namespace beginner_tutorials;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "talker");
  ros::NodeHandle n;
  ros::Publisher chatter_pub = n.advertise<Num>("chatter", 1000);
  ros::Rate loop_rate(10);
  int count = 0;
  while(ros::ok())
  {
    Num data;
    ROS_INFO("%d",data.age);
    chatter_pub.publish(data);
    ros::spinOnce();
    loop_rate.sleep();
    ++count;
   }
   return 0;
}
