#include "ros/ros.h"
#include "std_msgs/String.h"

using namespace ros;

int main(int argc, char **argv)
{
  init(argc, argv, "publisher");
  NodeHandle nh;

  Publisher chatter_pub = nh.advertise<std_msgs::String>("chatter", 1000);

  Rate loop_rate(10);
  while (ros::ok())
  {
    std_msgs::String msg;
    msg.data = "hello world";

    chatter_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
  }

  return 0;
}
