#include <ros/ros.h>
#include "std_msgs/String.h"
#include "sensor_msgs/Range.h"
#include "rosserial_arduino/Adc.h"

static double v_motor, v_pc, i_motor,i_pc;
static float level;

void chatterCallback(const sensor_msgs::Range::ConstPtr& range_msg)
{
  level = range_msg->range;
  //printf("level:[%.2f]", level);
}

void chatterCallback2(const rosserial_arduino::Adc::ConstPtr& adc_msg)
{
  v_motor = adc_msg->adc0;
  v_pc = adc_msg->adc1;
  i_motor = adc_msg->adc2;
  i_pc = adc_msg->adc3;

  printf("v_motor=[%.2f]->v_pc=[%.2f]->i_mtr=[%.2f]->i_pc=[%.2f]\n",v_motor,v_pc,i_motor,i_pc);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "read_sensor");
  ros::NodeHandle sub;
  ros::NodeHandle pub;

  ros::Subscriber sub1 = sub.subscribe("ultrasound", 1000, chatterCallback);
  ros::Subscriber sub2 = sub.subscribe("adc_data", 1000, chatterCallback2);

  ros::Publisher chatter_pub;
  ros::Publisher chatter_pub2;
  //ros::Publisher chatter_pub = pub.advertise<sensor_msgs::Range>("data_jarak", 1000);
  //ros::Publisher chatter_pub2 = pub.advertise<sensor_msgs::Range>("data_adc", 1000);

  chatter_pub2 = pub.advertise<rosserial_arduino::Adc>("data_adc", 1000);
  chatter_pub = pub.advertise<sensor_msgs::Range>("data_jarak",1000);
  ros::Rate loop_rate(10);

  while (ros::ok())
  {
    sensor_msgs::Range data;
    rosserial_arduino::Adc data2;
    data.range = level;
    data2.adc0 = v_motor;
    data2.adc1 = v_pc;
    data2.adc2 = i_motor;
    data2.adc3 = i_pc;
    //printf("jarak:%.2f\n",data.range);
    chatter_pub.publish(data);
    chatter_pub2.publish(data2);
    ros::spinOnce();
    loop_rate.sleep();
  }
  ros::spin();
  return 0;
}
