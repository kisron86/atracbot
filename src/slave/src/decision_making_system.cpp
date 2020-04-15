#include <ros/ros.h>
#include "sensor_msgs/Range.h"
#include "std_msgs/Float64.h"

#include <iostream>
#include <fstream>
#include <sstream>

using namespace std;

static double level;
static double Mtr_Est;
static double PC_Est;

/*
0 - 5 menit		Sangat Sebentar		1
5 - 15 menit	Sebentar			2
15 - 30 menit	Sedang				3
30 - 60 menit	Lama				4
> 60 Menit		Sangat Lama			5


0 - 7 cm		Sangat Sedikit		1
8 - 15 cm		Sedikit				2
16 - 23 cm		Medium				3
24 - 31 cm		Banyak				4
32 - 38 cm		Sangat Banyak		5

*/

int result = 0;
float p[2], px[2], py[2], pz[2];
float yp, ypx, ypy, ypz;
float np, npx, npy, npz;
float nr, yr;

double getBatteryLevel(double f){
  if (f <= 5.0){
    return 1;
  }
  else if (f>5.0 && f <= 15.0){
    return 2;
  }
  else if (f>15.00 && f <= 30.0){
    return 3;
  }
  else if (f>30.00 && f <= 60.0){
    return 4;
  }
  else if (f>60.00){
    return 5;
  }
  else return 1;
}

double getSampahLevel(double i){
  if (i >= 0 && i <= 7){
    return 1;
  }
  else if (i>7 && i <= 15){
    return 2;
  }
  else if (i>15 && i <= 23){
    return 3;
  }
  else if (i>23 && i <= 31){
    return 4;
  }
  else if (i>31 && i <= 38){
    return 5;
  }
  else return 1;
}

void calc()
{
      const int x = 30;
      const int y = 4;

      int map[x][y] = {
        { 1, 1, 2, 0 }, //1
        { 1, 1, 5, 0 }, //2
        { 1, 3, 1, 0 }, //3
        { 1, 3, 5, 0 }, //4
        { 1, 4, 4, 0 }, //5
        { 1, 5, 3, 0 }, //6
        { 2, 1, 1, 0 }, //7
        { 2, 1, 5, 0 }, //8
        { 2, 2, 4, 1 }, //9
        { 2, 4, 2, 1 }, //10
        { 2, 5, 1, 1 }, //11
        { 2, 5, 5, 1 }, //12
        { 3, 1, 2, 0 }, //13
        { 3, 2, 1, 1 }, //14
        { 3, 2, 5, 0 }, //15
        { 3, 3, 5, 0 }, //16
        { 3, 4, 4, 1 }, //17
        { 3, 5, 4, 1 }, //18
        { 4, 1, 1, 0 }, //19
        { 4, 2, 1, 1 }, //20
        { 4, 2, 3, 1 }, //21
        { 4, 3, 4, 1 }, //22
        { 4, 4, 4, 1 }, //23
        { 4, 5, 5, 0 }, //24
        { 5, 1, 3, 0 }, //25
        { 5, 2, 3, 1 }, //26
        { 5, 3, 1, 1 }, //27
        { 5, 3, 4, 1 }, //28
        { 5, 4, 4, 1 }, //29
        { 5, 5, 4, 1 }  //30
      };



      for (int i = 0; i<30; i++){
        if (map[i][3] == 1) p[1] = p[1] + 1;
        else p[0] = p[0] + 1;

        if (map[i][0] == getBatteryLevel(Mtr_Est) && map[i][3] == 1) px[1] = px[1] + 1;
        else if (map[i][0] == getBatteryLevel(Mtr_Est) && map[i][3] == 0) px[0] = px[0] + 1;

        if (map[i][1] == getBatteryLevel(PC_Est) && map[i][3] == 1) py[1] = py[1] + 1;
        else if (map[i][1] == getBatteryLevel(PC_Est) && map[i][3] == 0) py[0] = py[0] + 1;

        if (map[i][2] == getSampahLevel(level) && map[i][3] == 1) pz[1] = pz[1] + 1;
        else if (map[i][2] == getSampahLevel(level) && map[i][3] == 0) pz[0] = pz[0] + 1;

      }

      yp = p[1] / x;
      np = p[0] / x;
      ypx = px[1] / p[1];
      npx = px[0] / p[0];
      ypy = py[1] / p[1];
      npy = py[0] / p[0];
      ypz = pz[1] / p[1];
      npz = pz[0] / p[0];
      yr = yp * ypx * ypy * ypz;
      nr = np * npx * npy * npz;

      result = (yr >= nr ? 1 : 0);
      if (result == 1){
          //  cout << " RUN " << endl;
          //  m_decision.data = 0.0;
          //  m_decisionOut.write();
          }
          else{
          //  cout << " STOP " << endl;
          //  m_decision.data = 1.0;
          //  m_decisionOut.write();
          }
      cout << yr << " " << nr << " " <<(result == 1 ? "RUN" : "STOP") << endl;
      p[0] = p[1] = 0.0;
      px[0] = px[1] = 0.0;
      py[0] = py[1] = 0.0;
      pz[0] = pz[1] = 0.0;
}
void Motor_Est(const std_msgs::Float64::ConstPtr& msg1)
{
  Mtr_Est = msg1->data;
}

void PC_Estimation(const std_msgs::Float64::ConstPtr& msg2)
{
  PC_Est = msg2->data;
}

void Decision(const sensor_msgs::Range::ConstPtr& msg)
{
  level = msg->range;
  //printf("jarak: %f",level);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "decision_making_system");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe("data_jarak", 1000, Decision);
  ros::Subscriber sub1 = nh.subscribe("motor_estimation", 1000, Motor_Est);
  ros::Subscriber sub2 = nh.subscribe("pc_estimation", 1000, PC_Estimation);
  ros::Rate loop_rate(10);

  while (ros::ok())
  {
    calc();
    ros::spinOnce();
    loop_rate.sleep();
  }
  ros::spin();
  return 0;
}
