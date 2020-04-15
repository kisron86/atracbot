/* Subscriber string data
 * #include "ros/ros.h"
#include "std_msgs/String.h"

void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("I read: [%s]", msg->data.c_str());
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "subscriber");
  ros::NodeHandle nh;

  ros::Subscriber sub = nh.subscribe("chatter", 1000, chatterCallback);

  ros::spin();

  return 0;
}
*/

// Subscriber float data
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "sensor_msgs/Range.h"
#include "rosserial_arduino/Adc.h"

static double Vmtr, Vpc, Imtr,Ipc;
static double level;

//1st
static int i = 0;
static double x_est_last = 0, P_last = 0;	//initial values for the kalman filter
static double Q = 0.022, R = 0.617;			//the noise in the system

static double K, P, P_temp;
static double x_temp_est, x_est;
static double z_measured; 			//the 'noisy' value we measured
static double z_real = 200; 		//the ideal value we wish to measure

static double sum_error_kalman = 0, sum_error_measure = 0;

//2nd
static int i2 = 0;
static double x_est_last2 = 0, P_last2 = 0;	//initial values for the kalman filter
static double Q2 = 0.022, R2 = 0.617;			//the noise in the system

static double K2, P2, P_temp2;
static double x_temp_est2, x_est2;
static double z_measured2; 			//the 'noisy' value we measured
static double z_real2 = 200; 		//the ideal value we wish to measure

static double sum_error_kalman2 = 0, sum_error_measure2 = 0;

//prediction
#define v_min 11.1

static double x0, xx;
static double x_titik;
static double xx1;
static double xx11;
static double delta_t = 5.0;
static double delta_tnew = 0.1;
static double slope;
static double estimasi;

//prediction2
static double x02, xx2;
static double x_titik2;
static double xx12;
static double xx112;
static double delta_t2 = 5.0;
static double delta_tnew2 = 0.1;
static double slope2;
static double estimasi2;

void chatterCallback(const sensor_msgs::Range::ConstPtr& range_msg)
{
  level = range_msg->range;
  printf("level:[%.2f]", level);
}

void chatterCallback2(const rosserial_arduino::Adc::ConstPtr& adc_msg)
{
  Vmtr = adc_msg->adc0;
  Vpc = adc_msg->adc1;
  Imtr = adc_msg->adc2;
  Ipc = adc_msg->adc3;
  double Pmtr = Vmtr * Imtr;
  double Ppc  = Vpc * Ipc;

  double vNew_mtr, vNew_pc;

  //1st kalman
    x_temp_est = x_est_last;
    P_temp = P_last + Q;
    K = P_temp * (1.0 / (P_temp + R));		//calculate the Kalman gain
    //ganti coba
    //z_measured needs to be read from console
    z_measured = Vmtr;

    x_est = x_temp_est + K * (z_measured - x_temp_est);
    P = (1 - K) * P_temp;

    P_last = P;
    x_est_last = x_est;

  //2nd kalman
    x_temp_est2 = x_est_last2;
    P_temp2 = P_last2 + Q2;
    K2 = P_temp2 * (1.0 / (P_temp2 + R2));		//calculate the Kalman gain
    //ganti coba
    //z_measured needs to be read from console
    z_measured2 = Vpc;

    x_est2 = x_temp_est2 + K2 * (z_measured2 - x_temp_est2);
    P2 = (1 - K2) * P_temp2;

    P_last2 = P2;
    x_est_last2 = x_est2;

    //cout << " " << x_est << " " << z_measured << endl;
    //Sleep(1000);

    vNew_mtr = x_est / Imtr;
    vNew_pc = x_est2 / Ipc;


  //prediction1
    x0 = 12.60;
    xx = x_est;
    //xx = x_est;

    x_titik = (xx - x0) / 5;

    //menghitung x5'
    xx11 = x0 + x_titik*delta_t;

    //menghitung x 5,1
    double xx111 = xx11 + x_titik*delta_tnew;

    //menghitung slope
    slope = (xx111 - xx11) / delta_tnew;

    //menghitung estimasi waktu
    estimasi = (xx111 - v_min) / slope*(-1.0);

    //prediction2
    x02 = 12.60;
    xx2 = x_est2;

    x_titik2 = (xx2 - x02) / 5;

    //menghitung x5'
    xx112 = x02 + x_titik2*delta_t2;

    //menghitung x 5,1
    double xx1112 = xx112 + x_titik2*delta_tnew2;

    //menghitung slope
    slope2 = (xx1112 - xx112) / delta_tnew2;

    //menghitung estimasi waktu
    estimasi2 = (xx1112 - v_min) / slope2*(-1.0);

    if (estimasi < 0.0){ estimasi = 0.0; }
    if (estimasi2 < 0.0){ estimasi2 = 0.0; }
    printf("kalman: %f\n",x_est);

    //cout << "Kalman : " << x_est << " " << x_est2 << " Asli :" << Vmtr << " " << Vpc <<" " << estimasi<< " " << estimasi2<< endl;

//  printf("v_motor=[%.2f]->v_pc=[%.2f]->i_mtr=[%.2f]->i_pc=[%.2f]\n",v_motor,v_pc,i_motor,i_pc);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "subscriber");
  ros::NodeHandle nh;

  ros::Subscriber sub = nh.subscribe("data_jarak", 1000, chatterCallback);
  ros::Subscriber sub2 = nh.subscribe("data_adc", 1000, chatterCallback2);

  ros::Publisher chatter_pub_motor = nh.advertise<std_msgs::Float64>("motor_estimation", 1000);
  ros::Publisher chatter_pub_pc = nh.advertise<std_msgs::Float64>("pc_estimation", 1000);
  ros::Rate loop_rate(10);

  while (ros::ok())
  {
    std_msgs::Float64 est1;
    std_msgs::Float64 est2;

    est1.data = estimasi;
    est2.data = estimasi2;

    chatter_pub_motor.publish(est1);
    chatter_pub_pc.publish(est2);
    ros::spinOnce();
    loop_rate.sleep();
  }
  ros::spin();
  return 0;
}
