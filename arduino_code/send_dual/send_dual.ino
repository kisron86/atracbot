#include "NewPing.h" //ultrasonic sensor specific library
#include <ros.h>
#include <sensor_msgs/Range.h>
#include <std_msgs/Float64.h>

#if (ARDUINO >= 100)
 #include <Arduino.h>
#else
 #include <WProgram.h>
#endif
#include <rosserial_arduino/Adc.h>
 
//define the pins
#define trigPin 5
#define echoPin 4
 
//define the constants
#define maxDistance 400
#define intervalR 200
 
NewPing sonar(trigPin, echoPin, maxDistance);
 
//variables
float range;
unsigned long range_timer;
float duration, distance;
 
ros::NodeHandle nh;
 
sensor_msgs::Range range_msg;
//std_msgs::Float64 float_msg;
rosserial_arduino::Adc adc_msg;

ros::Publisher adc_data("adc_data", &adc_msg); 
ros::Publisher pub_range_ultrasound("/ultrasound", &range_msg);
//ros::Publisher pub_range_adc("/adc_data", &float_msg);
 
//float number = 12.20;

float returnDistance()
{
 
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
 
 
  duration = pulseIn(echoPin, HIGH);
 
  // convert the time into a distance
  return duration/58; //    duration/29/2, return centimeters
}
 
void setup() {
     //pin mode for the ultrasonic sensor
   pinMode(trigPin, OUTPUT);
   pinMode(echoPin, INPUT);
 
   nh.initNode();
   nh.advertise(pub_range_ultrasound);
   //nh.advertise(pub_range_adc);
   nh.advertise(adc_data);
}

long adc_timer;

//We average the analog reading to elminate some of the noise
int averageAnalog(int pin){
  int v=0;
  for(int i=0; i<4; i++) v+= analogRead(pin);
  return v/4;
}

void baca_adc(){
  adc_msg.adc0 = averageAnalog(0);
  adc_msg.adc1 = averageAnalog(1);
  adc_msg.adc2 = averageAnalog(2);
  adc_msg.adc3 = averageAnalog(3);
  adc_msg.adc4 = averageAnalog(4);
  adc_msg.adc5 = averageAnalog(5);
}
 
void loop() {
  unsigned long currentMillis = millis();
 
  if (currentMillis-range_timer >= intervalR)
  {
    range_timer = currentMillis+intervalR;
   
    range_msg.range = returnDistance();
    pub_range_ultrasound.publish(&range_msg);
    }
   //float_msg.data = number;
   //pub_range_adc.publish( &float_msg );
   baca_adc();
   adc_data.publish(&adc_msg);
   nh.spinOnce();
}
