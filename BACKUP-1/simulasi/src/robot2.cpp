#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <stdio.h>
#include <iostream>
#include <stdlib.h> 
#include <time.h> 
#include <math.h>
#include <signal.h>
#include <cstdlib>

using namespace std;
#define pi 3.14
#define map(x,in_min,in_max,out_min,out_max) ( (x-in_min) * (out_max-out_min) / (in_max-in_min) + out_min )
#define constrain(nilaix,bawah,atas) ( (nilaix)<(bawah) ? (bawah) : ( (nilaix)>(atas) ? (atas) : (nilaix) ) )


enum kondisi{satu,dua,tiga,empat};
kondisi stepK;



ros::Publisher motorf1pub;
ros::Publisher motorf2pub;
ros::Publisher motorf3pub;

std_msgs::Float32 motorf1_msg;
std_msgs::Float32 motorf2_msg;
std_msgs::Float32 motorf3_msg;

//posisi now
ros::Publisher posf1pub;
ros::Publisher posf2pub;
ros::Publisher posf3pub;

std_msgs::Float32 posf1_msg;
std_msgs::Float32 posf2_msg;
std_msgs::Float32 posf3_msg;


ros::Time current_time,prev_time;
ros::Time timeNow,timePrev;
float speedMotor1,speedMotor2,speedMotor3;
struct varRobot{
	float Motor1;
	float Motor2;
	float Motor3;

	float posx;
	float posy;
	float post;

	float pos1x;
	float pos1y;
	float pos1status;
};

struct varRobot robot,friends;



struct varObs{

	float posx;
	float posy;
	float post;
};

struct varObs obstacle[6];



struct sPIDtarget 
{
	float error,errorTheta;
	float Derror;
	float Ierror;
	float proportional,proportionalTheta;
	float derivative,derivativeTheta;
	float integral,integralTheta;
	float sumError,sumErrorTheta;
	float lasterror,lasterrorTheta;
	float speed;
	float errteta;

	float distanceX;
	float distanceY;
	float distance;

	float posx;
	float posy;
	float post;
	float theta;
	float speedTheta;
	float speedx,speedy,speedt;
	float forcealpha;
};
struct sPIDtarget target,targetf,targetk,targetkx; 



struct sFuzzy 
{
	float distoObsx;
	float distoObsy;
	float distoObs;
};
struct sFuzzy dis; 




float dt;



char step = 1;
void pidTarget(int xTarget,int yTarget,int tTarget);



char ts;
void  timeSamplingCallback(const ros::TimerEvent&)
  {
   ts=1;
  }

double getDegree(int x,int y){
double deg;
	deg=atan2((double)x,(double)y)*57.2957795;
return deg;
}


void motoinActfriend(float speed1,float speed2,float speed3){
	    motorf1_msg.data = speed1;
		motorf2_msg.data = speed2;
		motorf3_msg.data = speed3;

		motorf1pub.publish(motorf1_msg);
		motorf2pub.publish(motorf2_msg);
		motorf3pub.publish(motorf3_msg);
}



void mySigintHandler(int sig){
	
	for(int i=0;i<=5; i++){
	speedMotor1=0;speedMotor2=0;speedMotor3=0;

	//motoinAct(speedMotor1,speedMotor2,speedMotor3);
	motoinActfriend(speedMotor1,speedMotor2,speedMotor3);
	ROS_WARN("motor1 = %.3f | motor2 = %.3f | motor3 = %.3f",speedMotor1,speedMotor2,speedMotor3);
	
	}
	ros::shutdown();
}


void robotgerakFriend(float velX, float velY, float velW)
{	//Refference from Al-kwarizmi omni directional
	//Jarak roda ke pusat = 27.7350 cm >>18cm
	//VelX = w * r;//VelY = w * r; //VelW = w * r;//	r(cm)

	velX/=100;velY/=100;velW/=100;//convert from cm/s to m/s
	const float wheelRadius = 0.1 ; //5CM//0.05 meter >> 10cm//0.1m
	
	//INVERS KINEMATIC 
	//friends.Motor1 = -0.3333*velX + 0.5774*velY + 0.09245*velW; //Notation m/s
	//friends.Motor2 = -0.3333*velX - 0.5774*velY + 0.09245*velW;
	//friends.Motor3 =  0.6667*velX + 0.09245*velW;
	friends.Motor1 = -0.5*velX + 0.5774*velY + 0.09245*velW; //Notation m/s
	friends.Motor2 = -0.5*velX - 0.5774*velY + 0.09245*velW;
	friends.Motor3 =  0.6667*velX + 0.09245*velW;
	
	//Convert from m/s to RPM
	friends.Motor1 = (friends.Motor1/(wheelRadius)); //Notation Rad/s
	friends.Motor2 = (friends.Motor2/(wheelRadius));
	friends.Motor3 = (friends.Motor3/(wheelRadius));
	
	//Limit the RPM
	//friends.Motor1 = constrain(friends.Motor1,-450,450);
	//friends.Motor2 = constrain(friends.Motor2,-450,450);
	//friends.Motor3 = constrain(friends.Motor3,-450,450);
	printf("MotorF 1 = %.1f || MotorF 2 = %.1f || MotorF 3 = %.1f \n",friends.Motor1,friends.Motor2,friends.Motor3);
motoinActfriend(friends.Motor1,friends.Motor2,friends.Motor3);
}



struct sPoint
	{
		float x;
		float y;
		float r;
	};
sPoint Point[10],substarget;


struct sBezier
{
	float ControlPoint;
	float x,y;
};
sBezier Bezier;


int minimaObstacle;
char flagSampai ;
int GenerateSubstarget(int targx,int targy){

	const float Rrobot = 20;
	const float Robs   = 1;

	

	float gap = 5;
	//const float targetx= 300;
	//const float targety= 300;

	 //float targetx= 250;
	 //float targety= 500;

	 float targetx= targx;
	 float targety= targy;


	 float robotx = robot.posx ;
 	 float roboty = robot.posy ;
 	 // float robotx = 101 ;
 	 // float roboty = 86 ;


 	//float robotx = 243 ;
 	//float roboty = 221 ;

	//const float obsx = 3.5;
	//const float obsy = 2.5;

	int jumlahObject = 1+1;
	int groupB[jumlahObject];
	int grouptestObject[jumlahObject];

	int indexBlocking=1;
	float tmp;
    bool flag=false;
    float ai[jumlahObject],bi[jumlahObject];

	//============STEP 1========= first obstructor//
	for(int i=1;i<jumlahObject;i++){

		//===========Generate Line (robot to target) Equation===============//
		static float p,q,r;

		static float mtarget;
		mtarget = (targety - roboty) / (targetx - robotx);
		
		p = -mtarget;
		q = 1;
		r = -roboty+mtarget*robotx;
		
		//===========calculate bi =======================//
		static float disPQ;
		disPQ = sqrt((double)p*(double)p + (double)q*(double)q );	
		bi[i] = (p*Point[i].x + q*Point[i].y + r)/(float)disPQ;

	
		static float xp,yp,xpp,ypp,distxp;
		xp = targetx - robotx ;   yp = targety - roboty;
		xpp= Point[i].x - robotx; ypp= Point[i].y - roboty;

		distxp =  sqrt((double)xp*(double)xp + (double)yp*(double)yp );
		ai[i]  = ((xp * xpp) + (yp * ypp))/distxp;  


		//Determine Group Blocking
	    static float Distr,Distrx,Distry;
	    Distrx = targetx - robotx;
	    Distry = targety - roboty;
		Distr = sqrt((double)Distrx*(double)Distrx + (double)Distry*(double)Distry);
		//printf("ai[%d] = %f || bi[%d] = %f\n ",i,ai[i],i,bi[i]);	
		
		if( (0<ai[i] && ai[i]<Distr) && (fabs(bi[i]) < Rrobot+Point[i].r))
		{	flag=true;
			//printf("a[%d] = %f || b[%d] =%f || distance[%d] = %f \n",i,ai[i],i,bi[i],i,Distr);
			//printf("indexobstacle %d || indexBlocking = %d \n",i,indexBlocking);
			groupB[indexBlocking]=i;//printf("groupB[%d] = %d\n",indexBlocking,groupB[indexBlocking]);
			indexBlocking++;
		}
			
		 grouptestObject[i]=i; //input data for step2;

	}



	static int f;tmp=0;f=0;
	minimaObstacle = f;

	//Rejected
	if(flag==false){//printf("NO blocking\n");
	substarget.x = targetx;substarget.y = targety; 
			  // printf("substargetx = %f  || substargety = %f \n",substarget.x,substarget.y);
				return 0;}

	//printf("indexBlocking = %d ",indexBlocking);
	//find first obstructor with min groupB

	for(int i=1;i<indexBlocking;i++){ 

		if(i==1){ f=groupB[i]; tmp = ai[groupB[i]];}
		else if(tmp>ai[groupB[i]]){f=groupB[i];tmp=ai[groupB[i]]; 
		}
	}

  minimaObstacle = f;

  //=======STEP 2 ================Grouping//
	int groupG[jumlahObject];tmp=0;
	int indexgroupG=1;
	int g=1,n=jumlahObject;
	int tempG=0,tempN=0;
	char constant,counter=0;
	
	int jumlahtestObject = 0;

	counter=0;
	groupG[1]=f;

	jumlahtestObject = jumlahObject - 1;
	tempG = g;


	//printf("\n");
	//reject f from test object
	for(int i = 1;i<=jumlahtestObject;i++)
	{	
		//printf("f: %d\n", f);
		
		if(grouptestObject[i]==f){
			for(int j=0;j<jumlahtestObject;j++)	//Geser
			{
				grouptestObject[i+j] = grouptestObject[i+j+1];
			}
			jumlahtestObject--;
		} 
		//printf("grouptestObject[%d]: %d\n",i, grouptestObject[i]);
	}
	
	//printf("jumlah test object = %d \n\n",jumlahtestObject);
	 while(1){

		for(int i=1;i<=g; i++){
		
			for(int j=1;j<=jumlahtestObject;j++){
			
				float distancex_ =  Point[groupG[i]].x - Point[grouptestObject[j]].x;
				float distancey_ =  Point[groupG[i]].y - Point[grouptestObject[j]].y;
				float distance_  = sqrt((double)distancex_* (double)distancex_+ (double)distancey_*(double)distancey_ );	
		
				//printf("groupG[%d] = %d || grouptestObject[%d] = %d\n",i,groupG[i],j,grouptestObject[j]);

				 if((distance_-Point[groupG[i]].r-Point[grouptestObject[j]].r)<2*Rrobot){ 
					
					groupG[i+1]= grouptestObject[j];
					tempG = g+1;
					for(int k=0;k<jumlahtestObject;k++){//Geser
						grouptestObject[j+k] = grouptestObject[j+k+1];
					}
					jumlahtestObject--;
					j--;
				 }	
			}
		}
		

	 if(g == tempG){if(counter++ >5){counter=0; indexgroupG = g;break;} }
	 else { g = tempG;counter=0; } 


	 }


	//printf("\ngroupG [1] = %d  || groupG [2] = %d  || indexgroupG %d \n ",groupG[1],groupG[2],indexgroupG);

	//int groupGleft[jumlahObject];int indexleft=0;
	//int groupGright[jumlahObject];int indexright=0;
	
  //=========STEP 3 ==============Location of substarget//
	 float maxpositive=0,maxnegative=0;
	//indexleft=1;
	//indexright=1;
	for(int i=1; i<=indexgroupG ;i++){
		
		if(bi[groupG[i]]<0){ //groupGright[indexright] = groupG[i]; indexright++;
							//printf("Masuk negative\n");
							maxnegative+=bi[groupG[i]]-Rrobot;}
		else if(bi[groupG[i]]>0){ //groupGleft[indexleft] = groupG[i]; indexleft++;//printf("Masuk positive\n");
							maxpositive+=bi[groupG[i]]+Rrobot;}
									//printf("bi[%d] = %f ",groupG[i],bi[groupG[i]]);

	}

	//printf("Max positive = %f || Max negative = %f \n",maxpositive,maxnegative);

	char sign ;
	if((fabs(maxnegative)-fabs(maxpositive))<0){sign=-1;//printf("Kanan\n");
						}
	else if((fabs(maxnegative)-fabs(maxpositive))>=0){sign=1;//printf("Kiri\n");
						}


	//printf(" side = %d\n",sign);

	//find largest alpha
	float alpha[jumlahObject];tmp=0;
	float largestAlpha;
	int indexLarge = 0;
	


		for(int i=1;i<=indexgroupG;i++){	
			
			float distanceI_x = Point[groupG[i]].x - robotx;
			float distanceI_y = Point[groupG[i]].y - roboty; 
			float distance_   = sqrt((double)distanceI_x*(double)distanceI_x + (double)distanceI_y*(double)distanceI_y); 
				
			float Atheta = (Rrobot+Point[groupG[i]].r) / distance_ ;
			if(Atheta>1){Atheta = 1;}
			else if(Atheta<-1){Atheta = -1;}
			alpha[i]          = atan(bi[groupG[i]]/ai[groupG[i]])*57.2957795 + (sign * asin(Atheta)*57.2957795);
			
			 distanceI_x = targetx - robotx;
			 distanceI_y = targety - roboty; 
			
			float theta =  getDegree( distanceI_x, distanceI_y);
		
			if(theta <0 ){alpha[i] = -alpha[i] ;}
	

			if(alpha[i]<0){alpha[i]-=gap;}
			else if(alpha[i]>0){alpha[i]+=gap;}
			
		
			if(i==1){largestAlpha = alpha[i]; tmp = fabs(alpha[i]);indexLarge = groupG[i];}
			else if (tmp<fabs(alpha[i])){ largestAlpha = alpha[i];tmp=fabs(alpha[i]);indexLarge = groupG[i];}
			
			}

	//printf("indexlargest = %d \n",indexLarge);
	//printf("largestAlpha = %f \n",largestAlpha);

	//find substarget coordinat
	float distancex = targetx - robotx;
	float distancey = targety - roboty;
	float distance  = sqrt((double)distancex*(double)distancex+ (double)distancey*(double)distancey );
		
	float distance_Ox 		= Point[indexLarge].x - robotx;
	float distance_Oy 		= Point[indexLarge].y - roboty;
	float distance_Olarge 	= sqrt((double)distance_Ox*(double)distance_Ox+ (double)distance_Oy*(double)distance_Oy );

	substarget.x = robotx +  (cos(largestAlpha/57.2957795)*(distancex * distance_Olarge/distance) - sin(largestAlpha/57.2957795)*(distancey * distance_Olarge/distance)); 		
	substarget.y = roboty +  (sin(largestAlpha/57.2957795)*(distancex * distance_Olarge/distance) + cos(largestAlpha/57.2957795)*(distancey * distance_Olarge/distance)); 

	//printf("\nHasil akhir \n");
	//printf("substargetx = %.3f  || substargety = %.3f \n",substarget.x,substarget.y);
}




void pidFriend(int xTarget,int yTarget,int tTarget)
{

	const float timeSampling  = 0.01;
	const float target_radius = 1; 
	const float other_robot_radius = 50;

	targetf.posx = (float)xTarget;
    targetf.posy = (float)yTarget;
	targetf.post = (float)tTarget;

	targetf.distanceX = targetf.posx-friends.posx;
	targetf.distanceY = targetf.posy-friends.posy;
	
	//targetf.distanceX = (targetf.posx-friends.posx)-30;
	//targetf.distanceY = (targetf.posy-friends.posy)-30;

	targetf.distance  =  (float)(sqrt((double)targetf.distanceX*(double)targetf.distanceX + (double)targetf.distanceY*(double)targetf.distanceY)); 
	targetf.theta	 =  (float) getDegree(targetf.distanceX,targetf.distanceY);

		targetf.forcealpha = (float)map(targetf.distance,0,300,0.8,1.5);
		
		targetf.error = targetf.distance * targetf.forcealpha;
		/*==========PID TARGET============speed*/
		//if(ts==1){ts=0;
			targetf.proportional = 3.0* targetf.error;
			targetf.derivative   = 0.01*(targetf.error - targetf.lasterror)/timeSampling;
			targetf.integral     = 0.0*targetf.sumError*timeSampling;
			targetf.lasterror    = targetf.error;
			targetf.sumError    += targetf.error;
			if(targetf.sumError>4000){targetf.sumError=4000;}
			else if(targetf.sumError<-4000){targetf.sumError=-4000;}
			targetf.speed=targetf.proportional+targetf.derivative+targetf.integral;
			/*=================END==================*/ 

			/*==========PID Heading============*/
			/*Input 0<>179 ~ -1<>-180*///CW Imu negative
			targetf.errorTheta        = (-targetf.post) - friends.post;
			targetf.sumErrorTheta    += targetf.errorTheta*timeSampling;
			if(targetf.sumErrorTheta>1000){targetf.sumErrorTheta=1000;}
			else if(targetf.sumErrorTheta<-1000){targetf.sumErrorTheta=-1000;}
			
			targetf.proportionalTheta = 4.8*targetf.errorTheta;
			targetf.derivativeTheta   = 0.2*(targetf.errorTheta-targetf.lasterrorTheta)/timeSampling;
			targetf.integralTheta	  = 0.01*targetf.sumErrorTheta;
			targetf.lasterrorTheta    = targetf.errorTheta;
			targetf.speedTheta = targetf.proportionalTheta+targetf.derivativeTheta+targetf.integralTheta;
	
			/*============END=================*/
			//}
		
		/*============Convert theta from worldFrame To robotFrame=========*/
		targetf.errteta = targetf.theta + friends.post;	
		if(targetf.errteta > 180 ){ targetf.errteta -= 360;}
		else if(targetf.errteta < -180 ){ targetf.errteta += 360;}
		/*==============END Convert theta from worldFrame To robotFrame===*/
		targetf.speedx = targetf.speed * sin(targetf.errteta/57.2957795);
		targetf.speedy = targetf.speed * cos(targetf.errteta/57.2957795);
		targetf.speedt = targetf.speedTheta;

		//batas kecepatan
		//targetf.speedx = constrain(targetf.speedx,-50,50);
		//targetf.speedy = constrain(targetf.speedy,-50,50);
		//targetf.speedt = constrain(targetf.speedt,-50,50);

		if(targetf.distance > target_radius)
		robotgerakFriend(targetf.speedx ,targetf.speedy,targetf.speedt);
		else
		{ 
			robotgerakFriend(0,0,0); 
		} 
}



void AvoidanceMotionf(int targetx,int targety){

	GenerateSubstarget(targetx,targety);
	pidFriend(substarget.x,substarget.y,0);

}


void odoXcallback(const std_msgs::Float32::ConstPtr& msg)
{robot.posx = msg->data;
 // ROS_INFO("I heard: [%f]", robot.posx);
  
}
void odoYcallback(const std_msgs::Float32::ConstPtr& msg)
{	robot.posy =  msg->data;
 //ROS_INFO("I heard: [%f]", robot.posy);
  
}
void odoTcallback(const std_msgs::Float32::ConstPtr& msg)
{ robot.post =  msg->data;
 // ROS_INFO("I heard: [%f]", robot.post);
}





void friendxCallback(const std_msgs::Float32::ConstPtr& msg)
{ 
 friends.posx=msg->data;
}

void friendyCallback(const std_msgs::Float32::ConstPtr& msg)
{ 
 friends.posy=msg->data;
}
void friendtCallback(const std_msgs::Float32::ConstPtr& msg)
{ 
 friends.post=msg->data;
}

//posisi now status
void pstatus(const std_msgs::Float32::ConstPtr& msg)
{ 
 friends.pos1status=msg->data;
}
void px(const std_msgs::Float32::ConstPtr& msg)
{ 
 friends.pos1x=msg->data;
}
void py(const std_msgs::Float32::ConstPtr& msg)
{ 
 friends.pos1y=msg->data;
}



int main(int argc,char**argv){

ros::init(argc,argv,"targetPosisi2",ros::init_options::NoSigintHandler);
	ros::NodeHandle nh;
	//signal(SIGINT, mySigintHandler);


//Create timer object 
float timer=0.001;//10 ms
ros::Timer timer1 = nh.createTimer(ros::Duration(timer), timeSamplingCallback);


 motorf1pub = nh.advertise<std_msgs::Float32> ("/velf/motor1",500);
 motorf2pub = nh.advertise<std_msgs::Float32> ("/velf/motor2",500);
 motorf3pub = nh.advertise<std_msgs::Float32> ("/velf/motor3",500);


 
 ros::Subscriber sub_friendx   = nh.subscribe("/friend/posx",500,&friendxCallback);
 ros::Subscriber sub_friendy   = nh.subscribe("/friend/posy",500,&friendyCallback);
 ros::Subscriber sub_friendt   = nh.subscribe("/friend/post",500,&friendtCallback);

 //posisi now subscribe
 ros::Subscriber pos1pub   = nh.subscribe("/status1/pstatus",500,&pstatus);
 ros::Subscriber pos2pub   = nh.subscribe("/status1/px",500,&px);
 ros::Subscriber pos3pub   = nh.subscribe("/status1/py",500,&py);

	current_time = ros::Time::now();
    prev_time = ros::Time::now();

     timeNow  = ros::Time::now();
	 timePrev = ros::Time::now();
 

ros::Rate loop_rate(5000);
int counter;

while(ros::ok())
	{	
		current_time = ros::Time::now();
		dt = (current_time - prev_time).toSec();

		
		//pidFriend(-25,200,0);

		if (friends.pos1status == 1)
		{
			pidFriend((friends.pos1x),(friends.pos1y),0);
		}

		 ros::spinOnce();
		 prev_time = current_time;

		 loop_rate.sleep();
	}


	return 0;
}
