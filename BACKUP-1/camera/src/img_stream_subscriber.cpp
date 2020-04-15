#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

using namespace cv;



void r_imageCallback(const sensor_msgs::ImageConstPtr& msg){
    try {
        imshow("viewr", cv_bridge::toCvShare(msg, "bgr8")->image);
        waitKey(30);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("Could not conver from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}

void l_imageCallback(const sensor_msgs::ImageConstPtr& msg){
    try {
        imshow("viewl", cv_bridge::toCvShare(msg, "bgr8")->image);
        waitKey(30);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("Could not conver from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "img_subscriber");
    ros::NodeHandle nh;

    Mat imgl;
    Mat imgr;


    cv::namedWindow("viewr");
    cv::namedWindow("viewl");

    //cv::startWindowThread();
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber subr = it.subscribe("camera/imager", 1, r_imageCallback);
    image_transport::Subscriber subl = it.subscribe("camera/imagel", 1, l_imageCallback);
    ros::spin();
    destroyWindow("viewr");
    destroyWindow("viewl");

    }
