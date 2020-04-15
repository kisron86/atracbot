#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "img_publisher");
    ros::NodeHandle nh;

    image_transport::ImageTransport it(nh);
    image_transport::ImageTransport it2(nh);
    image_transport::Publisher pub = it.advertise("camera/left/rect",1);
    image_transport::Publisher pub2 = it2.advertise("camera/right/rect",1);

    //cv::Mat image = cv::imread(argv[1],CV_LOAD_IMAGE_COLOR);
    cv::Mat imagel = cv::imread("left00.jpg");
    cv::Mat imager = cv::imread("right00.jpg");

    cv::waitKey(30);
    //sensor_msgs::ImagePtr msg4 = cv_bridge::CvImage(std_msgs::Header(),)
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(),"mono16", imagel).toImageMsg();
    sensor_msgs::ImagePtr msg2 = cv_bridge::CvImage(std_msgs::Header(),"mono16", imager).toImageMsg();

    ros::Rate loop_rate(5);

    while (1) {
        pub.publish(msg);
        pub2.publish(msg2);
        ros::spinOnce();
        loop_rate.sleep();
    }
}

