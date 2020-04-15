#include "opencv2/objdetect.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/videoio.hpp"
#include <iostream>
using namespace std;
using namespace cv;
void detectAndDisplay( Mat frame );
CascadeClassifier face_cascade;
CascadeClassifier eyes_cascade;
int main( int argc, const char** argv )
{
Mat image;
  image = imread("/home/budi/Pictures/data/kertas1.jpg",CV_LOAD_IMAGE_COLOR);
  namedWindow("window1",1);
  imshow("window1",image);

  // Load Face cascade (.xml file)imp
  CascadeClassifier face_cascade;
  face_cascade.load("cascade_40x40.xml");

  if(face_cascade.empty())
  {
  cerr<<"Error Loading XML file"<<endl; return 0;
  }
  // Detect faces
  std::vector<Rect> faces;
  face_cascade.detectMultiScale(image,faces,1.1,2,0|CV_HAAR_SCALE_IMAGE,Size(30,30));

  // Draw circles on the detected faces
  for(int i = 0; i < faces.size(); i++)
  {
  Point center(faces[i].x + faces[i].width*0.5, faces[i].y + faces[i].height*0.5 );
  ellipse(image, center,Size( faces[i].width*0.5,faces[i].height*0.5),0,0,360,Scalar(255,0,255),4,8, 0);
  }
  imshow( "Detected Face", image ); waitKey(0);
  return 0;
}