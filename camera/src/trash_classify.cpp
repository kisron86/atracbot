#include <ros/ros.h>
#include "my_roscpp_library/my_hog.h"
#include "my_roscpp_library/my_super_roscpp_library.h"

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include <time.h> // to calculate time needed
#include <limits.h> // to get INT_MAX, to protect against overflow

#ifndef DWORD
#define WINAPI
typedef unsigned long DWORD;
typedef short WCHAR;
typedef void * HANDLE;
#define MAX_PATH    PATH_MAX
typedef unsigned char BYTE;
typedef unsigned short WORD;
typedef unsigned int BOOL;
#endif

#ifndef WIN32_NO_STATUS
# define WIN32_NO_STATUS
#endif

#include <iomanip>
#include <iostream>
#include <stdlib.h>
//#include <windows.h>

#ifndef WIN32_NO_STATUS
# define WIN32_NO_STATUS
#endif

#include <vector>
#include <string>
#include <algorithm>
#include <iostream>
#include <iterator>
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>

#include <stdio.h>
#include <iostream>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <fstream>

#include <sstream>
#include <iomanip>

//#include "StereoGrab.h" // deklaration camera
//#include "StereoFunction.h" // funsi eksecukis stereo


//#include <conio.h>
#include <opencv2/core/core.hpp>
#include <opencv2/ml/ml.hpp>
#include <opencv2/core/mat.hpp>
#include "HOG.h"
#include "glcm.h"

//using namespace cv::ml;

using namespace cv;
using namespace std;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "trash_classify");
  ros::NodeHandle nh;

  ini_coba();
}
