#include "opencv2/opencv.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/objdetect/objdetect.hpp"

#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <vector>

#include <iostream>
#include <stdio.h>

using namespace std;
using namespace cv;

#if defined(_MSC_VER)
#include <tchar.h>
#include <strsafe.h>
#include <windows.h>
#pragma comment(lib, "Ws2_32.lib")
#elif defined(__GNUC__) || defined(__GNUG__)
#include <dirent.h>
#endif

using namespace std;
using namespace cv;

/** Function Headers */
void detectAndDisplay( string& img_filename );

/** Global variables */
//string cascade_name = "/home/kisron/catkin_workspace/cpp_test/detectball-master/xml/eigenValues_All.xml";
string cascade_name = "waste.xml";
CascadeClassifier cascade;

string window_name = "SPQR TEAM - ball detection with LBP";

int main( int argc, const char** argv )
{
	string dirname;
	cout << "SPQR TEAM - ball detection with LBP" << endl;
	cout << endl;	

	if (argc == 2) {
		dirname.assign(argv[1]);
	}
	else if (argc == 4) {
		dirname.assign(argv[1]);
                string a2(argv[2]); 
                if(a2.compare("-c") == 0) {
                    cascade_name.assign(argv[3]);
                }
	}
	else {
		cout << "Usage is:" << endl;
		cout << argv[0] << "<dir name> [-c <cascade filename>]" << endl;
		cout << endl;
		return EXIT_FAILURE;
	}
	
	//-- 1. Load the cascade
	cout << "Loading the cascade " << cascade_name << "...";
	cout.flush();
	if (!cascade.load(cascade_name)) {
		cout << endl;
		cout << "--(!)Error loading CASCADE: " << cascade_name << endl;
                cout << "Please provide the cascade filename" << endl;
                cout << "example:" << endl;
                cout << "         detectball ../test -c ../ball_cascade.xml" << endl;
		return EXIT_FAILURE;
	}
	else {
		cout << "[OK]" << endl;
	}

#if defined(_MSC_VER)

	WIN32_FIND_DATA ffd;
	TCHAR szDir[MAX_PATH];
	size_t length_of_arg;
	HANDLE hFind = INVALID_HANDLE_VALUE;
	DWORD dwError = 0;

	// Check that the input path plus 3 is not longer than MAX_PATH.
	// Three characters are for the "\*" plus NULL appended below.

	StringCchLength(dirname.c_str(), MAX_PATH, &length_of_arg);

	if (length_of_arg > (MAX_PATH - 3))
	{
		_tprintf(TEXT("\nDirectory path is too long.\n"));
		exit(EXIT_FAILURE);
	}

	_tprintf(TEXT("\nTarget directory is %s\n\n"), dirname.c_str());

	// Prepare string for use with FindFile functions.  First, copy the
	// string to a buffer, then append '\*' to the directory name.

	StringCchCopy(szDir, MAX_PATH, dirname.c_str());
	StringCchCat(szDir, MAX_PATH, TEXT("\\*"));

	// Find the first file in the directory.

	hFind = FindFirstFile(szDir, &ffd);

	if (INVALID_HANDLE_VALUE == hFind)
	{
		exit(EXIT_FAILURE);
	}

	// List all the files in the directory with some info about them.
	string img_filename;
	do
	{
		if (ffd.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY)
		{
			continue;
		}
		else
		{
			PTSTR pszFileName = ffd.cFileName;
			std::string name(pszFileName);
			img_filename = dirname + "\\" + name;
			detectAndDisplay(img_filename);
		}
	} while (FindNextFile(hFind, &ffd) != 0);

	dwError = GetLastError();
	if (dwError != ERROR_NO_MORE_FILES)
	{
		exit(EXIT_FAILURE);
	}

	FindClose(hFind);

#elif defined(__GNUC__) || defined(__GNUG__)

	DIR *dir;
	struct dirent *ent;
	if ((dir = opendir(dirname.c_str())) != NULL) {
		string img_filename;
		while ((ent = readdir(dir)) != NULL) {			
			img_filename = dirname + "/" + ent->d_name;
			detectAndDisplay(img_filename);
		}
		closedir(dir);
	}
	else {
		/* could not open directory */
		perror("");
		return EXIT_FAILURE;
	}

#endif
	return EXIT_SUCCESS;
}


/**
* @function detectAndDisplay
*/
void detectAndDisplay(string& img_filename)
{
	Mat frame;
	frame = imread(img_filename, IMREAD_COLOR);
	if (!frame.data) {
		cout << "Unable to read input frame: " << img_filename << endl;
		exit(EXIT_FAILURE);
	}
    
	Mat resized_frame(Size(320, 240), CV_8UC3);
	resize(frame, resized_frame, resized_frame.size());
	frame = resized_frame.clone();

    std::vector<Rect> balls;
    Mat frame_gray;

    cvtColor( frame, frame_gray, COLOR_BGR2GRAY );
    cascade.detectMultiScale(frame_gray, balls, 1.1, 5, 8, Size(16, 16));

   Mat gui_frame = frame.clone();
   for( unsigned int i = 0; i < balls.size(); i++ )
   {
      Point center( balls[i].x + cvRound(balls[i].width*0.5), cvRound(balls[i].y + balls[i].height*0.5) );
      circle(gui_frame, center, cvRound(balls[i].width*0.5), Scalar( 255, 0, 255 ), 2, 8, 0 );
	  //Mat ballROI = frame_gray( balls[i] );
   } 

   //-- Show what you got
   imshow( window_name, gui_frame );
   int key = waitKey(0);
}
