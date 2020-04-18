#include <ros/ros.h>

#include <my_roscpp_library/stereofunction.h>
#include <my_roscpp_library/stereograb.h>

#if defined(_MSC_VER)
#include <tchar.h>
#include <strsafe.h>
#include <windows.h>
#pragma comment(lib, "Ws2_32.lib")
#elif defined(__GNUC__) || defined(__GNUG__)
#include <dirent.h>
#endif

string window_name = "Deteksi Manusia";

std::string cascadeName = "bismillah20.xml";
cv::CascadeClassifier cascade;
int scale = 1;
int i;
float meas_dist = 0.0;

#define CETAK 1

int fileNO = 0;
IplImage *r_detect, *g_detect, *b_detect, *r_detect_r, *g_detect_r, *b_detect_r ;
int threshold, blobArea;
CvFont font;
int col = 0;
int column[320];
cv::Point p_center;

using namespace cv;

void StereoFunction::stereoInit(StereoGrab* grabb)
{
  cvInitFont(&font, CV_FONT_HERSHEY_PLAIN, 1, 1, 0, 1.4f, CV_AA);

  _M1 = (CvMat *)cvLoad("CalibFile/M1.yml");
  _M2 = (CvMat *)cvLoad("CalibFile/M2.yml");
  _T  = (CvMat *)cvLoad("CalibFile/T.yml");
  mx1 = (CvMat *)cvLoad("CalibFile/mx1.yml");
  my1 = (CvMat *)cvLoad("CalibFile/my1.yml");
  mx2 = (CvMat *)cvLoad("CalibFile/mx2.yml");
  my2 = (CvMat *)cvLoad("CalibFile/my2.yml");
  //_Q = (CvMat *)cvLoad("CalibFile/Q.yml");
  _CamData = (CvMat *)cvLoad("CalibFile/CamData.yml");

  //READ In FOCAL LENGTH, SENSOR ELEMENT SIZE, XFOV, YFOV
  //0: fx(pixel), 1: fy(pixel), 2: B (baseline), 3: f(mm), 4: sensor element size, 5: baseline in mm
    /*reprojectionVars[0] = cvmGet(_M1,0,0);
    reprojectionVars[1] = cvmGet(_M1,0,0);
    reprojectionVars[2] = (-1)*cvmGet(_T,0,0);
    reprojectionVars[3] = cvmGet(_CamData, 0, 0);
    reprojectionVars[4] = cvmGet(_CamData, 0, 1);
    reprojectionVars[5] = cvmGet(_CamData, 0, 2);*/


    //Loading images
    img1 = cvCreateImage(cvSize(320,240), IPL_DEPTH_8U, 1);
    img2 = cvCreateImage(cvSize(320,240), IPL_DEPTH_8U, 1);
    imageSize = cvSize(img1 -> width,img1 ->height);

    img1r = cvCreateMat( imageSize.height,imageSize.width, CV_8U );		//rectified left image
    img2r = cvCreateMat( imageSize.height,imageSize.width, CV_8U );		//rectified right image
    disp  = cvCreateMat( imageSize.height,imageSize.width, CV_16S );	//disparity map
    vdisp = cvCreateMat( imageSize.height,imageSize.width, CV_8U );
    depthM = cvCreateMat(imageSize.height, imageSize.width, CV_32F);


    thres_img = cvCreateImage( imageSize, img1->depth, 1);
    blobs_img = cvCreateImage( imageSize, img1->depth, 3);

    img_detect = cvCreateImage(imageSize, IPL_DEPTH_8U, 3);
    r_detect = cvCreateImage(imageSize,8,1);//subpixel
    r_detect_r = cvCreateImage(imageSize,8,1);
    g_detect = cvCreateImage(imageSize,8,1);//subpixel
    g_detect_r = cvCreateImage(imageSize,8,1);
    b_detect = cvCreateImage(imageSize,8,1);//subpixel
    b_detect_r = cvCreateImage(imageSize,8,1);

    pair = cvCreateMat( imageSize.height, imageSize.width*2,CV_8UC3 );
}

void StereoFunction::stereoCalibration(StereoGrab* grabb){

  int  nx=11, ny=7, frame = 0, n_boards =30, N;
  int count1 = 0,count2 = 0, result1=0, result2=0;
    int  successes1 = 0,successes2 = 0;
    const int maxScale = 1;
  const float squareSize = 2.0f;		//Set this to your actual square size
  CvSize imageSize = {0,0};
  CvSize board_sz = cvSize( nx,ny );

  int i, j, n = nx*ny, N1 = 0, N2 = 0;

  vector<CvPoint2D32f> points[2];
  vector<int> npoints;
  vector<CvPoint3D32f> objectPoints;
  vector<CvPoint2D32f> temp1(n);
  vector<CvPoint2D32f> temp2(n);

    double M1[3][3], M2[3][3], D1[5], D2[5];
    double R[3][3], T[3], E[3][3], F[3][3];
  double Q[4][4];
  CvMat _Qcalib  = cvMat(4, 4, CV_64F, Q);
    CvMat _M1calib = cvMat(3, 3, CV_64F, M1 );
    CvMat _M2calib = cvMat(3, 3, CV_64F, M2 );
    CvMat _D1 	   = cvMat(1, 5, CV_64F, D1 );
    CvMat _D2      = cvMat(1, 5, CV_64F, D2 );
    CvMat _R       = cvMat(3, 3, CV_64F, R );
    CvMat _Tcalib  = cvMat(3, 1, CV_64F, T );
    CvMat _E       = cvMat(3, 3, CV_64F, E );
    CvMat _F       = cvMat(3, 3, CV_64F, F );

  //Start webcam
    printf("\nWebcams are starting ...\n");
    grabb->stereoGrabInitFrames();
    grabb->stereGrabFrames();
    IplImage *frame1 = grabb->imageLeft;
    IplImage* gray_fr1 = cvCreateImage( cvGetSize(frame1), 8, 1 );
    IplImage *frame2 = grabb->imageRight;
    IplImage* gray_fr2 = cvCreateImage( cvGetSize(frame2), 8, 1 );
    imageSize = cvGetSize(frame1);


    printf("\nWant to capture %d chessboards for calibrate:", n_boards);
    while((successes1<n_boards)||(successes2<n_boards))
    {
      //------------- cari & drw chessboard-------------///
      if((frame++ % 20) == 0){
        //---------------- CAM KIRI-------------------------//
        result1 = cvFindChessboardCorners( frame1, board_sz,&temp1[0], &count1,CV_CALIB_CB_ADAPTIVE_THRESH|CV_CALIB_CB_FILTER_QUADS);
        cvCvtColor( frame1, gray_fr1, CV_BGR2GRAY );
        //----------------CAM KANAN--------------------------------------------------------------------------------------------------------
        result2 = cvFindChessboardCorners( frame2, board_sz,&temp2[0], &count2,CV_CALIB_CB_ADAPTIVE_THRESH|CV_CALIB_CB_FILTER_QUADS);
        cvCvtColor( frame2, gray_fr2, CV_BGR2GRAY );

        if(count1==n&&count2==n&&result1&&result2){
          cvFindCornerSubPix( gray_fr1, &temp1[0], count1,cvSize(11, 11), cvSize(-1,-1),cvTermCriteria(CV_TERMCRIT_ITER+CV_TERMCRIT_EPS,30, 0.01) );
          cvDrawChessboardCorners( frame1, board_sz, &temp1[0], count1, result1 );
          cvShowImage( "Scan corners cam KI", frame1 );
          N1 = points[0].size();
          points[0].resize(N1 + n, cvPoint2D32f(0,0));
          copy( temp1.begin(), temp1.end(), points[0].begin() + N1 );
          ++successes1;

          cvFindCornerSubPix( gray_fr2, &temp2[0], count2,cvSize(11, 11), cvSize(-1,-1),cvTermCriteria(CV_TERMCRIT_ITER+CV_TERMCRIT_EPS,30, 0.01) );
          cvDrawChessboardCorners( frame2, board_sz, &temp2[0], count2, result2 );
          cvShowImage( "Scan corners cam KA", frame2 );
          N2 = points[1].size();
          points[1].resize(N2 + n, cvPoint2D32f(0,0));
          copy( temp2.begin(), temp2.end(), points[1].begin() + N2 );
          ++successes2;
          printf("\nNumber Chessboards ditemukan: %d", successes2);
          //cvWaitKey(3000);
          //Sleep(3000);
        }else{
          cvShowImage( "corners camera2", frame2 );
          cvShowImage( "corners camera1", frame1 );

        }
        grabb->stereGrabFrames();
        frame1 = grabb->imageLeft;
        cvShowImage("camera KI", frame1);
        frame2 = grabb->imageRight;
        cvShowImage("camera KA", frame2);

      if(cvWaitKey(1)==27) break;

      }
    }

    grabb->stereoGrabStopCam();
    cvDestroyWindow("camera KI");
    cvDestroyWindow("camera KA");
    cvDestroyWindow("corners camera1");
    cvDestroyWindow("corners camera2");
    printf("\nSelesai Capture!");


    //--------------Compute for calibration-------------------
    N = n_boards*n;
    objectPoints.resize(N);
    for( i = 0; i < ny; i++ )
      for(j = 0; j < nx; j++ )   objectPoints[i*nx + j] = cvPoint3D32f(i*squareSize, j*squareSize, 0);
    for( i = 1; i < n_boards; i++ ) copy( objectPoints.begin(), objectPoints.begin() + n, objectPoints.begin() + i*n );
    npoints.resize(n_boards,n);

    CvMat _objectPoints = cvMat(1, N, CV_32FC3, &objectPoints[0] );
    CvMat _imagePoints1 = cvMat(1, N, CV_32FC2, &points[0][0] );
    CvMat _imagePoints2 = cvMat(1, N, CV_32FC2, &points[1][0] );
    CvMat _npoints = cvMat(1, npoints.size(), CV_32S, &npoints[0] );
    cvSetIdentity(&_M1calib);
    cvSetIdentity(&_M2calib);
    cvZero(&_D1);
    cvZero(&_D2);

    printf("\nRunning stereo calibration ...");
    fflush(stdout);
    cvStereoCalibrate( &_objectPoints, &_imagePoints1, &_imagePoints2, &_npoints,&_M1calib, &_D1, &_M2calib, &_D2,imageSize, &_R, &_Tcalib, &_E, &_F,
        cvTermCriteria(CV_TERMCRIT_ITER+CV_TERMCRIT_EPS, 100, 1e-5),
            CV_CALIB_FIX_ASPECT_RATIO+CV_CALIB_ZERO_TANGENT_DIST + CV_CALIB_SAME_FOCAL_LENGTH );


    printf("\nDone Calibration");
    //-------------UNDISTORTION------------------------------------------
    cvUndistortPoints( &_imagePoints1, &_imagePoints1,&_M1calib, &_D1, 0, &_M1calib );
    cvUndistortPoints( &_imagePoints2, &_imagePoints2,&_M2calib, &_D2, 0, &_M2calib );
    //COMPUTE AND DISPLAY RECTIFICATION and find disparities
    CvMat* mx1calib = cvCreateMat( imageSize.height,imageSize.width, CV_32F );
        CvMat* my1calib = cvCreateMat( imageSize.height,imageSize.width, CV_32F );
        CvMat* mx2calib = cvCreateMat( imageSize.height,imageSize.width, CV_32F );
        CvMat* my2calib = cvCreateMat( imageSize.height,imageSize.width, CV_32F );

        double R1[3][3], R2[3][3], P1[3][4], P2[3][4];
        CvMat _R1 = cvMat(3, 3, CV_64F, R1);
        CvMat _R2 = cvMat(3, 3, CV_64F, R2);

            CvMat _P1 = cvMat(3, 4, CV_64F, P1);
            CvMat _P2 = cvMat(3, 4, CV_64F, P2);
      //compute variables needed for rectification using camera matrices, distortion vectors, rotation matrix, and translation vector
            cvStereoRectify( &_M1calib, &_M2calib, &_D1, &_D2, imageSize,&_R, &_Tcalib,&_R1, &_R2, &_P1, &_P2, &_Qcalib,0/*CV_CALIB_ZERO_DISPARITY*/ );
      //Precompute maps for cvRemap()
            cvInitUndistortRectifyMap(&_M1calib,&_D1,&_R1,&_P1,mx1calib,my1calib);
            cvInitUndistortRectifyMap(&_M2calib,&_D2,&_R2,&_P2,mx2calib,my2calib);



      printf("\nSaving matries for later use ...\n");
      cvSave("CalibFile//M1.yml",&_M1calib);
      cvSave("CalibFile//D1.yml",&_D1);
      cvSave("CalibFile//R1.yml",&_R1);
      cvSave("CalibFile//P1.yml",&_P1);
      cvSave("CalibFile//M2.yml",&_M2calib);
      cvSave("CalibFile//D2.yml",&_D2);
      cvSave("CalibFile//R2.yml",&_R2);
      cvSave("CalibFile//P2.yml",&_P2);
      cvSave("CalibFile//Q.yml",&_Qcalib);
      cvSave("CalibFile//T.yml",&_Tcalib);
      cvSave("CalibFile//mx1.yml",mx1calib);
      cvSave("CalibFile//my1.yml",my1calib);
      cvSave("CalibFile//mx2.yml",mx2calib);
      cvSave("CalibFile//my2.yml",my2calib);
}
