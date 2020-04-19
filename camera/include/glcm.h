#pragma once
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/ml/ml.hpp>
#include <opencv2/core/mat.hpp>

#include <iostream>
#include <fstream>

using namespace std;
using namespace cv;

void glcm(const Mat img, vector<float> &vec_energy, bool isShow, bool isPrint);

