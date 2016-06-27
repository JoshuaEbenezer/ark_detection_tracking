#include "common.h"
#include "opencv2/objdetect/objdetect.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/nonfree/features2d.hpp> 
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/opencv_modules.hpp"

#include"iostream"
#include<stdio.h>

using namespace std;
using namespace cv;

vector<Vec3f> hough(Mat img);
Rect roi(Mat img, vector<Vec3f> circles);
