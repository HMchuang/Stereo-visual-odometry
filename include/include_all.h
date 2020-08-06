/*
 * include_all.h
 *
 *  Created on: Aug 14, 2018
 *      Author: shomin
 */

#ifndef INCLUDE_ALL_H_
#define INCLUDE_ALL_H_

// General includes
#include <iostream>
#include <sstream>
#include <string>
#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <math.h>
#include <fstream>
#include <iomanip>
#include <chrono>
#include <vector>
#include <bitset>

#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
//#include <smmintrin.h>

// ZED includes
#include <sl_zed/Camera.hpp>
#include <sl_core/utils/types.hpp>

// Eigen include
#include <Eigen/Core>
#include <Eigen/Dense>

// OpenCV includes
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/core/ocl.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/flann.hpp>
#include <opencv2/ml.hpp>

// folder includes
#include <visual_odometry.h>

// Namespaces
using namespace cv;
using namespace cv::ml;
using namespace sl;
using namespace std;
using namespace std::chrono;
using namespace Eigen;

#define DEFAULT_WAITKEY_DELAY  1


cv::Mat slMat2cvMat(sl::Mat& input);
bool AbortRequested();


#endif /* INCLUDE_ALL_H_ */
