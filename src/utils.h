#ifndef utils
#define utils

#include "config.h"
#include <iostream>
#include <string>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <fstream>

using namespace std;
using namespace cv;

Mat read_calibration(string path);
Vec3i parse_arguments(int argc, char** argv);
Mat read_image(String path, int image_color=1);


#endif

