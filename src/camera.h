#ifndef CAMERA_H
#define CAMERA_H

#include <iostream>
#include <string>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <fstream>

using namespace std;
using namespace cv;

class Camera{
    
    private:
    string camera_name = "Camera1";

    public:
    Mat K;
    Mat K_inv;
    float fx;
    float fy;
    float cx;
    float cy;

    Camera();
    void print_name();
    void set_k(Mat K);
    void set_k_inv(Mat K);
};
 

#endif
