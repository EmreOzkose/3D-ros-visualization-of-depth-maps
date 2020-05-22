#include "camera.h"

Camera::Camera(){

}


void Camera::print_name(){
    cout << "Camera name: " << camera_name << endl;
}

void Camera::set_k(Mat K){
    this->K = K;
    this->fx = K.at<float>(0, 0);
    this->fy = K.at<float>(1, 1);
    this->cx = K.at<float>(0, 2);
    this->cy = K.at<float>(1, 2);

    cout << "K is set to:\n" << K << endl;
    cout << "fx: " << fx << "\nfy: " << fy << "\ncx: " << cx << "\ncy: " << cy << endl;
}

void Camera::set_k_inv(Mat K){
    this->K_inv = K.clone();
    
       /* """Inverse intrinsics (for lifting)"""
        Kinv = self.K.clone()
        Kinv[:, 0, 0] = 1. / self.fx
        Kinv[:, 1, 1] = 1. / self.fy
        Kinv[:, 0, 2] = -1. * self.cx / self.fx
        Kinv[:, 1, 2] = -1. * self.cy / self.fy
*/
}