#ifndef MODE_KF1_H
#define MODE_KF1_H

#include "opencv2/opencv.hpp"
#include <opencv/cv.h>
using namespace cv;
using namespace std;


class Mode_KF1
{
public:

    int rows;
    float T;
    Mat X,P,H,R,G,t_diag,Q,F,S;
    Mat LH;
    //--------------------------------------------------------------------------
    Mat x_est,p_est,x_pre,p_pre,z_pre;

//--------------------------------------------------------------------------
    Point2f sp_est;
    Mode_KF1(Point2f p,float dt,float Accel_noise_mag);
    ~Mode_KF1();
    Point2d Update(Point2f p, bool DataCorrect);
    void Prediction(Mat x_mix, Mat p_mix);
    Mat getlikelihood();
    Point2d getspeed();
    Mat getxstate();
    Mat getpstate();
    Mat getxpredict();
    Mat getppredict();
};


#endif // MODE_KF1_H
