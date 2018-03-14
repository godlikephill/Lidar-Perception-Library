#ifndef MODE_KF2_H
#define MODE_KF2_H
#include "opencv2/opencv.hpp"
#include <opencv/cv.h>
using namespace cv;
using namespace std;


class Mode_KF2
{
public:

    int rows;
    float T;
    Mat X,P,H,R,G,t_diag,Q,F,S;
    Mat LH;
    //--------------------------------------------------------------------------
    Mat x_est,p_est,x_pre,p_pre,z_pre;
    Mat x_last,p_last;
   //--------------------------------------------------------------------------
    Point2f sp_est;
    Mode_KF2(Point2f p,float dt,float Accel_noise_mag);
    ~Mode_KF2();
    Point2d Update(Point2f p, bool DataCorrect);
    void Prediction(Mat x_mix, Mat p_mix);
    Mat getlikelihood();
    Point2d getspeed();
    Mat Cal_Jac(Mat xhat);
    Mat F_turn(Mat xhat);
    Mat getxstate();
    Mat getpstate();
    Mat getxpredict();
    Mat getppredict();
};

#endif // MODE_KF2_H
