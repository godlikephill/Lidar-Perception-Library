#ifndef IMM_KF_H
#define IMM_KF_H

#include "opencv2/opencv.hpp"
#include <opencv/cv.h>
#include "Mode_KF1.h"
#include "Mode_KF2.h"
using namespace cv;
using namespace std;


class IMM_KF
{
public:

    int rows;
    float T,turn_rate;
    Mat X,P,H,R,G,t_diag,Q,F,S;
    Mat mu,p_ij,mix,cbar,p_zeros,x_zeros;
    Mat X_est,xMix;
    vector<Mat> P_est,pMix;
    //vector<Mat> X_est,xMix;
    Mode_KF1* kf1;
    Mode_KF2* kf2;
    Mat re_xstate,re_pstate;

    //--------------------------------------------------------------------------
    Mat x_est,p_est,x_pre,p_pre,z_pre;

//--------------------------------------------------------------------------
    Point2f sp_est;
    IMM_KF(Point2f p,float dt,float Accel_noise_mag);
    ~IMM_KF();
    Point2d Update(Point2f p, bool DataCorrect);
    void Prediction();
    Mat getpossibility();
    Point2d getspeed();
    void Preprocess();
    Mat getxstate();
    Mat getpstate();
    float getturnrate();
};


#endif // IMM_KF_H
