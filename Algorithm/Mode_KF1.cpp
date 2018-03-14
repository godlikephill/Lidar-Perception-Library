#include "Mode_KF1.h"
#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <math.h>
#include <QFileDialog>
using namespace cv;
using namespace std;



Mode_KF1::Mode_KF1(Point2f pt,float dt,float Accel_noise_mag)
{
    sp_est.x =0;
    sp_est.y =0;

    ///initial estimation
    // Sampling
    T = dt;
    // Covariance matrix
    G = (Mat_<float>(5, 2) << (dt*dt)/2,0,  0,(dt*dt)/2, dt,0, 0,dt, 0,0);
    double tu1 = 0.8;
    t_diag = (Mat_<float>(2, 2) << (tu1*tu1),0, 0,(tu1*tu1) );
    Q = G * t_diag * G.t();
    double tu2 = 0.3;
    R = (Mat_<float>(2, 2) << (tu2*tu2),0,  0,(tu2*tu2));
    // Transition Matrix
    //uniform motion model
    F = (Mat_<float>(5, 5) << 1,0,T,0,0,  0,1,0,T,0,  0,0,1,0,0,  0,0,0,1,0, 0,0,0,0,0);
    //Measurement Matrix
    H = (Mat_<float>(2,5) << 1,0,0,0,0, 0,1,0,0,0);
    rows = H.rows;

    // initilization
    //x_est = X;
    //p_est = P;

}
//---------------------------------------------------------------------------
Mode_KF1::~Mode_KF1()
{

}



void Mode_KF1::Prediction(Mat x_mix, Mat p_mix){

    ///Prediction(a priori) stage
    x_est = x_mix;
    p_est = p_mix;
    //qDebug("x_mix is %f",x_mix.at<float>(0));
    ///Predicted State
    //x_pre = F*x_est;
    x_pre = F*x_est;
    //qDebug("x_pre1 are %f",x_pre.at<float>(0));
    ///Predicted Measurement
    z_pre = H * x_pre;

    ///Predicted Covariance
    //p_pre = F*p_est+F.t() + Q;
    p_pre = F*p_est+F.t() + Q;
    ///innovation covariance
    S = H * p_pre * H.t() + R;

    //qDebug( "p_pre 1 %f %f",p_pre.at<float>(0),p_pre.at<float>(1));
    //Point2f predict;
    //predict.x = x_pre.at<float>(0);
    //predict.y = x_pre.at<float>(1);
    //return predict;
}


//---------------------------------------------------------------------------
Point2d Mode_KF1::Update(Point2f p, bool DataCorrect)
{
    //qDebug( "kf1 update...");


    Mat z_current(2,1,CV_32FC1);
    if(!DataCorrect)
    {
        //Point2f temp2;
        //temp2.x = x_est.at<float>(0);
        //temp2.y = x_est.at<float>(1);
        //sp_est.x = x_est.at<float>(2);
        //sp_est.y = x_est.at<float>(3);
        //return temp2;
        z_current.at<float>(0) = x_est.at<float>(0);
        z_current.at<float>(1) = x_est.at<float>(1);
    }

    else
    {   //update using current measurement
        z_current.at<float>(0) = p.x;
        z_current.at<float>(1) = p.y;
    }

    //error between current measurement and predicted measurement
    Mat errk = z_current - z_pre;
    //qDebug( "errk1 is %f",errk.at<float>(0));
    Mat W = p_pre*H.t()*S.inv();  //calculate the KF gain
 //-------------------------------------------------------------------------------------
 /// A posteriori state update
    x_est = x_pre + W*errk;
    Mat eye_5 = (Mat_<float>(5,5) << 1,0,0,0,0, 0,1,0,0,0, 0,0,1,0,0, 0,0,0,1,0, 0,0,0,0,1);
    p_est = (eye_5 - W*H)* p_pre* (eye_5-W*H).t() + W*R*W.t();

    Mat temp_exp;
    cv::exp(-0.5*errk.t()*S.inv()*errk,temp_exp);
    //Mat hi = -0.5*errk.t()*S.inv()*errk;
    //qDebug( "temp_exp1 is %f",temp_exp.at<float>(0));
    LH =  (1/sqrt(abs(determinant(2*M_PI*S)))) * temp_exp;
    //qDebug( "KF1 possibility is %f",LH.at<float>(0));


    //qDebug("kf1 state is %f",x_est.at<float>(0));
///%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
   Point2d temp1;
   temp1.x = x_est.at<float>(0);
   temp1.y = x_est.at<float>(1);
   sp_est.x = x_est.at<float>(2);
   sp_est.y = x_est.at<float>(3);
   //qDebug( "speed is %f %f",x_est.at<float>(2),x_est.at<float>(3));
   return temp1;
}


Point2d Mode_KF1::getspeed(){
    return sp_est;
}


Mat Mode_KF1::getlikelihood()
{
   return LH;
}

Mat Mode_KF1::getxstate()
{
    return x_est;
}

Mat Mode_KF1::getpstate()
{
    return p_est;
}

Mat Mode_KF1::getxpredict()
{
    return x_pre;
}

Mat Mode_KF1::getppredict()
{
    return p_pre;
}
