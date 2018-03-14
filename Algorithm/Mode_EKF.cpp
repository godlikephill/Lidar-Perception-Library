#include "Mode_EKF.h"
#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <math.h>
#include <QFileDialog>
using namespace cv;
using namespace std;



Mode_EKF::Mode_EKF(Point2f pt,float dt,float Accel_noise_mag)
{
    sp_est.x =0.1;
    sp_est.y =0.1;

    ///initial estimation
    // Sampling time
    T = dt;
    // Covariance matrix
    G = (Mat_<float>(5, 3) << (dt*dt)/2,0,0,  0,(dt*dt)/2,0,  dt,0,0, 0,dt,0,  0,0,dt);
    double tu1 = 0.8;
    double tuu1 = 0.05;//0.05
    double tu2 = 0.5;
    t_diag = (Mat_<float>(3, 3) << (tu1*tu1),0,0,  0,(tu1*tu1),0,  0,0,(tuu1*tuu1));
    Q = G * t_diag * G.t();
    R = (Mat_<float>(2, 2) << (tu2*tu2),0,  0,(tu2*tu2));
    //Measurement Matrix
    H = (Mat_<float>(2,5) << 1,0,0,0,0, 0,1,0,0,0);
    rows = H.rows;

    // initilization
    X = (Mat_<float>(5,1) <<pt.x,pt.y,sp_est.x,sp_est.y,0);
    P = 10 * (Mat_<float>(5, 5) << 1,0,0,0,0,   0,1,0,0,0,  0,0,1,0,0,  0,0,0,1,0,  0,0,0,0,0);

    x_est = X;
    p_est = P;

}
//---------------------------------------------------------------------------
Mode_EKF::~Mode_EKF()
{

}



void Mode_EKF::Prediction(){

    ///Prediction(a priori) stage
    // Jacobian of F
    Mat JF = Cal_Jac(x_est);
    Mat Fnon = F_turn(x_est);
    //Predicted State
    x_pre = Fnon*x_est;
    //qDebug("x_pre1 are %f",x_pre.at<float>(0));
    //Predicted Measurement
    z_pre = H * x_pre;
    //Predicted Covariance
    p_pre = JF*p_est+JF.t() + Q;
    //innovation covariance
    S = H * p_pre * H.t() + R;

    //qDebug( "p_pre 2 %f %f",p_pre.at<float>(0),p_pre.at<float>(1));
}


//---------------------------------------------------------------------------
Point2d Mode_EKF::Update(Point2f p, bool DataCorrect)
{

    Mat z_current(2,1,CV_32FC1);
    if(!DataCorrect)
    {   //std::cout<<"read previous estimated positions..."<<std::endl;
        //Point2f temp2;
        //temp2.x = x_est.at<float>(0);
        //temp2.y = x_est.at<float>(1);
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
    //qDebug( "errk2 is %f",errk.at<float>(0));
    Mat W = p_pre*H.t()*S.inv();  //calculate the KF gain
 //-------------------------------------------------------------------------------------
 /// A posteriori state update
    x_est = x_pre + W*errk;
    Mat eye_5 = (Mat_<float>(5,5) << 1,0,0,0,0, 0,1,0,0,0, 0,0,1,0,0, 0,0,0,1,0, 0,0,0,0,1);
    p_est = (eye_5 - W*H)* p_pre* (eye_5-W*H).t() + W*R*W.t();
    error = errk;

    //qDebug( "temp_exp2 is %f",temp_exp.at<float>(0));
    //qDebug( "KF2 possibility is %f",LH.at<float>(0));
    //qDebug("kf2 state is %f",x_est.at<float>(0));


///%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
   Point2d temp1;
   temp1.x = x_est.at<float>(0);
   temp1.y = x_est.at<float>(1);
   sp_est.x = x_est.at<float>(2);
   sp_est.y = x_est.at<float>(3);
   return temp1;
}



Point2d Mode_EKF::geterror(){
   Point2d t;
   t.x = error.at<float>(0);
   t.y = error.at<float>(1);
   return t;
}


Point2d Mode_EKF::getspeed(){
    return sp_est;
}


Mat Mode_EKF::getxstate()
{
    return x_est;
}

Mat Mode_EKF::getpstate()
{
    return p_est;
}

Mat Mode_EKF::getprexstate()
{
    return x_pre;
}

Mat Mode_EKF::Cal_Jac(Mat xhat)
{
    //qDebug( "Cal_Jac xhat is %f %f",xhat.at<float>(0),xhat.at<float>(1));

    double w = xhat.at<float>(4);
    //qDebug("kf2 w is %f",w);

    double coswt,coswto,coswtopw,sinwt,sinwtpw,dsinwtpw,dcoswtopw;

    if(w ==0){
       coswt = 1;
       coswtopw = 0;
       sinwt = 0;
       sinwtpw = T;
       dsinwtpw = 0;
       dcoswtopw = -0.5*(T*T);
    }
    else{
       coswt = cos(w*T);
       coswto = cos(w*T)-1;
       coswtopw = coswto / w;
       sinwt = sin(w*T);
       sinwtpw = sinwt / w;
       dsinwtpw = (w*T*coswt - sinwt) / (w*w);
       dcoswtopw =(-w*T*sinwt - coswto) / (w*w);
    }

    Mat df = (Mat_<float>(5, 5) << 0,0,0,0,0,  0,0,0,0,0,  0,0,0,0,0,  0,0,0,0,0,  0,0,0,0,0);

    df.at<float>(0,0) = 1;
    df.at<float>(0,2) = sinwtpw;
    df.at<float>(0,3) = coswtopw;
    df.at<float>(0,4) = dsinwtpw* xhat.at<float>(2) + dcoswtopw*xhat.at<float>(3);

    df.at<float>(1,1) = 1;
    df.at<float>(1,2) = -coswtopw;
    df.at<float>(1,3) = sinwtpw;
    df.at<float>(1,4) = -dcoswtopw* xhat.at<float>(2) + dsinwtpw*xhat.at<float>(3);

    df.at<float>(2,2) = coswt;
    df.at<float>(2,3) = -sinwt;
    df.at<float>(2,4) = -T*sinwt* xhat.at<float>(2) - T*coswt*xhat.at<float>(3);

    df.at<float>(3,2) = sinwt;
    df.at<float>(3,3) = coswt;
    df.at<float>(3,4) = T*coswt* xhat.at<float>(2) - T*sinwt*xhat.at<float>(3);

    df.at<float>(4,4) = 1;

    return df;
}




Mat Mode_EKF::F_turn(Mat xhat)
{
   double w = xhat.at<float>(4);
   double coswt,coswto,coswtopw,sinwt,sinwtpw;


   if(w ==0){
       coswt = 1;
       coswto = 0;
       coswtopw = 0;

       sinwt = 0;
       sinwtpw = T;
   }
   else{
       coswt = cos(w*T);
       coswto = cos(w*T)-1;
       coswtopw = coswto / w;

       sinwt = sin(w*T);
       sinwtpw = sinwt / w;
   }

   Mat fnon = (Mat_<float>(5, 5) << 0,0,0,0,0,  0,0,0,0,0,  0,0,0,0,0,  0,0,0,0,0,  0,0,0,0,0);

   fnon.at<float>(0,0) = 1;
   fnon.at<float>(0,2) = sinwtpw;
   fnon.at<float>(0,3) = coswtopw;

   fnon.at<float>(1,1) = 1;
   fnon.at<float>(1,2) = -coswtopw;
   fnon.at<float>(1,3) = sinwtpw;

   fnon.at<float>(2,2) = coswt;
   fnon.at<float>(2,3) = -sinwt;

   fnon.at<float>(3,2) = sinwt;
   fnon.at<float>(3,3) = coswt;

   fnon.at<float>(4,4) = 1;

   return fnon;
}


