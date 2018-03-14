#include "IMM_KF.h"
#include <opencv2/opencv.hpp>
#include <QtCore/QDateTime>
#include <QFileDialog>
#include <iostream>
#include <vector>
#include <math.h>
using namespace cv;
using namespace std;



IMM_KF::IMM_KF(Point2f pt,float dt,float Accel_noise_mag)
{
    sp_est.x =0;
    sp_est.y =0;
    turn_rate = 0;

    mu = (Mat_<float>(2,1) << 0.85, 0.15);
    p_ij = (Mat_<float>(2,2) << 0.95, 0.05, 0.05, 0.95);

    ///initial estimation
    ///X_est = (Mat_<float>(5,2) <<pt.x,pt.x, pt.y,pt.y, sp_est.x,sp_est.x, sp_est.y,sp_est.y, 0,0);
    X = (Mat_<float>(5,1) <<pt.x,pt.y,sp_est.x,sp_est.y,turn_rate);

    // Sampling time
    T = dt;
    // Covariance matrix
    P = (Mat_<float>(5, 5) << 100,0,0,0,0,  0,100,0,0,0,  0,0,100,0,0,  0,0,0,100,0,  0,0,0,0,1);
    //R = (Mat_<float>(2, 2) << (0.6*0.6),0,  0,(0.6*0.6));
    //Measurement Matrix
    H = (Mat_<float>(2,5) << 1,0,0,0,0, 0,1,0,0,0);
    rows = H.rows;

    /// initilization
    re_xstate = X;
    re_pstate = P;
    ///xhat12
    X_est = Mat(5,2, X.type(), Scalar(0));
    X_est.col(0) = X*1;
    X_est.col(1) = X*1;
    //qDebug( "X est check is %f",X_est.at<float>(0,0));

    ///phat12
    P_est.push_back(P);
    P_est.push_back(P);
    //qDebug( "P est check is %f",P_est[0].at<float>(0,0));
    kf1 = new Mode_KF1(pt,dt,Accel_noise_mag);
    kf2 = new Mode_KF2(pt,dt,Accel_noise_mag);

}
//---------------------------------------------------------------------------
IMM_KF::~IMM_KF()
{
   delete kf1;
   delete kf2;
}

void IMM_KF::Preprocess(){

    ///X_est, P_est

    mix = (Mat_<float>(2,2) << 0,0,0,0);
    cbar = p_ij.t()*mu;
    for(int j=0;j<2;j++)
    {
        for(int i=0;i<2;i++)
        {
            mix.at<float>(i,j) = p_ij.at<float>(i,j)*mu.at<float>(i) / cbar.at<float>(j);
            //qDebug( "mix is %f",mix.at<float>(i,j));
        }
    }
    //qDebug("mix is %f",mix.at<float>(0,0));

    xMix = Mat(5,2, X.type(), Scalar(0));
    p_zeros = Mat(5,5, X.type(), Scalar(0));
    pMix.clear();
    pMix.push_back(p_zeros);
    pMix.push_back(p_zeros);

    for(int j=0;j<2;j++)
    {
        for(int i=0;i<2;i++)
        {
         //Mat te = mix.at<float>(i,j)*X_est.col(i);
         //qDebug( "te is %f",te.at<float>(3));
         xMix.col(j) = xMix.col(j)+mix.at<float>(i,j)*X_est.col(i);
        }
        //qDebug( "check is %f",xMix.at<float>(3,1));

        for(int i=0;i<2;i++)
        {
          //Mat te = X_est.col(0);
          //Mat te2 = xMix.col(0);
          //qDebug( "te is %f",te.at<float>(0));
          //qDebug( "te2 is %f",te2.at<float>(0));

          pMix[j] =  pMix[j]+ mix.at<float>(i,j)*(P_est[i]+(X_est.col(i)-xMix.col(j))*(X_est.col(i)-xMix.col(j)).t());
        }
        //qDebug( "check is %f",pMix[1].at<float>(1,1));
    }

}

void IMM_KF::Prediction(){
     //qDebug("xMix are %f %f",xMix.col(0).at<float>(0),xMix.col(0).at<float>(1));

     //Mat x_mix =  xMix.col(0);
    //qDebug( "x mix check is %f",x_mix.at<float>(0));

     kf1-> Prediction(xMix.col(0),P_est[0]);
     kf2-> Prediction(xMix.col(1),P_est[1]);
}


//---------------------------------------------------------------------------
Point2d IMM_KF::Update(Point2f p, bool DataCorrect)
{
    Point2f t_update_kf1,t_update_kf2;
    t_update_kf1 = kf1->Update(p,DataCorrect);
    t_update_kf2 = kf2->Update(p,DataCorrect);

    X_est.col(0) = (kf1->getxstate())*1;
    X_est.col(1) = (kf2->getxstate())*1;

    //qDebug("kf1 state is %f and kf2 state is %f",X_est.at<float>(0,0),X_est.at<float>(0,1));

    P_est[0] = kf1->getpstate();
    P_est[1] = kf2->getpstate();

    //qDebug( "speed is %f %f",X_est.at<float>(0,2),X_est.at<float>(0,3));


    double likelihood_kf1 = kf1->getlikelihood().at<float>(0);
    double t_mu_kf1 = likelihood_kf1* cbar.at<float>(0);
    mu.at<float>(0) = t_mu_kf1;
    double likelihood_kf2 = kf2->getlikelihood().at<float>(0);
    double t_mu_kf2 = likelihood_kf2* cbar.at<float>(1);
    mu.at<float>(1) = t_mu_kf2;

    mu = mu / (mu.at<float>(0)+mu.at<float>(1));

    //qDebug("like are %f %f",likelihood_kf1,likelihood_kf2);
    //qDebug("mu are %f %f",mu.at<float>(0),mu.at<float>(1));


    re_xstate = Mat(5,1, X.type(), Scalar(0));
    for(size_t i=0;i<2;i++)
    {
      Mat t = Mat(5,1, X.type(), Scalar(0));
      t.col(0) = X_est.col(i)*mu.at<float>(i);
      re_xstate = re_xstate + t;
      //qDebug("re_xstate are %f",re_xstate.at<float>(3));
    }

    //qDebug( "speed is %f %f",re_xstate.at<float>(2),re_xstate.at<float>(3));

    re_pstate = Mat(5,5, X.type(), Scalar(0));
    for(size_t i=0;i<2;i++)
    {
      re_pstate = re_pstate +  mu.at<float>(i)*(P_est[i]+(X_est.col(i)-re_xstate)*(X_est.col(i)-re_xstate).t());
    }

    Point2d temp1;
    temp1.x = re_xstate.at<float>(0);
    temp1.y = re_xstate.at<float>(1);
    sp_est.x = re_xstate.at<float>(2);
    sp_est.y = re_xstate.at<float>(3);
    turn_rate = re_xstate.at<float>(4);
    //qDebug("turnrate is %f",turn_rate);
    return temp1;
}

Mat IMM_KF :: getpossibility()
{
   return mu;
}


Point2d IMM_KF::getspeed()
{
    return sp_est;
}

Mat IMM_KF::getxstate(){
    return re_xstate;
}

Mat IMM_KF::getpstate(){
    return re_pstate;
}

float IMM_KF::getturnrate(){
    return turn_rate;
}
