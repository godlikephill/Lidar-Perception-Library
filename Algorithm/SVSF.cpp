#include "SVSF.h"
#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <math.h>
using namespace cv;
using namespace std;



SVSF::SVSF(Point2f pt,float dt,float Accel_noise_mag)
{
    sp_est.x =0;
    sp_est.y =0;
    //Dummy Example
    ///initial process state
    //x_state = (Mat_<float>(4,1) <<pt.x,pt.y,sp.x,sp.y);
    ///initial estimation
    X = (Mat_<float>(4,1) <<pt.x,pt.y,sp_est.x,sp_est.y);
    // Sampling time
    T = dt;
    // Covariance matrix
    P = 10 * (Mat_<float>(4, 4) << 1,0,0,0,   0,1,0,0,  0,0,1,0,  0,0,0,1);
    //tr_svsf = (Mat_<float>(1,1) << 0);
    //tr_svsf = trace(P);
    // Transition Matrix
    F = (Mat_<float>(4, 4) << 1,0,T,0, 0,1,0,T,  0,0,1,0,  0,0,0,1);
    //Acceleration process noise
    Gama = (Mat_<float>(4,2) << 0.5*pow(T,2),0, 0, 0.5*pow(T,2),T,0, 0,T);
    //Measurement Matrix    
    H = (Mat_<float>(2,4) << 1,0,0,0,0,1,0,0);
    rows = H.rows;
    //Saturation function
    sat = (Mat_<float>(rows,1) <<0,0,0,0);
    //Process noise
    sigmav = 10;
    Q = pow(sigmav,2)*Gama*Gama.t();
    sigmaw = 1;
    // Measurment Noise Variance
    //R = sigmaw*(Mat_<float>(2,2) << 1,0,0,1);
    //Gama = [0.5*T^2 0; 0 0.5*T^2;T 0; 0 T];
    m_Boundary = (Mat_<float>(2,1) << 1,5);
    u_Boundary = (Mat_<float>(2,1) << 100,100);
    error = (1.0e-2)*(Mat_<float>(2,1) << 0,0);

    Phi11 = (Mat_<float>(2,2) <<1,0,0,1);
    Phi12 = (Mat_<float>(2,2) <<T,0,0,T);
    Phi21 = (Mat_<float>(2,2) <<0,0,0,0);
    Phi22 = (Mat_<float>(2,2) <<1,0,0,1);
    Psik1_lim = m_Boundary.diag();
    Psik2_lim = u_Boundary.diag();
    vconcat(Psik1_lim, Psik2_lim,Psik);

    Gam = 0.1;
    H1 = (Mat_<float>(2,2) <<1,0,0,1);
    // initilization
    x_est = X;
    p_est = P;
    //S_GVBL_SVSF = H * P_GVBL_SVSF * H.t() + R;
    //zp_GVBL_SVSF = H * x_state;

}
//---------------------------------------------------------------------------
SVSF::~SVSF()
{

}



Point2d SVSF::Prediction(){

    ///Prediction(a priori) stage
    //Predicted State
    x_pre = F*x_est;
    //Predicted Measurement
    z_pre = H * x_pre;
    //Predicted Covariance
    p_pre = F*p_est+F.t() + Q;
    //innovation covariance
    S = H * p_pre * H.t();

    Point2f predict;
    predict.x = x_pre.at<float>(0);
    predict.y = x_pre.at<float>(1);
    return predict;
}


//---------------------------------------------------------------------------
Point2d SVSF::Update(Point2f p, bool DataCorrect)
{
   // Mat x_est(4,1,CV_32FC1);
   // Mat p_est(4,4,CV_32FC1);

   // x_est = x_store;
   // p_est = p_store;
    //std::cout<<"read data begin..."<<std::endl;


    Mat z_current(2,1,CV_32FC1);
    if(!DataCorrect)
    {   //std::cout<<"read previous estimated positions..."<<std::endl;
        //std::cout<<"x_est is   "<<x_est.at<float>(0) <<"   "<<x_est.at<float>(1) <<std::endl;
        Point2f temp2;
        temp2.x = x_est.at<float>(0);
        temp2.y = x_est.at<float>(1);
        return temp2;
    }

    else
    {   //update using current measurement
        //std::cout<<"update current measurement..."<<std::endl;
        z_current.at<float>(0) = p.x;
        z_current.at<float>(1) = p.y;
    }

    //z_current = H*x_GVBL_SVSF;
    //error between current measurement and predicted measurement
    Mat innov = z_current - z_pre;
    //std::cout << "innov  " << innov << std::endl;

    Mat errk = innov;
    //std::cout<<"error calculation"<<std::endl;

    Mat Pk11 = p_pre(Range(0,2),Range(0,2));
    Mat Pk12 = p_pre(Range(0,2),Range(2,4));
    Mat Pk21 = p_pre(Range(2,4),Range(0,2));
    Mat Pk22 = p_pre(Range(2,4),Range(2,4));

    //std::cout<<"start error comcatenate"<<std::endl;
    Mat errky = Phi22*Phi12.inv()*errk;
    Mat errkzy;
    vconcat(errk,errky,errkzy);  // vertical concatenate matrices

    //std::cout<<"Start Variable Boundary Layer"<<std::endl;


    //std::cout << "error  " << error << std::endl;

    //Variable Boundary Layer
    Mat A1 = abs(errk) + Gam * abs(error);
    Mat A2 = (abs(errky) + Gam* abs(Phi12.inv()*errk));

     //std::cout << "A1  " << A1 << std::endl;

    // Creates a diagonal matrix
    Mat A1bar = (Mat_<float>(2,2) << A1.at<float>(0),0,0,A1.at<float>(1));
    Mat A2bar = (Mat_<float>(2,2) << A2.at<float>(0),0,0,A2.at<float>(1));



    //Mat A1bar = A1.diag();
    //Mat A2bar = A2.diag();



    Mat A1bar_pinv;
    invert(A1bar,A1bar_pinv,DECOMP_SVD);
    Mat A2bar_pinv;
    invert(A2bar,A2bar_pinv,DECOMP_SVD);
    Mat S_pinv;
    invert(S,S_pinv,DECOMP_SVD);

    Mat Psik1inv = A1bar_pinv*H1*Pk11*H1.t() *S_pinv;
    Mat Psik2inv = A2bar_pinv*Pk21*H1.t() *S_pinv;

/////////////////////////////////////////////////////////////////////////////////


    //std::cout<<"Start Saturation Function"<<std::endl;
    //-------------------------------------------------------------------------------------
    /// Saturation Functions
    for(size_t i=0;i<rows;i++)
    {
        auto a = errkzy.at<float>(i);
        auto b = Psik.at<float>(i);
        if(abs(a/b)>=1)
        {
           auto x = a/b;
           if (x > 0) sat.at<float>(i) = 1;
           if (x < 0) sat.at<float>(i) = -1;
           else sat.at<float>(i) = 0;
        }
        else if(abs(a/b)<1){
           sat.at<float>(i) = a/b;
        }

        if(i==1||i==2)
        {
          if(errk.at<float>(i)==0){
              errk.at<float>(i) = 1e-40;
              errky.at<float>(i) = 1e-40;
          }
        }
    }

//std::cout << "errky  " << errky << std::endl;

    Mat Psik1inv_pinv;
    invert(Psik1inv,Psik1inv_pinv,DECOMP_SVD);
    Mat Psik2inv_pinv;
    invert(Psik2inv,Psik2inv_pinv,DECOMP_SVD);

    //std::cout << "Psik1inv  " << Psik1inv  << std::endl;
    //std::cout << "Psik1inv_pinv  " << Psik1inv_pinv  << std::endl;



    Mat hi = (Mat_<float>(2,2) << 1,2,3,4);
    //Mat li = (Mat_<float>(2,2) << 0.010570526,0,0,0.017401695);
    //Mat ji = (Mat_<float>(2,2) << 1.290661226393136,0,0, 0.336851672921658);
    //Mat tttt = hi*li*ji;
    //std::cout << "tttt  " << tttt  << std::endl;



    //Mat hi = (Mat_<float>(2,2) << 11,1,2,8);
    //Mat li = (Mat_<float>(2,2) << 10,7,1,2);
    //Mat dstii = hi>li;
    //dstii = dstii/255;
    //auto tra = trace(dstii);
    //int trai = tra(0);
    //std::cout << "tra   " << trai  << std::endl;
    //int eq11 = cv::countNonZero(trace(dstii));
    //std::cout << "eq11  " << eq11 << std::endl;


    Mat dst1 = Psik1inv_pinv > Psik1_lim;
    dst1  = dst1 /255;
   // std::cout << "dst1  " << dst1 << std::endl;
    auto tra1 = trace(dst1);
    int tra1i = tra1(0);

    //bool eq1 = cv::countNonZero(trace(dst1));
    Mat dst2 = Psik2inv_pinv > Psik2_lim;
    dst2  = dst2 /255;
    auto tra2 = trace(dst2);
    int tra2i = tra2(0);


    //bool eq2 = cv::countNonZero(trace(dst2));


    Mat H1_pinv,K1,K2,errk_diag_pinv,errky_diag_pinv;
    invert(H1,H1_pinv,DECOMP_SVD);

    if(tra1i>1){
      // Creates a diagonal matrix
      Mat errk_diag = (Mat_<float>(2,2) << errk.at<float>(0),0,0,errk.at<float>(1));
      invert(errk_diag,errk_diag_pinv,DECOMP_SVD);

      Mat errk_sign = signum(errk);
      // Creates a diagonal matrix
      Mat errk_sign_diag = (Mat_<float>(2,2) << errk_sign.at<float>(0),0,0,errk_sign.at<float>(1));
      K1 = H1_pinv * A1bar *errk_sign_diag*errk_diag_pinv;
    }
    else{
      K1 = H1_pinv * A1bar *Psik1inv;
      //std::cout << "H1_pinv    "<< H1_pinv<<std::endl;
      //std::cout << "A1bar    "<< A1bar<<std::endl;
      //std::cout << "Psik1inv    "<< Psik1inv<<std::endl;
      //std::cout << "K1 else else    "<< K1<<std::endl;
    }

    //std::cout << "K1  " << K1.at<float>(0) << std::endl;



    if(tra2i>1){

     Mat errky_diag = (Mat_<float>(2,2) << errky.at<float>(0),0,0,errky.at<float>(1));
     invert(errky_diag,errky_diag_pinv,DECOMP_SVD);
     Mat errky_sign = signum(errky);
      // Creates a diagonal matrix
     Mat errky_sign_diag = (Mat_<float>(2,2) << errky_sign.at<float>(0),0,0,errky_sign.at<float>(1));
     K2 =  A2bar *errky_sign_diag*errky_diag_pinv;
    }
    else{
     K2 =  A2bar * Psik2inv;
    }




 //-------------------------------------------------------------------------------------
 /// A posteriori state update
    Mat K;
    vconcat(K1, K2,K);
    Mat xx = x_est;
    x_est = x_pre + K*errk;

    //std::cout << "K  " << K.at<float>(0) << "errk   " << errk.at<float>(0) << std::endl;

    //std::cout <<"x_pre   "<<x_pre.at<float>(0) <<"x_est  " << x_est.at<float>(0)<< std::endl;


    
    Mat H_inv,K_inv;
    invert(H,H_inv,DECOMP_SVD);
    invert(K,K_inv,DECOMP_SVD);

    p_est = p_pre - K*H*p_pre - p_pre*H_inv*K_inv + K*(H*p_pre*H_inv + R)*K_inv;
    errk = innov + H*xx - H*x_est;

    p_est = 0.5*(p_est + p_est.t());
    //std::cout << "errk  " << errk << std::endl;

///%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
   Point2d temp1;
   temp1.x = x_est.at<float>(0);
   temp1.y = x_est.at<float>(1);
   sp_est.x = x_est.at<float>(2);
   sp_est.y = x_est.at<float>(3);
   error = errk;
   std::cout<<"SVSF finished...." <<std::endl;
   std::cout<<"error is "<<error<<std::endl;
   return temp1;
}


Point2d SVSF::getspeed(){
    return sp_est;
}

Point2d SVSF::geterror(){
   Point2d t;
   t.x = error.at<float>(0);
   t.y = error.at<float>(1);
   return t;
}


Mat SVSF::signum(Mat src)
{
    Mat z = Mat::zeros(src.size(), src.type());
    Mat a = (z < src) & 1;
    Mat b = (src < z) & 1;
    Mat dst;
    addWeighted(a,1.0,b,-1.0,0.0,dst, CV_32F);
    return dst;
}

Mat SVSF::getxstate()
{
    return x_est;
}


Mat SVSF::getprexstate()
{
    return x_pre;
}
