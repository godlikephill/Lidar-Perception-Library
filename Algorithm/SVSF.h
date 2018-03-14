#ifndef SVSF_H
#define SVSF_H

#include "opencv2/opencv.hpp"
#include <opencv/cv.h>


using namespace cv;
using namespace std;

class SVSF
{
public:
    float T,sigmav,sigmaw;
    //Point2f LastResult;
    int rows,Gam;
    Mat X,P,F,Gama,H,Q,R,S;
    Mat m_Boundary,u_Boundary,error;
    Mat H1,Phi11,Phi12,Phi21,Phi22,Psik,Psik1_lim,Psik2_lim;
    Mat sat;
    //Mat x_GVBL_SVSF,err_GVBL_SVSF,P_GVBL_SVSF,S_GVBL_SVSF,zp_GVBL_SVSF;
//--------------------------------------------------------------------------
    Mat x_est,p_est,x_pre,p_pre,z_pre;
    Mat innov;
//--------------------------------------------------------------------------
    Point2f sp_est;
    SVSF(Point2f p,float dt,float Accel_noise_mag);
    ~SVSF();
    Point2d Update(Point2f p, bool DataCorrect);
    Point2d Prediction();
    Mat signum(Mat src);
    Point2d getspeed();
    Mat getxstate();
    Mat getprexstate();
    Point2d geterror();
};





#endif // SVSF_H
