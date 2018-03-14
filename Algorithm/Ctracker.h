#ifndef CTRACKER_H
#define CTRACKER_H

#include "Kalman.h"
#include "Hungarianalg.h"
#include "SVSF.h"
#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include "Mode_KF1.h"
#include "Mode_KF2.h"
#include "Mode_EKF.h"
#include "IMM_KF.h"

using namespace cv;
using namespace std;

class CTrack
{
public:
    CTrack(Point2f p, float dt, float Accel_noise_mag);
    ~CTrack();
	vector<Point2d> trace;
	static size_t NextTrackID;
    size_t track_id,assgn;
	size_t skipped_frames; 
    Point2d predict,estimation,previous;
    Point2d assigndetection;
    Point2d speed;
    double rmse_x, rmse_y;
    double dist;
    Mat x_store,p_store;
    //TKalmanFilter* KF;
    SVSF* svsf;
    Mode_KF1* kf1;
    Mode_KF2* kf2;
    IMM_KF* immkf;
    Mode_EKF* ekf;
    Mat mu;
    int dist_flag;
    float turn;
};


class CTracker
{
public:
	

	float dt; 

	float Accel_noise_mag;

	double dist_thres;

	int maximum_allowed_skipped_frames;

	int max_trace_length;

    size_t ini_flag;

    double min_dist;

	vector<CTrack*> tracks;
	void Update(vector<Point2d>& detections);
	CTracker(float _dt, float _Accel_noise_mag, double _dist_thres=60, int _maximum_allowed_skipped_frames=10,int _max_trace_length=10);
	~CTracker(void);
};

#endif // CTRACKER_H
