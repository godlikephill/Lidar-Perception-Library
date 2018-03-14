#include "Ctracker.h"
using namespace cv;
using namespace std;

size_t CTrack::NextTrackID=0;
// ---------------------------------------------------------------------------
// Track constructor
// The track begins from initial point(pt)
// ---------------------------------------------------------------------------
CTrack::CTrack(Point2f pt, float dt, float Accel_noise_mag)
{
    track_id=NextTrackID;
    NextTrackID++;
    // Every track have its own Kalman filter
    // it user for next point position prediction
    //svsf = new SVSF(pt,dt,Accel_noise_mag);
    ///kf1 = new Mode_KF1(pt,dt,Accel_noise_mag);
    //immkf = new IMM_KF(pt,dt,Accel_noise_mag);
    ekf = new Mode_EKF(pt,dt,Accel_noise_mag);
    // Here stored points coordinates, used for next position prediction.
    predict = pt;
    previous = pt;
	skipped_frames=0;
    assgn = -1;
    dist = 0;
    mu = (Mat_<float>(2,1) << 0, 0);
    turn = 0;
    rmse_x = 0;
    rmse_y = 0;
    dist_flag = 0;
    speed.x = 0;
    speed.y = 0;
    turn = 0;
    //x_store = new Mat(4,1,CV_32FC1);
    //p_store = new Mat(4,4,CV_32FC1);
    //Mat x_store(4,1,CV_32FC1);
    //Mat p_store(4,4,CV_32FC1);
}
// ---------------------------------------------------------------------------
//
// ---------------------------------------------------------------------------
CTrack::~CTrack()
{
    // free resources
    ///delete SF;
    ///delete kf1;
    //delete immkf ;
    //delete svsf;
    delete ekf;
}
// ---------------------------------------------------------------------------
// Tracker. Manage tracks. Create, remove, update.
// ---------------------------------------------------------------------------
CTracker::CTracker(float _dt, float _Accel_noise_mag, double _dist_thres, int _maximum_allowed_skipped_frames,int _max_trace_length)
{
dt=_dt;
Accel_noise_mag=_Accel_noise_mag;
dist_thres=_dist_thres;
maximum_allowed_skipped_frames=_maximum_allowed_skipped_frames;
max_trace_length=_max_trace_length;
ini_flag = 0;

}
// ---------------------------------------------------------------------------
//
// ---------------------------------------------------------------------------
void CTracker::Update(vector<Point2d>& detections)
{   // -----------------------------------
    // If there is no tracks yet, then every point begins its own track.
	// -----------------------------------
    std::cout<<"Start Update..."<<std::endl;


	if(tracks.size()==0)
    {
        size_t dsize = detections.size();
        if(dsize ==0){
           return;
        }

        // If no tracks yet
		for(int i=0;i<detections.size();i++)
        {
			CTrack* tr=new CTrack(detections[i],dt,Accel_noise_mag);
            //tr->x_store = tr->SF->X;
            //tr->p_store = tr->SF->P;
            tracks.push_back(tr);

		}	
	}


    /// Get prediction state for each track


    int N=tracks.size();
    int M=detections.size();
    std::cout<<"track number:  "<<N<<"    detect number:   "<< M <<std::endl;

    for(int i=0;i<N;i++)
    {
        tracks[i]->ekf->Prediction();
        //Mat x_state = tracks[i]->immkf->getxstate();
        Mat x_prestate = tracks[i]->ekf->getprexstate();
        tracks[i]->predict.x = x_prestate.at<float>(0);
        tracks[i]->predict.y = x_prestate.at<float>(1);

    }



	vector< vector<double> > Cost(N,vector<double>(M));
    vector<int> assignment;


	// -----------------------------------
    //
	// -----------------------------------
	double dist;
    for(size_t i=0;i<tracks.size();i++)
	{	
        for(size_t j=0;j<detections.size();j++)
		{
            Point2d diff=(tracks[i]->predict-detections[j]);
			dist=sqrtf(diff.x*diff.x+diff.y*diff.y);
			Cost[i][j]=dist;
		}
	}



	// -----------------------------------
    // Solving assignment problem (tracks and predictions of Kalman Filter)
	// -----------------------------------
	AssignmentProblemSolver APS;
	APS.Solve(Cost,assignment,AssignmentProblemSolver::optimal);
/*
    for (int i = 0; i < Cost.size(); i++)
    {
        for (int j = 0; j < Cost[i].size(); j++)
        {
            std::cout << Cost[i][j];
        }
        std::cout <<" "<<std::endl;
    }

    for (std::vector<int>::iterator it = assignment.begin() ; it != assignment.end(); ++it)
      std::cout << ' ' << *it<<std::endl;
*/


    // ----------------------------------------------------------------------------
    // assign value to tracks and clean assignment from pairs with large distance
    // ----------------------------------------------------------------------------

    // Not assigned tracks
    vector<int> not_assigned_tracks;

    for(size_t i=0;i<tracks.size();i++)
    {
       ///pass assigment values to tracks
       tracks[i]->assgn = assignment[i];       

       if(tracks[i]->assgn!=-1)
       {
           if(Cost[i][tracks[i]->assgn]>dist_thres)
           {
              // Mark unassigned tracks, and increment skipped frames counter,
              // when skipped frames counter will be larger than threshold, track will be deleted
              tracks[i]->assgn =-1;
              tracks[i]->skipped_frames++;
              not_assigned_tracks.push_back(i);
           }
       }
       else
       {
           //If track have no assigned detect, then increment skipped frames counter
           tracks[i]->skipped_frames++;
       }
    }


    // ----------------------------------------------------
    // If track didn't get detects long time, remove it
    // -----------------------------------------------------
    vector<CTrack*>::iterator pt;


    for ( pt = tracks.begin(); pt != tracks.end(); )
    {
       if( (*pt)->skipped_frames>maximum_allowed_skipped_frames )
       {
          delete * pt;
          pt = tracks.erase(pt);
       }
       else {
          ++pt;
       }
    }

    /// Update IMM filters state

    for(int i=0;i<tracks.size();i++)
    {
        //tracks[i]->immkf->Preprocess();
        if(tracks[i]->assgn!=-1) // If we have assigned detect, then update using its coordinates
        {
            std::cout<<"update using its measurement"<<std::endl;
            tracks[i]->skipped_frames=0;
            /// IMM preprocessing
            //tracks[i]->immkf->Preprocess();
            /// IMM prediction
            //tracks[i]->svsf->Prediction();
            ///IMM update
            tracks[i]->estimation=tracks[i]->ekf->Update(detections[tracks[i]->assgn],1);
            ///IMM Model Possibility
            //tracks[i]->mu = tracks[i]->ekf-> getpossibility();
            ///
            tracks[i]->assigndetection = detections[tracks[i]->assgn];

            double x =  tracks[i]->assigndetection.x;
            double y =  tracks[i]->assigndetection.y;
            double dista = sqrt(x*x + y*y);

            tracks[i]->dist = dista;
            ///estimate the relatively speed
            tracks[i]->speed = tracks[i]->ekf->getspeed();
            //tracks[i]->speed = Cost[i][tracks[i]->assgn]/0.1;
            ///IMM Turn Rate
            //tracks[i]->turn = tracks[i]->ekf->getturnrate();
            /// root mean square errors
            ///
            //std::cout<<"estimation "<< tracks[i]->estimation.x <<std::endl;
            //std::cout<<"detection x  "<< x <<std::endl;
            Point2d err;
            err = tracks[i]->ekf->geterror();
            tracks[i]->rmse_x = err.x;
            tracks[i]->rmse_y = err.y;
        }
        else				  // if not continue using predictions
        {
            std::cout<<"update using previous predictions"<<std::endl;
            /// IMM preprocessing
            //tracks[i]->immkf->Preprocess();
            /// IMM prediction
            //tracks[i]->svsf->Prediction();

            ///IMM update
            tracks[i]->estimation=tracks[i]->ekf->Update(Point2f(1,1),0);
            /// IMM Model Possibility
            //tracks[i]->mu = tracks[i]->immkf-> getpossibility();

            tracks[i]->assigndetection = tracks[i]->estimation;

            double x =  tracks[i]->assigndetection.x;
            double y =  tracks[i]->assigndetection.y;
            double dista = sqrt(x*x + y*y);

            tracks[i]->dist = dista;
            ///estimate the relatively speed
            tracks[i]->speed = tracks[i]->ekf->getspeed();

            ///IMM Turn Rate
           // tracks[i]->turn = tracks[i]->immkf->getturnrate();
            /// root mean square errors
            //tracks[i]->rmse_x = tracks[i]->predict.x - x;
            //tracks[i]->rmse_y = tracks[i]->predict.y - y;
            Point2d err;
            err = tracks[i]->ekf->geterror();
            tracks[i]->rmse_x = err.x;
            tracks[i]->rmse_y = err.y;

        }

        if(tracks[i]->trace.size()>max_trace_length)
        {
            tracks[i]->trace.erase(tracks[i]->trace.begin(),tracks[i]->trace.end()-max_trace_length);
        }
        //std::cout<<"Prediction    "<<tracks[i]->prediction.x<<"   "<<tracks[i]->prediction.y<<std::endl;
        tracks[i]->trace.push_back(tracks[i]->estimation);
        //tracks[i]->SF->LastResult=tracks[i]->prediction;
    }


	// -----------------------------------
    // Search for unassigned detects
	// -----------------------------------
	vector<int> not_assigned_detections;
	vector<int>::iterator it;
	for(int i=0;i<detections.size();i++)
	{
		it=find(assignment.begin(), assignment.end(), i);
		if(it==assignment.end())
		{
			not_assigned_detections.push_back(i);

		}
	}
    //std::cout<<"not assign detection   "<<not_assigned_detections.size()<<std::endl;
	// -----------------------------------
    // and start new tracks for them
	// -----------------------------------
	if(not_assigned_detections.size()!=0)
	{
		for(int i=0;i<not_assigned_detections.size();i++)
        {   std::cout<<"detection not assign for new track   "<<detections[not_assigned_detections[i]].x<<"  "<<detections[not_assigned_detections[i]].y<<std::endl;
			CTrack* tr=new CTrack(detections[not_assigned_detections[i]],dt,Accel_noise_mag);
			tracks.push_back(tr);
		}	
	}

    //std::cout<<"Update completed"<<std::endl;


}
// ---------------------------------------------------------------------------
//
// ---------------------------------------------------------------------------
CTracker::~CTracker(void)
{
	for(int i=0;i<tracks.size();i++)
	{
	delete tracks[i];
	}
	tracks.clear();
}
