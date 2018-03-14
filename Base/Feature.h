#ifndef FEATURE_H
#define FEATURE_H


#include <opencv2/opencv.hpp>
using namespace cv;
using namespace std;

class Feature
{

public:
     vector<double> length;
     vector<double> width;
     vector<double> height;
     vector<int> max_intensity;
     vector<double> mean_intensity;
     vector<double> var_intenisty;
     Feature();
     ~Feature();

};

#endif // FEATURE_H
