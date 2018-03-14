#ifndef DATAPROCESSCONTROL_H
#define DATAPROCESSCONTROL_H


#include "Library/Algorithm/Ctracker.h"
#include "Library/Base/VdynePointCloud.h"
#include "Library/Algorithm/LinearRegression.h"
#include <utility>

using namespace nanoflann;
using namespace cv;

struct bucket
{
    double minimum;
    double maximum;
    double frequency;
};

// This is an exampleof a custom data set class
template <typename T>
struct PointCloudType
{
    struct kPoint
    {
      T  x,y,z;
    };

    std::vector<kPoint> pts;

    // Must return the number of data points
    inline size_t kdtree_get_point_count() const { return pts.size(); }

    // Returns the distance between the vector "p1[0:size-1]" and the data point with index "idx_p2" stored in the class:
    inline T kdtree_distance(const T *p1, const size_t idx_p2,size_t /*size*/) const
    {
        const T d0=p1[0]-pts[idx_p2].x;
        const T d1=p1[1]-pts[idx_p2].y;
        const T d2=p1[2]-pts[idx_p2].z;
        return d0*d0+d1*d1+d2*d2;
    }

    // Returns the dim'th component of the idx'th point in the class:
    // Since this is inlined and the "dim" argument is typically an immediate value, the
    //  "if/else's" are actually solved at compile time.
    inline T kdtree_get_pt(const size_t idx, int dim) const
    {
        if (dim==0) return pts[idx].x;
        else if (dim==1) return pts[idx].y;
        else return pts[idx].z;
    }

    // Optional bounding-box computation: return false to default to a standard bbox computation loop.
    //   Return true if the BBOX was already computed by the class and returned in "bb" so it can be avoided to redo it again.
    //   Look at bb.size() to find out the expected dimensionality (e.g. 2 or 3 for point clouds)
    template <class BBOX>
    bool kdtree_get_bbox(BBOX& /*bb*/) const { return false; }

};



class DataProcessControl
{
public:
    DataProcessControl();
    DataProcessControl(std::vector<std::vector<VdynePointCloud::Point3D>>& Single_Point_Cloud, CTracker *trker);

    void GroundSegmentation(double& ma, double& mb, size_t &reducesize);
    void AssignCluster(double radius);
    void Replace(vector<int> &input,set<int> toReplace,int replacement);
    void Replace(vector<int> &input,int toReplace,int replacement);
    vector<int> ReNumber(vector<int> input);
    void BoundedBox(double MaxLength,double MinLength,double MaxWidth,double MinWidth);
    void Tracking();
    VdynePointCloud getClusterPointCloud();
    VdynePointCloud getSegmentationPointCloud();
    VdynePointCloud getGroundPointCloud();
    vector<double> getDistance();
    vector<double> getCenterPtsX();
    vector<double> getCenterPtsY();
    vector<double> getDeltaX();
    vector<double> getDeltaY();
    double getMinDistance();
    VdynePointCloud getMinCluster();
    int getMinPos();
    double getMinX();
    double getMinY();
    void getTrackingRMSE();

protected:
    std::vector<std::vector<VdynePointCloud::Point3D>> SinglePointCloud;
    VdynePointCloud Ng_PointCloud, G_PointCloud;
    CTracker* tracker;
    std::vector<double> in_x,in_y,in_z,in_intensity,loss;
    std::vector< std::vector<double> > input_data;
    int num_of_col,num_of_row;
    double init_value;
    std::vector<int> clusterLabels,Num,ClassifierLabel;
    set<int> ClusterNumbers;
    std::vector<double> CenterPtsX,CenterPtsY,DeltaX,DeltaY, distance;
    Feature feat;
    VdynePointCloud ClusterPointCloud;
    PointCloudType<double> cloud;
    vector<Point2d> mea;
    double dist_min;
    vector<VdynePointCloud> clusterpc;
    int min_pos;
    FILE * pFile;
};

#endif // DATAPROCESSCONTROL_H
