#ifndef KDTREE_H
#define KDTREE_H

#pragma once
#include <vector>    // point datatype
#include <math.h>    // fabs operation
#include "MyHeaps.h" // priority queues
#include "float.h"   // max floating point number


typedef vector<double> kPoint;
/// The root node is stored in position 0 of nodesPtrs
#define ROOT 0

/// L2 distance (in dimension ndim) between two points
inline double distance_squared( const vector<double>& a, const vector<double>& b){
    double d = 0;
    double N = a.size();
    for( int i=0; i<N; i++ )
        d += (a[i]-b[i])*(a[i]-b[i]);
    return d;
}

struct Node{
    double key;	///< the key (value along k-th dimension) of the split
    int LIdx;	///< the index to the left sub-tree (-1 if none)
    int	RIdx;	///< the index to the right sub-tree (-1 if none)
    int	pIdx;   ///< index of stored data-point (NOTE: only if isLeaf)

    Node(){ LIdx=RIdx=key=pIdx=-1; }
    inline bool isLeaf() const{ return pIdx>=0; }
};



class Kdtree
{
    /// @{ kdtree constructor/destructor
    public:
        Kdtree(){}                           ///< Default constructor (only for load/save)
        Kdtree(const vector<kPoint>& points); ///< tree constructor
        ~Kdtree();                           ///< tree destructor
    private:
        int build_recursively(vector< vector<int> >& sortidx, vector<char> &sidehelper, int dim);
        // int heapsort(int dim, vector<int>& idx, int len);
    /// @}

    /// @{ basic info
    public:
        inline int size(){ return points.size(); } ///< the number of points in the kd-tree
        inline int ndims(){ return ndim; } ///< the number of dimensions of a point in the kd-tree
    /// @}

    /// @{ core kdtree data
    private:
        int ndim;                 ///< Number of dimensions of the data (>0)
        int npoints;              ///< Number of stored points
        vector<kPoint> points;     ///< Points data, size ?x?
        vector<Node*> nodesPtrs;  ///< Tree node pointers, size ?x?
    /// @}

    /// @{ Debuggging helpers
    public:
        void linear_tree_print() const;
        void left_depth_first_print( int nodeIdx=0 ) const;
        void print_tree( int index=0, int level=0 ) const;
        void leaves_of_node( int nodeIdx, vector<int>& indexes );
    /// @}

    /// @{ Knn Search & helpers
    public:
        int closest_point(const kPoint& p);
        void closest_point(const kPoint &p, int &idx, double &dist);
        void k_closest_points(const kPoint& Xq, int k, vector<int>& idxs, vector<double>& distances);
    private:
        void knn_search( const kPoint& Xq, int nodeIdx = 0, int dim = 0);
        bool ball_within_bounds(const kPoint& Xq);
        double bounds_overlap_ball(const kPoint& Xq);
    private:
        int k;					  ///< number of records to search for
        kPoint Bmin;  		 	  ///< bounding box lower bound
        kPoint Bmax;  		      ///< bounding box upper bound
        MaxHeap<double> pq;  	  ///< <key,idx> = <distance, node idx>
        bool terminate_search;    ///< true if k points have been found
    /// @}

    /// @{ Points in hypersphere (ball) query
    public:
        void ball_query( const kPoint& point, const double radius, vector<int>& idxsInRange, vector<double>& distances );
    private:
        void ball_bbox_query(int nodeIdx, kPoint& pmin, kPoint& pmax, vector<int>& inrange_idxs, vector<double>& distances, const kPoint& point, const double& radiusSquared, int dim=0);
    /// @}

    /// @{ Range (box) query
    public:
        void range_query( const kPoint& pmin, const kPoint& pmax, vector<int>& inrange_idxs, int nodeIdx=0, int dim=0 );
    private:
        bool lies_in_range( const kPoint& p, const kPoint& pMin, const kPoint& pMax );

};

#endif // KDTREE_H
