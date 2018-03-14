#ifndef LINEARREGRESSION_H
#define LINEARREGRESSION_H

#include "Library/Base/VdynePointCloud.h"
#include "Library/Algorithm/Kdtree.h"
#include "Library/Algorithm/Ctracker.h"
#include "Library/Algorithm//nanoflann.hpp"
#include "Library/Base/Feature.h"
#include <set>
#include <ctime>
#include <array>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <QFileDialog>
#include <libiomp/omp.h>
#include <array>
#define PI 3.14159265
#include "global.h"



class LinearRegression
{
public:
    LinearRegression();
    int Init(const std::vector<Point2f>& points);
    int Run();
    int PrintResult();
    double getMa();
    double getMb();
    double CaculateCost(double a, double b);



    // a，b的取值范围
    double MIN_a;
    double MAX_a;
    double MIN_b;
    double MAX_b;
    // 梯度递增值
    double INC;
    // a，b，样本的保存
    double m_a;
    double m_b;
    std::vector<Point2f> m_points;


};

#endif // LINEARREGRESSION_H
