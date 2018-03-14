#include "LinearRegression.h"
#include <opencv2/opencv.hpp>
#include <QFileDialog>
#include <libiomp/omp.h>
#include <array>
#define PI 3.14159265
#include "global.h"

LinearRegression::LinearRegression()
{


    // a，b的取值范围
     MIN_a = -0.2;
     MAX_a = 0.2;
     MIN_b = -0.5;
     MAX_b = 0.5;
    // 梯度递增值
     INC = 0.001;
}


// one-dimensional
// Y=f(X)=aX+b


// 第一步骤：初始化
int LinearRegression::Init(const std::vector<Point2f>& points)
{
     if(points.size() == 0)
        {
                return -1;

        }

        m_points = points;
}

        // 第二步骤：计算a和b
int LinearRegression::Run()
{        /// initial ma = -0.004
         /// initial mb = -0.043
            // 先将a和b取个随机的初值，此处取了0
       m_a = -0.004;
       m_b = -0.043;

         ///m_a = 0;
         ///m_b = 0;
        double minCost = CaculateCost(m_a,m_b);

        double curCost = 0.0;
            // 先计算最优的a
        for(double a=MIN_a; a<=MAX_a; a+=INC)
        {
                curCost = CaculateCost(a,m_b);
                if(curCost< minCost)
                {
                    m_a = a;
                    minCost = curCost;
                }
        }

            // 再计算最优的b
         for(double b=MIN_b; b<=MAX_b; b+=INC)
         {
           curCost = CaculateCost(m_a,b);
           if(curCost< minCost)
             {
               m_b = b;
               minCost = curCost;
             }
         }

}

        // 第三步骤：输出结果
int LinearRegression::PrintResult()
{
       //printf("Y=f(X)=%lfX+(%lf)\n",m_a,m_b);
       //printf("minCost=%lf\n",CaculateCost(m_a,m_b));
}


        // 内部函数：给定a,b，输出当前所有样本的预计与实际值的方差
double LinearRegression::CaculateCost(double a, double b)
{
    double cost = 0.0;
    double xReal = 0.0;
    double yReal = 0.0;
    double yPredict = 0.0;
    double yDef = 0.0;
    for(size_t i=0;i< m_points.size();++i)
       {
        // x实际值
        xReal = m_points[i].x;
        ///qDebug("x is %f",xReal);
        // y实际值
        yReal = m_points[i].y;
        ///qDebug("y is %f",yReal);
        // y预测值
        yPredict = a*xReal + b;

        yDef = yPredict - yReal;
        // 累加方差
        cost += (yDef*yDef);
        }
        return cost;
}

double LinearRegression::getMa(){
    return m_a;
}


double LinearRegression::getMb(){
    return m_b;
}
