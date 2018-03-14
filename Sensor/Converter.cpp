/******************************************************************************
 * Copyright (C) 2011 by Jerome Maye                                          *
 * jerome.maye@gmail.com                                                      *
 *                                                                            *
 * This program is free software; you can redistribute it and/or modify       *
 * it under the terms of the Lesser GNU General Public License as published by*
 * the Free Software Foundation; either version 3 of the License, or          *
 * (at your option) any later version.                                        *
 *                                                                            *
 * This program is distributed in the hope that it will be useful,            *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of             *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the              *
 * Lesser GNU General Public License for more details.                        *
 *                                                                            *
 * You should have received a copy of the Lesser GNU General Public License   *
 * along with this program. If not, see <http://www.gnu.org/licenses/>.       *
 ******************************************************************************/
#include <QFileDialog>
#include "Library/Sensor/Converter.h"
#include <typeinfo>
#include <QFileDialog>
#include <cmath>
#include <boost/assign/std/vector.hpp>
#define PI 3.14159265
/******************************************************************************/
/* Methods                                                                    */
/******************************************************************************/

namespace Converter {

void toPointCloud(const DataPacket& dataPacket, const Calibration&
    calibration, VdynePointCloud& pointCloud, std::vector<std::vector<VdynePointCloud::Point3D>>& SinglePointCloud,double minDistance, double
    maxDistance) {
  pointCloud.setTimestamp(dataPacket.getTimestamp());


  for (size_t i = 0; i < dataPacket.mDataChunkNbr; ++i)
  {
    /// 1 packet = 12 shoots
    size_t idxOffs = 0;
    const DataPacket::DataChunk& data = dataPacket.getDataChunk(i);
    if (data.mHeaderInfo == dataPacket.mLowerBank)
      idxOffs = data.mLasersPerPacket;
    const double rotation =
      calibration.deg2rad((double)data.mRotationalInfo /
      (double)dataPacket.mRotationResolution);
    if (i == 0)
      pointCloud.setStartRotationAngle(rotation);
    else if (i == dataPacket.mDataChunkNbr -1)
      pointCloud.setEndRotationAngle(rotation);

    std::vector<VdynePointCloud::Point3D> col_point;
    col_point.reserve(23);


    for (size_t j = 0; j < 32; j=j+1)
    {

      size_t laserIdx = idxOffs + j;
      const double distance = (calibration.getDistCorr(laserIdx)
        + (double)data.mLaserData[j].mDistance /
        (double)dataPacket.mDistanceResolution) /
        (double)mMeterConversion;
      //qDebug("distance is %f",distance);
      VdynePointCloud::Point3D point;
      if ((distance < minDistance) || (distance > maxDistance))
      {

          point.mX = 0;
          point.mY = 0;
          point.mZ = 0;
          ///2.6 for suv, 2 for sedan
          point.mZ = 0;
          point.mIntensity = 0;
          point.mLaserID = laserIdx;
          point.mDist = 0;
          point.mRotation = 0;
      }
      else
      {
          const double sinRot = sin(rotation) *
            calibration.getCosRotCorr(laserIdx) -
            cos(rotation) * calibration.getSinRotCorr(laserIdx);
          const double cosRot = cos(rotation) *
            calibration.getCosRotCorr(laserIdx) +
            sin(rotation) * calibration.getSinRotCorr(laserIdx);
          const double horizOffsCorr =
            calibration.getHorizOffsCorr(laserIdx) /
            (double)mMeterConversion;
          const double vertOffsCorr =
            calibration.getVertOffsCorr(laserIdx) /
            (double)mMeterConversion;
          const double xyDist = distance *
            calibration.getCosVertCorr(laserIdx) -
            vertOffsCorr * calibration.getSinVertCorr(laserIdx);


          point.mX = xyDist * sinRot - horizOffsCorr * cosRot;
          point.mY = xyDist * cosRot + horizOffsCorr * sinRot;
          point.mZ = distance *
            calibration.getSinVertCorr(laserIdx) + vertOffsCorr *
            calibration.getCosVertCorr(laserIdx);
          ///2.6 for suv, 2 for sedan
          point.mZ = point.mZ + 2;
          point.mIntensity = data.mLaserData[j].mIntensity;
          point.mLaserID = laserIdx;
          point.mDist = distance;
          point.mRotation = rotation;
      }
      //std::cout<<"ID is  " <<point.mLaserID << std::endl;
      /// Reduce Points
      //***************************************************

      ///excluse blind spot area
      ///if(!( abs(point.mY) <1 && point.mX< 1.5 &&point.mX>-3))


           pointCloud.insertPoint(point);

           if(j!=15&&j!=17&&j!=19&&j!=21&&j!=23&&j!=25&&j!=27&&j!=29&&j!=31)
           {
               col_point.push_back(point);
           }
    }
    //qDebug("check is %d",col_point.size());
    if(!col_point.empty())
    {
    SinglePointCloud.push_back(col_point);
    }


    ///****************************************
    ///  1..... 32
    ///  2..... 32
    ///  .
    ///  .
    ///  .
    ///  n..... 32
    /// ****************************************
  }

  //qDebug("SinglePointCloud size is %d and %d",SinglePointCloud.size(),SinglePointCloud[0].size());

}




double normalizeAnglePositive(double angle) {
  return fmod(fmod(angle, 2.0 * M_PI) + 2.0 * M_PI, 2.0 * M_PI);
}

double normalizeAngle(double angle) {
  double value = normalizeAnglePositive(angle);
  if (value > M_PI)
    value -= 2.0 * M_PI;
  return value;
}


void getthre(std::vector<double> thre_dist){

  ///float h = 2;
  thre_dist[0]=2*sin(30.67*PI/180.0);
  thre_dist[1]=2*sin(9.33*PI/180.0);
  thre_dist[2]=2*sin(29.33*PI/180.0);
  thre_dist[3]=2*sin(8.00*PI/180.0);
  thre_dist[4]=2*sin(28*PI/180.0);
  thre_dist[5]=2*sin(6.66*PI/180.0);
  thre_dist[6]=2*sin(26.66*PI/180.0);
  thre_dist[7]=2*sin(5.33*PI/180.0);
  thre_dist[8]=2*sin(25.33*PI/180.0);
  thre_dist[9]=2*sin(4*PI/180.0);
  thre_dist[10]=2*sin(24*PI/180.0);
  thre_dist[11]=2*sin(2.67*PI/180.0);
  thre_dist[12]=2*sin(22.67*PI/180.0);
  thre_dist[13]=2*sin(1.33*PI/180.0);
  thre_dist[14]=2*sin(21.33*PI/180.0);
  thre_dist[15]= 0;
  thre_dist[16]=2*sin(20*PI/180.0);
  thre_dist[17]=0;
  thre_dist[18]=2*sin(18.67*PI/180.0);
  thre_dist[19]=0;
  thre_dist[20]=2*sin(17.33*PI/180.0);
  thre_dist[21]=0;
  thre_dist[22]=2*sin(16*PI/180.0);
  thre_dist[23]=0;
  thre_dist[24]=2*sin(14.67*PI/180.0);
  thre_dist[25]=0;
  thre_dist[26]=2*sin(13.33*PI/180.0);
  thre_dist[27]=0;
  thre_dist[28]=2*sin(12*PI/180.0);
  thre_dist[29]=0;
  thre_dist[30]=2*sin(10.67*PI/180.0);
  thre_dist[31]=0;

}


}
