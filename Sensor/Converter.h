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

/** \file Converter.h
    \brief This file defines utilities to convert Velodyne data packets to point
           clouds or scan clouds.
  */

#ifndef CONVERTER_H
#define CONVERTER_H

#include "Library/Sensor/DataPacket.h"
#include "Library/Sensor/Calibration.h"
#include "Library/Base/VdynePointCloud.h"


/** The Converter namespace contains utilities to convert Velodyne data packets
     to point clouds or scan clouds.
    \brief Velodyne data packets converter
  */
namespace Converter {
  /** \name Constants
    @{
    */
  /// Minimum distance for representing points
  static const double mMinDistance = 0.9;
  /// Maximum distance for representing points
  static const double mMaxDistance = 70.0;
  /// Conversion in meters
  static const size_t mMeterConversion = 100;



  /** @}
    */

  /** \name Methods
    @{
    */
  /// The toPointCloud function converts a data packet into a point cloud
  void toPointCloud(const DataPacket& dataPacket, const Calibration&
    calibration, VdynePointCloud& pointCloud, std::vector<std::vector<VdynePointCloud::Point3D>>& SingleLaser, double minDistance =
    Converter::mMinDistance, double maxDistance = Converter::mMaxDistance);
  /// Normalize an angle positive
  double normalizeAnglePositive(double angle);
  /// Normalize an angle
  double normalizeAngle(double angle);
  ///
  void getthre(std::vector<double> thre_dist);
  /** @}
    */
}


#endif // CONVERTER_H
