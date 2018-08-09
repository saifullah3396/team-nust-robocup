/**
 * A simple 2D gridmap structure
 *
 * Copyright 2011 Armin Hornung, University of Freiburg
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the University of Freiburg nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "MotionModule/include/PathPlanner/GridMap2D.h"

namespace PathPlannerSpace
{

  GridMap2D::GridMap2D()
  {
  }

  GridMap2D::GridMap2D(const OccupancyMap& occMap, bool unknownAsObstacle)
  {
    setMap(occMap, unknownAsObstacle);
  }

  GridMap2D::GridMap2D(const GridMap2D& other) :
    binaryMap(other.getBinaryMap().clone()),
      distMap(other.getDistanceMap().clone()),
      resolution(other.getResolution()), mapOriginPose(other.getMapOriginPose())
  {
    width = other.getWidth();
    height = other.getHeight();
  }

  GridMap2D::~GridMap2D()
  {
  }

  void
  GridMap2D::updateDistanceMap()
  {
    //high_resolution_clock::time_point tStart = high_resolution_clock::now();
    //cout << "performing distnace transform" << endl;
    //VisionUtils::displayImage(binaryMap, "binaryMap");
    Mat binaryMapR;
    Size fSize(width / 8, height / 8);
    resize(binaryMap, binaryMapR, fSize);
    Mat distMapR = Mat(binaryMapR.size(), CV_32FC1);
    distanceTransform(binaryMapR, distMapR, CV_DIST_L2, 3);
    //! distance map now contains distance in meters:
    resize(distMapR, distMap, Size(width, height));
    distMap = distMap * resolution * 8;
    //high_resolution_clock::time_point t1 = 
    high_resolution_clock::now();
    //duration<double> time_span = t1 - tStart;
    //cout << "distTransform: " << time_span.count() << "secs." << endl;
  }

  void
  GridMap2D::setMap(const OccupancyMap& occMap, bool unknownAsObstacle)
  {
    //! allocate map structs so that x/y in the world correspond to x/y in the image
    //! (=> Mat is rotated by 90 deg, because it's row-major!)
    resolution = occMap.resolution;
    mapOriginPose = occMap.originPose;
    binaryMap = occMap.data;
    width = occMap.data.cols;
    height = occMap.data.rows;
    distMap = Mat(binaryMap.size(), CV_32FC1);
    //updateDistanceMap();
    /*cout << "GridMap2D created with "
     << width
     << "x"
     << height
     << " cells at "
     << resolution
     << " resolution."
     << endl;*/
  }

  void
  GridMap2D::inflateMap(double inflationRadius)
  {
    binaryMap = (distMap > inflationRadius);
    //! recompute distance map with new binary map:
    distanceTransform(binaryMap, distMap, CV_DIST_L2, CV_DIST_MASK_PRECISE);
    distMap = distMap * resolution;
  }

//! See Costmap2D for mapToWorld / worldToMap implementations:
  void
  GridMap2D::mapToWorld(unsigned int mx, unsigned int my, double& wx,
    double& wy) const
  {
    //wx = (mx + mapOriginPose.x + 0.5) * resolution;
    //wy = (mapOriginPose.y - my - 0.5) * resolution;
    wx = (mx + 0.5) * resolution;
    wy = (-my - 0.5) * resolution;
    cout << "maptoWorld:" << endl;
    cout << "mx " << mx << endl;
    cout << "my " << my << endl;
    cout << "wx " << wx << endl;
    cout << "wy " << wy << endl;

  }

  void
  GridMap2D::worldToMapNoBounds(double wx, double wy, unsigned int& mx,
    unsigned int& my) const
  {
    mx = (int) (wx / resolution + mapOriginPose.x);
    my = (int) (mapOriginPose.y - wy / resolution);
  }

  bool
  GridMap2D::worldToMap(double wx, double wy, unsigned int& mx,
    unsigned int& my) const
  {
    //if(wx  mapOriginPose.x || wy < mapOriginPose.y)
    //  return false;
    mx = (int) (wx / resolution + mapOriginPose.x);
    my = (int) (mapOriginPose.y - wy / resolution);
    if (mx < width && my < height) return true;
    return false;
  }

  bool
  GridMap2D::inMapBounds(double wx, double wy) const
  {
    unsigned mx, my;
    return worldToMap(wx, wy, mx, my);
  }

  float
  GridMap2D::distanceMapAt(double wx, double wy) const
  {
    unsigned mx, my;
    if (worldToMap(wx, wy, mx, my)) return distMap.at<float>(my, mx);
    else return -1.0f;
  }

  uchar
  GridMap2D::binaryMapAt(double wx, double wy) const
  {
    unsigned mx, my;
    if (worldToMap(wx, wy, mx, my)) return binaryMap.at < uchar > (my, mx);
    else return 0;
  }

  float
  GridMap2D::distanceMapAtCell(unsigned int mx, unsigned int my) const
  {
    return distMap.at<float>(my, mx);
  }

  uchar
  GridMap2D::binaryMapAtCell(unsigned int mx, unsigned int my) const
  {
    return binaryMap.at < uchar > (my, mx);
  }

  uchar&
  GridMap2D::binaryMapAtCell(unsigned int mx, unsigned int my)
  {
    return binaryMap.at < uchar > (my, mx);
  }

  bool
  GridMap2D::isOccupiedAtCell(unsigned int mx, unsigned int my) const
  {
    return (binaryMap.at < uchar > (my, mx) < 255);
  }

  bool
  GridMap2D::isOccupiedAt(double wx, double wy) const
  {
    unsigned mx, my;
    if (worldToMap(wx, wy, mx, my)) return isOccupiedAtCell(mx, my);
    else return true;
  }

}
;
