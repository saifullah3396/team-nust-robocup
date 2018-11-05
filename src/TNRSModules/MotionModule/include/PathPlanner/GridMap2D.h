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

#pragma once

#include <chrono>
#include "Utils/include/DataUtils.h"
#include "Utils/include/VisionUtils.h"
#include "Utils/include/OccupancyMap.h"

using namespace std::chrono;
using namespace Utils;

namespace PathPlannerSpace
{

  /**
   * @brief Stores an OccupancyGrid in a convenient opencv Mat
   * as binary map (free: 255, occupied: 0) and as distance map (distance
   * to closest obstacle in meter).
   */
  class GridMap2D
  {
  public:
    GridMap2D();
    ///@brief Create from OccupancyGrid
    GridMap2D(const OccupancyMap& occMap, bool unknownAsObstacle = false);
    ///@brief Copy constructor, performs a deep copy of underlying data structures
    GridMap2D(const GridMap2D& other);
    virtual
    ~GridMap2D();

    void
    mapToWorld(unsigned int mx, unsigned int my, double& wx, double& wy) const;
    bool
    worldToMap(double wx, double wy, unsigned int& mx, unsigned int& my) const;
    void
    worldToMapNoBounds(double wx, double wy, unsigned int& mx,
      unsigned int& my) const;

    /// check if a coordinate is covered by the map extent (same as worldToMap)
    bool
    inMapBounds(double wx, double wy) const;

    /**
     * Inflate occupancy map by inflationRadius
     */
    void
    inflateMap(double inflationRaduis);

    /// Distance (in m) between two map coordinates (indices)
    inline double
    worldDist(unsigned x1, unsigned y1, unsigned x2, unsigned y2)
    {
      return worldDist(Point(x1, y1), Point(x2, y2));
    }

    inline double
    worldDist(const Point& p1, const Point& p2)
    {
      return GridMap2D::pointDist(p1, p2) * resolution;
    }

    /// Euclidean distance between two points:
    static inline double
    pointDist(const Point& p1, const Point& p2)
    {
      return sqrt(pointDist2(p1, p2));
    }

    /// Squared distance between two points:
    static inline double
    pointDist2(const Point& p1, const Point& p2)
    {
      return (p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y);
    }

    /// Returns distance (in m) at world coordinates <wx,wy> in m; -1 if out of bounds!
    float
    distanceMapAt(double wx, double wy) const;

    /// Returns distance (in m) at map cell <mx, my> in m; -1 if out of bounds!
    float
    distanceMapAtCell(unsigned int mx, unsigned int my) const;

    /// Returns map value at world coordinates <wx, wy>; out of bounds will be returned as 0!
    uchar
    binaryMapAt(double wx, double wy) const;

    /// Returns map value at map cell <mx, my>; out of bounds will be returned as 0!
    uchar
    binaryMapAtCell(unsigned int mx, unsigned int my) const;

    /// Returns map value at map cell <mx, my>; out of bounds will be returned as 0!
    uchar&
    binaryMapAtCell(unsigned int mx, unsigned int my);

    /// @return true if map is occupied at world coordinate <wx, wy>. Out of bounds
    ///     will be returned as occupied.
    bool
    isOccupiedAt(double wx, double wy) const;

    /// @return true if map is occupied at cell <mx, my>
    bool
    isOccupiedAtCell(unsigned int mx, unsigned int my) const;

    ///@brief Initialize map from a ROS OccupancyGrid message
    void
    setMap(const OccupancyMap& gridMap, bool unknownAsObstacle = false);

    ///@brief Initialize from an existing Map. mapInfo (in particular resolution) remains the same!
    void
    setMap(const Mat& binaryMap);

    ///@brief Recalculate the internal distance map. Required after manual changes to the grid map data.
    void
    updateDistanceMap();

    inline Point3f
    getMapOriginPose() const
    {
      return mapOriginPose;
    }

    inline float
    getResolution() const
    {
      return resolution;
    }
    /// @return the Mat distance image.
    const Mat&
    getDistanceMap() const
    {
      return distMap;
    }
    /// @return the Mat binary image.
    const Mat&
    getBinaryMap() const
    {
      return binaryMap;
    }
    /// @return the Mat distance image.
    const int&
    getWidth() const
    {
      return width;
    }
    /// @return the Mat binary image.
    const int&
    getHeight() const
    {
      return height;
    }
    /// @return the size of the Mat binary image. Note that x/y are swapped wrt. height/width
    inline const Size
    size() const
    {
      return binaryMap.size();
    }

    const static uchar FREE = 255; ///< char value for "free": 255
    const static uchar OCCUPIED = 0; ///< char value for "free": 0

  protected:
    Mat binaryMap; ///< binary occupancy map. 255: free, 0 occupied.
    Mat distMap; ///< distance map (in meter)
    float resolution;
    Point3f mapOriginPose;
    int width;
    int height;
  };

  typedef boost::shared_ptr<GridMap2D> GridMap2DPtr;
  typedef boost::shared_ptr<const GridMap2D> GridMap2DConstPtr;

}
;
