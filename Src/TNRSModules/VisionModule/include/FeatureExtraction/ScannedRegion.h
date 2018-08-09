/**
 * @file FeatureExtraction/ScannedRegion.h
 *
 * This file defines the struct ScannedRegion.
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 12 Mar 2018
 */

#pragma once

#include "Utils/include/VisionUtils.h"

/**
 * @struct ScannedRegion
 * @brief Holds information about the regions resulting from scanning 
 *   the image.
 */
struct ScannedRegion
{
  /**
   * Constructor
   * 
   * @param pts: Points defining the bounding box of the region
   */
  ScannedRegion(const vector<Point>& pts)
  {
    searched = false;
    rect = boundingRect(pts);
    center = Point(rect.x + rect.width / 2, rect.y + rect.height / 2);
    int yBase = rect.y + rect.height;
    centerBase = Point(center.x, yBase);
    leftBase = Point(rect.x, yBase);
    rightBase = Point(rect.x + rect.width, yBase);
  }

  /**
   * Constructor
   * 
   * @param rect: Rect defining the bounding box of the region
   */
  ScannedRegion(const Rect& rect)
  {
    searched = false;
    this->rect = rect;
    center = Point(rect.x + rect.width / 2, rect.y + rect.height / 2);
    int yBase = rect.y + rect.height;
    centerBase = Point(center.x, yBase);
    leftBase = Point(rect.x, yBase);
    rightBase = Point(rect.x + rect.width, yBase);
    horizontal = false;
  }

  Rect rect; //! Rect defining the bounding box of the region
  Point center; //! Region center
  Point centerBase; //! Center of the base
  Point leftBase; //! Left base corner
  Point rightBase; //! Right base corner
  float closestDist; //! Distance to other closest scanned region
  bool searched; //! Whether this region has already been searched
  bool horizontal; //! Whether this region has width / height > 1
  boost::shared_ptr<ScannedRegion> pred; //! Preceding ScannedRegion in search
  boost::shared_ptr<ScannedRegion> bestNeighbor; //! Best neighbouring ScannedRegion
};

typedef boost::shared_ptr<ScannedRegion> ScannedRegionPtr;
