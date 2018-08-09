/**
 * @file FeatureExtraction/ScannedEdge.h
 *
 * This file defines the struct ScannedEdge.
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 12 Mar 2018
 */

#pragma once

#include "Utils/include/VisionUtils.h"

/**
 * @struct ScannedEdge
 * @brief Holds information about the line edge points resulting from
 *   scanning the image.
 */
struct ScannedEdge
{
  /**
   * Constructor
   * 
   * @param p: Point defining region edge in the image
   */
  ScannedEdge(const Point2f& pI, const Point2f& pW) :
    pI(pI), pW(pW), searched(false)
  {
  }

  Point2f pI; //! Point defining region edge in the image
  Point2f pW; //! Point defining region edge in the world
  float angleW;
  float closestDist; //! Distance to other closest scanned edge
  bool searched; //! Whether this line segment has already been searched
  boost::shared_ptr<ScannedEdge> pred; //! Preceding ScannedEdge in search
  boost::shared_ptr<ScannedEdge> bestNeighbor; //! Best neighbouring ScannedEdge
  boost::shared_ptr<ScannedEdge> bestTo; //! The edge to which this is the best neighbor
};

typedef boost::shared_ptr<ScannedEdge> ScannedEdgePtr;

