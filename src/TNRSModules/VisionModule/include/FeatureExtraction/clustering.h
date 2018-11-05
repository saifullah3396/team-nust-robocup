#pragma once

#include <vector>
#include <opencv2/core/core.hpp>
#include "Utils/include/MathsUtils.h"

using namespace Utils;

class Cluster
{
  vector<Point3f> points;
  Point3f mean_p, variance_p, sum_p;
public:
  Cluster()
  {
  }
  Cluster(Point3f point)
  {
    points.push_back(point);
    mean_p = point;
    sum_p.x = mean_p.x * mean_p.x;
    sum_p.y = mean_p.y * mean_p.y;
  }
  const Point3f &
  getMean_p()
  {
    return mean_p;
  }
  const Point3f &
  getVariance_p()
  {
    return variance_p;
  }
  const vector<Point3f> &
  getPoints()
  {
    return points;
  }
  //returns true if point within threshold of mean point of cluster
  bool
  checkPoint(float threshold, Point3f point)
  {
//		cout<<"Th: "<<threshold<<" Point: "<<point<<" meanP: "<<mean_p<<" s: "<<points.size()<<endl;
    if (points.size() > 0) {
      if (norm(Point2f(point.x, point.y) - Point2f(mean_p.x, mean_p.y)) < threshold) {
        return true;
      }
    }
    return false;
  }

  //add point to cluster updating mean variance and sum
  void
  addPoint(Point3f point)
  {
    points.push_back(point);
    sum_p = sum_p + Point3f(
      point.x * point.x,
      point.y * point.y,
      point.z * point.z);
    float s = points.size();
    mean_p.x = (mean_p.x * (s - 1) + point.x);
    mean_p.y = (mean_p.y * (s - 1) + point.y);
    if (mean_p.z * point.z >= 0) {
      mean_p.z = mean_p.z * (s - 1) / s + point.z * 1 / s;
    } else if (mean_p.z > 0) {
      mean_p.z = mean_p.z * (s - 1) / s + (point.z + M_PI * 2) * 1 / s;
    } else {
      mean_p.z = (mean_p.z + M_PI * 2) * (s - 1) / s + (point.z) * 1 / s;
    }
    mean_p.z = MathsUtils::rangeToPi(mean_p.z);
    mean_p = Point3f(mean_p.x / s, mean_p.y / s, mean_p.z);
    variance_p.x = sqrt(sum_p.x / s - mean_p.x * mean_p.x);
    variance_p.y = sqrt(sum_p.y / s - mean_p.y * mean_p.y);
    variance_p.z = 0; //todo: Correct variance of angle of robot
  }
};

class Clustering
{
  vector<Cluster> clusters;
  int count;
  float threshold;
public:
  Clustering()
  {
    count = 0;
    threshold = 10;
  }
  vector<Cluster> &
  getClusters()
  {
    return clusters;
  }
  //threshold for adding new point to cluster. it is the distance from mean of cluster
  void
  setThreshold(float threshold)
  {
    this->threshold = threshold;
  }
  void
  addNewPoint(Point3f point)
  {
    vector<Cluster>::iterator beg = clusters.begin(), end = clusters.end();
    for (; beg != end; beg++) {
      if ((*beg).checkPoint(threshold, point)) {
        (*beg).addPoint(point);
        return;
      }
    }

    //if reach here means no addition in any cluster then make new cluster
    clusters.push_back(Cluster(point));
    count++;
  }

  void
  addNewPoint(Point point)
  {
    addNewPoint(Point3f((float) point.x, (float) point.y, 0.0f));
  }

  const int &
  getClusterCount()
  {
    return count;
  }
  void
  reset()
  {
    clusters.clear();
    count = 0;
  }
};
