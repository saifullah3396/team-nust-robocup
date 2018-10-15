/**
 * @file Utils/include/VisionUtils.h
 *
 * This file defines the vision utilities class for assisting with vision
 * processing and debugging.
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 05 Feb 2017
 */

#ifndef _VISION_UTILS_H_
#define _VISION_UTILS_H_

#include "Eigen/Dense"
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/video/video.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/core/eigen.hpp>
#include <stack>
#include "Utils/include/DebugUtils.h"

using namespace cv;
using namespace Eigen;

namespace Utils
{

#define INF 10000
  /**
   * @class VisionsUtils
   * @brief This class is defined for handling various functions related to
   *   vision processing and debugging.
   */
  class VisionUtils
  {
  public:

    /**
     * @brief Default class constructor.
     */
    VisionUtils()
    {
    }

    /**
     * @brief Default class destructor.
     */
    ~VisionUtils()
    {
    }

    /**
     * @brief This function creates a window with the given name using
     *   openCv.
     * @param name: Name of the window.
     * @return void
     */
    static inline void
    createWindow(const string& name)
    {
      namedWindow(name, CV_WINDOW_NORMAL);
    }

    /**
     * @brief This function displays an image with the given name using
     *   openCv.
     * @param name: Name of the window.
     * @return void
     */
    static inline void
    displayImage(const Mat& image, const string& name, const float& sizeMult =
      0.75)
    {
      //namedWindow(name, WINDOW_NORMAL);
      //resizeWindow(name, image.cols * 0.75,  image.rows * 0.75);
      Mat tmp = image.clone();
      Mat dst;
      Size size(image.cols * sizeMult, image.rows * sizeMult);
      resize(tmp, dst, size);
      imshow(name, dst);
      waitKey(1);
    }

    /**
     * @brief This function can be used to add a trackbar for given
     *   variables.
     * @param trackName: Name of the track.
     * @param windowName: Name of the window.
     * @param value: The variable pointer to be changed by the trackbar.
     * @param maxValue: maximum allowed value of the track.
     * @return void
     */
    static inline void
    addTrackBar(const string& trackName, const string& windowName, int *value,
      const int& maxValue)
    {
      createTrackbar(trackName, windowName, value, maxValue, 0);
    }
    
    /**
     * @brief Draws a rotated rect on the image
     * @param imageIn: Input Image
     * @param rect: Rotated rect
     * @param color: Color
     * @return void
     */
    static inline void 
    drawRRect(
      Mat& image,
      const RotatedRect& rect,
      cv::Scalar color)
    {
      Point2f vertices[4];
      rect.points(vertices);
      for (size_t j = 0; j < 4; j++)
        line(image, vertices[j], vertices[(j + 1) % 4], color);
    }
    
    /**
     * @brief This function performs the color segmentation on the image
     *   using opencv function inRange().
     * @param imageIn: Input Image.
     * @param lower: The lower color threshold range.
     * @param upper: The upper color threshold range.
     * @param imageOut: Output Image.
     * @return void
     */
    static inline void
    applyThreshold(const Mat& imageIn, const vector<int>& lower,
      const vector<int>& upper, Mat& imageOut)
    {
      ASSERT(lower.size() == 3 && upper.size() == 3)
      inRange(
        imageIn,
        Scalar(lower[0], lower[1], lower[2]),
        Scalar(upper[0], upper[1], upper[2]),
        imageOut);
    }

    template<typename T>
      static inline double
      minContourDist(vector<Point_<T> > c1, vector<Point_<T> > c2)
      {
        double minDist = 10000000;
        for (size_t i = 0; i < c1.size(); ++i) {
          for (size_t j = 0; j < c2.size(); ++j) {
            double dist = norm(c1[i] - c2[j]);
            minDist = dist < minDist ? dist : minDist;
          }
        }
        return minDist;
      }

    static inline int
    clip(const double& d)
    {
      if (d < 0) return 0;
      if (d > 255) return 255;
    }

    /*vector<Point> fieldHull = fieldExt->getFieldHull();
     vector<Vec4i> fieldLines;
     float maxX = fieldHull[0].x;
     float minX = fieldHull[0].x;
     float maxY = fieldHull[0].y;
     float minY = fieldHull[0].y;
     for (int i = 1; i < fieldHull.size(); ++i) {
     maxX = fieldHull[i].x < maxX ? maxX : fieldHull[i].x;
     minX = fieldHull[i].x > minX ? minX : fieldHull[i].x;
     maxY = fieldHull[i].y < maxY ? maxY : fieldHull[i].y;
     minY = fieldHull[i].y > minY ? minY : fieldHull[i].y;
     }
     VisionUtils::drawPoint(Point(maxX, maxY), image);
     VisionUtils::drawPoint(Point(maxX, minY), image);
     VisionUtils::drawPoint(Point(minX, maxY), image);
     VisionUtils::drawPoint(Point(minX, minY), image);
     */

    /*for (int i = 0; i < contours.size(); i++) {
     if (contourArea(contours[i]) >= 200) {
     std::vector<std::vector<cv::Point> > tcontours;
     std::vector<std::vector<cv::Point> > hulls(1);
     std::vector<std::vector<int> > hullsI(1);
     tcontours.push_back(contours[i]);
     //cv::drawContours(image, tcontours, -1, cv::Scalar(0, 0, 255), 2);
     cv::convexHull(cv::Mat(tcontours[0]), hulls[0], false, true);
     cv::convexHull(cv::Mat(tcontours[0]), hullsI[0], false, true);
     //cv::drawContours(image, hulls, -1, cv::Scalar(0, 255, 0), 1);
     cv::RotatedRect rect = cv::minAreaRect(cv::Mat(tcontours[0]));
     std::vector<std::vector<cv::Vec4i> >convDef(contours.size());
     cv::convexityDefects(tcontours[0], hullsI[0], convDef[i]);
     for (int k = 0; k < hullsI[0].size(); k++) {
     for (int j = 0; j < convDef[i].size(); j++) {
     if (convDef[i][j][3] > 20 * 256) {
     int ind_0 = convDef[i][j][0]; //start point
     int ind_1 = convDef[i][j][1]; //end point
     int ind_2 = convDef[i][j][2]; //defect point
     cv::circle(image, contours[i][ind_0], 5, cv::Scalar(0, 0, 255), -1);
     cv::circle(image, contours[i][ind_1], 5, cv::Scalar(255, 0, 0), -1);
     cv::circle(image, contours[i][ind_2], 5, cv::Scalar(0, 255, 0), -1);
     cv::line(image, contours[i][ind_2], contours[i][ind_0], cv::Scalar(0, 255, 255), 1);
     cv::line(image, contours[i][ind_2], contours[i][ind_1], cv::Scalar(0, 255, 255), 1);
     } else {
     //  fContours.push_back(contours[i]);
     }
     }
     }
     }
     }
     */

    /*vector<vector<Point> > contours;
     vector<Vec4i> hierarchy;
     findContours(contourImage, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));
     Mat drawing = Mat::zeros(mm.size(), CV_8UC3);
     for (int i = 0; i < contours.size(); i++) {
     drawContours(contourImage, contours, i, Scalar(255, 255, 255), 2, 8, hierarchy, 0, Point());
     }*/
    /*Mat fieldMask = fieldExt->getFieldMask();
     Mat cannyField;
     Canny(fieldMask, cannyField, 0, 100, 3, true);
     vector<Vec4i> boundaryLines;
     HoughLinesP(cannyField, boundaryLines, 1, CV_PI / 180, hltSettings[0], hltSettings[1], hltSettings[2]);
     for (size_t i = 0; i < boundaryLines.size(); ++i) {
     Scalar color = Scalar(255, 255, 230);
     Vec4f fl = boundaryLines[i];
     line(image, Point(fl[0], fl[1]), Point(fl[2], fl[3]), color, 1);
     }
     vector<Vec4i> lines;//, filteredLines;
     HoughLinesP(contourImage, lines, 1, CV_PI / 180, hltSettings[0], hltSettings[1], hltSettings[2]);
     */
    //cout << "FittedLineSize: " << fittedLines.size() << endl;
    /*ellipsefit ef;
     if(highProbLines.size() == 0)
     {
     for (int i = 0; i < fContours.size(); i++) {
     if (fContours[i].size() < 5)
     continue;
     vector<Point> ellipse_contour = ef.get_ellipse(fContours[i]);
     vector<vector<Point> > contour_vector;
     contour_vector.push_back(ellipse_contour);
     drawContours(drawing, contour_vector, 0, Scalar(0,0,255), 1.5);
     }
     } else {
     vector<vector<Point> > contoursE;
     vector<Vec4i> hierarchyE;
     findContours(goodLinesRemoved, contoursE, hierarchyE, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));
     for (int i = 0; i < contoursE.size(); i++) {
     if (contoursE[i].size() < 5)
     continue;
     vector<Point> ellipse_contour = ef.get_ellipse(contoursE[i]);
     vector<vector<Point> > contour_vector;
     contour_vector.push_back(ellipse_contour);
     contour_vector.push_back(contoursE[i]);
     drawContours(drawing, contour_vector, 0, Scalar(0,0,255), 1.5);
     }
     }
     imshow("ellipse", drawing);*/
    /*if(highProbLines.size() == 0)
     {
     vector<RotatedRect> ellipses;
     for (int i = 0; i < fContours.size(); i++) {
     if (fContours[i].size() < 5)
     continue;
     RotatedRect e = fitEllipse(fContours[i]);
     ellipse(drawing, e, Scalar(0, 255, 0));
     }
     imshow("ellipse", drawing);
     } else {
     vector<vector<Point> > contoursE;
     vector<Vec4i> hierarchyE;
     findContours(goodLinesRemoved, contoursE, hierarchyE, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));
     drawContours(drawing, contoursE, 0, Scalar(0,0,255), 1.5);
     vector<RotatedRect> ellipses;
     for (int i = 0; i < contoursE.size(); i++) {
     if (contoursE[i].size() < 5)
     continue;
     RotatedRect e = fitEllipse(contoursE[i]);
     ellipse(drawing, e, Scalar(0, 255, 0));
     }
     imshow("ellipse", drawing);
     }
     */

    /**
     * @brief This function draws a point with given color on the image
     *   matrix.
     * @param point: Point to be drawn.
     * @param image: Image matrix.
     * @param color: color.
     * @return void
     */
    template<typename T>
      static inline void
      drawPoint(const Point_<T>& point, Mat& image, Scalar color)
      {
        if (image.type() != CV_8UC1) {
          circle(
            image,
            Point(static_cast<int>(point.x), static_cast<int>(point.y)),
            2,
            color,
            -1);
          circle(
            image,
            Point(static_cast<int>(point.x), static_cast<int>(point.y)),
            5,
            color,
            2);
        } else {
          circle(
            image,
            Point(static_cast<int>(point.x), static_cast<int>(point.y)),
            2,
            Scalar(255),
            -1);
          circle(
            image,
            Point(static_cast<int>(point.x), static_cast<int>(point.y)),
            5,
            Scalar(255),
            2);
        }
      }

    /**
     * @brief This function draws a point on the image matrix.
     * @param point: Point to be drawn.
     * @param image: Image matrix.
     * @return void
     */
    template<typename T>
      static inline void
      drawPoint(const Point_<T>& point, Mat& image)
      {
        if (image.type() != CV_8UC1) {
          circle(
            image,
            Point(static_cast<int>(point.x), static_cast<int>(point.y)),
            1,
            Scalar(255, 0, 0),
            -1);
          circle(
            image,
            Point(static_cast<int>(point.x), static_cast<int>(point.y)),
            2,
            Scalar(0, 255, 0),
            2);
        } else {
          circle(
            image,
            Point(static_cast<int>(point.x), static_cast<int>(point.y)),
            1,
            Scalar(255),
            -1);
          circle(
            image,
            Point(static_cast<int>(point.x), static_cast<int>(point.y)),
            2,
            Scalar(255),
            2);
        }
      }

    /**
     * @brief This function draws points on the image matrix.
     * @param points: Points to be drawn.
     * @param image: Image matrix.
     * @return void
     */
    template<typename T>
      static void
      drawPoints(const vector<Point_<T> >& points, Mat& image)
      {
        for (size_t i = 0; i < points.size(); ++i) {
          circle(
            image,
            Point(static_cast<int>(points[i].x), static_cast<int>(points[i].y)),
            1,
            Scalar(255, 0, 0),
            -1);
          circle(
            image,
            Point(static_cast<int>(points[i].x), static_cast<int>(points[i].y)),
            2,
            Scalar(0, 255, 0),
            2);
        }
      }

    /**
     * @brief This function compares the y-coordinate of two lines using
     *   first points.
     * @param l1: First line.
     * @param l2: Second line.
     * @return bool
     */
    template<typename T>
      static inline bool
      compareLinesY(const Vec<T, 4>& l1, const Vec<T, 4>& l2)
      {
        return l1[1] < l2[1];
      }

    /**
     * @brief This function compares the y-coordinate of two lines using
     *   second points.
     * @param l1: First line.
     * @param l2: Second line.
     * @return bool
     */
    template<typename T>
      static inline bool
      compareLinesY2(const Vec<T, 4>& l1, const Vec<T, 4>& l2)
      {
        return l1[3] < l2[3];
      }

    /**
     * @brief This function compares the x-coordinate of two lines using
     *   first points.
     * @param l1: First line.
     * @param l2: Second line.
     * @return bool
     */
    template<typename T>
      static inline bool
      compareLinesX(const Vec<T, 4>& l1, const Vec<T, 4>& l2)
      {
        return l1[0] < l2[0];
      }

    /**
     * @brief This function compares the x-coordinate of two lines using
     *   second points.
     * @param l1: First line.
     * @param l2: Second line.
     * @return bool
     */
    template<typename T>
      static inline bool
      compareLinesX2(const Vec<T, 4>& l1, const Vec<T, 4>& l2)
      {
        return l1[2] < l2[2];
      }

    /**
     * @brief The function checks if the two lines defined by OpenCv type
     *   Vec<float, 4> are approximately parallel by comparing slopes with
     *   a given threshold.
     * @param l1: First line.
     * @param l2: Second line.
     * @return bool
     */
    template<typename T>
      static inline bool
      linesParallel(const Vec<T, 4>& l1, const Vec<T, 4>& l2,
        const double& threshold)
      {
        auto slope1 = ((double) (l1[3] - l1[1])) / ((double) (l1[2] - l1[0]));
        auto slope2 = ((double) (l2[3] - l2[1])) / ((double) (l2[2] - l2[0]));
        if (abs(slope1 - slope2) < threshold) return true;
        else return false;
      }

    /**
     * @brief The function checks if the two lines are collinear using
     *   triangle area property or by using slope with y-intercept.
     * @param l1: First line.
     * @param l2: Second line.
     * @param method: If true uses slope otherwise uses triangle area
     *   method.
     * @param threshold: Threshold for slopes and yItercept or area.
     * @return bool
     */
    template<typename T>
      static inline bool
      linesCollinear(const Vec<T, 4>& l1, const Vec<T, 4>& l2,
        const float& thresholdTheta, const float& thresholdDist,
        const bool& method)
      {
        if (method) {
          auto angle1 = atan2((float) (l1[3] - l1[1]), (float) (l1[2] - l1[0]));
          auto angle2 = atan2((float) (l2[3] - l2[1]), (float) (l2[2] - l2[0]));
          if (abs(angle1 - angle2) < thresholdTheta) {
            float angle, mag, perp;
            angle = atan2(
              ((float) l2[3] - (float) l1[3]),
              ((float) l2[2] - (float) l1[2])) - (angle1 + angle2) / 2;
            mag = norm(Point(l2[2], l2[3]) - Point(l1[2], l1[3]));
            perp = abs(mag * sin(angle));
            if (perp < thresholdDist) {
              return true;
            } else {
              return false;
            }
          } else {
            return false;
          }
        } else {
          Matrix3i areaMat;
          areaMat << l1[0], l1[2], l2[0], l1[1], l1[3], l2[1], 1, 1, 1;
          auto area = 0.5 * abs(areaMat.determinant());
          if (area < 200) return true;
          else return false;
        }
      }

    /**
     * @brief Given three collinear points p, q, r, the function checks if
     * point q lies on line segment 'pr'
     * @param p: opencv Point with x and y.
     * @param q: opencv Point with x and y.
     * @param r: opencv Point with x and y.
     * @return bool
     */
    template<typename T>
      static inline bool
      onSegment(const Point_<T>& p, const Point_<T>& q, const Point_<T>& r)
      {
        if (q.x <= max(p.x, r.x) && q.x >= min(p.x, r.x) && q.y <= max(p.y, r.y) && q.y >= min(
          p.y,
          r.y)) return true;
        return false;
      }

    /**
     * @brief To find orientation of ordered triplet (p, q, r).
     * The function returns following values:
     * 0 --> p, q and r are colinear
     * 1 --> Clockwise
     * 2 --> Counterclockwise
     * @param p: opencv Point with x and y.
     * @param q: opencv Point with x and y.
     * @param r: opencv Point with x and y.
     * @return unsigned
     */
    template<typename T>
      static inline unsigned
      orientation(const Point_<T>& p, const Point_<T>& q, const Point_<T>& r)
      {
        auto val = (q.y - p.y) * (r.x - q.x) - (q.x - p.x) * (r.y - q.y);
        if (val == 0) return 0; //! collinear
        return (val > 0) ? 1 : 2; //! clock or counterclock wise
      }

    /**
     * @brief This functions finds if the two lines intersect.
     * @param p1: Point with x and y on first line.
     * @param q1: Point with x and y on first line.
     * @param p2: Point with x and y on second line.
     * @param q2: Point with x and y on second line.
     * @return bool
     */
    template<typename T>
      static inline bool
      doIntersect(const Vec<T, 4>& l1, const Vec<T, 4>& l2)
      {
        //! Find the four orientations needed for general and
        //! special cases
        auto p1 = Point_ < T > (l1[0], l1[1]);
        auto q1 = Point_ < T > (l1[2], l1[3]);
        auto p2 = Point_ < T > (l2[0], l2[1]);
        auto q2 = Point_ < T > (l2[2], l2[3]);
        auto o1 = orientation(p1, q1, p2);
        auto o2 = orientation(p1, q1, q2);
        auto o3 = orientation(p2, q2, p1);
        auto o4 = orientation(p2, q2, q1);
        //! General case
        if (o1 != o2 && o3 != o4) return true;
        //! Special Cases
        //! p1, q1 and p2 are colinear and p2 lies on segment p1q1
        if (o1 == 0 && onSegment(p1, p2, q1)) return true;
        //! p1, q1 and p2 are colinear and q2 lies on segment p1q1
        if (o2 == 0 && onSegment(p1, q2, q1)) return true;
        //! p2, q2 and p1 are colinear and p1 lies on segment p2q2
        if (o3 == 0 && onSegment(p2, p1, q2)) return true;
        //! p2, q2 and q1 are colinear and q1 lies on segment p2q2
        if (o4 == 0 && onSegment(p2, q1, q2)) return true;
        return false; //! Doesn't fall in any of the above cases
      }

    /**
     * @brief This function checks whether a point lies within a polygon.
     * @param polygon: The polygon defined by several points.
     * @param p: Point with x and y to be checked.
     * @return bool
     */
    /*template<typename T>
     bool isInside(const vector<Point_<T> >& polygon, const Point_<T>& p)
     {
     //! There must be at least 3 vertices in polygon
     if (polygon.size() < 3)  return false;
     //! Create a point for line segment from p to infinite
     auto extreme = Point_<T>(INF, p.y);
     //! Count intersections of the above line with sides of polygon
     auto count = 0, i = 0;
     do
     {
     auto next = (i+1)%n;
     //! Check if the line segment from 'p' to 'extreme' intersects
     //! with the line segment from 'polygon[i]' to 'polygon[next]'
     if (doIntersect(polygon[i], polygon[next], p, extreme))
     {
     //! If the point 'p' is colinear with line segment 'i-next',
     //! then check if it lies on segment. If it lies, return true,
     //! otherwise false
     if (orientation(polygon[i], p, polygon[next]) == 0)
     return onSegment(polygon[i], p, polygon[next]);
     count++;
     }
     i = next;
     } while (i != 0);
     //! Return true if count is odd, false otherwise
     return count&1;
     }*/

    /**
     * @brief This function finds the point of intersection between two lines.
     * @param l1: First line.
     * @param l2: Second line.
     * @return Point of intersection
     */
    template<typename T>
      static inline Point_<T>
      findIntersection(const Vec<T, 4>& l1, const Vec<T, 4>& l2)
      {
        auto p1 = Point_ < T > (l1[0], l1[1]);
        auto p2 = Point_ < T > (l1[2], l1[3]);
        auto p3 = Point_ < T > (l2[0], l2[1]);
        auto p4 = Point_ < T > (l2[2], l2[3]);
        auto p1p2 = p1 - p2;
        auto p3p4 = p3 - p4;
        auto det = p1p2.x * p3p4.y - p1p2.y * p3p4.x;
        auto c1 = p1.x * p2.y - p1.y * p2.x;
        auto c2 = p3.x * p4.y - p3.y * p4.x;
        if (det != 0.0) {
          //! det == 0 -> Lines are parallel
          auto pX = (c1 * p3p4.x - p1p2.x * c2) / det;
          auto pY = (c1 * p3p4.y - p1p2.y * c2) / det;
          return Point_ < T > (pX, pY);
        } else {
          return Point_ < T > (-100, -100);
        }
      }

    /**
     * @brief This function finds all the connected components in an image
     *   and updates the labelled image.
     * @param image: Input image matrix.
     * @param labels: Labelled image matrix.
     * @return Number of labels
     */
    static inline unsigned
    connectedComponents(const Mat1i& image, Mat1i& labels)
    {
      vector < vector<Point> > components;
      return connectedComponents(image, labels, components);
    }

    /**
     * @brief This function finds all the connected components in an image
     *   and updates the labelled image.
     * @param image: Input image matrix.
     * @param labels: Labelled image matrix.
     * @param components: Separate component contours.
     * @return Number of labels
     */
    static inline unsigned
    connectedComponents(const Mat1i& image, Mat1i& labels,
      vector<vector<Point> >& components)
    {
      ASSERT(!image.empty());
      Mat1i src = image;
      labels = Mat1i(image.rows, image.cols, 0);
      int label = 0;
      int w = src.cols;
      int h = src.rows;
      int i;
      Point point;
      for (int y = 0; y < h; ++y) {
        for (int x = 0; x < w; ++x) {
          if ((src(y, x)) > 0) // Seed found
          {
            stack<int, vector<int> > stack2;
            i = x + y * w;
            stack2.push(i);
            vector < Point > comp;
            while (!stack2.empty()) {
              i = stack2.top();
              stack2.pop();
              int x2 = i % w;
              int y2 = i / w;
              src(y2, x2) = 0;
              point.x = x2;
              point.y = y2;
              comp.push_back(point);
              //! 4 connected
              if (x2 > 0 && (src(y2, x2 - 1) != 0)) {
                stack2.push(i - 1);
                src(y2, x2 - 1) = 0;
              }
              if (y2 > 0 && (src(y2 - 1, x2) != 0)) {
                stack2.push(i - w);
                src(y2 - 1, x2) = 0;
              }
              if (y2 < h - 1 && (src(y2 + 1, x2) != 0)) {
                stack2.push(i + w);
                src(y2 + 1, x2) = 0;
              }
              if (x2 < w - 1 && (src(y2, x2 + 1) != 0)) {
                stack2.push(i + 1);
                src(y2, x2 + 1) = 0;
              }

              //! 8 connected
              if (x2 > 0 && y2 > 0 && (src(y2 - 1, x2 - 1) != 0)) {
                stack2.push(i - w - 1);
                src(y2 - 1, x2 - 1) = 0;
              }
              if (x2 > 0 && y2 < h - 1 && (src(y2 + 1, x2 - 1) != 0)) {
                stack2.push(i + w - 1);
                src(y2 + 1, x2 - 1) = 0;
              }
              if (x2 < w - 1 && y2 > 0 && (src(y2 - 1, x2 + 1) != 0)) {
                stack2.push(i - w + 1);
                src(y2 - 1, x2 + 1) = 0;
              }
              if (x2 < w - 1 && y2 < h - 1 && (src(y2 + 1, x2 + 1) != 0)) {
                stack2.push(i + w + 1);
                src(y2 + 1, x2 + 1) = 0;
              }
            }
            ++label;
            components.push_back(comp);
            for (int k = 0; k < comp.size(); ++k) {
              labels(comp[k]) = label;
            }
          }
        }
      }
      return label;
    }

    /*    static void getEllipseParam(double a,double b,double c,double d,double f,double g, Ellipse& ellipse){
     ellipse.x = (c * d - b * f)/(b * b - a * c);
     ellipse.y = (a * f - b * d)/(b * b - a * c);

     ellipse.majorAxis = sqrt( (2*(a*f*f+c*d*d+g*b*b-2*b*d*f-a*c*g))/((b*b-a*c)*(sqrt((a-c)*(a-c)+4*b*b)-(a+c))));
     ellipse.minorAxis = sqrt( (2*(a*f*f+c*d*d+g*b*b-2*b*d*f-a*c*g))/((b*b-a*c)*(sqrt((a-c)*(a-c)+4*b*b)+(a+c))));

     ellipse.angle=0;
     if(b == 0 && a < c){
     ellipse.angle = 0;
     }
     else if(b == 0 && a > c){
     ellipse.angle = 90;
     }
     else if(b != 0 && a < c){
     ellipse.angle = 0.5 * MathsUtils::aCotan( (a-c)/(2*b) ) * 180 / M_PI;
     }
     else if(b != 0 && a > c){
     ellipse.angle = 90 + 0.5 * MathsUtils::aCotan( (a-c)/(2*b) ) * 180 / M_PI;
     }
     if(ellipse.minorAxis > ellipse.majorAxis){
     double temp = ellipse.majorAxis;
     ellipse.majorAxis = ellipse.minorAxis;
     ellipse.minorAxis = temp;
     ellipse.angle += 90;
     }

     double temp_c;
     if(ellipse.majorAxis > ellipse.minorAxis)
     temp_c = sqrt(ellipse.majorAxis * ellipse.majorAxis - ellipse.minorAxis * ellipse.minorAxis);
     else
     temp_c = sqrt(ellipse.minorAxis * ellipse.minorAxis - ellipse.majorAxis * ellipse.majorAxis);
     ellipse.f1_x = ellipse.x - temp_c * cos(ellipse.angle*M_PI/180);
     ellipse.f1_y = ellipse.y - temp_c * sin(ellipse.angle*M_PI/180);
     ellipse.f2_x = ellipse.x + temp_c * cos(ellipse.angle*M_PI/180);
     ellipse.f2_y = ellipse.y + temp_c * sin(ellipse.angle*M_PI/180);
     }

     static bool pointInEllipse(Point point,Ellipse ellipse){
     double dist1 = sqrt((point.x - ellipse.f1_x) * (point.x - ellipse.f1_x) +
     (point.y - ellipse.f1_y) * (point.y - ellipse.f1_y));
     double dist2 = sqrt((point.x - ellipse.f2_x) * (point.x - ellipse.f2_x) +
     (point.y - ellipse.f2_y) * (point.y - ellipse.f2_y));
     double max;
     if(ellipse.majorAxis > ellipse.minorAxis)
     max = ellipse.majorAxis;
     else
     max = ellipse.minorAxis;
     if(dist1+dist2 <= 2*max)
     return true;
     else
     return false;
     }

     static Ellipse fitEllipseRANSAC(vector<Point> points,int &count){
     Ellipse ellipse;
     count=0;
     int index[5];
     bool match=false;
     for(int i=0;i<5;i++){
     do {
     match = false;
     index[i]=rand()%points.size();
     for(int j=0;j<i;j++){
     if(index[i] == index[j]){
     match=true;
     }
     }
     }
     while(match);
     }
     double aData[] = {
     points[index[0]].x * points[index[0]].x, 2 * points[index[0]].x * points[index[0]].y, points[index[0]].
     y * points[index[0]].y, 2 * points[index[0]].x, 2 * points[index[0]].y,

     points[index[1]].x * points[index[1]].x, 2 * points[index[1]].x * points[index[1]].y, points[index[1]].
     y * points[index[1]].y, 2 * points[index[1]].x, 2 * points[index[1]].y,

     points[index[2]].x * points[index[2]].x, 2 * points[index[2]].x * points[index[2]].y, points[index[2]].
     y * points[index[2]].y, 2 * points[index[2]].x, 2 * points[index[2]].y,

     points[index[3]].x * points[index[3]].x, 2 * points[index[3]].x * points[index[3]].y, points[index[3]].
     y * points[index[3]].y, 2 * points[index[3]].x, 2 * points[index[3]].y,

     points[index[4]].x * points[index[4]].x, 2 * points[index[4]].x * points[index[4]].y, points[index[4]].
     y * points[index[4]].y, 2 * points[index[4]].x, 2 * points[index[4]].y };
     Mat matA=Mat(5,5,CV_64F,aData);
     Mat *D,*U,*V;
     D=new Mat(5,5,CV_64F);
     U=new Mat(5,5,CV_64F);
     V=new Mat(5,5,CV_64F);

     SVD svd(matA);

     Mat matV(5, 5, CV_64F);
     matV = svd.vt;

     double a,b,c,d,f,g;
     a=matV.at<double>(0,4);
     b=matV.at<double>(1,4);
     c=matV.at<double>(2,4);
     d=matV.at<double>(3,4);
     f=matV.at<double>(4,4);
     g=1;

     getEllipseParam(a,b,c,d,f,g,ellipse);

     vector<Point>::iterator point_iter;
     if(ellipse.majorAxis > 0 && ellipse.minorAxis > 0){
     for(point_iter=points.begin();point_iter!=points.end();point_iter++){
     Point point = *point_iter;
     if(pointInEllipse(point,ellipse)){
     count++;
     }
     }
     }

     return ellipse;
     }*/
  };
}
#endif //! _VISION_UTILS_H_
