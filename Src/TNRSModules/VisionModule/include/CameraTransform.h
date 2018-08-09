/**
 * @file VisionModule/CameraTransform.h
 *
 * This file declares the class CameraTransform.
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 17 Oct 2017
 */

#pragma once

#include "TNRSBase/include/MemoryBase.h"
#include "VisionModule/include/VisionModule.h"
#include "VisionModule/include/CameraModule/CameraModule.h"
#include "Utils/include/Camera.h"
#include "Utils/include/HardwareIds.h"
#include "Utils/include/MathsUtils.h"
#include "Utils/include/VisionUtils.h"

/**
 * @class CameraTransform
 * @brief A class for performing image to world transformation
 */
class CameraTransform : public MemoryBase
{
public:

  /**
   * The default constructor for image processing class.
   */
  CameraTransform(VisionModule* visionModule) :
    MemoryBase(visionModule), camModule(visionModule->getCameraModule())
  {
    transformed = false;
    cams = camModule->getCameraPtrs();
    A.resize(NUM_CAMS);
    b.resize(NUM_CAMS);
    //scale.resize(NUM_CAMS, NAN);
    invExtMatrix.resize(NUM_CAMS);
    extMatrix.resize(NUM_CAMS);
    camMatrix.resize(NUM_CAMS);
    camMatrixCv.resize(NUM_CAMS);
    projMatrix.resize(NUM_CAMS);
    //invProjMatrix.resize(NUM_CAMS);
    computeCamMatrix();
    update();
  }

  /**
   * The default destructor for image processing class.
   */
  ~CameraTransform()
  {
  }

  /*Point3f compute3DOnPlaneFrom2D(const Point2f& imagePt) 
   {
   Point2f normalizedImagePt;
   normalizedImagePt.x = (imagePt.x - cams[camIndex]->centerOffX) / cams[camIndex]->focalX;
   normalizedImagePt.y = (imagePt.y - cams[camIndex]->centerOffY) / cams[camIndex]->focalY;

   float s = -plane[3] / (plane[0]*normalizedImagePt.x + plane[1]*normalizedImagePt.y + plane[2]);

   Point3f pt;
   pt.x = s*normalizedImagePt.x;
   pt.y = s*normalizedImagePt.y;
   pt.z = s;

   return pt;
   }
   
   Vector4f computePlaneEquation(const vector<Vector4f>& ps) 
   {
   //Vector p0_p1
   Vector3f p0_p1 = (ps[0] - ps[1]).block(0, 0, 3, 1);
   Vector3f p0_p2 = (ps[0] - ps[2]).block(0, 0, 3, 1);

   //Normal vector
   Vector3f n = p0_p1.cross(p0_p2);
   
   Vector4f coeffs;
   coeffs[0] = n[0];
   coeffs[1] = n[1];
   coeffs[2] = n[2];
   coeffs[3] = -(coeffs[0]*ps[0][0] + coeffs[1]*ps[0][1] + coeffs[2]*ps[0][2]);

   float norm =  sqrt(coeffs[0]*coeffs[0] + coeffs[1]*coeffs[1] + coeffs[2]*coeffs[2]);
   coeffs /= norm;
   return coeffs;
   }*/

  void
  update()
  {
    //cout << "Upper camera in foot: " << endl; 
    //cout << IVAR(Matrix4f, VisionModule::upperCamInFeet) << endl;
    //cout << "Lower camera in foot: " << endl; 
    //cout << IVAR(Matrix4f, VisionModule::lowerCamInFeet) << endl;
    for (int i = 0; i < NUM_CAMS; ++i) {
      Matrix4f T;
      if (i == TOP_CAM) T = IVAR(Matrix4f, VisionModule::upperCamInFeet);
      else T = IVAR(Matrix4f, VisionModule::lowerCamInFeet);
      //T = MathsUtils::getTInverse(T);

      invExtMatrix[i] = MathsUtils::getTInverse(T);
      extMatrix[i] = T.block(0, 0, 3, 4);
      b[i][0] = -T(0, 3);
      b[i][1] = -T(1, 3);
      b[i][2] = -T(2, 3);
      A[i](0, 0) = T(0, 0);
      A[i](0, 1) = T(0, 1);
      A[i](0, 2) = 1;
      A[i](1, 0) = T(1, 0);
      A[i](1, 1) = T(1, 1);
      A[i](1, 2) = 1;
      A[i](2, 0) = T(2, 0);
      A[i](2, 1) = T(2, 1);
      A[i](2, 2) = -1;
      projMatrix[i] = camMatrix[i] * extMatrix[i];
      //MatrixXf proj = projMatrix[i];
      //invProjMatrix[i] = MathsUtils::pseudoInverse(proj);
      /*vector<Vector4f> points;
       points.push_back(Vector4f(3.0, 0.0, 0.0, 1.0));
       points.push_back(Vector4f(2.0, 1.0, 0.0, 1.0));
       points.push_back(Vector4f(2.0, -1.0, 0.0, 1.0));
       for (int i = 0; i < points.size(); ++i) {
       points[i] = T * points[i];
       }
       plane = computePlaneEquation(points);*/
    }
    //cout << "upper cam: " << endl;
    Vector4f wP1(4.5f, 0.f, 0.f, 1.f);
    Vector4f wP2(0.f, -4.5f, 0.f, 1.f);
    //cout << "twp1: "<< extMatrix[0] * wP1 << endl;
    //cout << "twp2: "<< extMatrix[0] * wP2 << endl;
    wPoints.clear();
    imgPoints.clear();
    transformed = false;
  }

  /**
   * Converts the point in image coordinates on height Z to the
   * world coordinates. The static frame for the world coordinates
   * lies between the robot feet on the ground and so the world
   * coordinate Z = Constant value. The simplified equations to solve 
   * for world coordinates X, Y and Camera coordinate Z are found and used.
   *
   * @param camIndex: The camera for which transfomration is to be done.
   * @param worldPoint: Extracted X, Y in world frame.
   * @param imagePoint: Image coordinates x and y in image frame.
   * @param worldZ: Known Z coordinate in world frame.
   */
  void
  imageToWorld(int camIndex, Point2f& worldPoint, Point2f imagePoint,
    float worldZ = 0.f)
  {
    if (worldZ != 0.f) {
      A[camIndex](0, 2) =
        (cams[camIndex]->centerOffX - imagePoint.x) / cams[camIndex]->focalX;
      A[camIndex](1, 2) =
        (cams[camIndex]->centerOffY - imagePoint.y) / cams[camIndex]->focalY;
      Vector3f tempB;
      tempB[0] = b[camIndex][0] - extMatrix[camIndex](0, 2) * worldZ;
      tempB[1] = b[camIndex][1] - extMatrix[camIndex](1, 2) * worldZ;
      tempB[2] = b[camIndex][2] - extMatrix[camIndex](2, 2) * worldZ;
      Vector3f sol = A[camIndex].inverse() * tempB;
      worldPoint.x = sol[0];
      worldPoint.y = sol[1];
      wPoints.push_back(worldPoint);
    } else {
      if (!transformed) {
        imgPoints.push_back(imagePoint);
      } else {
        vector<Point2f> src, dst;
        src.push_back(imagePoint);
        perspectiveTransform(src, dst, pTransform);
        worldPoint = dst[0];
        return;
      }
      A[camIndex](0, 2) =
        (cams[camIndex]->centerOffX - imagePoint.x) / cams[camIndex]->focalX;
      A[camIndex](1, 2) =
        (cams[camIndex]->centerOffY - imagePoint.y) / cams[camIndex]->focalY;
      Vector3f tempB;
      tempB[0] = b[camIndex][0] - extMatrix[camIndex](0, 2) * worldZ;
      tempB[1] = b[camIndex][1] - extMatrix[camIndex](1, 2) * worldZ;
      tempB[2] = b[camIndex][2] - extMatrix[camIndex](2, 2) * worldZ;
      Vector3f sol = A[camIndex].inverse() * tempB;
      worldPoint.x = sol[0];
      worldPoint.y = sol[1];
      wPoints.push_back(worldPoint);
      if (imgPoints.size() == 4 && !transformed) {
        //cout << "worldPoints: " << worldPoints << endl;
        /*float minX = worldPoints[0].x;
         float minY = worldPoints[0].y;
         for (int i = 1; i < worldPoints.size(); ++i)
         {
         minX = min(worldPoints[i].x, minX);
         minY = min(worldPoints[i].y, minY);
         }
         //Point2f worldOffset = Point2f(minX, minY);
         for (int i = 0; i < worldPoints.size(); ++i) {
         worldPoints[i] = worldPoints[i] * 50;
         //float x = worldPoints[i].x;
         worldPoints[i].x += 125;
         worldPoints[i].y = 350 / 2 - worldPoints[i].y;
         }*/
        ///cout << "imgPoints: " << imgPoints << endl;
        ///cout << "wPoints: " << wPoints << endl;
        pTransform = findHomography(imgPoints, wPoints);
        //cout << "p: " << pTransform << endl;
        transformed = true;
        //warpPerspective(inImage,  transformed, H,  transformed.size());
        //pTransform = getPerspectiveTransform(imgPoints, worldPoints);
      }
    }
  }

  /**
   * Converts a vector of points in image coordinates on height Z to the
   * world coordinates. The static frame for the world coordinates
   * lies between the robot feet on the ground and so the world
   * coordinate Z = Constant value. The simplified equations to solve 
   * for world coordinates X, Y and Camera coordinate Z are found and used.
   *
   * @param camIndex: The camera for which transfomration is to be done.
   * @param worldPoints: Extracted X, Y in world frame.
   * @param imagePoints: Image coordinates x and y in image frame.
   * @param worldZ: Known Z coordinate in world frame.
   */
  void
  imageToWorld(const int& camIndex, vector<Point2f>& worldPoints,
    vector<Point2f> imagePoints, float worldZ = 0.f)
  {
    worldPoints.resize(imagePoints.size());
    undistortPoints(
      imagePoints,
      imagePoints,
      camMatrixCv[camIndex],
      cams[camIndex]->distCoeffs,
      noArray(),
      camMatrixCv[camIndex]);
    if (worldZ != 0.f) {
      for (int i = 0; i < imagePoints.size(); ++i) {
        imageToWorld(camIndex, worldPoints[i], imagePoints[i], worldZ);
      }
    } else {
      if (transformed) {
        perspectiveTransform(imagePoints, worldPoints, pTransform);
      } else {
        for (int i = 0; i < imagePoints.size(); ++i) {
          imageToWorld(camIndex, worldPoints[i], imagePoints[i], worldZ);
        }
        perspectiveTransform(imgPoints, worldPoints, pTransform);
      }
    }
  }

  /**
   * Converts the point in world coordinates to the image coordinates.
   * 
   * @param camIndex: The camera for which transfomration is to be done.
   * @param worldPoint: Extracted X, Y, Z in world frame.
   * @param imagePoint: Image coordinates x and y in image frame.
   */
  void
  worldToImage(const int& camIndex, const Point3f& worldPoint,
    Point2f& imagePoint)
  {
    Vector4f wP(worldPoint.x, worldPoint.y, worldPoint.z, 1.f);
    Vector3f iP = projMatrix[camIndex] * wP;
    //cout << "camMatrix[camIndex]: " << endl << camMatrix[camIndex] << endl;
    //cout << "cams[camIndex]->width : " << endl << cams[camIndex]->width << endl;
    //cout << "cams[camIndex]->height : " << endl << cams[camIndex]->height << endl;
    imagePoint.x = iP[0] / iP[2];
    imagePoint.y = iP[1] / iP[2];
    //imagePoint.x = imagePoint.x + cams[camIndex]->width / 2;
    //imagePoint.y = imagePoint.y + cams[camIndex]->height / 2;
    //cout << "worldToImageXY: " << imagePoint << endl;
    //cout << "worldToImageScale: " << iP[2] << endl;
  }

  /**
   * Converts the points in world coordinates to the image coordinates.
   * 
   * @param camIndex: The camera for which transfomration is to be done.
   * @param worldPoints: Extracted points in world frame.
   * @param imagePoints: Image coordinates x and y in image frame.
   */
  void
  worldToImage(const int& camIndex, const vector<Point3f>& worldPoints,
    vector<Point2f>& imagePoints)
  {
    imagePoints.resize(worldPoints.size());
    for (int i = 0; i < worldPoints.size(); ++i) {
      Vector4f wP(worldPoints[i].x, worldPoints[i].y, worldPoints[i].z, 1.f);
      Vector3f iP = projMatrix[camIndex] * wP;
      //cout << "camMatrix[camIndex]: " << endl << camMatrix[camIndex] << endl;
      //cout << "cams[camIndex]->width : " << endl << cams[camIndex]->width << endl;
      //cout << "cams[camIndex]->height : " << endl << cams[camIndex]->height << endl;
      if (iP[2] > 0) {
        imagePoints[i].x = iP[0] / iP[2];
        imagePoints[i].y = iP[1] / iP[2];
      } else {
        imagePoints[i].x = 1000;
        imagePoints[i].y = 1000;
      }
      //cout << "scale: " << iP[2] << endl;
      //imagePoint.x = imagePoint.x + cams[camIndex]->width / 2;
      //imagePoint.y = imagePoint.y + cams[camIndex]->height / 2;
      //cout << "worldToImageXY: " << imagePoint << endl;
      //cout << "worldToImageScale: " << iP[2] << endl;
    }
    //cout << "imagepoints: " << imagePoints << endl;
    /*Mat R = Mat(3, 3, CV_64F);
     Mat t = Mat(3, 1, CV_64F);
     R.at<double>(0, 0) = extMatrix[camIndex](0, 0);
     R.at<double>(0, 1) = extMatrix[camIndex](0, 1);
     R.at<double>(0, 2) = extMatrix[camIndex](0, 2);
     R.at<double>(1, 0) = extMatrix[camIndex](1, 0);
     R.at<double>(1, 1) = extMatrix[camIndex](1, 1);
     R.at<double>(1, 2) = extMatrix[camIndex](1, 2);
     R.at<double>(2, 0) = extMatrix[camIndex](2, 0);
     R.at<double>(2, 1) = extMatrix[camIndex](2, 1);
     R.at<double>(2, 2) = extMatrix[camIndex](2, 2);
     
     t.at<double>(0, 0) = extMatrix[camIndex](0, 3);
     t.at<double>(1, 0) = extMatrix[camIndex](1, 3);
     t.at<double>(2, 0) = extMatrix[camIndex](2, 3);*/

    //cout << "eigen: " << extMatrix[camIndex] << endl;
    //cout << "R: " << R << endl;
    //cout << "t: " << t << endl;
    //projectPoints(worldPoints, R, t, camMatrixCv[camIndex], cams[camIndex]->distCoeffs, imagePoints);
  }

  Mat&
  getCamMatrixCv(const unsigned& index)
  {
    return camMatrixCv[index];
  }

  void
  computeCamMatrix()
  {
    for (int i = 0; i < NUM_CAMS; ++i) {
      camMatrix[i](0, 0) = cams[i]->focalX;
      camMatrix[i](0, 1) = 0.f;
      camMatrix[i](0, 2) = cams[i]->centerOffX;
      camMatrix[i](1, 0) = 0.f;
      camMatrix[i](1, 1) = cams[i]->focalY;
      camMatrix[i](1, 2) = cams[i]->centerOffY;
      camMatrix[i](2, 0) = 0.f;
      camMatrix[i](2, 1) = 0.f;
      camMatrix[i](2, 2) = 1;
      eigen2cv(camMatrix[i], camMatrixCv[i]);
    }
  }

  Matrix<float, 3, 4>
  getExtMatrix(const unsigned& camIndex)
  {
    return extMatrix[camIndex];
  }

  Matrix4f
  getExtInvMatrix(const unsigned& camIndex)
  {
    return invExtMatrix[camIndex];
  }

  Mat
  getPTransform()
  {
    return pTransform;
  }

private:
  //! The matrix A, for finding the solution to the system of equations
  //! defined by camera, and perspective transformation relations. 
  //! Used as inv(A) * b
  vector<Matrix3f> A;

  //! The matrix b, for finding the solution to the system of equations
  //! defined by camera, and perspective transformation relations. 
  //! Used as inv(A) * b
  vector<Vector3f> b;

  //! The camera matrices vector.
  vector<Matrix3f> camMatrix;

  //! The camera matrices vector in opencv mat.
  vector<Mat> camMatrixCv;

  //! The extrinsic matrices vector
  vector<Matrix<float, 3, 4> > extMatrix;

  //! The inverse extrinsic matrices vector
  vector<Matrix4f> invExtMatrix;

  //! The inverse projection transformation matrices
  //vector<Matrix<float, 4, 3> > invProjMatrix;

  //! The forward projection transformation matrices
  vector<Matrix<float, 3, 4> > projMatrix;

  //! Vector of cams
  vector<CameraPtr> cams;

  //! CameraModule ptr
  CameraModulePtr camModule;

  //! Perspective transform matrix
  Mat pTransform;

  //! Known points in image
  vector<Point2f> imgPoints;

  //! Known points in world
  vector<Point2f> wPoints;

  //! 
  bool transformed;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

typedef boost::shared_ptr<CameraTransform> CameraTransformPtr;

