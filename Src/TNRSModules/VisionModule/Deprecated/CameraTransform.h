/**
 * @file VisionModule/CameraTransform.h
 *
 * This file declares the class CameraTransform.
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 17 Oct 2017
 */

#pragma once

#include "MemoryModule/MemoryBase.h"
#include "VisionModule/VisionModule.h"
#include "Utils/VisionUtils.h"

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

  void
  update()
  {
    for (int i = 0; i < NUM_CAMS; ++i) {
      Matrix4f T;
      if (i == TOP_CAM) T = IVAR(Matrix4f, VisionModule::upperCamInFeet);
      else T = IVAR(Matrix4f, VisionModule::lowerCamInFeet);
      T = MathsUtils::getTInverse(T);
      invExtMatrix[i] = T;
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
    }
    worldPoints.clear();
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
  imageToWorld(int camIndex, Point2f& worldPoint, Point imagePoint,
    float worldZ = 0.f)
  {
    if (!transformed) imgPoints.push_back(Point2f(imagePoint.x, imagePoint.y));
    if (transformed) {
      vector<Point2f> src, dst;
      src.push_back(Point2f(imagePoint.x, imagePoint.y));
      perspectiveTransform(imgPoints, dst, pTransform);
      worldPoint = dst[0];
      return;
    }
    //imagePoint.x = imagePoint.x - (int) cams[camIndex]->width / 2;
    //imagePoint.y = imagePoint.y - (int) cams[camIndex]->height / 2;
    //if (scale[camIndex] != scale[camIndex]) { // checking if it is NAN
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
    worldPoints.push_back(worldPoint);
    if (imgPoints.size() == 4) {
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
      //cout << "worldPoints: " << worldPoints << endl;
      //cout << "imgPoints: " << imgPoints<< endl;
      pTransform = findHomography(imgPoints, worldPoints);
      //cout << "p: " << pTransform << endl;
      transformed = true;
      //warpPerspective(inImage,  transformed, H,  transformed.size());
      //pTransform = getPerspectiveTransform(imgPoints, worldPoints);
    }
    //scale[camIndex] = sol[2];
    /*} else {
     Vector3f imageP;
     imageP[0] = scale[camIndex] * imagePoint.x;
     imageP[1] = scale[camIndex] * imagePoint.y;
     imageP[2] = scale[camIndex];
     Vector4f sol = invProjMatrix[camIndex] * imageP;
     worldPoint.x = sol[0];
     worldPoint.y = sol[1];
     }*/
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
  imageToWorld(int camIndex, vector<Point2f>& worldPoints,
    vector<Point> imagePoints, float worldZ = 0.f)
  {
    worldPoints.resize(imagePoints.size());
    for (int i = 0; i < imagePoints.size(); ++i) {
      imageToWorld(camIndex, worldPoints[i], imagePoints[i], worldZ);
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
  worldToImage(int camIndex, Point3f worldPoint, Point& imagePoint)
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
    }
  }

  Matrix4f
  getTInvMatrix(const unsigned& camIndex)
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
  vector<Point2f> worldPoints;

  //! 
  bool transformed;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

typedef boost::shared_ptr<CameraTransform> CameraTransformPtr;

