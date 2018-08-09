/**
 * @file Utils/include/RobotStateDefinitions.h
 *
 * This file declares the structs RobotPose2D, PositionInput, and VelocityInput.
 *
 * @author Team-Nust 2015
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 18 Feb 2017
 */

#pragma once

#include <Eigen/Dense>
#include <vector>

using namespace std;
using namespace Eigen;

/**
 * @struct RobotPose2D
 * @brief A struct that defines the pose of the robot in x-y-theta.
 */
template<typename Scalar>
struct RobotPose2D
{

  /**
   * @brief Empty constructor.
   */
  RobotPose2D()
  {
  }

  /**
   * @brief Constructor.
   * @param x X-coordinate of the pose.
   * @param y Y-coordinate of the pose.
   * @param theta Theta-coordinate of the pose.
   */
  RobotPose2D(const Scalar& x, const Scalar& y, const Scalar& theta) :
    x(x), y(y), theta(theta)
  {
  }

  RobotPose2D<Scalar>&
  operator+=(const RobotPose2D<Scalar>& other)
  {
    x += other.x;
    y += other.y;
    theta += other.theta;
    return (*this);
  }

  RobotPose2D<Scalar>&
  operator-=(const RobotPose2D<Scalar>& other)
  {
    x -= other.x;
    y -= other.y;
    theta -= other.theta;
    return (*this);
  }

  RobotPose2D<Scalar>&
  operator*=(const Scalar constant)
  {
    x *= constant;
    y *= constant;
    theta *= constant;
    return (*this);
  }

  const RobotPose2D<Scalar>
  operator+(const RobotPose2D<Scalar>& other) const
  {
    return RobotPose2D<Scalar>(*this) += other;
  }

  const RobotPose2D<Scalar>
  operator-(const RobotPose2D<Scalar>& other) const
  {
    return RobotPose2D<Scalar>(*this) -= other;
  }

  Scalar x;
  Scalar y;
  Scalar theta;
};
  
template struct RobotPose2D<float> ;
template struct RobotPose2D<double> ;

/**
 * @struct VelocityInput
 * @brief The struct for defintion of the velocity control input to the
 *   humanoid in 2D.
 */
template<typename Scalar>
  struct VelocityInput
  {

    /**
     * @brief Empty constructor.
     */
    VelocityInput()
    {
    }

    /**
     * @brief Constructor.
     * @param dX X-coordinate of the input velocity.
     * @param dY Y-coordinate of the input velocity.
     * @param dTheta Theta-coordinate of the input velocity.
     */
    VelocityInput(Scalar dX, Scalar dY, Scalar dTheta) :
      dX(dX), dY(dY), dTheta(dTheta)
    {
    }
    Scalar dX;
    Scalar dY;
    Scalar dTheta;
  };

template struct VelocityInput<float> ;
template struct VelocityInput<double> ;
