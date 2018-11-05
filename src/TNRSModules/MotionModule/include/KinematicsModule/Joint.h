/**
 * @file MotionModule/include/KinematicsModule/Joint.h
 *
 * This file defines the structs JointState and Joint
 * 
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date Jul 22 2018
 */

#pragma once

#include <boost/make_shared.hpp>
#include "Utils/include/MathsUtils.h"
#include "LinkInfo.h"
#include "JointStateType.h"
#include "MTypeHeader.h"

/**
 * @struct DHParams
 * @brief Definition of dh parameters for revolute joints
 */
template<typename Scalar>
struct DHParams
{
/**
   * Constructor
   * 
   * @param a: Link length
   * @param alpha: Link twist
   * @param d: Joint offset
   * @param theta: Joint angle offset
   */ 
  DHParams(
    const Scalar& a, 
    const Scalar& alpha,
    const Scalar& d,
    const Scalar& theta) : 
    a(a), 
    alpha(alpha), 
    d(d), 
    theta(theta)
  {
    calpha = cos(alpha);
    salpha = sin(alpha);
  }
  
  //! Link length
  Scalar a;
  
  //! Link twist
  Scalar alpha;
  
  //! Joint offset
  Scalar d;
  
  //! Joint angle offset
  Scalar theta;
  
  //! Cosine of alpha
  Scalar calpha;
  
  //! Sine of alpha
  Scalar salpha;
};
template struct DHParams<float>;
template struct DHParams<double>;

/**
 * @struct JointState
 * @brief Definition of a joint state
 */
template<typename Scalar>
struct JointState
{
  /**
   * Constructor
   */ 
  JointState() : 
    position(0.0), 
    prevPosition(0.0), 
    velocity(0.0), 
    prevVelocity(0.0),
    accel(0.0),
    trans(Matrix<Scalar, 4, 4>::Identity())
  {
  }

  //! Joint position
  Scalar position;
  
  //! Transformation matrix for this joint
  Matrix<Scalar, 4, 4> trans;
  
  //! Transformation matrix of the current joint in base frame of 
  //! the chain
  Matrix<Scalar, 4, 4> transInBase;
  
  //! Joint position in base;
  Matrix<Scalar, 3, 1> posInBase;
  
  //! A vector representing the joint rotation axis in base frame;
  Matrix<Scalar, 3, 1> zInBase;
  
  //! A vector representing the center of mass of this link in base frame;
  Matrix<Scalar, 3, 1> comInBase;
  
  //! Joint previous positions
  Scalar prevPosition;

  //! Joint velocity
  Scalar velocity;
  
  //! Joint previous velocities
  Scalar prevVelocity;
  
  //! Joint acceleration
  Scalar accel;
};
template struct JointState<float>;
template struct JointState<double>;

/**
 * @struct Joint
 * @brief Definition of a joint
 */
template<typename Scalar>
struct Joint
{
  /**
   * Constructor
   */ 
  Joint(
    const string& name,
    const Scalar& maxPosition,
    const Scalar& minPosition,
    const Scalar& maxVelocity,
    DHParams<Scalar>* dhParams) :
    name(name), 
    maxPosition(maxPosition), 
    minPosition(minPosition), 
    maxVelocity(maxVelocity),
    dhParams(dhParams)
  {
    for (size_t i = 0; i < (unsigned)JointStateType::COUNT; ++i) {
      states.push_back(boost::make_shared<JointState<Scalar> >());
      makeDHMatrix(states[i]->trans);
    }
  }
  
  /**
   * Destructor
   */ 
  ~Joint() {
    delete dhParams;
    dhParams = NULL;
  }
  
  /**
   * Sets up the link transformation matrix as a dh matrix with 
   * unchanging variables set up on start
   * 
   * @param mat: The matrix to be updated
   */
  void makeDHMatrix(Matrix<Scalar, 4, 4>& mat)
  {
    mat(0, 2) = 0;
    mat(0, 3) = dhParams->a;
    mat(1, 2) = -dhParams->salpha;
    mat(1, 3) = -dhParams->salpha * dhParams->d;
    mat(2, 2) = dhParams->calpha;
    mat(2, 3) = dhParams->calpha * dhParams->d;
  }
  
  /**
   * Updates the link transformation matrix for the given angle
   * 
   * @param mat: The matrix to be updated
   * @param theta: The given angle
   */
  void updateDHMatrix(
    Matrix<Scalar, 4, 4>& mat,
    const Scalar& theta)
  {
    auto ct = cos(theta);
    auto st = sin(theta);
    mat(0, 0) = ct;
    mat(0, 1) = -st;
    mat(1, 0) = st * dhParams->calpha;
    mat(1, 1) = ct * dhParams->calpha;
    mat(2, 0) = st * dhParams->salpha;
    mat(2, 1) = ct * dhParams->salpha;
  }

  const Matrix<Scalar, 4, 4>& computeLinkTrans(const JointStateType& type) {
    updateDHMatrix(
      states[(unsigned)type]->trans,
      states[(unsigned)type]->position + dhParams->theta
    );
    return states[(unsigned)type]->trans;
  }
  
  void setTransInBase(
    const Matrix<Scalar, 4, 4> T, 
    const JointStateType& type) 
  {
    states[(unsigned)type]->transInBase = T;
    states[(unsigned)type]->posInBase = T.template block<3, 1>(0, 3);
    states[(unsigned)type]->zInBase = T.template block<3, 1>(0, 2);
    states[(unsigned)type]->comInBase = (T * link->com).template block<3, 1>(0, 0);
  }

  //! Joint name
  string name;

  //! Upper limit of joint position
  Scalar maxPosition;
  
  //! Upper limit of joint position
  Scalar minPosition;
  
  //! Joint velocity limit
  Scalar maxVelocity;
  
  //! Dh parameters
  DHParams<Scalar>* dhParams;
  
  //! Joint state vectors
  vector<boost::shared_ptr<JointState<Scalar> > > states;
  
  //! Associated link
  boost::shared_ptr<LinkInfo<Scalar> > link;
};
template struct Joint<MType>;
