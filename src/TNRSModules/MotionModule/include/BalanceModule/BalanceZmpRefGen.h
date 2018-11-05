/**
 * @file MotionModule/BalanceZmpRefGen/BalanceZmpRefGen.h
 *
 * This file declares the class BalanceZmpRefGen
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date Jul 22 2018
 */

#pragma once

#include "MotionModule/include/MotionModule.h"
#include "ZmpRefGenerator.h"

/**
 * @class BalanceZmpRefGen
 * @brief The class for generating simple zmp reference for shifting 
 *   robot balance to the required support leg
 */
template <typename Scalar>
class BalanceZmpRefGen : public ZmpRefGenerator<Scalar>
{
public:
  /**
   * Constructor
   * 
   * @param motionModule: pointer to parent MotionModule.
   */
  BalanceZmpRefGen(
    MotionModule* motionModule, 
    const unsigned& refFrame,
    const unsigned& nReferences,
    const Scalar& totalTime,
    const Matrix<Scalar, 2, 1>& targetZmp) : 
    ZmpRefGenerator<Scalar>(motionModule, refFrame, nReferences), 
    totalTime(totalTime),
    targetZmp(targetZmp),
    timeStep(0.f)
  {
    initZmpPosition.setZero();
  }

  /**
   * Destructor
   */
  virtual ~BalanceZmpRefGen()
  {
  }

  /**
   * @derived
   */ 
  void initiate() ;

  /**
   * @derived
   */ 
  void update();

private:
  Scalar timeStep;
  Scalar totalTime;
  Matrix<Scalar, 2, 1> targetZmp;
  Matrix<Scalar, 2, 1> initZmpPosition;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

typedef boost::shared_ptr<BalanceZmpRefGen<MType> > BalanceZmpRefGenPtr;
