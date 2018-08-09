/**
 * @file MotionModule/BalanceZmpRefGen/BalanceZmpRefGen.h
 *
 * This file declares the class BalanceZmpRefGen
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date Jul 22 2018
 */

#pragma once

#include "ZmpRefGenerator.h"

/**
 * @class BalanceZmpRefGen
 * @brief The class for generating simple zmp reference for shifting 
 *   robot balance to the required support leg
 */
class BalanceZmpRefGen : public ZmpRefGenerator
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
    const float& totalTime,
    const Vector2f& targetZmp) : 
    ZmpRefGenerator(motionModule, refFrame, nReferences), 
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
  float timeStep;
  float totalTime;
  Vector2f targetZmp;
  Vector2f initZmpPosition;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

typedef boost::shared_ptr<BalanceZmpRefGen> BalanceZmpRefGenPtr;
