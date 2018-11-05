/**
 * @file MotionModule/ZmpRefGenerator/ZmpRefGenerator.h
 *
 * This file declares the class ZmpRefGenerator
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date Jul 22 2018
 */

#pragma once

#include "MotionModule/include/MotionModule.h"
#include "MotionModule/include/BalanceModule/ZmpRef.h"
#include "MotionModule/include/KinematicsModule/KinematicsModule.h"

/**
 * @class ZmpRefGenerator
 * @brief The base class for zmp reference generators
 */
template <typename Scalar>
class ZmpRefGenerator
{
public:
  /**
   * Constructor
   * 
   * @param motionModule: pointer to parent MotionModule.
   */
  ZmpRefGenerator(
    MotionModule* motionModule, 
    const unsigned& refFrame,
    const unsigned& nReferences) :
    kM(motionModule->getKinematicsModule()), 
    refFrame(refFrame),
    nReferences(nReferences)
  {
    cycleTime = kM->getCycleTime();
    zmpRef.setLength(nReferences); 
  }

  /**
   * Destructor
   */
  virtual ~ZmpRefGenerator()
  {
  }

  /**
   * Initiates the zmp reference generator
   */ 
  virtual void initiate() = 0;
  
  /**
   * Finds the current zmp reference and updates the output
   * 
   * @param outZmpRef Output reference to be updated
   */ 
  virtual void update() = 0;

  ZmpRef<Scalar>& getCurrentRef() { return zmpRef; }
  
  unsigned& getRefFrame() { return refFrame; }

protected:
  //! Kinematics module object.
  boost::shared_ptr<KinematicsModule<Scalar> > kM;
  
  //! References container
  ZmpRef<Scalar> zmpRef;
  
  //! The time step for each reference generated.
  Scalar cycleTime;

  //! Number of references required
  unsigned nReferences;

  //! Base frame of reference
  // To be defined in kinematics class. Currently its CHAIN_L_LEG or CHAIN_R_LEG
  unsigned refFrame; 
  
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

typedef boost::shared_ptr<ZmpRefGenerator<MType> > ZmpRefGeneratorPtr;
