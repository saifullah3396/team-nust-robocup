/**
 * @file MotionModule/include/KickModule/KickModule.h
 *
 * This file declares the class KickModule
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 15 May 2017  
 */

#pragma once

#include "MotionModule/include/PostureModule/PostureModule.h"
#include "MotionModule/include/KickModule/KickUtils.h"
#include "MotionModule/include/KickModule/MaxMomentumEEOpt.h"
#include "MotionModule/include/MotionBehavior.h"
#include "MotionModule/include/MotionConfigs/MBKickConfig.h"
#include "MotionModule/include/KinematicsModule/ComState.h"
#include "MotionModule/include/KinematicsModule/Joint.h"
#include "MotionModule/include/KinematicsModule/LinkChain.h"
#include "MotionModule/include/KinematicsModule/KinematicsConsts.h"
#include "Utils/include/ConfigMacros.h"
#include "Utils/include/BSpline.h"
#include "Utils/include/EnvConsts.h"

/** 
 * @class KickModule
 * @brief The base class for defining different kinds of kick engines
 */
template <typename Scalar>
class KickModule : public MotionBehavior<Scalar>
{
public:
  /**
   * Constructor
   *
   * @param motionModule: Pointer to base motion module
   * @param config: Configuration of the behavior
   * @param name: Name of the behavior
   */
  KickModule(
    MotionModule* motionModule,
    const BehaviorConfigPtr& config,
		const string& name = "Not assigned.") :
    MotionBehavior<Scalar>(motionModule, config, name),
    kickTimeStep(0.0)
  {
    kickLeg = 0;
    supportLeg = 0;
    endEffector.setIdentity();
    supportToKick.setIdentity();
    torsoToSupport.setIdentity();
    kickFailed = false;
  }
  
  /**
   * Destructor
   */
  ~KickModule()
  {
  }

  /**
   * Returns its own child based on the given type
   * 
   * @param motionModule: Pointer to base motion module
   * @param cfg: Config of the requested behavior
   * 
   * @return BehaviorConfigPtr
   */
  static boost::shared_ptr<KickModule<Scalar>  > getType(
    MotionModule* motionModule, const BehaviorConfigPtr& cfg);
    
  /**
   * Derived from Behavior. Child type may or may not use the same 
   * behavior config as parent.
   */
  virtual void loadExternalConfig();
  
protected:
  /**
   * Defines the static constants used for computing bezier based foot
   * contour perpendiculars
   */ 
  //static void setBezierContourConstants();

  /**
   * Sets up the posture module with the input config as a child
   */
  void setupPosture();
	
  /**
   * Sets the required transformation frames from support to end effector
   * frame and torso to support leg frame
   * 
   * @return false if transformations are not set correctly
   */
  virtual bool setTransformFrames() throw(BehaviorException);

  /**
   * Sets the kicking and support leg frames
   */
  virtual bool setKickSupportLegs();

  /**
   * Sets the end-effector xy-coordinates based on foot contour 
   * approximation
   * 
   * @param angle: Angle for the normal to contour. Ranges from -90 to 
   *   +90 degrees
   */
  bool setEndEffectorXY(const Scalar& angle);

  /**
   * Finds the point on the given bezier where vec becomes normal to it.
   * 
   * @param contourPoint: output point
   * @param contourMat: bezier curve matrix
   * @param contourConsts: bezier vector constants
   * @param vec: vector that is required to be the normal
   * 
   * @return true if a point is found
   */ 
  /*bool findBezierAtVecNormal(
	  Matrix<Scalar, 3, 1>& contourPoint,
	  const Matrix<Scalar, 4, 3>& contourMat,
	  const vector<Matrix<Scalar, 3, 1>>& contourConsts, 
	  const Matrix<Scalar, 3, 1>& vec);*/
    
  /**
   * Sets the end-effector zx coordinates based on foot contour 
   * approximation
   * 
   * @param t: ZX bezier curve parameter [0,...,1] .
   */
  void
  setEndEffectorZX(const Scalar& t);

  /**
   * Returns true if the kicking leg foot is colliding with support leg foot
   *
   * @return bool
   */
  bool checkFootCollision(
    const Matrix<Scalar, Dynamic, 1>& kickAngles);

  /**
   * Logs foot contours in their respective files
   */
  void logFootContours();


  /** 
   * Plots the kick parameters in 2D using GnuPlot
   */
  //virtual void setupGnuPlotConfig();
  
  /**
   * Saves the foot contours in logs 
   * 'FootContourLeft.txt' and 'FootContourRight.txt'
   * if not already present.
   */
  //void logFootContours();
  
  /**
   * Plots the foot front surfaces in 3D
   */
  //void plotFootSurfaces();
  
  /**
   * Makes the feet surface matrix
   */ 
  //void makeFootSurfaces3D(
  //  const bool& leftFoot, fstream& log, fstream& zxLog, Mat& surfaceMat);
  
  //! Shift these two to Utils/include/PlotEnv.h and use PlotEnv for 
  //! further plotting operations
  //void plotPoint(
  //  const Scalar& x, const Scalar& y, const string& title, const unsigned& ls);
  //Gnuplot gp;
  
  //! Whether the requested kick is not acheivable
  bool kickFailed;

  //! Ball mass
  static Scalar ballMass;

  //! Ball radius
  static Scalar ballRadius;
  
  //! Ball static friction coefficient
  static Scalar sf;
  
  //! Ball rolling friction coefficient
  static Scalar rf;
  
  //! Coefficient of damping for ball if damping equation is used
  Scalar coeffDamping;

  //! Ball Position in base leg frame
  Matrix<Scalar, 3, 1> ballPosition;

  //! Ball to target direction vector
  Matrix<Scalar, 3, 1> ballToTargetUnit;

  //! Angle from ball to target
  Scalar targetAngle;

  //! Distance between the feet frames.
  Scalar footSpacing;

  //! Kicking Leg 
  unsigned kickLeg;

  //! Support Leg 
  unsigned supportLeg;
  
  //! End-Effector frame wrt kick leg base
  Matrix<Scalar, 4, 4> endEffector;

  //! End-effector pose at impact wrt the support leg frame
  Matrix<Scalar, 4, 4> impactPose;	

  //! Transformation matrix from support leg frame to end effector frame
  Matrix<Scalar, 4, 4> supportToKick;
  
  //! Transformation matrix from torso to support leg frame
  Matrix<Scalar, 4, 4> torsoToSupport;

  //! Time step for updating kick execution.
  Scalar kickTimeStep;
  
  //! Time taken by the overall kick trajectory
  Scalar totalTimeToKick;

  //! BSpline defining the left foot contour 
  static BSpline<Scalar>* lFootContour;
  
  //! BSpline defining the right foot contour 
  static BSpline<Scalar>* rFootContour;
  
  //! Maximum momentum optimizer is friend as it uses kick module
  //! functions and variables
  friend class MaxMomentumEEOpt<Scalar>;
private:
  /**
   * Returns the cast of config to MBKickConfigPtr
   */ 
  MBKickConfigPtr getBehaviorCast();

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

typedef boost::shared_ptr<KickModule<MType> > KickModulePtr;

/**
 * Defines the kick trajectory based on virtual mass calculations.
 * 
 * @param jointPoses desired joint configurations
 * @param initialJointV joint velocities at start 
 * @param finalointV joint velocities at end
 * 
 * @return a spline trajectory for each joint with cycle time difference
 */
//MatrixXf
//vmKickPlanner(const MatrixXf& jointPoses, const MatrixXf& initialJointV,
//  const MatrixXf& finalJointV);

/**
 * Calculates the virtual mass for desired leg configuration 
 * (only considers five joints)
 */
//Scalar
//calcVirtualMass(const VectorXf& jointAngles, const Matrix<Scalar, 3, 1>& endEff,
//  const Matrix<Scalar, 3, 1>& direction);

/**
 * Forward kinematics solution for legs.   
 */
//Matrix<Scalar, 3, 1>
//forwardKinematics(const Matrix<Scalar, 3, 1>& endEffector, const string& baseFrame,
//  const VectorXf& jointAngles, Matrix<Scalar, 4, 4>& endFrame);

/**
 * Forward kinematics solution for legs.   
 */
//Matrix<Scalar, 3, 1>
//forwardKinematics(const Matrix<Scalar, 3, 1>& endEffector, const string& baseFrame,
//  const VectorXf& jointAngles);

/**
 * Solves inverse kinematics solution for legs.
 */
//bool
//inverseKinematics(const string& state, const Matrix<Scalar, 3, 1>& initialPosition,
//  const Matrix<Scalar, 3, 1>& finalPosition, const Matrix<Scalar, 3, 1>& endEff,
//  VectorXf& jointAngles);

/**
 * Sets up the initial kick parameters for KickTypes::VM_BASED.
 */
//bool
//setupVMBasedKick();
/**
 * Defines the kick trajectory based on virtual mass calculations.
 */
//void
//virtualMassKick();

//! Reference desired position in current time step. This variable is
//! for real-time trajectory generation
//Matrix<Scalar, 3, 1> refPosition;

//! Reference desired velocity in current time step. This variable is
//! for real-time trajectory generation
//Matrix<Scalar, 3, 1> refVelocity;

//! Reference desired acceleration in current time step. This variable
//! is for real-time trajectory generation
//Matrix<Scalar, 3, 1> refAcceleration;
