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
#include "MotionModule/include/KickModule/FootContours.h"
#include "MotionModule/include/KickModule/KickUtils.h"
#include "MotionModule/include/KickModule/MaxMomentumEEOpt.h"
#include "MotionModule/include/MotionBehavior.h"
#include "MotionModule/include/MotionConfigs/MBKickConfig.h"

/** 
 * @class KickModule
 * @brief The base class for defining different kinds of kick engines
 */
class KickModule : public MotionBehavior
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
    MotionBehavior(motionModule, config, name),
    kickTime(0.f)
  {
    kickLeg = 0;
    supportLeg = 0;
    endEffector.setIdentity();
    supportToKick.setIdentity();
    torsoToSupport.setIdentity();
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
  static boost::shared_ptr<KickModule> getType(
    MotionModule* motionModule, const BehaviorConfigPtr& cfg);
    
  /**
   * Derived from Behavior. Child type may or may not use the same 
   * behavior config as parent.
   */
  virtual void loadExternalConfig();
  
protected:
  /**
   * Defines the foot contour static constants
   */ 
  static void setContourConstants();

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
   * Sets the end-effector xy-coordinates
   * 
   * @param angle: Angle for the normal to contour. Ranges from -90 to 
   *   +90 degrees
   */
  virtual bool setEndEffectorXY(const float& angle);

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
  bool findBezierAtVecNormal(
	  Vector3f& contourPoint,
	  const Matrix<float, 4, 3>& contourMat,
	  const vector<Vector3f>& contourConsts, 
	  const Vector3f& vec);

  /**
   * Sets the end-effector zx coordinates
   * 
   * @param t: ZX bezier curve parameter [0,...,1] .
   */
  void
  setEndEffectorZX(const float& t);

  /** 
   * Plots the kick parameters in 2D using GnuPlot
   */
  virtual void setupGnuPlotConfig();
  
  /**
   * Saves the foot contours in logs 
   * 'FootContourLeft.txt' and 'FootContourRight.txt'
   * if not already present.
   */
  void logFootContours();
  
  /**
   * Plots the foot front surfaces in 3D
   */
  void plotFootSurfaces();
  
  /**
   * Makes the feet surface matrix
   */ 
  void makeFootSurfaces3D(
    const bool& leftFoot, fstream& log, fstream& zxLog, Mat& surfaceMat);
  
  //! Shift these two to Utils/include/PlotEnv.h and use PlotEnv for 
  //! further plotting operations
  void plotPoint(
    const float& x, const float& y, const string& title, const unsigned& ls);
  Gnuplot gp;
  
  //! Ball mass
  static float ballMass;

  //! Ball radius
  static float ballRadius;
  
  //! Current static friction coefficient
  static float sf;
  
  //! Current rolling friction coefficient
  static float rf;

  //! Ball Position in base leg frame
  Vector3f ballPosition;

  //! Ball to target direction vector
  Vector3f ballToTargetUnit;

  //! Angle from ball to target
  float targetAngle;

  //! Distance between the feet frames.
  float footSpacing;

  //! Kicking Leg 
  unsigned kickLeg;

  //! Support Leg 
  unsigned supportLeg;
  
  //! End-Effector frame wrt kick leg base
  Matrix4f endEffector;

  //! End-effector pose at impact wrt the support leg frame
  Matrix4f impactPose;	

  //! Transformation matrix from support leg frame to end effector frame
  Matrix4f supportToKick;
  
  //! Transformation matrix from torso to support leg frame
  Matrix4f torsoToSupport;

  //! Time step for updating kick execution.
  float kickTime;
  
  //! Time taken by the overall kick trajectory
  float totalTimeToKick;

  //! Left half bezier control points for the left foot. 
  //! Same as contour with angle > 0
  static Matrix<float, 4, 3> lLeftContour; 
  
  //! Right half bezier control points for the left foot. 
  //! Same as contour with angle < 0
  static Matrix<float, 4, 3> lRightContour; 
  
  //! Left half bezier control points for the right foot. 
  //! Same as contour with angle > 0
  static Matrix<float, 4, 3> rLeftContour; 
  
  //! Right half bezier control points for the right foot. 
  //! Same as contour with angle < 0
  static Matrix<float, 4, 3> rRightContour; 

  //! Constants used for finding vector normal for left foot.
  static vector<Vector3f> lLeftCurveConsts;
  static vector<Vector3f> lRightCurveConsts;
  
  //! Constants used for finding vector normal for right foot.
  static vector<Vector3f> rLeftCurveConsts;
  static vector<Vector3f> rRightCurveConsts;
  
  //! Maximum momentum optimizer is friend as it uses kick module
  //! functions and variables
  friend class MaxMomentumEEOpt;
private:
  /**
   * Returns the cast of config to MBKickConfigPtr
   */ 
  MBKickConfigPtr getBehaviorCast();

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

typedef boost::shared_ptr<KickModule> KickModulePtr;

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
//float
//calcVirtualMass(const VectorXf& jointAngles, const Vector3f& endEff,
//  const Vector3f& direction);

/**
 * Forward kinematics solution for legs.   
 */
//Vector3f
//forwardKinematics(const Vector3f& endEffector, const string& baseFrame,
//  const VectorXf& jointAngles, Matrix4f& endFrame);

/**
 * Forward kinematics solution for legs.   
 */
//Vector3f
//forwardKinematics(const Vector3f& endEffector, const string& baseFrame,
//  const VectorXf& jointAngles);

/**
 * Solves inverse kinematics solution for legs.
 */
//bool
//inverseKinematics(const string& state, const Vector3f& initialPosition,
//  const Vector3f& finalPosition, const Vector3f& endEff,
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
//Vector3f refPosition;

//! Reference desired velocity in current time step. This variable is
//! for real-time trajectory generation
//Vector3f refVelocity;

//! Reference desired acceleration in current time step. This variable
//! is for real-time trajectory generation
//Vector3f refAcceleration;
