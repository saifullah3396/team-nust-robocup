/**
 * @file LocalizationModule/include/ParticleFilter.h
 *
 * The class for implementing particle filter for robot localization.
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @author TeamNust 2015
 * @date 17 Sep 2017
 */

#pragma once

#include <boost/circular_buffer.hpp>
#include <boost/make_shared.hpp>
#include <queue>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "LocalizationModule/include/LocalizationModule.h"
#include "LocalizationModule/include/LandmarkDefinitions.h"
#include "LocalizationModule/include/FieldLandmarkIds.h"
#include "Resources/RandomLib/include/Random.hpp"
#include "Resources/RandomLib/include/RandomSelect.hpp"
#include "Resources/RandomLib/include/NormalDistribution.hpp"
#include "Utils/include/HardwareIds.h"
#include "Utils/include/Landmark.h"
#include "Utils/include/MathsUtils.h"
#include "Utils/include/PositionInput.h"

using namespace RandomLib;

#define MAX_STATE_ITERATIONS 100

/**
 * @struct Particle
 * @brief The struct for defining a single particle of the filter.
 */
struct Particle
{
  RobotPose2D<float> state;
  double weight;
};

/**
 * @class ParticleFilter
 * @brief The class for implementing particle filter for robot
 *   localization.
 */
class ParticleFilter : public MemoryBase
{
public:

  /**
   * Default constructor for this class.
   *
   * @param lModule: Pointer to parent LocalizationModule.
   */
  ParticleFilter(LocalizationModule* lModule);
  /**
   * Defualt destructor for this class.
   */
  ~ParticleFilter()
  {
  }

  /**
   * Initializes the particles based on a normal distribution
   * around the initial state estimate for the given standard deviations
   *
   * @param state: Initial state estimate of the robot
   * @param stdDeviation: Standard deviations for the uncertainty in 
   *   state estimate
   */
  void
  init(const RobotPose2D<float>& state);

  /**
   * Initializes the particles based on a normal distribution
   * around the initial state estimates for the given standard deviations.
   *
   * @param states: Initial state estimates of the robot
   */
  void
  init(const vector<RobotPose2D<float> >& states);

  /**
   * Initializes the particles based on a normal distribution
   * around the state estimates.
   */
  void
  init();

  /**
   * Sets up the landmarks information for the filter
   */
  void
  setupLandmarks();

  /**
   * Sets up the voronoi map for landmark labels
   */
  void
  setupVoronoiMap();

  /**
   * Sets up the camera range vectors
   */
  void
  setupViewVectors();

  /**
   * Tries to determine the robots position according to known landmarks 
   * information. // Currently only goal posts information is used here
   */
  void
  getRobotStateEstimate();

  void
  estimateFromLandmarks();
  void
  estimateForSideLines();

  /**
   * Updates the particle filter
   */
  void
  update();

  /**
   * This function updates the particles according to the motion
   * model for a given velocity input and adds random gaussian noise.
   */
  void
  prediction();

  void
  addPredictionNoise();

  /**
   * This function updates the particles according to the motion
   * model for a given velocity input and adds random gaussian noise.
   *
   * @param velocityInput: Velocity input that updates a particle state 
   *   from time t to t+1.
   */
  void
  prediction(const VelocityInput<double>& vI);

  /**
   * Updates particle weights based on the likelihood of the observed 
   *   measurements.
   * @param obsLandmarks: Vector of observed landmarks
   */
  void
  updateWeights(const vector<LandmarkPtr>& obsLandmarks);

  void
  checkIfLocalized();

  void
  updateSideConfidence();

  void
  minMaxGroundDist();

  bool
  normalizeWeights(const bool& noData);

  /**
   * Resampling from the updated set of particles to form the new set
   * of particles.
   */
  void
  resample();

  /**
   * Returns whether particle filter is initiated yet or not.
   */
  const bool
  isInitiated() const
  {
    return initiated;
  }

  void
  reset();

  /**
   * Gets the current average state of the robot.
   *
   * @return RobotPose2D<float>
   */
  RobotPose2D<float>
  getFilteredState()
  {
    return avgFilteredState;
  }

  vector<Particle>
  getParticles()
  {
    return particles;
  }

  vector<boost::shared_ptr<Landmark> >
  getFieldLandmarks()
  {
    return fieldLandmarks;
  }

  void
  setupWithRandomSideLine();
  void
  setupWithTeamPositions();

  bool
  getLocalized()
  {
    return localized;
  }
  
  void addPositionInput(const PositionInput<float>& input) {
    this->positionInputs.push(input);
  }
  
  void setKnownLandmarks(const vector<KnownLandmarkPtr>& landmarks) {
    this->knownLandmarks = landmarks;
  }
  
  void setUnknownLandmarks(const vector<UnknownLandmarkPtr>& landmarks) {
    this->unknownLandmarks = landmarks;
  }

private:
  //! Whether the filter has been initiated with some initial estimate
  bool initiated;

  //! Whether the robot has been localized to a good extent
  bool localized;

  //! Total number of generated particles
  int nParticles;

  //! Vector containing all the particles
  vector<Particle> particles;

  //! The gaussian covariance constant parameter
  float gaussianConst;

  //! The exponential constant value for x-axis
  float expConstX;

  //! The exponential constant value for y-axis
  float expConstY;

  //! Sum of weights of all the particles
  double sumWeights;

  //! Maximum weight from all the particles
  double maxWeight;

  int lostCount;
  //int recheckCount;
  //bool inRecheck;

  //! Estimated poses of the robot found from known landmarks
  //! Currnetly only goal info is used for this estimate
  boost::circular_buffer<RobotPose2D<float> > estimatedStates;

  //! Vector of the actual field landmarks
  vector<LandmarkPtr> fieldLandmarks;

  //! Vector of the actual field landmarks, labelled according to their
  //! position using voronoi map of the environment
  vector<vector<LandmarkPtr> > labelledLandmarks;

  //! Number of landmarks in each type
  vector<unsigned> landmarkTypeCount;

  //! Starting index of each type of landmark
  vector<unsigned> landmarkTypeStarts;

  //! The unit vector that represents the minimum field of view angle
  //! from the lower camera frame and maximum field of view angle from
  //! the upper camera frame
  vector<Vector3f> unitVecY;

  //! The minimum distance from the camera view on the ground
  float minDistGround;

  //! The maximum distance from the camera view on the ground
  float maxDistGround;

  //! Random  library object
  Random random;

  //! Parameters for augmented monte-carlo localization method
  double wSlow, wFast;

  //! Current average state of the robot
  RobotPose2D<float> avgFilteredState;

  //! Last known half the robot is in: 0 us/1 opponents.
  unsigned lastKnownHalf;

  //! Filter prediction standard deviation when the robot is standing
  Vector3d predictionStd;

  //! Filter prediction standard deviation when the robot is in motion
  Vector3d motionStd;

  //! Filter measurement standard deviation
  Vector3d measurementStd;

  //! Standard deviation used for states generation during initialization
  Vector3d genStd;

  //! Cycle time of the filter loop
  float cycleTime;

  //! Queue containing all the prediction control inputs
  queue<PositionInput<float> > positionInputs;
  
  //! Queue containing latest known landmarks observation.
  vector<KnownLandmarkPtr> knownLandmarks;
  
  //! Queue containing latest unknown landmarks observation.
  vector<UnknownLandmarkPtr> unknownLandmarks;

  //! Id of the last received goal info
  int prevGoalInfoId;

  //! Number of labels found 
  int numLabels;

  //! A matrix containing the labelled voronoi image
  Mat voronoiLabels;

  //! Pointer to localization module object
  LocalizationModule* lModule;
};
