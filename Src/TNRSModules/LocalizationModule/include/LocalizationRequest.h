/**
 * @file LocalizationModule/include/LocalizationRequest.h
 *
 * This file defines the class LocalizationRequest
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 04 Feb 2017
 */

#pragma once

#include <opencv2/core/core.hpp>
#include "TeamNUSTSPL/include/TNSPLModuleIds.h"
#include "TNRSBase/include/ModuleRequest.h"
#include "Utils/include/SwitchRequest.h"
#include "Utils/include/PositionInput.h"
#include "Utils/include/Landmark.h"

/**
 * Types of request valid for LocalizationModule
 * 
 * @enum LocalizationRequestIds
 */ 
enum class LocalizationRequestIds {
  SWITCH_LOCALIZATION,
  SWITCH_BALL_OBSTACLE,
  RESET_LOCALIZER,
  POSITION_UPDATE,
  KNOWN_LANDMARKS_UPDATE,
  UNKNOWN_LANDMARKS_UPDATE
};

/**
 * @class LocalizationRequest
 * @brief A module request that can be handled by LocalizationModule
 */ 
class LocalizationRequest : public ModuleRequest 
{
public:
  /**
   * Constructor
   * 
   * @param id: Id of the control request
   */ 
  LocalizationRequest(const LocalizationRequestIds& id) :
    ModuleRequest((unsigned)TNSPLModules::LOCALIZATION, (unsigned)id)
  {
  }
};
typedef boost::shared_ptr<LocalizationRequest> LocalizationRequestPtr;

/**
 * @class SwitchLocalization
 * @brief A request to switch on and off the localization module
 */ 
struct SwitchLocalization : public LocalizationRequest, SwitchRequest
{
  /**
   * Constructor
   */ 
  SwitchLocalization(const bool& state) :
    LocalizationRequest(LocalizationRequestIds::SWITCH_LOCALIZATION),
    SwitchRequest(state)
  {
  }
};
typedef boost::shared_ptr<SwitchLocalization> SwitchLocalizationPtr;

/**
 * @class SwitchBallObstacle
 * @brief A request to switch on and off the ball to be used as obstacle
 */ 
struct SwitchBallObstacle : public LocalizationRequest, SwitchRequest
{
  /**
   * Constructor
   */ 
  SwitchBallObstacle(const bool& state) :
    LocalizationRequest(LocalizationRequestIds::SWITCH_BALL_OBSTACLE),
    SwitchRequest(state)
  {
  }
};
typedef boost::shared_ptr<SwitchBallObstacle> SwitchBallObstaclePtr;

/**
 * @class ResetLocalizer
 * @brief A request to reset localizer of the robot
 */ 
struct ResetLocalizer : public LocalizationRequest
{
  /**
   * Constructor
   */ 
  ResetLocalizer() :
    LocalizationRequest(LocalizationRequestIds::RESET_LOCALIZER)
  {
  }
};
typedef boost::shared_ptr<ResetLocalizer> ResetLocalizerPtr;

/**
 * @class PositionUpdate
 * @brief A request to update the position of the robot
 */ 
struct PositionUpdate : public LocalizationRequest
{
  /**
   * Constructor
   */ 
  PositionUpdate(const PositionInput<float>& input) :
    LocalizationRequest(LocalizationRequestIds::POSITION_UPDATE),
    input(input)
  {
  }
  
  PositionInput<float> input;
};

typedef boost::shared_ptr<PositionUpdate> PositionUpdatePtr;

/**
 * @class KnownLandmarksUpdate
 * @brief A request to update the observed known landmarks for the localizer
 */ 
struct KnownLandmarksUpdate : public LocalizationRequest
{
  /**
   * Constructor
   */ 
  KnownLandmarksUpdate(const vector<KnownLandmarkPtr>& landmarks) :
    LocalizationRequest(LocalizationRequestIds::KNOWN_LANDMARKS_UPDATE),
    landmarks(landmarks)
  {
  }
  
  vector<KnownLandmarkPtr> landmarks;
};
typedef boost::shared_ptr<KnownLandmarksUpdate> KnownLandmarksUpdatePtr;

/**
 * @class UnknownLandmarksUpdate
 * @brief A request to update the observed unknown landmarks for the localizer
 */ 
struct UnknownLandmarksUpdate : public LocalizationRequest
{
  /**
   * Constructor
   */ 
  UnknownLandmarksUpdate(const vector<UnknownLandmarkPtr>& landmarks) :
    LocalizationRequest(LocalizationRequestIds::UNKNOWN_LANDMARKS_UPDATE),
    landmarks(landmarks)
  {
  }
  
  vector<UnknownLandmarkPtr> landmarks;
};
typedef boost::shared_ptr<UnknownLandmarksUpdate> UnknownLandmarksUpdatePtr;
