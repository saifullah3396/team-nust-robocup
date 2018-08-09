/**
 * @file Utils/include/BehaviorInfo.h
 *
 * This file declares the base struct for the configuration of all the
 * behaviors.
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 10 Sep 2017
 */

#pragma once

#include <boost/shared_ptr.hpp>
#include "BehaviorManager/include/BehaviorConfig.h"
#include "Utils/include/Exceptions/TNRSException.h"
#include "Utils/include/DataUtils.h"

using namespace Utils;

/**
 * @struct BehaviorInfo
 * @brief A struct that holds information about the behavior
 */
struct BehaviorInfo
{
  BehaviorInfo() :
    initiated(false), running(false)
  {
  }
  
  virtual ~BehaviorInfo()
  {
  }
  
  const bool& isInitiated() const
  {
    return initiated;
  }
  
  const bool& isRunning() const
  {
    return running;
  }
  
  const BehaviorConfigPtr& getConfig() const
  {
    return config;
  }

private:
  void setConfig(const BehaviorConfigPtr& config) {
    this->config = config;
  }
  
  void resetConfig() {
    config.reset();
  }

  void setInitiated(const bool& initiated) {
    this->initiated = initiated;
  }

  void setRunning(const bool& running) {
    this->running = running;
  }

  bool initiated;
  bool running;
  
  BehaviorConfigPtr config;
  
  //! Behavior manager can access its private members
  friend class BehaviorManager;
};

typedef boost::shared_ptr<BehaviorInfo> BehaviorInfoPtr;
