/**
 * @file Utils/include/MotionLogger.h
 *
 * This file defines the class MotionLogger
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 05 Feb 2017
 */

#pragma once

#include "Utils/include/JsonLogger.h"
#include <alvalue/alvalue.h>
#include <chrono>

using namespace std::chrono;

/**
 * @class MotionLogger
 * @brief Class that provides a json based motion data logger
 */
class MotionLogger : public Utils::JsonLogger
{
public:

  /**
   * @brief Class constructor.
   * 
   * @param path: Path to json file
   * @param root: Root of json object
   * @param reftime: Reference start time for motion data logs
   */
  MotionLogger(
    const string& path, 
    const Json::Value& root = Json::Value(),
    const bool& logJointCmds = false,
    const bool& logJointStates = false,
    const high_resolution_clock::time_point& refTime = 
      high_resolution_clock::now()) :
    Utils::JsonLogger(path, root),
    logJointCmds(logJointCmds), 
    logJointStates(logJointStates),
    refTime(refTime)
  {
  }

  /**
   * @brief Class destructor.
   */
  ~MotionLogger()
  {
  }
  
  /**
   * Records joint commands in the output file
   * 
   * @param time: Time vector
   * @param cmds: Matrix of joint commands for each time frame
   */ 
  void recordJointCmds(
    const AL::ALValue& time, const AL::ALValue& cmds, const vector<unsigned>& ids);
  
  /**
   * Records current joint state in the output file
   */ 
  template <typename Scalar>
  void recordJointStates(const vector<Scalar>& joints);
  
  //! Setters
  void setRefTime(const high_resolution_clock::time_point& refTime) 
    { this->refTime = refTime; }
  void setLogJointStates(const bool& logJointStates) 
    { this->logJointStates = logJointStates; }
  void setLogJointCmds(const bool& logJointCmds) 
    { this->logJointCmds = logJointCmds; }
    
private:
  //! Reference time from which all other readings are recorded
  high_resolution_clock::time_point refTime;
  
  //! Whether to log joint states
  bool logJointStates;
  
  //! Whether to log joint commands
  bool logJointCmds;
};
typedef boost::shared_ptr<MotionLogger> MotionLoggerPtr;
