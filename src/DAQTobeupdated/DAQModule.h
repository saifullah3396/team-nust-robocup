/**
 * @file DAQModule/DAQModule.h
 *
 * This file declares a class for data filteration. All the functions
 * and algorithms for implementing kalman or complementary filters for
 * sensors will be defined under this module.
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 30 Sep 2017
 */

#pragma once

#include "DAQModuleIds.h"
#include "ProcessingModule/ProcessingModule.h"

class Filter;

/**
 * @class DAQModule
 * @brief A class for sensors data filteration.
 */
class DAQModule : public BaseModule
{
  CREATE_INPUT_CONNECTOR(
    (int, daqThreadPeriod),
    (vector<float>, inertialSensors),
  )

  CREATE_OUTPUT_CONNECTOR(
    (vector<float>, imuDataFilterOutput),
  )
public:
  /**
   * Initializes the daq module with its thread.
   *
   * @param processingModule: Pointer to base class which is in
   *   this case the ProcessingModule.
   */
  DAQModule(void* processingModule);

  /**
   * Defualt destructor for this class.
   */
  ~DAQModule()
  {
  }

  void
  mainRoutine();
  void
  initMemoryConn();
  void
  displayToLog(int priority, string str);
  void
  syncThreadRelatedConfig();

  /**
   * Returns the pointer to the required filter class.
   *
   * @return Pointer to Filter class.
   */
  boost::shared_ptr<Filter>
  getFilterClass(const unsigned &index) const
  {
    return filters[index];
  }

private:
  //! Pointer to a vector of feature extraction classes.
  vector<boost::shared_ptr<Filter> > filters;
};
