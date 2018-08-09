/**
 * @file DAQModule/DAQModule.cpp
 *
 * This file declares a class for data filteration. All the functions
 * and algorithms for implementing kalman or complementary filters for
 * sensors will be defined under this module.
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 30 Sep 2017
 */

#include "DAQModule.h"
#include "Filter.h"
#include "ImuDataFilter/ImuDataFilter.h"

DAQModule::DAQModule(void* processingModule) :
  BaseModule(processingModule, DAQ_MODULE, "DAQModule")
{
  initMemoryConn();
  onIterationComplete(true);
  filters.resize(NUM_FILTER_ClASSES);
  filters[IMU_DATA_FILTER] =
    boost::make_shared < ImuDataFilter<double> > (this);
  filters[IMU_DATA_FILTER]->initiate();
}

void
DAQModule::displayToLog(int priority, string str)
{
}

void
DAQModule::syncThreadRelatedConfig()
{
periodMinMS = IVAR(int, daqThreadPeriod);
}

void
DAQModule::mainRoutine()
{
  while (true) {
    setIterationStartTime();
    filters[IMU_DATA_FILTER]->update();
    onIterationComplete();
  }
}

void
DAQModule::initMemoryConn()
{
  blackBoard = ((ProcessingModule*) parent)->getBlackBoard();
  genericInputConnector = new InputConnector(
    this,
    moduleName + "InputConnector");
  genericOutputConnector = new OutputConnector(
    this,
    moduleName + "OutputConnector");
  genericInputConnector->initConnector();
  genericOutputConnector->initConnector();
}
