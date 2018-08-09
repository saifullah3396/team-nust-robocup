/**
 * @file MotionModuleTests/MotionModuleTests.h
 *
 * This file declares the class MotionModuleTests
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 01 Feb 2017
 */

#pragma once

#include <iostream>
#include <string.h>
#include <assert.h>
#include <boost/shared_ptr.hpp>
#include <alcommon/almodule.h>
#include <alcommon/albroker.h>
#include <alcommon/albrokermanager.h>
#include <alcommon/alproxy.h>
#include <alproxies/almemoryproxy.h>
#include <alproxies/almotionproxy.h>
#include <alproxies/alvideodeviceproxy.h>
#include <alproxies/dcmproxy.h>
#include "TNRSBase/include/SharedMemory.h"
#include "TNRSBase/include/BaseModuleHandler.h"
#include "Utils/include/DebugUtils.h"

/**
 * These constants are defined to remove the bug that causes the error
 * "cannot allocate memory in static TLS block." while loading the module
 * on the robot. For details on this bug, see:
 * https://sourceware.org/bugzilla/show_bug.cgi?format=multiple&id=14898
 */
#ifndef NO_TLS_OFFSET
# define NO_TLS_OFFSET  0
#endif
#ifndef FORCED_DYNAMIC_TLS_OFFSET
# if NO_TLS_OFFSET == 0
#  define FORCED_DYNAMIC_TLS_OFFSET -1
# elif NO_TLS_OFFSET == -1
#  define FORCED_DYNAMIC_TLS_OFFSET -2
# else
#  error "FORCED_DYNAMIC_TLS_OFFSET is not defined"
# endif
#endif

extern string ROBOT_NAME;

class SharedMemory;

typedef boost::shared_ptr<AL::ALBroker> ALBrokerPtr;
typedef boost::shared_ptr<AL::ALMemoryProxy> ALMemoryProxyPtr;
typedef boost::shared_ptr<AL::ALMotionProxy> ALMotionProxyPtr;
typedef boost::shared_ptr<AL::DCMProxy> ALDCMProxyPtr;

/**
 * @class MotionModuleTests
 * @brief A class for testing MotionModule.
 */
class MotionModuleTests : public AL::ALModule, public BaseModuleHandler
{
public:
  /**
   * Constructor
   *
   * @param parentBroker: a broker for establishing connection with naoqi
   *   architecture
   * @param moduleName: name of the module with which it is loaded into
   *   naoqi
   */
  MotionModuleTests(ALBrokerPtr parentBroker, const string& moduleName);

  /**
   * Destructor
   */
  ~MotionModuleTests()
  {
  }

  /**
   * Initializes the class along with its child modules
   *
   * @returns void
   */
  void
  init();

  /**
   * Returns the pointer to SharedMemory object
   *
   * @return SharedMemoryPtr
   */
  SharedMemoryPtr
  getLocalSharedMemory()
  {
    return sharedMemory;
  }

private:
  //! Pointer to local data shared memory object
  SharedMemoryPtr sharedMemory;

  //! Pointer to NaoQi internal device communication manager (DCM)
  ALDCMProxyPtr dcmProxy;

  //! Pointer to NaoQi internal memory
  ALMemoryProxyPtr memoryProxy;

  //! Pointer to NaoQi internal motion class
  ALMotionProxyPtr motionProxy;
};
