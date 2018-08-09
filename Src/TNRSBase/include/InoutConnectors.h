/**
 * @file TNRSBase/include/InoutConnectors.h
 *
 * This file declares the macros for declaring the shared memory
 * connections for each class
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 03 Feb 2017
 */

#include "TNRSBase/include/ConnectorMacro.h"

#define CREATE_INPUT_CONNECTOR(InputEnum, ...) CREATE_INPUT_CONNECTOR_(InputEnum, __VA_ARGS__)
#define CREATE_OUTPUT_CONNECTOR(OutputEnum, ...) CREATE_OUTPUT_CONNECTOR_(OutputEnum, __VA_ARGS__)

/**
 * \def CREATE_INPUT_CONNECTOR_(...)
 *
 * Defines the input connector struct for the inialization
 * of input/output connection of a BaseModule with the shared memory
 *
 * @param ... paranthesis enclosed array of variables to be
 *   defined (type, name), (type, name), ... (typelast, namelast),
 */
#define CREATE_INPUT_CONNECTOR_(InputEnum, ...) \
  public: \
  /** \
   * Enumerator for all the input variables of the class \
   * written to shared memory \
   * \
   * @enum InputEnum \
   */ \
  DEFINE_ENUM(InputEnum, 0, SEPARATE(__VA_ARGS__), numInput); \
  struct InputConnector : public InputMemoryConnector \
  { \
    /** \
     * Initializes the thread connection for shared memory read \
     * access \
     * \
     * @param connectingThread pointer to BaseModule \
     * @param connectorName name of the module \
     */ \
    InputConnector( \
      BaseModule* connectingThread, string connectorName) : \
      InputMemoryConnector(connectingThread, connectorName) \
    {} \
    \
    /** \
    * Derived from MemoryConnector \
    */ \
    void initConnector() \
    { \
      SharedMemoryPtr sharedMemory = \
        connectingThread->getLocalSharedMemory(); \
      INIT_INPUT; \
      FOR_EACH(DECLARE_IVARS_, __VA_ARGS__) \
    } \
  };

/**
 * \def CREATE_OUTPUT_CONNECTOR_(...)
 *
 * Defines the output connector struct for the inialization
 * of input/output connection of a BaseModule with the shared memory
 *
 * @param ... paranthesis enclosed array of variables to be
 *   defined (type, name), (type, name), ... (typelast, namelast),
 */
#define CREATE_OUTPUT_CONNECTOR_(OutputEnum, ...) \
  public: \
  /** \
   * Enumerator for all the output variables of the class \
   * written to shared memory \
   * \
   * @enum OutputEnum \
   */ \
  DEFINE_ENUM(OutputEnum, 0, SEPARATE(__VA_ARGS__), numOutput); \
  struct OutputConnector : public OutputMemoryConnector \
  { \
    /** \
     * Initializes the BaseModule connection for shared memory read \
     * access \
     * \
     * @param connectingThread pointer to BaseModule \
     * @param connectorName name of the module \
     */ \
    OutputConnector( \
      BaseModule* connectingThread, string connectorName) : \
      OutputMemoryConnector(connectingThread, connectorName) \
    {} \
    \
    /** \
    * Derived from MemoryConnector \
    */ \
    void initConnector() \
    { \
      boost::shared_ptr<SharedMemory> sharedMemory = \
      connectingThread->getLocalSharedMemory(); \
      INIT_OUTPUT; \
      FOR_EACH(DECLARE_OVARS_, __VA_ARGS__) \
    } \
  };
