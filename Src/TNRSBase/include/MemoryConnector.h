/**
 * @file TNRSBase/include/MemoryConnector.h
 *
 * This file declares the classes MemoryConnector, InputMemoryConnector,
 * and OutputMemoryConnector
 *
 * @author Team-Nust 2015
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 03 Feb 2017
 */

#pragma once

#include "TNRSBase/include/MemoryDefinition.h"
#include "TNRSBase/include/Multiplexer.h"

class BaseModule;
class SharedMemory;

/**
 * @class MemoryConnector
 * @brief A class that defines the connection to SharedMemory
 */
class MemoryConnector
{
public:
  /**
   * Constructor
   *
   * @param connectingThread BaseModule that wants to connect to memory
   * @param connecterName name of the memory connector
   */
  MemoryConnector(BaseModule* connectingThread, const string& connectorName);

  /**
   * Destructor
   */
  virtual
  ~MemoryConnector();

  /**
   * Pure virtual function for the initialization of input/output
   * connections to memory
   *
   * @return void
   */
  virtual void
  initConnector() = 0;

  /**
   * Returns a vector of pointers to memory variables
   *
   * @return vector of void pointers to variables
   */
  vector<void*>
  getVariables()
  {
    return variables;
  }

  //! Vector of pointers to memory variables
  vector<void*> variables;

  //! Vector of memory variable names
  vector<string> variableNames;

protected:
  //! Name of this memory connector
  string connectorName;

  //! The thread that connects with the memory through this
  //! memory connector
  BaseModule* connectingThread;
};

/**
 * @class InputMemoryConnector
 * @brief A class that is defines an input connection to SharedMemory
 */
class InputMemoryConnector : public MemoryConnector
{
public:
  /**
   * Constructor
   *
   * @param connectingThread BaseModule that wants to connect to memory
   * @param connecterName name of the memory connector
   */
  InputMemoryConnector(BaseModule* connectingThread,
    const string& connectorName) :
    MemoryConnector(connectingThread, connectorName)
  {
  }

  /**
   * Destructor
   */
  virtual
  ~InputMemoryConnector()
  {
    for (size_t i = 0; i < multiplexedVariables.size(); ++i)
      delete multiplexedVariables[i];
  }

  /**
   * Syncs the input variables of the connected module from memory
   *
   * @return void
   */
  void
  syncFromMemory();

  /**
   * Returns the vector of multiplexed variables
   *
   * @return vector<Multiplexer*>&
   */
  vector<Multiplexer*>&
  getMUX()
  {
    return multiplexedVariables;
  }

  /**
   * Returns the multiplexed variable with the given index
   *
   * @return Pointer to Multiplexer
   */
  Multiplexer*
  getMultiplexedVariables(const unsigned& index)
  {
    return
      (index < multiplexedVariables.size()) ? multiplexedVariables[index] : NULL;
  }

protected:
  //! A vector of multiplexed variables
  vector<Multiplexer*> multiplexedVariables;
};

/**
 * @class OutputMemoryConnector
 * @brief A class that defines an output connection to SharedMemory
 */
class OutputMemoryConnector : public MemoryConnector
{
public:
  /**
   * Constructor
   *
   * @param connectingThread BaseModule that wants to connect to memory
   * @param connecterName name of the memory connector
   */
  OutputMemoryConnector(BaseModule* connectingThread,
    const string& connectorName) :
    MemoryConnector(connectingThread, connectorName)
  {
  }

  /**
   * Virtual destructor
   */
  virtual
  ~OutputMemoryConnector()
  {
  }

  /**
   * Syncs the output variables from the connected module to memory
   */
  void
  syncToMemory();

  /**
   * Returns the vector of pointer to thread-safe shared memory variables
   *
   * @return vector<ThreadSafeVariable*>&
   */
  vector<ThreadSafeVariable*>&
  getSharedMemoryVariables()
  {
    return sharedMemoryVariables;
  }

protected:
  //! The vector of pointer to threadsafe shared memory variables
  vector<ThreadSafeVariable*> sharedMemoryVariables;
};
