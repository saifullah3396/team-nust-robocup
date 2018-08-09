/**
 * @file TNRSBase/include/MemoryBase.h
 *
 * This file declares the class MemoryBase
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @author Team-Nust 2015
 * @date 10 Jun 2017
 */

#pragma once

#include "TNRSBase/include/MemoryConnector.h"

class BaseModule;

/**
 * @class MemoryBase
 * @brief A class to act as parent for memory connection
 */
class MemoryBase
{
public:
  /**
   * Constructor
   */
  MemoryBase()
  {
  }

  /**
   * Constructor in case of a child module with parent memory
   * connector
   *
   * @param connectingModule the BaseModule through which the class will
   *   be accessing the memory
   */
  MemoryBase(BaseModule* connectingModule);

  /**
   * Destructor
   */
  virtual ~MemoryBase()
  {
  }

  /**
   * Returns the shared memory input connector of the module
   *
   * @return Pointer to InputMemoryConnector
   */
  InputMemoryConnector*
  getGenericInputConnector()
  {
    return genericInputConnector;
  }

  /**
   * Returns the shared memory output connector of the module
   *
   * @return Pointer to OutputMemoryConnector
   */
  OutputMemoryConnector*
  getGenericOutputConnector()
  {
    return genericOutputConnector;
  }
protected:
  //! Shared memory input connector of the thread
  InputMemoryConnector* genericInputConnector;

  //! Shared memory output connector of the thread
  OutputMemoryConnector* genericOutputConnector;
};
