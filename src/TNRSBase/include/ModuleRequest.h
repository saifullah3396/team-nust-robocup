/**
 * @file TeamNUSTSPL/include/ModuleRequests.h
 *
 * This file declares the struct ModuleRequest
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 04 Feb 2017
 */

#pragma once

#include <boost/shared_ptr.hpp>
#include "TeamNUSTSPL/include/TNSPLModuleIds.h"

/**
 * @class ModuleRequest
 * @brief A base class for defining a request. The module that receives
 *   the request performs some operation based on the id.
 */
class ModuleRequest
{
public:
  /**
   * Constructor
   */
  ModuleRequest(
    const unsigned& moduleId,
    const unsigned& id) :
    moduleId(moduleId),
    id(id)
  {
  }

  /**
   * Gets the module id
   */ 
  unsigned getModuleId() { return moduleId; }

  /**
   * Sets the id
   */ 
  unsigned setId(const unsigned& id) { this->id = id; }
  
  /**
   * Gets the id
   */ 
  unsigned getId() { return id; }

private:
  //! Id of the module for which this request is valid
  unsigned moduleId;

  //! Id of the request
  unsigned id;
};
typedef boost::shared_ptr<ModuleRequest> ModuleRequestPtr;
