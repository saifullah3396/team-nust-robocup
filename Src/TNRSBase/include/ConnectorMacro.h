/**
 * @file TNRSBase/include/ConnectorMacro.h
 *
 * This file declares the macros for defining SharedMemory connectors,
 * and for defining and providing access to the SharedMemory variables
 *
 * @author Team-Nust 2015
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 03 Feb 2017
 */

#pragma once

#include "TNRSBase/include/BaseModule.h"
#include "TNRSBase/include/Multiplexer.h"
#include "TNRSBase/include/SharedMemory.h"
#include "Utils/include/DataUtils.h"
#include "Utils/include/VariadicMacros.h"
#include "Utils/include/Variable.h"

using namespace Utils;

/**
 * \def INIT_INPUT
 *
 * Inializies the local module input variables
 */
#define INIT_INPUT \
{ \
  variables.assign(numInput, NULL); \
  multiplexedVariables.assign(numInput, NULL); \
  variableNames.assign(numInput, string("  ") ); \
}

/**
 * \def INIT_OUTPUT
 *
 * Inializies the local module output variables
 */
#define INIT_OUTPUT \
{ \
  variables.assign(numOutput, NULL); \
  sharedMemoryVariables.assign(numOutput, NULL); \
  variableNames.assign(numOutput, string("  ") ); \
}

/**
 * \def DECLARE_IVARS_(TYPE_VAR)
 *
 * Takes a single input variable (type, name), removes
 * paranthesis and sends it to declaration
 *
 * @param TYPE_VAR (typeOfVarX, indexOfVarX) in paranthesis
 */
#define DECLARE_IVARS_(TYPE_VAR) \
{ \
  DECLARE_IVAR( \
    M_GET_ELEM(0,UNPAREN(TYPE_VAR)), \
    M_GET_ELEM(1,UNPAREN(TYPE_VAR)) \
  ) \
}

/**
 * \def DECLARE_IVAR(type, LOCAL_INDEX)
 *
 * Separates local and global memory indexes for input variables
 *
 * @param type variable type
 * @param LOCAL_INDEX local index name or enum of the variable
 */
#define DECLARE_IVAR(type, LOCAL_INDEX) \
{ \
  IVAR_EXTENDED( \
    type, \
    LOCAL_INDEX, \
    GlobalMemory::LOCAL_INDEX \
  ) \
}

/**
 * \def DECLARE_OVARS_(TYPE_VAR)
 *
 * Takes a single output variable (type, name), removes
 * paranthesis and sends it to declaration
 *
 * @param TYPE_VAR (typeOfVarX, indexOfVarX) in paranthesis
 */
#define DECLARE_OVARS_(TYPE_VAR) \
{ \
  DECLARE_OVAR( \
    M_GET_ELEM(0,UNPAREN(TYPE_VAR)), \
    M_GET_ELEM(1,UNPAREN(TYPE_VAR)) \
  ) \
}

/**
 * \def DECLARE_OVAR(type, LOCAL_INDEX)
 *
 * Separates local and global memory indexes for output variables
 *
 * @param type variable type
 * @param LOCAL_INDEX local index name or enum of the variable
 */
#define DECLARE_OVAR(type, LOCAL_INDEX) \
{ \
  OVAR_EXTENDED( \
    type, \
    LOCAL_INDEX, \
    GlobalMemory::LOCAL_INDEX \
  ) \
}

/**
 * \def IVAR_EXTENDED(type, LOCAL_INDEX, SHAREDMEMORY_INDEX)
 *
 * Defines a local input variable and its relationship with global
 * memory variable
 *
 * @param type variable type
 * @param LOCAL_INDEX local index name or enum of the variable
 * @param SHAREDMEMORY_INDEX shared memory index name or enum of the
 *   variable defined
 */
#define IVAR_EXTENDED(type, LOCAL_INDEX, SHAREDMEMORY_INDEX) \
{ \
  variables[LOCAL_INDEX] = new type; \
    Multiplexer *  multiplexer = new Multiplexer; \
    multiplexer->addInput(sharedMemory->variables[SHAREDMEMORY_INDEX]); \
    variableNames[LOCAL_INDEX] = \
    sharedMemory->variables[SHAREDMEMORY_INDEX]->getVariableName(); \
    multiplexedVariables[LOCAL_INDEX] = multiplexer; \
}

/**
 * \def OVAR_EXTENDED(type, LOCAL_INDEX, SHAREDMEMORY_INDEX)
 *
 * Defines a local output variable and its relationship with global
 * memory variable
 *
 * @param type variable type
 * @param LOCAL_INDEX local index name or enum of the variable
 * @param SHAREDMEMORY_INDEX shared memory index name or enum of the
 *   variable defined
 */
#define OVAR_EXTENDED(type, LOCAL_INDEX, SHAREDMEMORY_INDEX) \
{ \
  sharedMemoryVariables[LOCAL_INDEX] = \
    sharedMemory->variables[SHAREDMEMORY_INDEX]; \
    variableNames[LOCAL_INDEX] = \
    sharedMemory->variables[SHAREDMEMORY_INDEX]->getVariableName(); \
    type * tempVar = new type; \
    *tempVar = (static_cast< Variable< type > *>(sharedMemory->variables[SHAREDMEMORY_INDEX]))->getValue(); \
    variables[LOCAL_INDEX] = tempVar; \
}

/**
 * \def IVAR(type, var)
 *
 * Provides access to a local input variable
 *
 * @param type variable type
 * @param var variable index
 */
#define IVAR(type, var) (*(static_cast< type *>(this->genericInputConnector->variables[var])))

/**
 * \def OVAR(type, var)
 *
 * Provides access to a local output variable
 *
 * @param type variable type
 * @param var variable index
 */
#define OVAR(type, var) (*(static_cast< type *>(this->genericOutputConnector->variables[var])))

/**
 * \def IVAR_PTR(type, var)
 *
 * Provides pointer access to a local input variable
 *
 * @param type variable type
 * @param var variable index
 */
#define IVAR_PTR(type, var) static_cast<boost::shared_ptr<type> >(static_cast< type *>(this->genericInputConnector->variables[var]))

/**
 * \def OVAR_PTR(type, var)
 *
 * Provides pointer access to a local output variable
 *
 * @param type variable type
 * @param var variable index
 */
#define OVAR_PTR(type, var) static_cast<boost::shared_ptr<type> >(static_cast< type *>(this->genericOutputConnector->variables[var]))
