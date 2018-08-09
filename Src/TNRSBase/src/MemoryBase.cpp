/**
 * @file TNRSBase/src/MemoryBase.cpp
 *
 * This file implements the class MemoryBase
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @author Team-Nust 2015
 * @date 10 Jun 2017
 */

#include "TNRSBase/include/BaseModule.h"
#include "TNRSBase/include/MemoryBase.h"

MemoryBase::MemoryBase(BaseModule* connectingModule)
{
  genericInputConnector = connectingModule->getGenericInputConnector();
  genericOutputConnector = connectingModule->getGenericOutputConnector();
}
