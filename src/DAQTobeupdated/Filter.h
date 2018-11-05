/**
 * @file Filter.h
 *
 * This file declares the base class for all the filters used on data
 * for estimation.
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 30 Sep 2017
 */

#pragma once

#include "DAQModule/DAQModule.h"
#include "MemoryModule/MemoryBase.h"
#include "Utils/MathsUtils.h"

using namespace Utils;

class Filter : public MemoryBase
{
public:

  /**
   * Default constructor for this class.
   *
   * @param daqModule: Pointer to parent DAQModule
   */
  Filter(DAQModule* daqModule) :
    MemoryBase(daqModule)
  {
  }

  /**
   * Default destructor for this class.
   */
  ~Filter()
  {
  }

  /**
   * Virtual function that must be defined for each filter.
   */
  virtual void
  initiate() = 0;

  /**
   * Virtual function that must be defined for each filter.
   */
  virtual void
  update() = 0;
};
