/**
 * @file TeamNUSTSPL/TNSPLModuleIds.h
 *
 * This file defines the enumeration TNSPLModuleIds
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 03 Feb 2017
 */

#pragma once

/**
 * Enumeration for BaseModules defined under TeamNUSTSPL class
 *
 * @enum TNSPLModules
 */
enum class TNSPLModules
: unsigned int {
  PLANNING,
  MOTION,
  STATIC,
  VISION,
  LOCALIZATION,
  COMM,
  COUNT
};
