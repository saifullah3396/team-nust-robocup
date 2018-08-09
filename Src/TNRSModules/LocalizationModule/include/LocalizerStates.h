/**
 * @file LocalizationModule/include/LocalizerStates.h
 *
 * This file declares the Enumeration LocalizerStates
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date 12 Sep 2017
 */

#pragma once

/**
 * Enumeration for localizer states according to game situation.
 *
 * @enum LocalizerStates
 */
enum LocalizerStates
{
  LOCALIZER_STARTUP,
  LOCALIZER_IN_PLAYING,
  LOCALIZER_IN_PENALIZED,
  LOCALIZER_AFTER_PENALIZED,
  LOCALIZER_STATES
};
