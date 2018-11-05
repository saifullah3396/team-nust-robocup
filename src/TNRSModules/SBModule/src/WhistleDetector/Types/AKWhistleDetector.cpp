/**
 * @file SBModule/src/WhistleDetector/Types/AKWhistleDetector.cpp
 *
 * This file implements the class AKWhistleDetector
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date Jul 21 2018
 */

#include "SBModule/include/WhistleDetector/Types/AKWhistleDetector.h"

ProcessingRecord AKWhistleDetector::config;

void AKWhistleDetector::loadExternalConfig() 
{
  static bool loaded = false;
  if (!loaded) {
    GET_CONFIG("WhistleDetector",
      (float, AKWhistleDetector.whistleBegin, config.fWhistleBegin), 
      (float, AKWhistleDetector.whistleEnd, config.fWhistleEnd), 
      (int, AKWhistleDetector.sampleRate, config.fSampleRate), 
      (int, AKWhistleDetector.windowSize, config.nWindowSize), 
      (int, AKWhistleDetector.windowSizePadded, config.nWindowSizePadded), 
      (int, AKWhistleDetector.windowSkipping, config.nWindowSkipping), 
      (float, AKWhistleDetector.threshold, config.vWhistleThreshold), 
      (unsigned, AKWhistleDetector.frameOkays, config.nWhistleOkayFrames), 
      (unsigned, AKWhistleDetector.frameMisses, config.nWhistleMissFrames), 
    )
    loaded = true;
  }
}

void AKWhistleDetector::initiate()
{
  runFrequencyExtraction();
  std::cout << "Listening ..." << std::endl;
  inBehavior = true;
}

void AKWhistleDetector::update()
{
  if (reader->stopRecording()) {
    finish();
  }

  if (execTime <= timeToDetect) {
    reader->main();
    execTime += cycleTime;
  } else {
    std::cout << "... stopped listening." << std::endl;
    finish();
  }
}

void
AKWhistleDetector::finish()
{
  reader->destroyAlsa();
  inBehavior = false;
}

int
AKWhistleDetector::runFrequencyExtraction()
{
  /* load window times */
  config.nWhistleBegin =
    (config.fWhistleBegin * config.nWindowSizePadded) / config.fSampleRate;
  config.nWhistleEnd =
    (config.fWhistleEnd * config.nWindowSizePadded) / config.fSampleRate;
  /* back calculation for displaying purposes */
  const float fWhistleBegin =
    (config.nWhistleBegin * static_cast<float>(config.fSampleRate)) / config.nWindowSizePadded;
  const float fWhistleEnd =
    (config.nWhistleEnd * static_cast<float>(config.fSampleRate)) / config.nWindowSizePadded;
  std::cout << "---------------------------------------------------" << std::endl << "Window:" << std::endl << "  Real Window:      " << config.nWindowSize << " bins" << std::endl << "  Padded Window:    " << config.nWindowSizePadded << " bins" << std::endl << "  Window Skip:      " << config.nWindowSkipping << " samples" << std::endl << "---------------------------------------------------" << std::endl << "  Whistle Begin:    " << fWhistleBegin << " Hz" << std::endl << "  Whistle End:      " << fWhistleEnd << " Hz" << std::endl;
  std::cout << "---------------------------------------------------" << std::endl;
  if (fWhistleBegin < 0) {
    std::cerr << "Whistle begin is below zero!" << std::endl;
    return -1;
  }
  if (fWhistleEnd < 0) {
    std::cerr << "Whistle end is below zero!" << std::endl;
    return -1;
  }
  if (fWhistleBegin > (config.fSampleRate / 2)) {
    std::cerr << "Whistle begin is above Nyquist frequency!" << std::endl;
    return -1;
  }
  if (fWhistleEnd > (config.fSampleRate / 2)) {
    std::cerr << "Whistle end is above Nyquist frequency!" << std::endl;
    return -1;
  }
  if (fWhistleBegin > fWhistleEnd) {
    std::cerr << "Whistle begin is above Whistle end!" << std::endl;
    return -1;
  }
  setupExecution();
  return 0;
}

void
AKWhistleDetector::setupExecution()
{
  stft = new STFT(
    0,
    config.nWindowSize,
    config.nWindowSkipping,
    config.nWindowSizePadded,
    config);
  reader = new AlsaRecorder(
    std::bind(
      &STFT::newData,
      stft,
      std::placeholders::_1,
      std::placeholders::_2,
      std::placeholders::_3));

  reader->initAlsa();
  reader->setVolume(SOUND_SUBDEVICE_RX);
}
