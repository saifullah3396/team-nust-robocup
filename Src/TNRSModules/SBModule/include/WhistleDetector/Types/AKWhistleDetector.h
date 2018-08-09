/**
 * @file SBModule/include/WhistleDetector/Types/AKWhistleDetector.h
 *
 * This file declares the class AKWhistleDetector
 *
 * @author <A href="mailto:saifullah3396@gmail.com">Saifullah</A>
 * @date Jul 21 2018
 */

#pragma once

#include "SBModule/include/WhistleDetector/WhistleDetector.h"
/**
 * @class AKWhistleDetector
 * @brief Whistle detector based on Austrian Kangaroos Opensource
 *   whistle detector
 */ 
class AKWhistleDetector : public WhistleDetector
{
public:
  /**
   * Constructor
   * 
   * @param sbModule: Pointer to base static behaviors module
   * @param config: Configuration of the behavior
   */
  AKWhistleDetector(
	SBModule* sbModule,
	const BehaviorConfigPtr& config) :
    WhistleDetector(sbModule, config, "WhistleDetector"), 
    reader(NULL)
  {
  }

  /**
   * Destructor
   */
  ~AKWhistleDetector()
  {
    delete stft;
    delete reader;
  }

  /**
   * Derived from Behavior
   */
  void initiate();
  void update();
  void finish();
  void loadExternalConfig(); 

private:
  int runFrequencyExtraction();
  void stopListening();
  void setupExecution();
  AlsaRecorder *reader;
  STFT* stft;
  static ProcessingRecord config;  
};

typedef boost::shared_ptr<AKWhistleDetector> AKWhistleDetectorPtr;
