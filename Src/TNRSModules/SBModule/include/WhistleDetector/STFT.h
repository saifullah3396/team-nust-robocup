/*!
 * \brief Short Time Fourier Transform.
 * \author Thomas Hamboeck, Austrian Kangaroos 2014
 */

#ifndef __AK_STFT__
#define __AK_STFT__

#include <fftw3.h>
#include <complex>
#include <functional>
#include <iostream>

using namespace std;

struct ProcessingRecord
{
  float fWhistleBegin, fWhistleEnd;
  int nWhistleBegin, nWhistleEnd;
  int fSampleRate;
  int nWindowSize, nWindowSizePadded;
  int nWindowSkipping;
  float vDeviationMultiplier;
  float vWhistleThreshold;
  unsigned nWhistleMissFrames, nWhistleOkayFrames;
};

class STFT
{
public:
  STFT(const int channelOffset, const int windowTime, const int windowTimeStep,
    const int windowFrequency, const ProcessingRecord& config);
  virtual
  ~STFT();

  void
  newData(const int16_t *data, int length, short channels);

protected:
  void
  intToFloat(const int16_t &in, float &out);

  const int offset;
  const int windowTime, windowTimeStep, windowFrequency, windowFrequencyHalf;

  void
  calcMeanDeviation(const float *data, int length, float &mean, float &dev);
  void
  handleSpectrum(const float *spectrum, int length);
  void
  whistleAction()
  {
    cout << "Whistle heard" << endl;
  }

  int nOverflow;
  int16_t *overflownData;
  float *input;
  fftwf_complex *output;
  float *outputMag;

  fftwf_plan plan;
  ProcessingRecord config;
};

#endif
