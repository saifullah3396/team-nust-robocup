/*!
 * \brief Short Time Fourier Transform.
 * \author Thomas Hamboeck, Austrian Kangaroos 2014
 */

#include <limits>
#include <complex>
#include <iostream>
#include "SBModule/include/WhistleDetector/STFT.h"

using namespace std;

#define WARN(cond, str)     do { if(!(cond)) { std::cerr << "Warning: " << str << std::endl; } } while(0);

STFT::STFT(const int channelOffset, const int windowTime,
  const int windowTimeStep, const int windowFrequency,
  const ProcessingRecord& config) :
  offset(channelOffset), windowTime(windowTime), windowTimeStep(windowTimeStep),
    windowFrequency(windowFrequency),
    windowFrequencyHalf(windowFrequency / 2 + 1), config(config), nOverflow(0),
    overflownData(NULL), input(NULL), output(NULL), outputMag(NULL)
{
  overflownData = new int16_t[windowTime]; /* actually a max of (windowTime - 1) */
  input = static_cast<float*>(fftwf_malloc(sizeof(float) * windowFrequency));
  output = static_cast<fftwf_complex*>(fftwf_malloc(
    sizeof(fftwf_complex) * windowFrequencyHalf));
  outputMag = new float[windowFrequencyHalf];

  WARN(
    windowFrequency >= windowTime,
    "Frequency window must be greater than Time Window.");

  for (int i = 0; i < windowFrequency; ++i) {
    input[i] = 0.0f;
  }

  plan = fftwf_plan_dft_r2c_1d(windowFrequency, input, output, FFTW_MEASURE);
}

STFT::~STFT()
{
  if (overflownData) {
    delete[] overflownData;
  }
  if (input) {
    fftwf_free(input);
  }
  if (output) {
    fftwf_free(output);
  }
  if (outputMag) {
    delete[] outputMag;
  }
  if (plan) {
    fftwf_destroy_plan(plan);
  }
}

void
STFT::intToFloat(const int16_t &in, float &out)
{
  out = static_cast<float>(in) / (std::numeric_limits < int16_t > ::max() + 1);
}

void
STFT::newData(const int16_t *data, int length, short channels)
{
  int iBegin, iEnd, iDataChannel, iBuffer;

  iBuffer = 0;
  /* for each overflown data */
  while (iBuffer < nOverflow) {
    intToFloat(overflownData[iBuffer], input[iBuffer]);
    ++iBuffer;
  }
  iBegin = 0;
  while (true) {
    iEnd = iBegin + windowTimeStep;
    if (iEnd > length) {
      break;
    }
    iDataChannel = iBegin * channels + offset;
    while (iBuffer < windowTime) {
      intToFloat(data[iDataChannel], input[iBuffer]);
      ++iBuffer;
      iDataChannel += channels;
    }
    /* and the rest is zero */

    fftwf_execute(plan);
    /* calc magnitude */
    for (int i = 0; i < windowFrequencyHalf; ++i) {
      outputMag[i] = std::abs(
        *reinterpret_cast<std::complex<float>*>(&output[i]));
    }
    handleSpectrum(outputMag, windowFrequencyHalf);
    /* next cycle */
    iBuffer = 0;
    iBegin += windowTimeStep;
  }
  nOverflow = 0;
  iDataChannel = iBegin * channels + offset;
  while (iBegin < length) {
    /* copy to overflow buffer */
    overflownData[nOverflow] = data[iDataChannel];
    ++nOverflow;
    ++iBegin;
    iDataChannel += channels;
  }
}

void
STFT::calcMeanDeviation(const float *data, int length, float &mean, float &dev)
{
  mean = dev = 0;
  for (int i = 0; i < length; ++i) {
    mean += data[i];
    dev += data[i] * data[i];
  }
  dev = std::sqrt(length * dev - mean * mean) / length;
  mean /= length;
}

/* start fft stuff */
void
STFT::handleSpectrum(const float *spectrum, int length)
{
  static unsigned whistleCounter(0), whistleMissCounter(0), whistleDone(false);

  float mean, dev;
  calcMeanDeviation(spectrum, length, mean, dev);

  bool found;
  const float whistleThresh = mean + config.vWhistleThreshold * dev;
  found = false;

  int i;
  for (i = config.nWhistleBegin; i < config.nWhistleEnd; ++i) {
    if (spectrum[i] > whistleThresh) {
      found = true;
      break;
    }
  }

  if (whistleDone) {
    if (!found) {
      ++whistleMissCounter;
      if (whistleMissCounter > config.nWhistleMissFrames) {
        whistleCounter = 0;
        whistleMissCounter = 0;
        whistleDone = false;
      }
    }
  } else {
    if (found) {
      ++whistleCounter;
      whistleMissCounter = 0;
    } else if (whistleCounter > 0) {
      ++whistleMissCounter;
      if (whistleMissCounter > config.nWhistleMissFrames) {
        whistleCounter = 0;
        whistleMissCounter = 0;
        whistleDone = false;
      }
    }
    if (whistleCounter >= config.nWhistleOkayFrames) {
      whistleAction();
      whistleCounter = 0;
      whistleMissCounter = 0;
      whistleDone = true;
    }
  }
}
