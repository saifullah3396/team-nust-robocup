/*!
 * \brief An interface to ALSA for recording sound data.
 * \author Thomas Hamboeck, Austrian Kangaroos 2014
 */

#include <cmath>
#include <iostream>
#include "SBModule/include/WhistleDetector/ALSARecorder.h"
#include "SBModule/include/WhistleDetector/SoundConfig.h"

using namespace std;

AlsaRecorder::AlsaRecorder(std::function<void
(const int16_t*, int, short)> handler) :
  audioBuffer(NULL), handler(handler), stopRecorder(false)
{
}

AlsaRecorder::~AlsaRecorder()
{
}

void
AlsaRecorder::main()
{
  int err;
  if ((err = snd_pcm_readi(captureHandle, audioBuffer, bufferSize)) != bufferSize) {
    std::cerr << "read from audio interface failed " << snd_strerror(err) << std::endl;
    if ((err = xrunRecovery(captureHandle, err)) < 0) {
      std::cerr << "couldn't recover " << snd_strerror(err) << std::endl;
      stopRecorder = true;
    }
    return;
  }
  /* process */
  handler(audioBuffer, bufferSize, NUM_CHANNELS_RX);
}

/*******************************************************************/
void
AlsaRecorder::initAlsa()
{
  if (audioBuffer) {
    std::cerr << "Double initialization" << std::endl;
    return;
  }

  int err;
  snd_pcm_hw_params_t *hwParams;

  bufferSize = BUFFER_SIZE_RX;

  if ((err = snd_pcm_open(
    &captureHandle,
    SOUND_DEVICE_RX,
    SND_PCM_STREAM_CAPTURE,
    0)) < 0) {
    std::cerr << "cannot open audio device " << SOUND_DEVICE_RX << "(" << snd_strerror(
      err) << ")" << std::endl;
    return;
  }

  if ((err = snd_pcm_hw_params_malloc(&hwParams)) < 0) {
    std::cerr << "cannot allocate hardware parameter structure " << snd_strerror(
      err) << std::endl;
    ;
    return;
  }

  if ((err = snd_pcm_hw_params_any(captureHandle, hwParams)) < 0) {
    std::cerr << "cannot initialize hardware parameter structure " << snd_strerror(
      err) << std::endl;
    return;
  }

  if ((err = snd_pcm_hw_params_set_access(
    captureHandle,
    hwParams,
    SND_PCM_ACCESS_RW_INTERLEAVED)) < 0) {
    std::cerr << "cannot set access type " << snd_strerror(err) << std::endl;
    return;
  }

  if ((err = snd_pcm_hw_params_set_format(
    captureHandle,
    hwParams,
    SND_PCM_FORMAT_S16_LE)) < 0) {
    std::cerr << "cannot set sample format " << snd_strerror(err) << std::endl;
    return;
  }

  unsigned oSR;
  oSR = SAMPLE_RATE_RX;
  if ((err = snd_pcm_hw_params_set_rate_near(captureHandle, hwParams, &oSR, 0)) < 0) {
    std::cerr << "cannot set sample rate " << snd_strerror(err) << std::endl;
    return;
  }

  if (oSR != SAMPLE_RATE_RX) {
    std::cerr << "cannot set sample rate, correct should be: " << SAMPLE_RATE_RX << ", is " << oSR << std::endl;
    return;
  }

  std::cout << "ALSA-RX opened with a samplerate of " << oSR << " (requested: " << SAMPLE_RATE_RX << ")." << std::endl;

  if ((err = snd_pcm_hw_params_set_channels(
    captureHandle,
    hwParams,
    NUM_CHANNELS_RX)) < 0) {
    std::cerr << "cannot set channel count " << snd_strerror(err) << std::endl;
    return;
  }

  if ((err = snd_pcm_hw_params(captureHandle, hwParams)) < 0) {
    std::cerr << "cannot set parameters " << snd_strerror(err) << std::endl;
    return;
  }

  snd_pcm_hw_params_free(hwParams);

  if ((err = snd_pcm_prepare(captureHandle)) < 0) {
    std::cerr << "cannot prepare audio interface for use " << snd_strerror(err) << std::endl;
    return;
  }

  audioBuffer = new int16_t[NUM_CHANNELS_RX * bufferSize];
}

void
AlsaRecorder::setVolume(const char *subdevice)
{
  int err;
  long vmin;
  long vmax;
  snd_mixer_t *mixer;
  snd_mixer_selem_id_t *selemid;

  int mode = 0;
  if ((err = snd_mixer_open(&mixer, mode)) < 0) {
    std::cerr << "unable to open mixer " << snd_strerror(err) << std::endl;
    return;
  }
  if ((err = snd_mixer_attach(mixer, SOUND_DEVICE_RX_VOL)) < 0) {
    std::cerr << "unable to attach card to mixer " << snd_strerror(err) << std::endl;
    return;
  }
  if ((err = snd_mixer_selem_register(mixer, NULL, NULL)) < 0) {
    std::cerr << "unable to register mixer " << snd_strerror(err) << std::endl;
    return;
  }
  if ((err = snd_mixer_load(mixer)) < 0) {
    std::cerr << "unable to load mixer " << snd_strerror(err) << std::endl;
    return;
  }
  snd_mixer_selem_id_malloc(&selemid);
  if (selemid == NULL) {
    std::cerr << "unable to allocate selemid." << std::endl;
    return;
  }
  snd_mixer_selem_id_set_index(selemid, 0);
  snd_mixer_selem_id_set_name(selemid, subdevice);
  snd_mixer_elem_t* elem = snd_mixer_find_selem(mixer, selemid);
  if (elem == NULL) {
    std::cerr << "unable to find selem." << std::endl;
    return;
  }
  if ((err = snd_mixer_selem_get_capture_volume_range(elem, &vmin, &vmax)) < 0) {
    std::cerr << "unable to get capture volume range " << snd_strerror(err) << std::endl;
    return;
  }
  if ((err = snd_mixer_selem_set_capture_volume_all(
    elem,
    (vmax * VOLUME_RX) / 100)) < 0) {
    std::cerr << "unable to set capture volume " << snd_strerror(err) << std::endl;
    return;
  }
  snd_mixer_selem_id_free(selemid);
  if ((err = snd_mixer_close(mixer)) < 0) {
    std::cerr << "unable to close mixer " << snd_strerror(err) << std::endl;
    return;
  }
}

int
AlsaRecorder::xrunRecovery(snd_pcm_t *handle, int err)
{
  std::cerr << "stream recovery" << std::endl;

  if (err == -EPIPE) {
    /* underrun */
    err = snd_pcm_prepare(handle);
    if (err < 0) {
      std::cerr << "Can't recover from underrun, prepare failed: " << snd_strerror(
        err) << std::endl;
    }
    return 0;
  }
  return err;
}

void
AlsaRecorder::destroyAlsa()
{
  if (!audioBuffer) {
    std::cerr << "Not initialized!" << std::endl;
    return;
  }

  std::cout << "ALSA-RX closed." << std::endl;

  snd_pcm_drop(captureHandle);
  snd_pcm_close(captureHandle);

  delete audioBuffer;
}
