/*!
 * \brief An interface to ALSA for recording sound data.
 * \author Thomas Hamboeck, Austrian Kangaroos 2014
 */

#ifndef __AK_ALSA_RECORDER__
#define __AK_ALSA_RECORDER__

#include <stdint.h>
#include <alsa/asoundlib.h>
#include <vector>
#include <functional>

class AlsaRecorder
{
public:
  /* handler: samples, count, channels */
  AlsaRecorder(std::function<void
  (const int16_t*, int, short)> handler);
  virtual
  ~AlsaRecorder();

  void
  main();

  void
  initAlsa();
  void
  setVolume(const char *subdevice);
  void
  destroyAlsa();

  bool
  stopRecording() const
  {
    return stopRecorder;
  }

protected:
  int
  xrunRecovery(snd_pcm_t *handle, int err);

  int16_t *audioBuffer;
  int bufferSize;

  snd_pcm_t *captureHandle;

  std::function<const void
  (int16_t*, int, short)> handler;

  bool stopRecorder;
};

#endif

