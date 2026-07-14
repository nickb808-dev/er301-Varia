// SlopeGraphic — phosphor filter-slope scope for Varia.
//
// Draws the live variable-BP magnitude response as a glowing curve: X = log
// frequency (20 Hz … 20 kHz), Y = magnitude in dB.  A phosphor buffer with a
// −1/frame decay makes the curve TRAIL as Freq / Bandwidth / Resonance move —
// so you watch the band slide and the resonant edges rise as you play.
//
// Modeled directly on Dirac's DiracFieldGraphic (which follows Habitat's
// MirrorPhosphorGraphic): header-only with INLINE virtuals, a uint8 brightness
// buffer as a CLASS MEMBER (never a stack array — Cortex-A8 :64 trap), fade −1
// per frame, additive hits, monochrome fb.fill(BLACK) + fb.pixel render.
//
// am335x TRIG: the curve uses varBpMag (mults/divs/sqrtf) and pexp2 for the
// log-frequency axis — NO sinf/cosf.  logf (for the dB axis and edge ticks) is
// allowed on am335x (only package sinf/cosf miscompute; see the trig note).

#pragma once

#include "VariableBW.h"
#include "filter_response.h"
#include "svf_math.h"
#include <od/graphics/Graphic.h>
#include <od/graphics/constants.h>
#include <string.h>
#include <math.h>   // logf, sqrtf

namespace vbf
{

  class SlopeGraphic : public od::Graphic
  {
  public:
    SlopeGraphic(int left, int bottom, int width, int height)
        : od::Graphic(left, bottom, width, height), mpFilter(0) {}

    virtual ~SlopeGraphic()
    {
      if (mpFilter)
        mpFilter->release();
    }

    void follow(VariableBW *p)
    {
      if (mpFilter)
        mpFilter->release();
      mpFilter = p;
      if (mpFilter)
        mpFilter->attach();
    }

  private:
    VariableBW *mpFilter;

    static const int kMaxW = 128;
    static const int kMaxH = 64;
    uint8_t mPixels[kMaxW * kMaxH];
    bool    mCleared = false;

    // Display axes.
    static constexpr float kFMin    = 20.0f;
    static constexpr float kLogSpan = 9.96578f;  // = log2(20000 / 20)
    static constexpr float kDbMin   = -24.0f;
    static constexpr float kDbMax   = 36.0f;      // covers the self-osc peak
    static constexpr float kDbToPx  = 1.0f / (kDbMax - kDbMin);

    inline void addPix(int w, int h, int x, int y, int amt)
    {
      if (x < 0 || x >= w || y < 0 || y >= h) return;
      int idx = y * w + x;
      int b = mPixels[idx] + amt;
      if (b > 15) b = 15;
      mPixels[idx] = (uint8_t)b;
    }

    // x pixel column for a given frequency (log axis).  logf allowed on am335x.
    inline int freqToX(int w, float f) const
    {
      float t = 1.44269504f * logf(f / kFMin) / kLogSpan;  // log2(f/fMin)/span
      if (t < 0.0f) t = 0.0f; else if (t > 1.0f) t = 1.0f;
      return (int)(t * (w - 1) + 0.5f);
    }

  public:
    virtual void draw(od::FrameBuffer &fb)
    {
      int w = mWidth  < kMaxW ? mWidth  : kMaxW;
      int h = mHeight < kMaxH ? mHeight : kMaxH;

      if (!mCleared) { memset(mPixels, 0, sizeof(mPixels)); mCleared = true; }

      // Phosphor decay — every pixel fades one level per frame (trails).
      for (int i = 0; i < w * h; i++)
        if (mPixels[i] > 0) mPixels[i]--;

      if (mpFilter)
      {
        const float fL = mpFilter->getFL();
        const float fH = mpFilter->getFH();
        const float k  = mpFilter->getK();

        // Rasterise the response curve into the phosphor buffer, connecting
        // consecutive columns so steep resonant edges stay continuous.
        int prevY = -1;
        for (int x = 0; x < w; x++)
        {
          const float f  = kFMin * vbfmath::pexp2(kLogSpan * (float)x / (float)(w - 1));
          float m = varBpMag(f, fL, fH, k);
          if (!(m == m)) m = 0.0f;                          // NaN guard
          const float db = 8.68589f * logf(m > 1e-4f ? m : 1e-4f);  // 20·log10
          float t = (db - kDbMin) * kDbToPx;
          if (t < 0.0f) t = 0.0f; else if (t > 1.0f) t = 1.0f;
          const int y = (int)(t * (h - 1) + 0.5f);

          if (prevY < 0) prevY = y;
          int y0 = prevY < y ? prevY : y;
          int y1 = prevY < y ? y : prevY;
          for (int yy = y0; yy <= y1; yy++) addPix(w, h, x, yy, 10);  // curve body
          addPix(w, h, x, y, 5);                                      // bright crest
          prevY = y;
        }

        // Dim edge ticks at fL / fH (bottom two rows) so the band reads clearly.
        int xL = freqToX(w, fL), xH = freqToX(w, fH);
        addPix(w, h, xL, 0, 6); addPix(w, h, xL, 1, 4);
        addPix(w, h, xH, 0, 6); addPix(w, h, xH, 1, 4);
      }

      // Background + phosphor render (monochrome; v is the 4-bit intensity).
      fb.fill(BLACK, mWorldLeft, mWorldBottom,
              mWorldLeft + mWidth - 1, mWorldBottom + mHeight - 1);
      for (int y = 0; y < h; y++)
        for (int x = 0; x < w; x++)
        {
          uint8_t v = mPixels[y * w + x];
          if (v > 0)
            fb.pixel(v, mWorldLeft + x, mWorldBottom + y);
        }
    }
  };

} // namespace vbf
