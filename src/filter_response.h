/* filter_response.h — analytic magnitude of Varia's variable-BP, for the graphic.
 *
 * The analog-prototype magnitude of the series highpass@fL → lowpass@fH bandpass
 * with shared damping k (= 1/Q).  Used by SlopeGraphic to draw the live filter
 * slope, and host-tested against the measured FFT response.
 *
 * Pure arithmetic + sqrtf only — NO libm trig (sqrtf is a hardware instruction,
 * not the flagged package→firmware sin/cos path).  So the slope display is
 * am335x-safe by construction (see svf_math.h / the package-trig note).
 */
#pragma once

#include <math.h>   // sqrtf only

namespace vbf {

// |H(f)| of highpass@fL · lowpass@fH, 2nd-order each, shared damping k.
// k is clamped to a small floor so the display peak stays finite at self-osc.
static inline float varBpMag(float f, float fL, float fH, float k)
{
    if (k < 0.02f) k = 0.02f;           // finite display peak at k→0

    const float w  = f / fL;            // highpass @ fL
    const float w2 = w * w;
    const float dHp = (1.0f - w2) * (1.0f - w2) + (k * w) * (k * w);
    const float hp = w2 / sqrtf(dHp > 1e-20f ? dHp : 1e-20f);

    const float u  = f / fH;            // lowpass @ fH
    const float u2 = u * u;
    const float dLp = (1.0f - u2) * (1.0f - u2) + (k * u) * (k * u);
    const float lp = 1.0f / sqrtf(dLp > 1e-20f ? dLp : 1e-20f);

    return hp * lp;
}

// |BP(f)| of one 2nd-order band-pass (an SVF's v1 tap) at f0, damping k.
// Peaks at f0 with height 1/k, which is why the interior resonators get a k
// floor in updateCoeffs — an undamped parallel resonator has no finite peak.
static inline float bpMag(float f, float f0, float k)
{
    if (k < 0.02f) k = 0.02f;
    const float w  = f / f0;
    const float w2 = w * w;
    const float d  = (1.0f - w2) * (1.0f - w2) + (k * w) * (k * w);
    return w / sqrtf(d > 1e-20f ? d : 1e-20f);
}

// Full response including the PEAKS bank: the series band plus the parallel
// interior resonators, scaled by the same normalisation the DSP applies.
//
// APPROXIMATION, on purpose: the terms are summed as MAGNITUDES, not as
// complex values, because the display has no phase to show and the peaks are
// narrow and well separated — at any frequency one term dominates. Where they
// overlap this reads slightly high. The `curve` suite checks the drawn peak
// POSITIONS against the measured FFT, which is what the display is actually
// promising; it does not claim dB accuracy in the overlaps.
static inline float varBpMagN(float f, float fL, float fH, float k,
                              const float *innerHz, const float *innerG,
                              int n, float innerK, float norm)
{
    float m = varBpMag(f, fL, fH, k);
    for (int i = 0; i < n; ++i) m += innerG[i] * bpMag(f, innerHz[i], innerK);
    return m * norm;
}

} // namespace vbf
