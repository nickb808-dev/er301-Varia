/* VariableBW.h — Serge Variable Bandwidth Filter (+ resonance) for ER-301 v0.3.0
 *
 * CONCEPT
 * ───────
 * Modelled on the Serge Variable Bandwidth Filter (VCF2): a bandpass whose
 * CENTRE FREQUENCY and BANDWIDTH are independently voltage-controllable.  Unlike
 * the flat, non-resonant original, this version adds RESONANCE that emphasises
 * the two band EDGES and can self-oscillate.
 *
 * TOPOLOGY
 * ────────
 * The variable-bandwidth bandpass is two Cytomic (Andrew Simper) state-variable
 * filters in SERIES per channel:
 *     input → highpass @ fL  →  lowpass @ fH  →  output
 * with the band edges set from the centre and bandwidth:
 *     fL = fc · 2^(−bw·kMaxBwOct/2),   fH = fc · 2^(+bw·kMaxBwOct/2)
 * so the pass-band spans fL…fH, widening with Bandwidth and sliding with Freq.
 * Resonance is shared damping k = 1/Q on both stages: the highpass stage peaks
 * at fL and the lowpass stage at fH, so both edges resonate; as the band narrows
 * (bw → 0) the two peaks merge into one tall, high-Q peak.  At full resonance
 * the edges self-oscillate — bounded by the output soft-limiter.
 *
 * am335x TRIG: cutoff is variable, so g = tan(π·f/fs) and the octave spread are
 * recomputed every block — using the libm-free svf_math.h (no runtime sinf/cosf
 * from the package .so; see that header).
 *
 * CONTROLS: Freq (Hz, CV via a pitch map in Lua) · Bandwidth [0,1] ·
 *           Resonance [0,1] · Level [0,2].  Stereo in → stereo out (shared
 *           coefficients, per-channel state).
 *
 * STEREO PEAK PHASE (v0.3.0 — stereo-only)
 * ────────────────────────────────────────
 * Phase [−1,1] offsets the relative phase of the resonant peaks between L and R.
 * Each Cytomic SVF exposes its band-pass output v1, which is in QUADRATURE (≈90°)
 * with the peak at that stage's edge; q = v1(stage L) + v1(stage H) is therefore
 * the 90° partner of BOTH edges at once (only the matching stage is large at each
 * edge).  The output becomes  OutL = v2 + w·q,  OutR = v2 − w·q  (w from Phase),
 * so L and R gain opposite phase at the peaks — up to ±45° each (±90° relative)
 * at |w| = 1 — swinging the ringing edges across the stereo field.  At w = 0 the
 * output is bit-identical to before; the mono sum is always 2·v2 (mono-safe).
 * The Phase inlet is wired only in a STEREO lane (Lua), so a mono lane is
 * unchanged.  Feed-forward from bounded states → no new stability concern.
 */

#pragma once

#include <od/objects/Object.h>
#include <od/config.h>
#include "svf_math.h"

namespace vbf {

static constexpr int   kSampleRate = 48000;
static constexpr float kMaxBwOct   = 6.0f;     // full Bandwidth span (octaves)
static constexpr float kQmin       = 0.5f;     // Resonance 0 → damped/flat
static constexpr float kQmax       = 1500.0f;  // Resonance →1 → very sharp
static constexpr float kResSpan    = 11.5507f; // = log2(kQmax / kQmin)
// Resonance == 1.0 forces k = 0 (undamped) → genuine self-oscillation.
static constexpr float kFcMin      = 10.0f;
static constexpr float kFcMax      = 0.45f * kSampleRate;   // 21.6 kHz
static constexpr float kEdgeMax    = 0.49f * kSampleRate;   // 23.5 kHz (tan safe)

class VariableBW : public od::Object
{
public:
    VariableBW();
    virtual ~VariableBW();

#ifndef SWIGLUA
    void process() override;

    // Live state for the slope graphic (read each UI frame).
    float getFL()    const { return mFL; }
    float getFH()    const { return mFH; }
    float getK()     const { return mK;  }
    float getPhase() const { return mPhase; }   // [-1,1] stereo peak phase

private:
    od::Inlet  mInL   {"InL"};
    od::Inlet  mInR   {"InR"};
    od::Inlet  mFreqIn      {"Freq"};       // centre frequency, Hz
    od::Inlet  mBandwidthIn {"Bandwidth"};  // [0,1] → 0..kMaxBwOct octaves
    od::Inlet  mResonanceIn {"Resonance"};  // [0,1] → edges peak / self-osc
    od::Inlet  mLevelIn     {"Level"};      // [0,2]
    od::Inlet  mPhaseIn     {"Phase"};      // [-1,1] stereo peak phase (stereo-only)

    od::Outlet mOutL {"OutL"};
    od::Outlet mOutR {"OutR"};

    // Per-block coefficients (shared L/R).
    float mA1L = 0, mA2L = 0, mA3L = 0;     // stage L (highpass @ fL)
    float mA1H = 0, mA2H = 0, mA3H = 0;     // stage H (lowpass  @ fH)
    float mK = 1.0f;                        // damping (1/Q)
    float mFL = 500.0f, mFH = 500.0f;       // current band edges (Hz) — for the graphic
    float mPhase = 0.0f;                    // last stereo phase amount — for the graphic

    // SVF state, per channel (two series stages each).
    float mIc1L_L = 0, mIc2L_L = 0, mIc1H_L = 0, mIc2H_L = 0;   // left channel
    float mIc1L_R = 0, mIc2L_R = 0, mIc1H_R = 0, mIc2H_R = 0;   // right channel

    float mLastLevel = 1.0f;

    void updateCoeffs();

#endif // SWIGLUA
};

} // namespace vbf
