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
// ER-301 convention: a full-scale 1.0 on a V/oct port is 10 octaves, matching
// the firmware's own oscillators (and Landau's kVoctToSemis = 120 semitones).
// PEAKS. Two peaks are the band edges themselves (the HP at fL and the LP at
// fH resonating), so only the INTERIOR ones need extra filters: kMaxPeaks - 2.
static constexpr int   kMaxPeaks   = 7;
static constexpr int   kMaxInner   = kMaxPeaks - 2;   // 5 interior resonators
static constexpr float kVoctOctaves = 10.0f;
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
    // For the slope graphic: how many interior peaks are live, and where.
    int   getInnerCount()    const { return mInnerN; }
    float getInnerHz(int i)  const { return (i >= 0 && i < mInnerN) ? mInnerHz[i] : 0.0f; }
    float getInnerGain(int i) const { return (i >= 0 && i < mInnerN) ? mInnerG[i] : 0.0f; }
    float getInnerK()        const { return mInnerK; }
    float getInnerNorm()     const { return mInnerNorm; }
    const float *innerHzPtr()   const { return mInnerHz; }
    const float *innerGainPtr() const { return mInnerG; }
    float getK()     const { return mK;  }
    float getPhase() const { return mPhase; }   // [-1,1] stereo peak phase

private:
    od::Inlet  mInL   {"InL"};
    od::Inlet  mInR   {"InR"};
    od::Inlet  mFreqIn      {"Freq"};       // centre frequency, Hz (LINEAR)
    // V/OCT — EXPONENTIAL centre-frequency CV (v0.3.1).
    //
    // Freq is a linear-Hz sum, which is what the header used to promise was
    // "V/oct trackable" without implementing it. Linear-Hz CV is why a SINE
    // modulator sounded like a SAW: pitch is logarithmic, so a symmetric
    // +/-300 Hz swing around 500 Hz is 0.68 octaves up but 1.32 down (and
    // +/-450 Hz is 0.93 up vs 3.32 down, then flat against the 10 Hz floor).
    // Slow rise, fast plunge, hard bottom.
    //
    //     fc = Freq * 2^(voct * kVoctOctaves)
    //
    // so a symmetric input is symmetric in OCTAVES. Unconnected reads the
    // firmware's zero buffer -> 2^0 = 1 -> bit-identical to v0.3.0.
    od::Inlet  mVoctIn      {"V/Oct"};
    // PEAKS — how many resonant peaks span the band, continuously (v0.4.0).
    // [0,1] maps to 2..kMaxPeaks. At 0 the interior resonators are all at
    // zero gain and are not run at all: bit-identical to v0.3.x, and free.
    od::Inlet  mPeaksIn     {"Peaks"};
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

    // ── interior resonators (Peaks) ─────────────────────────────────────
    // Parallel band-pass SVFs tuned INSIDE [fL, fH]. Geometric spacing:
    // interior peak j sits at fL * (fH/fL)^(j/(P-1)), so the peaks are equal
    // musical intervals apart and the pattern keeps its shape as Freq sweeps.
    //
    // Gain g_j = clamp(P-1-j, 0, 1) is what makes the count CONTINUOUS: a new
    // peak buds off the upper edge at zero gain and slides down into place as
    // P rises, while the outer two stay pinned at fL and fH. At P = 2 every
    // g_j is 0, mInnerN is 0, and the whole block is skipped.
    int   mInnerN = 0;                      // resonators actually running
    float mInnerG[kMaxInner]  = {0};        // per-peak gain (includes the fade)
    float mInnerA1[kMaxInner] = {0};
    float mInnerA2[kMaxInner] = {0};
    float mInnerA3[kMaxInner] = {0};
    float mInnerK  = 0.0f;                  // damping for the interior peaks
    float mInnerHz[kMaxInner] = {0};        // exposed to the slope graphic
    float mInnerNorm = 1.0f;                // level compensation
    // state, per channel
    float mInnerIc1L[kMaxInner] = {0}, mInnerIc2L[kMaxInner] = {0};
    float mInnerIc1R[kMaxInner] = {0}, mInnerIc2R[kMaxInner] = {0};
    float mPhase = 0.0f;                    // last stereo phase amount — for the graphic

    // SVF state, per channel (two series stages each).
    float mIc1L_L = 0, mIc2L_L = 0, mIc1H_L = 0, mIc2H_L = 0;   // left channel
    float mIc1L_R = 0, mIc2L_R = 0, mIc1H_R = 0, mIc2H_R = 0;   // right channel

    float mLastLevel = 1.0f;

    void updateCoeffs();

#endif // SWIGLUA
};

} // namespace vbf
