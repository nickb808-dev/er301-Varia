/* VariableBW.cpp — Serge Variable Bandwidth Filter (+ resonance) for ER-301
 *
 * See VariableBW.h.  Two Cytomic SVFs in series per channel (HP @ fL → LP @ fH);
 * band edges from Freq ± Bandwidth; shared resonance damping.  All coefficient
 * math is libm-free (svf_math.h) — no runtime sinf/cosf on am335x. */

#include "VariableBW.h"

#include <algorithm>

namespace vbf {

static inline float sanitize(float x)
{
    union { float f; uint32_t u; } v;
    v.f = x;
    return ((v.u & 0x7F800000u) == 0x7F800000u) ? 0.0f : x;
}

static inline float softLimit(float x)
{
    const float T  = 0.9f;
    const float ax = (x >= 0.0f) ? x : -x;
    if (ax <= T) return x;
    const float e   = ax - T;
    const float lim = T + (1.0f - T) * (e / (e + (1.0f - T)));
    return (x >= 0.0f) ? lim : -lim;
}

static inline float clampf(float x, float lo, float hi)
{
    return x < lo ? lo : (x > hi ? hi : x);
}

VariableBW::VariableBW()
{
    addInput(mInL);
    addInput(mInR);
    addInput(mFreqIn);
    addInput(mBandwidthIn);
    addInput(mResonanceIn);
    addInput(mLevelIn);
    addInput(mPhaseIn);
    addOutput(mOutL);
    addOutput(mOutR);
    updateCoeffs();
}

VariableBW::~VariableBW() {}

/* ── updateCoeffs — control-rate: band edges + SVF coefficients ─────────────── */

void VariableBW::updateCoeffs()
{
    const float fc  = clampf(mFreqIn.buffer()[0], kFcMin, kFcMax);
    const float bw  = clampf(mBandwidthIn.buffer()[0], 0.0f, 1.0f);
    const float res = clampf(mResonanceIn.buffer()[0], 0.0f, 1.0f);

    // Band edges: fL/fH straddle fc by ±(bw·kMaxBwOct/2) octaves.
    const float r  = vbfmath::pexp2(bw * kMaxBwOct * 0.5f);
    const float fL = clampf(fc / r, kFcMin, kEdgeMax);
    const float fH = clampf(fc * r, kFcMin, kEdgeMax);
    mFL = fL; mFH = fH;                         // expose to the slope graphic

    const float gL = vbfmath::ptan(vbfmath::kPi * fL / float(kSampleRate));
    const float gH = vbfmath::ptan(vbfmath::kPi * fH / float(kSampleRate));

    // Resonance → Q (exponential map) → shared damping k = 1/Q.  At the very top
    // (res = 1) k is forced to 0: the SVFs become undamped and self-oscillate at
    // the band edges (bounded by the output soft-limiter).
    if (res >= 0.999f) {
        mK = 0.0f;
    } else {
        const float Q = kQmin * vbfmath::pexp2(res * kResSpan);
        mK = 1.0f / Q;
    }

    const float a1L = 1.0f / (1.0f + gL * (gL + mK));
    mA1L = a1L; mA2L = gL * a1L; mA3L = gL * mA2L;

    const float a1H = 1.0f / (1.0f + gH * (gH + mK));
    mA1H = a1H; mA2H = gH * a1H; mA3H = gH * mA2H;
}

/* ── process() ───────────────────────────────────────────────────────────────── */

void VariableBW::process()
{
    const float *inL = mInL.buffer();
    const float *inR = mInR.buffer();
    float *outL = mOutL.buffer();
    float *outR = mOutR.buffer();
    const int N = FRAMELENGTH;

    updateCoeffs();

    const float k = mK;
    const float a1L = mA1L, a2L = mA2L, a3L = mA3L;
    const float a1H = mA1H, a2H = mA2H, a3H = mA3H;

    const float level  = std::max(0.0f, std::min(mLevelIn.buffer()[0], 2.0f));
    const float lvStep = (level - mLastLevel) * (1.0f / float(N));
    float lv = mLastLevel;

    // Stereo peak phase (stereo-only; inlet unconnected → 0 in a mono lane, so
    // the output is bit-identical to the no-phase path).  w scales a quadrature
    // side signal added +/- to L/R → up to +/-45deg each (+/-90deg relative).
    const float w = clampf(mPhaseIn.buffer()[0], -1.0f, 1.0f);

    // Local state copies (register-friendly across the sample loop).
    float ic1Ll = mIc1L_L, ic2Ll = mIc2L_L, ic1Hl = mIc1H_L, ic2Hl = mIc2H_L;
    float ic1Lr = mIc1L_R, ic2Lr = mIc2L_R, ic1Hr = mIc1H_R, ic2Hr = mIc2H_R;

    for (int s = 0; s < N; ++s) {
        // ── Left channel ──────────────────────────────────────────────────
        {
            const float x = sanitize(inL[s]);
            // stage L: highpass @ fL.  v1L = band-pass tap (quadrature at fL).
            float v3  = x - ic2Ll;
            float v1L = a1L * ic1Ll + a2L * v3;
            float v2  = ic2Ll + a2L * ic1Ll + a3L * v3;
            ic1Ll = 2.0f * v1L - ic1Ll;
            ic2Ll = 2.0f * v2  - ic2Ll;
            const float hp = x - k * v1L - v2;
            // stage H: lowpass @ fH.  v1H = band-pass tap (quadrature at fH).
            v3 = hp - ic2Hl;
            float v1H = a1H * ic1Hl + a2H * v3;
            v2 = ic2Hl + a2H * ic1Hl + a3H * v3;
            ic1Hl = 2.0f * v1H - ic1Hl;
            ic2Hl = 2.0f * v2  - ic2Hl;
            const float q = v1L + v1H;                  // 90deg partner of both edges
            outL[s] = softLimit(sanitize(v2 + w * q)) * lv;
        }
        // ── Right channel ─────────────────────────────────────────────────
        {
            const float x = sanitize(inR[s]);
            float v3  = x - ic2Lr;
            float v1L = a1L * ic1Lr + a2L * v3;
            float v2  = ic2Lr + a2L * ic1Lr + a3L * v3;
            ic1Lr = 2.0f * v1L - ic1Lr;
            ic2Lr = 2.0f * v2  - ic2Lr;
            const float hp = x - k * v1L - v2;
            v3 = hp - ic2Hr;
            float v1H = a1H * ic1Hr + a2H * v3;
            v2 = ic2Hr + a2H * ic1Hr + a3H * v3;
            ic1Hr = 2.0f * v1H - ic1Hr;
            ic2Hr = 2.0f * v2  - ic2Hr;
            const float q = v1L + v1H;
            outR[s] = softLimit(sanitize(v2 - w * q)) * lv;   // opposite sign → L↔R phase
        }
        lv += lvStep;
    }

    mIc1L_L = ic1Ll; mIc2L_L = ic2Ll; mIc1H_L = ic1Hl; mIc2H_L = ic2Hl;
    mIc1L_R = ic1Lr; mIc2L_R = ic2Lr; mIc1H_R = ic1Hr; mIc2H_R = ic2Hr;
    mLastLevel = level;
}

} // namespace vbf
