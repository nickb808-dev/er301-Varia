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
    addInput(mVoctIn);
    addInput(mPeaksIn);
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
    // Exponential V/oct on top of the linear Freq sum. pexp2 is the same
    // fast pow2 the bandwidth/resonance maps already use, so this costs one
    // extra call per BLOCK, not per sample.
    // pexp2's CONTRACT IS x >= 0 (`if (x < 0) x = 0;` — it was written for the
    // bandwidth half-spread, which is never negative). Feeding it a negative
    // exponent silently returns 2^0 = 1, so downward CV did nothing and the
    // filter only ever swept UP. Use the magnitude and reciprocate:
    // 2^-y == 1/2^y, exactly, and stays inside the contract.
    const float voct = clampf(mVoctIn.buffer()[0], -2.0f, 2.0f);   // +/-20 oct
    float fcBase = mFreqIn.buffer()[0];
    if (voct != 0.0f) {
        const float mag = vbfmath::pexp2((voct < 0.0f ? -voct : voct) * kVoctOctaves);
        fcBase = (voct > 0.0f) ? (fcBase * mag) : (fcBase / mag);
    }
    const float fc  = clampf(fcBase, kFcMin, kFcMax);
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

    // ── PEAKS: interior resonators ──────────────────────────────────────
    // P in [2, kMaxPeaks], continuous. Interior peak j (j = 1..P-2) sits at
    //     f_j = fL * (fH/fL)^(j/(P-1))
    // i.e. equal musical intervals across the band, and
    //     g_j = clamp(P-1-j, 0, 1)
    // fades the newest one in from zero, so the count sweeps smoothly. The
    // outer two peaks are the band edges themselves and are untouched.
    const float pk = clampf(mPeaksIn.buffer()[0], 0.0f, 1.0f);
    const float P  = 2.0f + pk * float(kMaxPeaks - 2);
    mInnerN = 0;
    if (P > 2.0f + 1e-4f && res > 1e-4f) {
        const float D = P - 1.0f;                    // intervals across the band
        // Span in OCTAVES without a log: the edges were built as
        // fL = fc/r, fH = fc*r with r = 2^(bw*kMaxBwOct/2), so log2(fH/fL) is
        // exactly bw*kMaxBwOct. (This file has no libm — vbfmath exists
        // precisely so the audio path never calls one.) If the edges hit a
        // clamp the nominal span overshoots, and clampf on f_j absorbs it.
        const float spanOct = bw * kMaxBwOct;
        // Interior peaks share the edges' Q but never self-oscillate: a
        // parallel bank at k = 0 would ring forever with nothing damping it,
        // and unlike the edges it is not bounded by the series path. Floor it.
        const float kIn = (mK > 0.02f) ? mK : 0.02f;
        mInnerK = kIn;
        for (int j = 1; j <= kMaxInner; ++j) {
            float g = D - float(j);                  // clamp(P-1-j, 0, 1)
            if (g <= 0.0f) break;
            if (g > 1.0f) g = 1.0f;
            const float fj = clampf(fL * vbfmath::pexp2(spanOct * float(j) / D),
                                    kFcMin, kEdgeMax);
            const float gj = vbfmath::ptan(vbfmath::kPi * fj / float(kSampleRate));
            const float a1 = 1.0f / (1.0f + gj * (gj + kIn));
            const int   i  = mInnerN++;
            mInnerA1[i] = a1; mInnerA2[i] = gj * a1; mInnerA3[i] = gj * mInnerA2[i];
            // Scale by resonance: with no resonance there are no peaks to
            // multiply, so Peaks must do nothing — same logic as the edges.
            mInnerG[i]  = g * res;
            mInnerHz[i] = fj;
        }
    }
    // Parallel resonators ADD energy. Normalise by the summed gain so turning
    // Peaks up does not just turn the filter up (softLimit catches the rest).
    float gSum = 0.0f;
    for (int i = 0; i < mInnerN; ++i) gSum += mInnerG[i];
    mInnerNorm = 1.0f / (1.0f + 0.7f * gSum);
}

// One sample through the interior resonator bank (Peaks). Band-pass tap v1 of
// a Cytomic SVF, summed with per-peak gain. ic1/ic2 are the caller's state
// arrays so the two channels stay independent but share the coefficients.
//
// Written once and used by both channels ON PURPOSE: the L and R blocks below
// are already near-duplicates, and the stereo phase bug class this unit is
// prone to comes from the two drifting apart.
static inline float innerBank(const float x, const int n,
                              const float *a1, const float *a2, const float *a3,
                              const float *g, const float k,
                              float *ic1, float *ic2)
{
    float acc = 0.0f;
    for (int i = 0; i < n; ++i) {
        const float v3 = x - ic2[i];
        const float v1 = a1[i] * ic1[i] + a2[i] * v3;
        const float v2 = ic2[i] + a2[i] * ic1[i] + a3[i] * v3;
        ic1[i] = 2.0f * v1 - ic1[i];
        ic2[i] = 2.0f * v2 - ic2[i];
        acc += g[i] * v1;              // v1 IS the band-pass output
    }
    return acc;
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
    mPhase = w;                                   // expose to the slope graphic

    const int   innerN = mInnerN;
    const float innerK = mInnerK, innerNorm = mInnerNorm;

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
            // Interior peaks are fed the INPUT and summed with the band
            // output: they are inside [fL, fH], so the series path would pass
            // them anyway — this just adds the resonant emphasis there.
            float y = v2 + w * q;
            if (innerN) y = (y + innerBank(x, innerN, mInnerA1, mInnerA2, mInnerA3,
                                           mInnerG, innerK, mInnerIc1L, mInnerIc2L))
                            * innerNorm;
            outL[s] = softLimit(sanitize(y)) * lv;
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
            float y = v2 - w * q;                       // opposite sign → L↔R phase
            if (innerN) y = (y + innerBank(x, innerN, mInnerA1, mInnerA2, mInnerA3,
                                           mInnerG, innerK, mInnerIc1R, mInnerIc2R))
                            * innerNorm;
            outR[s] = softLimit(sanitize(y)) * lv;
        }
        lv += lvStep;
    }

    mIc1L_L = ic1Ll; mIc2L_L = ic2Ll; mIc1H_L = ic1Hl; mIc2H_L = ic2Hl;
    mIc1L_R = ic1Lr; mIc2L_R = ic2Lr; mIc1H_R = ic1Hr; mIc2H_R = ic2Hr;
    mLastLevel = level;
}

} // namespace vbf
