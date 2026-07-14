/* svf_math.h — libm-free sin/cos/tan/exp2 for the Variable Bandwidth Filter.
 *
 * WHY (am335x): runtime sinf/cosf (and, by extension, tanf) called from a
 * package .so miscompute on the ER-301's am335x — a package→firmware libm
 * call-boundary bug (stolmine/habitat feedback_package_trig_lut; sinf/cosf
 * confirmed bad, logf/expf not flagged, emulator unaffected).  Unlike a
 * fixed-frequency filter, a VARIABLE-cutoff filter must recompute
 * g = tan(π·f/fs) every block, so these run at control rate — hence pure-
 * arithmetic versions that reference NO libm symbol.
 *
 * All functions assume their documented input ranges (non-negative, bounded)
 * and avoid floorf (also libm) via int casts on non-negative values.
 *   psin/pcos : ≈3.5e-6 over the circle
 *   ptan      : psin/pcos, valid for x in [0, ~0.49π) (cutoff < ~0.49·fs)
 *   pexp2     : x ≥ 0 (used for the bandwidth octave half-spread), ≈1e-4
 */
#pragma once

namespace vbfmath {

static constexpr float kPi     = 3.14159265358979f;
static constexpr float kTwoPi  = 6.28318530717959f;
static constexpr float kHalfPi = 1.57079632679490f;

// sine for any x ≥ 0 — degree-9 odd Taylor after range reduction, no libm/floorf.
static inline float psin(float x)
{
    x -= kTwoPi * float(int(x * (1.0f / kTwoPi)));   // → [0, 2π)  (x ≥ 0)
    if (x < 0.0f) x += kTwoPi;
    if (x > kPi) x -= kTwoPi;                         // → [-π, π]
    if (x > kHalfPi) x = kPi - x;                     // → [-π/2, π/2]
    else if (x < -kHalfPi) x = -kPi - x;
    const float x2 = x * x;
    return x * (1.0f + x2 * (-0.166666667f + x2 * (0.00833333333f
             + x2 * (-0.000198412698f + x2 * 0.0000027557319f))));
}

static inline float pcos(float x) { return psin(x + kHalfPi); }

// tan for x in [0, ~0.49π): pcos stays comfortably positive there.
static inline float ptan(float x)
{
    const float c = pcos(x);
    return psin(x) / (c > 1e-6f ? c : 1e-6f);
}

// 2^x for x ≥ 0 (bandwidth octave half-spread; x is small, ≲ 4).
static inline float pexp2(float x)
{
    if (x < 0.0f) x = 0.0f;
    const int   i = int(x);           // integer octaves (x ≥ 0 → trunc = floor)
    const float f = x - float(i);     // fractional part in [0,1)
    // 2^f — Taylor of exp(f·ln2), coefficients (ln2)^n/n!.
    float m = 1.0f + f * (0.693147181f + f * (0.240226507f + f * (0.0555041087f
            + f * (0.00961812910f + f * 0.00133335581f))));
    for (int k = 0; k < i; ++k) m *= 2.0f;   // 2^i (i small)
    return m;
}

} // namespace vbfmath
