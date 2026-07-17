# Varia — Variable Bandwidth Filter

A resonant take on the Serge Variable Bandwidth Filter (VCF2). A bandpass
whose **centre frequency** and **bandwidth** are independently voltage-controllable,
with an added **resonance** that emphasises the two band edges and, at the top of
its range, self-oscillates. Stereo in, stereo out.

Package name on the ER-301: **Varia** · folder on disk: `variable-bw/` ·
internal C++ namespace: `vbf`.

---

## What it does

The classic Serge VCF2 is a flat, non-resonant bandpass with independently
sweepable low and high edges. Varia keeps that behaviour and adds resonance:

- **Freq** — centre frequency in Hz. V/oct trackable (`oscFreq` map + octave
  scaling), so you can play it as a pitched voice.
- **Bandwidth** — `0` = narrow (tight band) … `1` = wide (~6 octaves). The band
  edges spread symmetrically around the centre: `fL = fc·2^(−bw·3)`,
  `fH = fc·2^(+bw·3)`, geometric centre stays at `fc`.
- **Resonance** — `0` = flat/gentle … `1` = self-oscillation. As resonance rises,
  a peak grows at **each** band edge; as bandwidth narrows the two peaks merge
  into one tall high-Q peak. At the very top the filter runs undamped and
  self-oscillates (playable as a sine voice), held bounded by a soft-limiter.
- **Level** — output gain (unity = 1).

### Live filter-slope scope

The unit's expanded view leads with a **phosphor slope scope** (v0.2.0) that
draws the live magnitude response — X = log frequency (20 Hz…20 kHz),
Y = dB — and **trails** as you move Freq / Bandwidth / Resonance. You watch the
band slide, widen, and grow its resonant edges in real time. It's modeled
directly on Dirac's animated "triangles" graphic and uses the same width and
render pattern, so the code reads the same across the two packages.

---

## How it works (DSP)

Two [Cytomic](https://cytomic.com/technical-papers/) state-variable filters in
**series** per channel:

```
input → highpass @ fL → lowpass @ fH → output
```

That series HP→LP puts a resonant peak at each edge when the damping `k = 1/Q` is
shared between the stages — which is exactly the "resonant band edges" behaviour.
(A difference-of-lowpasses bandpass would have inverted the lower peak into a
notch instead.) Resonance maps exponentially to Q (0.5 → ~1500); `res = 1.0`
forces `k = 0` for genuine self-oscillation.

### am335x trig-safe

The ER-301 runs a TI am335x (Cortex-A8). Runtime `sinf`/`cosf` called from a
package `.so` miscompute on that part (a package→firmware libm call-boundary
bug). Because Varia's cutoff is variable, `g = tan(π·f/fs)` must recompute every
block — so the DSP uses `src/svf_math.h`, a libm-free `psin/pcos/ptan/pexp2`
(pure arithmetic, ≈3.7e-6 / 8e-5 accuracy vs libm). The slope graphic is likewise
trig-free: the analytic magnitude is mults/divs + `sqrtf` (a hardware
instruction), the log-frequency axis uses `pexp2`, and the dB axis/edge ticks use
`logf` (unaffected by the bug). No `sinf`/`cosf`/`tanf` anywhere.

Both input and output are NaN/Inf sanitised, and the output soft-limiter
(transparent below ±0.9) keeps resonant and self-oscillating signals bounded.

---

## Layout

```
variable-bw/  (package name: varia)
  Makefile / Dockerfile
  src/
    VariableBW.h / .cpp   od::Object — 6 inlets, stereo; series HP→LP + resonance
    svf_math.h            libm-free sin/cos/tan/exp2 (variable cutoff, am335x-safe)
    filter_response.h     analytic varBpMag(f, fL, fH, k) for the scope (sqrtf only)
    SlopeGraphic.h        header-only phosphor slope scope (Dirac-style)
    libvaria.swig         SWIG interface (module varia_libvaria)
    compat.cpp / compat_swig.h
  assets/
    toc.lua               package manifest
    VariableBW.lua        unit wrapper (slope + Freq/Bandwidth/Resonance/Level)
    SlopeView.lua         display-only ViewControl hosting the slope scope
  test/host/              host simulation harness + od stubs
```

---

## Build

Requires the ER-301 SDK and Docker (cross-compiles to am335x). From the
`variable-bw/` folder:

```bash
make docker-image                     # first time only (shared image)
make swig-docker  ER301_SDK=~/er-301  # generate the SWIG Lua wrapper
make docker-build ER301_SDK=~/er-301  # cross-compile the .so
make pkg                              # → build/am335x/varia-0.2.0.pkg
```

Copy the resulting `.pkg` to the ER-301's SD card (`ER-301/packages/`) and load
it from the unit browser.

### Host tests

The DSP and the scope's curve math are verified off-device (graphics only render
on the emulator/hardware, but the sampled magnitude is host-testable):

```bash
g++ -std=c++17 -O2 -ffast-math -Dprivate=public -Itest/host -Isrc \
    src/VariableBW.cpp test/host/main.cpp -o t
./t curve      # curve band-limited, centred at √(fL·fH), dual edges, finite
./t center     # peak tracks Freq
./t width      # −3 dB bandwidth grows with Bandwidth
./t reson      # edge peak grows; res=1 self-oscillates (bounded)
./t stereo     # L/R bit-identical
./t stab       # sweep + full res + wide + noise → finite, bounded
./t nan        # NaN input → finite output
```

---

## Status

**v0.2.0** — host-verified. Pending: on-device confirmation of the slope scope
(the curve math is verified; the pixels only run on emulator/hardware) and a
hardware pass on V/oct tracking, the bandwidth sweep, and self-oscillation feel.

## Credits

Modeled after the Serge Modular Variable Bandwidth Filter (RS-VCF2). Filter core
follows Andrew Simper's (Cytomic) state-variable filter design. Built for the
Orthogonal Devices ER-301.
