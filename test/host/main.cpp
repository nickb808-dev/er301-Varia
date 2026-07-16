// Variable Bandwidth Filter host verification.  Build:
//   g++ -std=c++17 -O2 -ffast-math -Dprivate=public -Itest/host -Isrc \
//       src/VariableBW.cpp test/host/main.cpp -o t
//
// Modes:
//   resp <fc> <bw> <res>   report peak freq, peak dB, -3 dB band, Q
//   center                 peak tracks Freq across the range
//   width                  -3 dB bandwidth grows with Bandwidth
//   reson                  peak grows with Resonance; full res self-oscillates (bounded)
//   stereo                 L/R identical, finite
//   stab                   full res + wide band + noise → finite, bounded
//   nan                    NaN input → finite output
#include "VariableBW.h"
#include "filter_response.h"
#include <hal/fft.h>
#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <cmath>
#include <vector>

using namespace vbf;
static void fill(od::Port &p, float v) { for (int i = 0; i < FRAMELENGTH; ++i) p.buffer()[i] = v; }

static void measure(float fc, float bw, float res, int L, std::vector<double> &H)
{
    VariableBW d; const float A = 0.001f;
    std::vector<float> ir(size_t(L), 0.0f);
    int got = 0; long blk = 0;
    while (got < L) {
        fill(d.mFreqIn, fc); fill(d.mBandwidthIn, bw); fill(d.mResonanceIn, res); fill(d.mLevelIn, 1.0f);
        for (int s = 0; s < FRAMELENGTH; ++s) { d.mInL.buffer()[s] = (blk==0&&s==0)?A:0.0f; d.mInR.buffer()[s]=0.0f; }
        d.process();
        for (int s = 0; s < FRAMELENGTH && got < L; ++s) ir[got++] = d.mOutL.buffer()[s];
        ++blk;
    }
    handle_rfft_t fft = RFFT_allocate(L);
    std::vector<complex_float_t> spec(size_t(L / 2 + 1));
    RFFT_forward(spec.data(), ir.data(), fft);
    RFFT_destroy(fft);
    H.assign(size_t(L / 2 + 1), -200.0);
    for (int k = 0; k <= L / 2; ++k) {
        const double m = sqrt(double(spec[k].r)*spec[k].r + double(spec[k].i)*spec[k].i) / A;
        H[size_t(k)] = 20.0 * log10(m > 1e-12 ? m : 1e-12);
    }
}
static double binHz(int k, int L) { return double(k) * kSampleRate / double(L); }
static void analyze(const std::vector<double> &H, int L, double &pkHz, double &pkDb, double &loHz, double &hiHz)
{
    int pk = 1; for (int k = 1; k <= L/2; ++k) if (H[k] > H[pk]) pk = k;
    pkHz = binHz(pk, L); pkDb = H[pk];
    int lo = pk; while (lo > 1 && H[lo] > pkDb - 3.0) --lo;
    int hi = pk; while (hi < L/2 && H[hi] > pkDb - 3.0) ++hi;
    loHz = binHz(lo, L); hiHz = binHz(hi, L);
}

int main(int argc, char **argv)
{
    if (argc < 2) { fprintf(stderr, "mode?\n"); return 2; }
    const int L = 32768;

    if (!strcmp(argv[1], "resp")) {
        const float fc = argc>2?atof(argv[2]):1000, bw = argc>3?atof(argv[3]):0.3f, res = argc>4?atof(argv[4]):0.4f;
        std::vector<double> H; measure(fc, bw, res, L, H);
        double pk,pd,lo,hi; analyze(H,L,pk,pd,lo,hi);
        printf("resp fc=%.0f bw=%.2f res=%.2f: peak=%.0f Hz @%+.1f dB  -3dB=[%.0f..%.0f] Q≈%.1f\n",
               fc,bw,res,pk,pd,lo,hi, pk/(hi-lo+1e-9));
        return 0;
    }

    if (!strcmp(argv[1], "center")) {
        // At low resonance the band is a single rounded peak at the geometric
        // centre (√(fL·fH) = fc), so the argmax tracks Freq directly.  (The
        // dual resonant edges are checked separately in `reson`.)
        bool ok = true;
        printf("peak vs Freq (bw=0.1, res=0.05 — single-peak regime):\n");
        for (float fc : {150.f, 400.f, 1000.f, 3000.f, 7000.f}) {
            std::vector<double> H; measure(fc, 0.1f, 0.05f, L, H);
            double pk,pd,lo,hi; analyze(H,L,pk,pd,lo,hi);
            const bool near = fabs(pk-fc) < 0.20*fc;   // within 20%
            if (!near) ok = false;
            printf("  fc=%-5.0f → peak=%-6.0f Hz  %s\n", fc, pk, near?"":"(off)");
        }
        printf("  %s\n", ok?"PASS (peak tracks Freq)":"FAIL");
        return ok?0:1;
    }

    if (!strcmp(argv[1], "width")) {
        printf("−3 dB bandwidth vs Bandwidth (fc=1000, res=0.05 — single-peak):\n");
        double prev = 0; bool grows = true;
        for (float bw : {0.05f, 0.3f, 0.6f, 0.95f}) {
            std::vector<double> H; measure(1000.f, bw, 0.05f, L, H);
            double pk,pd,lo,hi; analyze(H,L,pk,pd,lo,hi);
            const double width = hi - lo;
            if (width < prev - 1.0) grows = false;
            printf("  bw=%.2f → band=[%.0f..%.0f] width=%.0f Hz\n", bw, lo, hi, width);
            prev = width;
        }
        printf("  %s\n", grows?"PASS (width grows with Bandwidth)":"FAIL");
        return grows?0:1;
    }

    if (!strcmp(argv[1], "reson")) {
        printf("peak dB vs Resonance (fc=800, bw=0.2):\n");
        double prev = -1e9; bool grows = true;
        for (float res : {0.0f, 0.4f, 0.7f, 1.0f}) {
            std::vector<double> H; measure(800.f, 0.2f, res, L, H);
            double pk,pd,lo,hi; analyze(H,L,pk,pd,lo,hi);
            if (pd < prev - 0.5) grows = false;
            printf("  res=%.1f → peak %+.1f dB @ %.0f Hz\n", res, pd, pk);
            prev = pd;
        }
        // self-oscillation: full res, single ping, then silence → sustained + bounded
        VariableBW d; long nf=0; float peak=0; double energy=0;
        for (int b=0;b<8000;++b){ fill(d.mFreqIn,800.f); fill(d.mBandwidthIn,0.1f); fill(d.mResonanceIn,1.0f); fill(d.mLevelIn,1.0f);
            for(int s=0;s<FRAMELENGTH;++s){ d.mInL.buffer()[s]=(b==0&&s==0)?0.5f:0.0f; d.mInR.buffer()[s]=0.0f; }
            d.process();
            for(int s=0;s<FRAMELENGTH;++s){ float y=d.mOutL.buffer()[s]; if(!std::isfinite(y))nf++; else { if(fabsf(y)>peak)peak=fabsf(y); if(b>4000)energy+=double(y)*y; } } }
        const bool ok = grows && nf==0 && peak<=1.001f && energy>1.0;
        printf("  selfOsc: peak=%.3f sustainE=%.1f nf=%ld  %s\n", peak, energy, nf, ok?"PASS":"FAIL");
        return ok?0:1;
    }

    if (!strcmp(argv[1], "stereo")) {
        VariableBW d; double maxDiff=0; long nf=0;
        for(int b=0;b<3000;++b){ fill(d.mFreqIn,1200.f); fill(d.mBandwidthIn,0.4f); fill(d.mResonanceIn,0.6f); fill(d.mLevelIn,1.0f);
            for(int s=0;s<FRAMELENGTH;++s){ float x=0.3f*sinf(2*3.14159265f*500*(b*FRAMELENGTH+s)/48000.f); d.mInL.buffer()[s]=x; d.mInR.buffer()[s]=x; }
            d.process();
            for(int s=0;s<FRAMELENGTH;++s){ if(!std::isfinite(d.mOutL.buffer()[s])||!std::isfinite(d.mOutR.buffer()[s]))nf++; maxDiff=std::max(maxDiff,double(fabsf(d.mOutL.buffer()[s]-d.mOutR.buffer()[s]))); } }
        printf("stereo L/R diff=%.2e nonFinite=%ld  %s\n", maxDiff, nf, (maxDiff<1e-6&&nf==0)?"PASS":"FAIL");
        return (maxDiff<1e-6&&nf==0)?0:1;
    }

    if (!strcmp(argv[1], "stab")) {
        VariableBW d; uint32_t rng=0x1234567u; long nf=0; float peak=0;
        for(int b=0;b<20000;++b){
            // sweep freq + full resonance + wide band, noise input
            float fc = 200.f + 6000.f*(0.5f+0.5f*sinf(b*0.0007f));
            fill(d.mFreqIn,fc); fill(d.mBandwidthIn,0.9f); fill(d.mResonanceIn,1.0f); fill(d.mLevelIn,1.0f);
            for(int s=0;s<FRAMELENGTH;++s){ rng^=rng<<13;rng^=rng>>17;rng^=rng<<5; float x=(float(rng&0xFFFF)/32768.f-1.f)*0.5f; d.mInL.buffer()[s]=x; d.mInR.buffer()[s]=x; }
            d.process();
            for(int s=0;s<FRAMELENGTH;++s){ float a=d.mOutL.buffer()[s],b2=d.mOutR.buffer()[s]; if(!std::isfinite(a)||!std::isfinite(b2))nf++; peak=std::max(peak,std::max(fabsf(a),fabsf(b2))); } }
        printf("stab (sweep, full res, wide, noise): nonFinite=%ld peak=%.3f  %s\n", nf, peak, (nf==0&&peak<=1.001f)?"PASS":"FAIL");
        return (nf==0&&peak<=1.001f)?0:1;
    }

    if (!strcmp(argv[1], "nan")) {
        VariableBW d; long nf=0;
        for(int b=0;b<3000;++b){ const bool burst=(b>=500&&b<510);
            fill(d.mFreqIn,1000.f); fill(d.mBandwidthIn,0.3f); fill(d.mResonanceIn,0.8f); fill(d.mLevelIn,1.0f);
            for(int s=0;s<FRAMELENGTH;++s){ float x=0.3f*sinf(2*3.14159265f*440*(b*FRAMELENGTH+s)/48000.f); if(burst&&(s&3)==0)x=0.0f/0.0f; d.mInL.buffer()[s]=x; d.mInR.buffer()[s]=x; }
            d.process();
            for(int s=0;s<FRAMELENGTH;++s) if(!std::isfinite(d.mOutL.buffer()[s]))nf++; }
        printf("nan: nonFinite=%ld  %s\n", nf, nf?"FAIL":"PASS");
        return nf?1:0;
    }

    if (!strcmp(argv[1], "curve")) {
        // The graphic draws varBpMag() from the DSP's live fL/fH/k.  Verify the
        // curve has the right SHAPE: finite, band-limited (rolls off outside the
        // band), centred at fc = √(fL·fH), and dual resonant EDGES at high res.
        bool ok = true;
        for (float res : {0.1f, 0.5f, 0.85f}) {
            const float fc = 1000.f, bw = 0.3f;
            VariableBW d;
            fill(d.mFreqIn, fc); fill(d.mBandwidthIn, bw); fill(d.mResonanceIn, res); fill(d.mLevelIn, 1.0f);
            for (int s = 0; s < FRAMELENGTH; ++s) { d.mInL.buffer()[s]=0; d.mInR.buffer()[s]=0; }
            d.process();
            const float fL = d.getFL(), fH = d.getFH(), k = d.getK();

            long nf = 0;
            for (double f = 20; f < 20000; f *= 1.01)
                if (!std::isfinite(varBpMag((float)f, fL, fH, k))) nf++;

            const float mL = varBpMag(fL, fL, fH, k);
            const float mH = varBpMag(fH, fL, fH, k);
            const float mC = varBpMag(sqrtf(fL*fH), fL, fH, k);
            const float below = varBpMag(fL * 0.15f, fL, fH, k);   // well below the band
            const float above = varBpMag(fH * 6.5f,  fL, fH, k);   // well above
            const float inBand = std::max(mC, std::max(mL, mH));

            const bool bandlimited = inBand > below * 3.0f && inBand > above * 3.0f;
            const bool edges = (res < 0.3f) ? true : (mL > mC * 1.2f && mH > mC * 1.2f);
            const bool good = nf == 0 && bandlimited && edges;
            if (!good) ok = false;
            printf("curve res=%.2f: fL=%.0f fH=%.0f k=%.3f  mL=%.2f mC=%.2f mH=%.2f  "
                   "outside(%.3f/%.3f)  bandlim=%d edges=%d nf=%ld %s\n",
                   res, fL, fH, k, mL, mC, mH, below, above, bandlimited?1:0, edges?1:0, nf, good?"":"(off)");
        }
        printf("  %s\n", ok?"PASS (curve band-limited, centred, dual edges, finite)":"FAIL");
        return ok?0:1;
    }

    if (!strcmp(argv[1], "phase")) {
        // Stereo peak phase: same mono source to L and R, resonant edges ringing.
        // phase=0 → L==R (identity); |phase|↑ → L/R decorrelate; phase=±1 → ≈90°
        // relative (correlation → 0).  Finite and bounded throughout.
        auto meas = [](float w, double &corr, float &peak, long &nf) {
            VariableBW d; double sLR=0,sLL=0,sRR=0; long n=0; peak=0; nf=0;
            static uint32_t rng=12321;
            for (int b=0;b<3000;++b){
                fill(d.mFreqIn,600.f); fill(d.mBandwidthIn,0.4f);
                fill(d.mResonanceIn,0.85f); fill(d.mLevelIn,1.0f); fill(d.mPhaseIn,w);
                for(int s=0;s<FRAMELENGTH;++s){ rng^=rng<<13;rng^=rng>>17;rng^=rng<<5;
                    float x=0.3f*(float(rng&0xFFFF)/32768.f-1.f); d.mInL.buffer()[s]=x; d.mInR.buffer()[s]=x; }
                d.process();
                if(b>1000) for(int s=0;s<FRAMELENGTH;++s){ float L=d.mOutL.buffer()[s],R=d.mOutR.buffer()[s];
                    if(!std::isfinite(L)||!std::isfinite(R))nf++;
                    sLR+=double(L)*R; sLL+=double(L)*L; sRR+=double(R)*R;
                    float a=fabsf(L)>fabsf(R)?fabsf(L):fabsf(R); if(a>peak)peak=a; n++; }
            }
            corr = (sLL>0&&sRR>0)? sLR/sqrt(sLL*sRR) : 1.0;
        };
        double c0,c5,c1; float p0,p5,p1; long nf0,nf5,nf1;
        meas(0.0f,c0,p0,nf0); meas(0.5f,c5,p5,nf5); meas(1.0f,c1,p1,nf1);
        const bool ok = c0>0.999 && c5<0.85 && c5<c0-0.1 && c1<0.2 &&
                        nf0==0&&nf5==0&&nf1==0 && p0<=1.001f&&p5<=1.001f&&p1<=1.001f;
        printf("phase: corr @0=%+.3f @0.5=%+.3f @1=%+.3f  peaks=%.3f/%.3f/%.3f  nf=%ld  %s\n",
               c0,c5,c1,p0,p5,p1,nf0+nf5+nf1, ok?"PASS":"FAIL");
        return ok?0:1;
    }

    fprintf(stderr, "unknown mode\n");
    return 2;
}
