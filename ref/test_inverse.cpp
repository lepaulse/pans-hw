// SPDX-License-Identifier: Apache-2.0
// test_inverse: the exhaustive check of the hardware's decoder algorithm decode_hw against decode.
// Specification: SPEC.md §6. It also checks the intervals, the round trip, the division constants and C itself.
// Exit status 0 iff every check has 0 errors.
//
//   make inverse-<cfg>    builds it into build/<cfg>/ and runs it (every config: make inverse); by hand:
//   g++ -O2 -std=c++17 -fopenmp -Wall -Wextra -Werror -Iref <config -D flags> ref/test_inverse.cpp -o test_inverse
//   test_inverse [scale_stride]
#include <cstdio>
#include <cstdlib>
#include <vector>

#include "pans1_config.h"

using namespace pans1hw;

int main(int argc, char** argv) {
    const Config c = build_config();
    const long stride = argc > 1 ? std::atol(argv[1]) : 1;
    const long contract = contract_violations(c);
    const DivMagic dm = div_magic(c);
    std::printf("P=%d SW=%d E=%d vf=%d T(0)=%lld (<= 2^(P-1) - E = %lld), scale mantissa %d bits; division shift %d; "
                "contract %ld (version %d); alphabet [%lld, %lld]%s\n", c.precision, c.symbol_width, c.escape_slots,
                c.v_frac_bits, (long long)tail_max(c), (long long)(c.half() - c.escape_slots), c.scale_man_width,
                dm.shift, contract, CONTRACT_VERSION, (long long)c.symbol_min(), (long long)c.symbol_max(),
                c.symmetric_alphabet ? " (symmetric)" : "");
    if (dm.shift < 0) return 1;
    long div_bad = 0;
#pragma omp parallel for reduction(+ : div_bad)
    for (int man = 0; man < (1 << c.scale_man_width); ++man) {
        const i64 km = (i64(1) << c.scale_man_width) + man, d = 2 * km;
        for (i64 n = 0; n <= dm.n_max; ++n) div_bad += div_q(dm, n, dm.m[man]) != (n + km) / d;  // the decoder's use
    }
    std::printf("division (shift %d, %d low columns dropped, %ld of %d partial-product bits kept): %ld errors over every "
                "mantissa and numerator in [0, %lld]\n", dm.shift, dm.drop, dm.pp_kept, mag_width(c) * dm.magic_width,
                div_bad, (long long)dm.n_max);
    // Every R' >= 2^MAG_WIDTH decodes beyond the alphabet, for every mantissa.
    long clamp_bad = 0;
    for (int man = 0; man < (1 << c.scale_man_width); ++man) {
        const i64 km = (i64(1) << c.scale_man_width) + man;
        if (((i64(1) << mag_width(c)) + km) / (2 * km) <= std::max(-c.symbol_min(), c.symbol_max())) ++clamp_bad;
    }
    // The threshold's closed form against its definition (bisection on T), for every z' of both halves.
    long thr_bad = 0;
    for (i64 zp = 0; zp < c.half(); ++zp) thr_bad += a_plus_hw(c, zp) != a_plus_def(c, zp);
    std::printf("clamp: %ld mantissas decode inside the alphabet at 2^MAG_WIDTH; threshold: %ld of %lld z' differ "
                "between a_plus_hw and a_plus_def\n", clamp_bad, thr_bad, (long long)c.half());
    const long n_scales = long(1) << (c.scale_exp_width + c.scale_man_width);
    const i64 mp = c.coded_slots(), n_sym = c.symbol_max() - c.symbol_min() + 1;
    long mismatch = 0, owner_bad = 0, roundtrip = 0, slots = 0, coded = 0;
    long tile_bad = 0, enc_bad = 0, mirror_bad = 0, beyond_scales = 0, ends_gained = 0, outside_bad = 0;
    i64 beyond_slots = 0;
    const bool symmetric = c.symbol_input_min() < c.symbol_min();  // the input -2^(S-1) lies outside the alphabet
#pragma omp parallel for schedule(dynamic, 16) reduction(+ : mismatch, owner_bad, roundtrip, slots, coded, tile_bad, \
                                                            enc_bad, mirror_bad, beyond_scales, ends_gained, \
                                                            beyond_slots, outside_bad)
    for (long code = 0; code < n_scales; code += stride) {
        const Scale s{unsigned(code >> c.scale_man_width), unsigned(code & ((1 << c.scale_man_width) - 1))};
        const Slope k = slope_of(c, s);
        // For every slot, decode_hw equals decode and the decoded interval holds the slot.
        for (i64 slot = 0; slot < c.slots(); ++slot) {
            const DecOut a = decode(c, s, slot), b = decode_hw(c, dm, s, slot);
            ++slots;
            if (a.symbol != b.symbol || a.start != b.start || a.freq != b.freq || a.escape != b.escape) {
                if (++mismatch <= 5)
#pragma omp critical
                    std::printf("MISMATCH exp %u man %u slot %lld: decode %lld [%lld, +%lld) hw %lld [%lld, +%lld)\n",
                                s.exp, s.man, (long long)slot, (long long)a.symbol, (long long)a.start,
                                (long long)a.freq, (long long)b.symbol, (long long)b.start, (long long)b.freq);
            }
            const bool owned = slot < mp ? !b.escape && b.freq >= 1 && b.start <= slot && slot < b.start + b.freq &&
                                               b.symbol >= c.symbol_min() && b.symbol <= c.symbol_max()
                                         : b.escape && b.start == mp && b.freq == c.escape_slots;
            if (!owned && ++owner_bad <= 5)
#pragma omp critical
                std::printf("NOT OWNED exp %u man %u slot %lld: hw %lld [%lld, +%lld) escape %d\n", s.exp, s.man,
                            (long long)slot, (long long)b.symbol, (long long)b.start, (long long)b.freq, b.escape);
        }
        // The intervals tile [0, M') in symbol order. encode codes exactly the non-empty ones.
        std::vector<i64> f(n_sym);
        i64 edge = 0;
        for (i64 x = c.symbol_min(); x <= c.symbol_max(); ++x) {
            const i64 lo = interval_lo(c, k, x), hi = interval_hi(c, k, x);
            tile_bad += lo != edge || hi < lo;
            edge = hi;
            f[x - c.symbol_min()] = hi - lo;
            const EncOut e = encode(c, s, x);
            enc_bad += hi > lo ? e.escape || e.start != lo || e.freq != hi - lo
                               : !e.escape || e.start != mp || e.freq != c.escape_slots;
            if (e.escape) continue;
            ++coded;
            for (i64 slot = e.start; slot < e.start + e.freq; ++slot) {
                const DecOut d = decode_hw(c, dm, s, slot);
                if (d.escape || d.symbol != x || d.start != e.start || d.freq != e.freq) ++roundtrip;
            }
        }
        tile_bad += edge != mp;
        // An input outside the alphabet escapes at every scale.
        if (symmetric) {
            const EncOut e = encode(c, s, c.symbol_input_min());
            outside_bad += !e.escape || e.start != mp || e.freq != c.escape_slots;
        }
        // The mirror f(x) = f(-x), with f(MAX) = f(MIN) + f(MIN + 1) in an asymmetric alphabet.
        const auto fq = [&](i64 x) { return f[x - c.symbol_min()]; };
        if (symmetric) {
            for (i64 x = c.symbol_min(); x <= c.symbol_max(); ++x) mirror_bad += fq(x) != fq(-x);
        } else {
            for (i64 x = c.symbol_min() + 2; x <= c.symbol_max() - 1; ++x) mirror_bad += fq(x) != fq(-x);
            mirror_bad += fq(c.symbol_max()) != fq(c.symbol_min()) + fq(c.symbol_min() + 1);
        }
        // What contract version 2 changed at this scale (SPEC.md §6).
        const i64 below = cdf(c, k, c.symbol_min()), above = mp - cdf(c, k, c.symbol_max() + 1);
        beyond_slots += below + above;
        beyond_scales += below + above > 0;
        ends_gained += (below > 0 && cdf(c, k, c.symbol_min() + 1) == below) +
                       (above > 0 && cdf(c, k, c.symbol_max()) == mp - above);
    }
    std::printf("%ld scale codes (stride %ld), %ld slots: %ld decode mismatches, %ld slots outside the interval "
                "decoded for them or decoded outside the alphabet; %ld coded symbols: %ld round-trip failures\n",
                (n_scales + stride - 1) / stride, stride, slots, mismatch, owner_bad, coded, roundtrip);
    std::printf("intervals: %ld tiling errors (MIN's from 0 to MAX's at M', each from the last's end), %ld encode "
                "mismatches, %ld mirror errors (%s); beyond the alphabet (owned by MIN and MAX): %lld slots at %ld "
                "scale codes, %ld end symbols coded there whose own [C(x), C(x + 1)) is empty\n", tile_bad, enc_bad,
                mirror_bad,
                symmetric ? "f(x) = f(-x) for every x" : "f(x) = f(-x) inside, f(MAX) = f(MIN) + f(MIN + 1)",
                (long long)beyond_slots, beyond_scales, ends_gained);
    if (symmetric)
        std::printf("outside the alphabet: the input %lld coded at %ld scale codes (must escape at every one)\n",
                    (long long)c.symbol_input_min(), outside_bad);

    // C against what it approximates: the symmetry C(x) + C(1 - x) = M', the half boundary, the tail (SPEC.md §6).
    long sym_bad = 0, sym_checked = 0, half_bad = 0, tail_bad = 0;
    double worst_lo = 1e9, worst_hi = 0;
    const double lo = std::exp2(-std::ldexp(1.0, -c.v_frac_bits)), hi = 1.0615 * (1.0 + double(c.escape_slots) / c.coded_slots());
#pragma omp parallel for schedule(dynamic, 16) reduction(+ : sym_bad, sym_checked, half_bad, tail_bad) reduction(min : worst_lo) reduction(max : worst_hi)
    for (long code = 0; code < n_scales; code += stride) {
        const Scale s{unsigned(code >> c.scale_man_width), unsigned(code & ((1 << c.scale_man_width) - 1))};
        const Slope k = slope_of(c, s);
        if (cdf(c, k, 0) > c.half() || cdf(c, k, 1) < c.half()) ++half_bad;
        if (k.special) continue;
        const double kabs = std::ldexp((double)k.mant, k.exp2);
        for (i64 x = c.symbol_min(); x <= c.symbol_max() + 1; ++x) {
            const i64 cx = cdf(c, k, x);
            const i64 xm = 1 - x;
            if (x <= 0 && xm <= c.symbol_max() + 1) {
                ++sym_checked;
                if (cx + cdf(c, k, xm) != c.coded_slots()) ++sym_bad;
            }
            const double a = kabs * std::fabs((double)x - 0.5);
            const double t_exact = 0.5 * c.coded_slots() * std::exp2(-a);
            const i64 t = std::min(cx, c.coded_slots() - cx);
            if (t_exact < 4.0) continue;  // below a few slots: rounding dominates
            const double ratio = (double)t / t_exact;
            if (t_exact >= 256.0) {  // the reported band: where one slot of rounding is below 0.4%
                worst_lo = std::min(worst_lo, ratio);
                worst_hi = std::max(worst_hi, ratio);
            }
            if ((double)t < lo * t_exact - 1.0 || (double)t > hi * t_exact + 1.0) ++tail_bad;
        }
    }
    std::printf("C symmetry: %ld of %ld pairs violate C(x) + C(1 - x) == M'; half boundary: %ld scales violate "
                "C(0) <= 2^(P-1) <= C(1)\n", sym_bad, sym_checked, half_bad);
    if (worst_hi == 0)
        std::printf("C tail vs exact Laplace: bound [%.4f, %.4f] +- 1 slot: %ld outside (no tail >= 256 slots)\n", lo,
                    hi, tail_bad);
    else
        std::printf("C tail vs exact Laplace: ratio in [%.4f, %.4f] (tails >= 256 slots), bound [%.4f, %.4f] +- 1 "
                    "slot: %ld outside\n", worst_lo, worst_hi, lo, hi, tail_bad);
    return (contract || div_bad || clamp_bad || thr_bad || mismatch || owner_bad || roundtrip || tile_bad || enc_bad ||
            mirror_bad || outside_bad || sym_bad || half_bad || tail_bad)
               ? 1
               : 0;
}
