// SPDX-License-Identifier: Apache-2.0
// pans1_ref.h: the bit-exact model of the cores pans_enc_core and pans_dec_core, the normative reference.
// The RTL implements it in rtl/pans_pkg_functions.svh, which names each counterpart.
// Specification: SPEC.md §1 and §2. Build identity: SPEC.md §4.
// decode_hw is the hardware's algorithm. ref/test_inverse.cpp checks it against decode on every scale code and slot.
#pragma once
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <map>
#include <mutex>
#include <vector>

namespace pans1hw {

using i64 = int64_t;
using i128 = __int128;

// ---- Configuration ------------------------------------------------------------------------------

constexpr int BYTE_BITS = 8;  // the stream's words and stored states are whole bytes (STREAM.md)

// The scale input b and its table, both fixed by the contract (SPEC.md §1).
constexpr int SCALE_B_FRAC = 8;  // fraction bits of b
constexpr int SCALE_B_K = 4;     // index bits below b's leading one, 16 buckets per octave

inline int bits_for(i64 v) {  // the bits of v >= 0 (1 for v = 0)
    int w = 1;
    while ((i64(1) << w) <= v) ++w;
    return w;
}

struct Config {
    int precision;        // M = 2^precision slots
    int escape_slots;     // E >= 1
    int symbol_width;     // signed symbols, <= precision
    int scale_exp_width;  // scale input: exponent bits
    int scale_man_width;  // scale input: mantissa bits
    int v_frac_bits;      // fraction bits of a (and of the threshold)
    const char* units = "";  // name of the unit set (reports only)
    bool symmetric_alphabet = false;  // MIN = -MAX, set by pans1_config.h
    int b_frac = SCALE_B_FRAC;        // fraction bits of the scale input b
    int b_k = SCALE_B_K;              // the scale table's index bits below b's leading one

    i64 slots() const { return i64(1) << precision; }
    i64 coded_slots() const { return slots() - escape_slots; }  // M' = M - E
    i64 half() const { return i64(1) << (precision - 1); }       // the half boundary of the slots

    // The symbol port takes [symbol_input_min, symbol_max]. The alphabet is [symbol_min, symbol_max].
    i64 symbol_input_min() const { return -(i64(1) << (symbol_width - 1)); }
    i64 symbol_min() const { return symmetric_alphabet ? -symbol_max() : symbol_input_min(); }
    i64 symbol_max() const { return (i64(1) << (symbol_width - 1)) - 1; }
    int v_sat() const { return precision; }  // C saturates at a >= v_sat
    i64 v_lim() const { return i64(v_sat()) << v_frac_bits; }
    int scale_bias() const { return (1 << (scale_exp_width - 1)) - 1; }
    unsigned scale_exp_inf() const { return (1u << scale_exp_width) - 1; }
    // The stream's widths (STREAM.md §1.1).
    int word_width() const { return BYTE_BITS * ((precision + BYTE_BITS - 1) / BYTE_BITS); }
    int state_width() const { return precision + word_width(); }
    int state_bytes() const { return (state_width() + BYTE_BITS - 1) / BYTE_BITS; }
    // The scale input's widths (pans_pkg B_WIDTH, B_LEAD_BITS, SCALE_INDEX_BITS, SCALE_TABLE_SIZE).
    int b_width() const { return symbol_width - 1 + b_frac; }
    i64 b_max() const { return (i64(1) << b_width()) - 1; }
    int b_lead_bits() const { return bits_for(b_width() - 1); }  // lead in [0, B_WIDTH - 1]
    int scale_index_bits() const { return b_lead_bits() + b_k; }
    i64 scale_table_size() const { return i64(b_width()) << b_k; }
};

// The scale exponent's width EW (SPEC.md §4).
inline int scale_exp_width_for(int precision, int symbol_width) {
    int lower = 0;  // ceil(log2(2 (P + 2)))
    while ((1 << lower) < 2 * (precision + 2)) ++lower;
    const int bias_min = std::max(lower + 2, symbol_width - 2);
    int w = 2;
    while ((1 << (w - 1)) - 1 < bias_min) ++w;
    return w;
}

inline Config make_config(int precision, int symbol_width, int vf, int scale_man_bits) {
    return Config{precision, 1, symbol_width, scale_exp_width_for(precision, symbol_width), scale_man_bits, vf};
}

// ---- Scale input --------------------------------------------------------------------------------

struct Scale {
    unsigned exp = 0;  // 0 and all ones are the special scales
    unsigned man = 0;
};

// The b whose discretized Laplace has standard deviation sigma (the discrete variance match).
inline double laplace_b(double sigma) {
    double lo = 0.0, hi = 1.0;  // rho = e^(-1/b)
    for (int i = 0; i < 200; ++i) {
        const double rho = 0.5 * (lo + hi);
        const double var = std::sqrt(rho) * (1.0 + rho) / ((1.0 - rho) * (1.0 - rho));
        (var < sigma * sigma ? lo : hi) = rho;
    }
    return -1.0 / std::log(0.5 * (lo + hi));
}

// The scale code of the slope k = 1 / (b ln2), rounded to nearest (SPEC.md §1).
inline Scale scale_from_b(const Config& c, double b) {
    const double k = 1.0 / (b * std::log(2.0));
    if (!(k > 0.0)) return {0, 0};
    if (std::isinf(k)) return {c.scale_exp_inf(), 0};
    int e;
    const double m = std::frexp(k, &e);  // k = m 2^e, m in [1/2, 1)
    i64 man = (i64)std::nearbyint((2.0 * m - 1.0) * std::ldexp(1.0, c.scale_man_width));
    int exp = e - 1;
    if (man == (i64(1) << c.scale_man_width)) {
        man = 0;
        ++exp;
    }
    const int biased = exp + c.scale_bias();
    if (biased <= 0) return {0, 0};
    if (biased >= (int)c.scale_exp_inf()) return {c.scale_exp_inf(), 0};
    return {(unsigned)biased, (unsigned)man};
}

// The scale code of a sigma, scale_from_b of laplace_b(sigma).
inline Scale scale_from_sigma(const Config& c, double sigma) {
    if (std::isnan(sigma) || std::isinf(sigma)) return {0, 0};
    if (!(sigma > 0.0)) return {c.scale_exp_inf(), 0};
    return scale_from_b(c, laplace_b(sigma));
}

// ---- The scale input b and the scale table (contract version 4) ---------------------------------
// The wire value round(b 2^B_FRAC) of a Laplace scale b, saturated to [0, b_max].
inline i64 b_fixed_of(const Config& c, double b) {
    const double v = std::nearbyint(std::ldexp(b, c.b_frac));
    if (!(v > 0.0)) return 0;
    return v >= double(c.b_max()) ? c.b_max() : (i64)v;
}

// The table index {lead, below} of b, b = 0 indexed as b = 1 (SPEC.md §1).
inline i64 scale_index_of_b(const Config& c, i64 b) {
    const int lead = b > 0 ? 63 - __builtin_clzll((uint64_t)b) : 0;
    const i64 below = lead >= c.b_k ? b >> (lead - c.b_k) : b << (c.b_k - lead);
    return (i64(lead) << c.b_k) | (below & ((i64(1) << c.b_k) - 1));
}

// Entry j's b, the geometric midpoint of its bucket, 0 for index 0 (SPEC.md §1).
inline double scale_b_mid(const Config& c, i64 j) {
    if (j == 0) return 0.0;
    const int lead = int(j >> c.b_k);
    const i64 mant = (i64(1) << c.b_k) + (j & ((i64(1) << c.b_k) - 1));  // 2^K + below = lo / 2^(lead-K)
    return std::ldexp(std::sqrt(double(mant) * double(mant + 1)), lead - c.b_k - c.b_frac);
}

// s itself if it is a normal code, else the nearest finite code (SPEC.md §1).
inline Scale finite_scale(const Config& c, Scale s) {
    if (s.exp == 0) return {1, 0};
    if (s.exp == c.scale_exp_inf()) return {c.scale_exp_inf() - 1, (1u << c.scale_man_width) - 1};
    return s;
}

// The scale table, finite_scale(scale_from_b(scale_b_mid(j))) for every index j (SPEC.md §1).
inline std::vector<Scale> build_scale_table(const Config& c) {
    std::vector<Scale> t((size_t)c.scale_table_size());
    for (size_t j = 0; j < t.size(); ++j) t[j] = finite_scale(c, scale_from_b(c, scale_b_mid(c, (i64)j)));
    return t;
}

// The table of a config, built once and cached.
inline const std::vector<Scale>& scale_table(const Config& c) {
    static std::mutex mu;
    static std::map<std::vector<int>, std::vector<Scale>> cache;
    const std::vector<int> key = {c.scale_exp_width, c.scale_man_width, c.symbol_width, c.b_frac, c.b_k};
    const std::lock_guard<std::mutex> lock(mu);
    const auto hit = cache.find(key);
    if (hit != cache.end()) return hit->second;
    return cache[key] = build_scale_table(c);
}

// The scale code of a scale input b (RTL: pans_scale_of_b).
inline Scale scale_of_b(const Config& c, i64 b) { return scale_table(c)[(size_t)scale_index_of_b(c, b)]; }

// ---- The slope k = 1 / (b ln2) of a scale code (SPEC.md §1) -------------------------------------
// The special scales saturate every C, so symbol 0 owns [0, M').

struct Slope {
    bool special = false;  // exponent 0 or all ones, C saturates
    i64 mant = 0;          // k = mant 2^exp2, mant = k_m, 0 when special
    int exp2 = 0;
};

inline Slope slope_of(const Config& c, Scale s) {
    if (s.exp == c.scale_exp_inf() || s.exp == 0) return {true, 0, 0};
    const int eb = (int)s.exp - c.scale_bias();
    return {false, (i64(1) << c.scale_man_width) + s.man, eb - c.scale_man_width};
}

// ---- The tail T(a) and C(x) ---------------------------------------------------------------------

// The tail T(a), Mitchell's 2^-a as wiring, 0 from a_hi = P - 1 on (SPEC.md §1).
inline i64 tail(const Config& c, i64 a) {
    const int vf = c.v_frac_bits;
    const i64 a_hi = a >> vf, a_lo = a & ((i64(1) << vf) - 1);
    if (a_hi >= c.precision - 1) return 0;
    return (((i64(2) << vf) - 1 - a_lo) << (c.precision - 2)) >> (vf + a_hi);
}

inline i64 tail_max(const Config& c) { return tail(c, 0); }

// C from a and the half, T or M' - T (RTL: cdf_from_a).
inline i64 cdf_from_a(const Config& c, i64 a, bool upper) {
    const i64 t = tail(c, a);
    return upper ? c.coded_slots() - t : t;
}

// The exponent a = floor(|P| 2^(exp2 - 1 + vf)), false where it saturates (RTL: a_of_pm).
inline bool a_of_pm(const Config& c, const Slope& k, i128 pm, i64* a) {
    const i64 lim = c.v_lim();
    if (k.special) return false;
    const int sh = k.exp2 - 1 + c.v_frac_bits;
    i128 aa;
    if (sh >= 0) {
        aa = sh > 90 ? (pm > 0 ? i128(lim) << 1 : 0) : (pm << sh);
    } else {
        aa = pm >> (-sh);  // floor
    }
    if (aa >= i128(lim)) return false;
    *a = (i64)aa;
    return true;
}

inline bool a_of(const Config& c, const Slope& k, i64 x, i64* a) {
    const i128 p = i128(2 * x - 1) * k.mant;
    return a_of_pm(c, k, p < 0 ? -p : p, a);
}

// C(x), the half's limit where a saturates.
inline i64 cdf(const Config& c, const Slope& k, i64 x) {
    const bool upper = x >= 1;
    i64 a;
    if (!a_of(c, k, x, &a)) return upper ? c.coded_slots() : 0;
    return cdf_from_a(c, a, upper);
}

// C from |P| and the half as the hardware forms it, sat forcing the half's limit.
inline i64 cdf_hw(const Config& c, const Slope& k, i64 pm, bool upper, bool sat = false) {
    i64 a;
    if (sat || !a_of_pm(c, k, i128(pm), &a)) return upper ? c.coded_slots() : 0;
    return cdf_from_a(c, a, upper);
}

// ---- The intervals ------------------------------------------------------------------------------
// The contract version, hashed into config_id. SPEC.md §4 lists what each version defines.
constexpr int CONTRACT_VERSION = 5;

// Symbol x owns [interval_lo(x), interval_hi(x)), [C(x), C(x + 1)) with the end symbols extended (SPEC.md §1).
inline i64 interval_lo(const Config& c, const Slope& k, i64 x) { return x == c.symbol_min() ? 0 : cdf(c, k, x); }

inline i64 interval_hi(const Config& c, const Slope& k, i64 x) {
    return x == c.symbol_max() ? c.coded_slots() : cdf(c, k, x + 1);
}

// ---- The two cores ------------------------------------------------------------------------------

struct EncOut {
    i64 start, freq;
    bool escape;
};

// The encoder core's function: the symbol's interval, or the escape (RTL: pans_enc_core).
inline EncOut encode(const Config& c, Scale s, i64 sym) {
    if (sym >= c.symbol_min() && sym <= c.symbol_max()) {
        const Slope k = slope_of(c, s);
        const i64 c0 = interval_lo(c, k, sym), f = interval_hi(c, k, sym) - c0;
        if (f >= 1) return {c0, f, false};
    }
    return {c.coded_slots(), c.escape_slots, true};
}

struct DecOut {
    i64 symbol, start, freq;
    bool escape;
};

// The decoder's function: the symbol whose interval holds the slot, by bisection (SPEC.md §1).
inline DecOut decode(const Config& c, Scale s, i64 slot) {
    if (slot >= c.coded_slots()) return {0, c.coded_slots(), c.escape_slots, true};
    const Slope k = slope_of(c, s);
    i64 lo = c.symbol_min(), hi = c.symbol_max();
    while (lo < hi) {
        const i64 mid = lo + (hi - lo + 1) / 2;
        if (interval_lo(c, k, mid) <= slot) lo = mid;
        else hi = mid - 1;
    }
    const i64 c0 = interval_lo(c, k, lo);
    return {lo, c0, interval_hi(c, k, lo) - c0, false};
}

// ---- The hardware's decoder ---------------------------------------------------------------------

// The bits kept of R'. Every R' at or above 2^mag_width clamps (SPEC.md §2).
inline int mag_width(const Config& c) { return c.symbol_width + c.scale_man_width + 1; }

// The exact division by 2 k_m as a truncated multiply by a stored constant (SPEC.md §2 step 3).
struct DivMagic {
    int shift = -1;
    int drop = 0;          // partial-product columns below this are not formed
    i64 n_max = 0;
    i64 comp = 0;          // the compensation constant, without the rounding term 2^(shift - 1)
    int magic_width = 0;
    std::vector<i64> m;    // per mantissa value
    long pp_kept = 0;      // partial-product bits formed (reports)
};

inline int bits_of(i64 v) {
    int w = 0;
    while ((i64(1) << w) <= v) ++w;
    return w;
}

inline DivMagic div_magic(const Config& c) {
    const i64 n_max = (i64(1) << mag_width(c)) - 1;
    const int B = mag_width(c), nm = 1 << c.scale_man_width;
    DivMagic best;
    int s_min = -1;
    for (int shift = 1; shift < 62; ++shift) {
        DivMagic r;
        r.shift = shift;
        r.n_max = n_max;
        r.m.resize(nm);
        i128 room = -1;  // the largest d_max every mantissa allows
        for (int man = 0; man < nm; ++man) {
            const i64 d = 2 * ((i64(1) << c.scale_man_width) + man);
            r.m[man] = (i64)(((i128(1) << shift) + d - 1) / d);
            const i128 e = i128(r.m[man]) * d - (i128(1) << shift);
            const i128 slack = (i128(1) << shift) - e * n_max;
            const i128 rm = slack <= 0 ? -1 : (slack + d - 1) / d - 1;
            if (man == 0 || rm < room) room = rm;
            r.magic_width = std::max(r.magic_width, bits_of(r.m[man]));
        }
        if (room < 0) continue;  // inexact even untruncated
        if (s_min < 0) s_min = shift;
        if (shift > s_min + 3) break;  // the kept count stops improving once the dropped triangle is as wide as R'
        i128 d_max = 0;
        int drop = 0;
        for (int col = 0; col <= B + r.magic_width; ++col) {
            int cnt = 0;
            for (int i = 0; i < B; ++i)
                for (int j = 0; j < r.magic_width; ++j) cnt += (i + j == col);
            if (d_max + (i128(cnt) << col) > room) break;
            d_max += i128(cnt) << col;
            drop = col + 1;
        }
        r.drop = drop;
        r.comp = (i64)d_max;
        r.pp_kept = 0;
        for (int i = 0; i < B; ++i)
            for (int j = 0; j < r.magic_width; ++j) r.pp_kept += (i + j >= drop);
        if (best.shift < 0 || r.pp_kept < best.pp_kept) best = r;
    }
    return best;
}

// The hardware's product without the dropped columns, plus comp and the rounding term.
inline i128 div_product(const DivMagic& dm, i64 rp, i64 magic) {
    i128 acc = i128(dm.comp) + (i128(1) << (dm.shift - 1));
    for (int j = 0; j < dm.magic_width; ++j) {
        if (!((magic >> j) & 1)) continue;
        const int lo = std::max(0, dm.drop - j);
        acc += (i128(rp >> lo) << lo) << j;
    }
    return acc;
}

inline i64 div_q(const DivMagic& dm, i64 rp, i64 magic) { return (i64)(div_product(dm, rp, magic) >> dm.shift); }

// The threshold's definition a+ = min{a : T(a) <= z'}, by bisection on T.
inline i64 a_plus_def(const Config& c, i64 zp) {
    i64 lo = 0, hi = i64(c.precision - 1) << c.v_frac_bits;
    while (lo < hi) {
        const i64 mid = lo + (hi - lo) / 2;
        if (tail(c, mid) <= zp) hi = mid;
        else lo = mid + 1;
    }
    return lo;
}

// The threshold in closed form, {n*, ~w} (SPEC.md §2 step 1, RTL: a_threshold).
inline i64 a_plus_hw(const Config& c, i64 zp) {
    const int vf = c.v_frac_bits, p = c.precision;
    const int msb = zp == 0 ? -1 : 63 - __builtin_clzll((uint64_t)zp);
    const int norm = p - msb;  // the left shift that puts the leading one at bit P
    const i64 n = p - 2 - msb;
    const i64 fill = (i64(1) << norm) - 1;
    const i64 w = (((zp << norm) | fill) >> (p - vf)) & ((i64(1) << vf) - 1);  // bits [P-1 : P-vf]
    return (n << vf) | (w ^ ((i64(1) << vf) - 1));
}

// The hardware's exact inverse (SPEC.md §2, RTL: pans_dec_core).
inline DecOut decode_hw(const Config& c, const DivMagic& dm, Scale s, i64 slot) {
    const i64 mp = c.coded_slots();
    if (slot >= mp) return {0, mp, c.escape_slots, true};
    const Slope k = slope_of(c, s);
    const bool upper = slot >= c.half(), lower = !upper;
    const int kw = c.scale_man_width + 1, B = mag_width(c);
    const i64 zp = upper ? mp - 1 - slot : slot;
    const i64 a_plus = a_plus_hw(c, zp);
    const bool none = a_plus == 0;
    const i64 ap = none ? 0 : a_plus - 1;  // A'
    i128 rp = 0;                           // R' on mag_width bits, 0 when special or none
    bool ovf = false;
    if (!k.special && !none) {
        const int shv = k.exp2 - 1 + c.v_frac_bits;
        if (shv >= 0) {
            rp = shv > 90 ? 0 : (i128(ap) >> shv);
        } else {
            const int t = std::min(-shv, 90);
            rp = (i128(ap) << t) | ((i128(1) << t) - 1);
        }
        ovf = (rp >> B) != 0;
        rp &= (i128(1) << B) - 1;
    }
    const i64 q = div_q(dm, (i64)rp, dm.m[s.man]);
    const i64 abs_x_max = lower ? -c.symbol_min() : c.symbol_max();  // the half's largest |x|
    // q > |x|max iff R' >= (2 |x|max + 1) k_m. The special scales never clamp.
    const bool big = ovf || (!k.special && (i64)rp >= (2 * abs_x_max + 1) * k.mant);
    const i64 abs_x = big ? abs_x_max : q;
    const i64 x = upper ? abs_x : -abs_x;
    const i64 km = k.mant, mask = (i64(1) << (kw + 1)) - 1;
    const i64 rem = (((i64)rp & mask) + km - 2 * ((km * (q & ((i64(1) << kw) - 1))) & ((i64(1) << kw) - 1))) & mask;
    const i64 pm_lo = big ? (2 * abs_x - 1) * km : (i64)rp - rem;  // (2|x| - 1) k_m; -k_m for |x| = 0, unused then
    const i64 pm_hi = pm_lo + 2 * km;                              // (2|x| + 1) k_m
    const bool nz = abs_x != 0;
    const bool side_x = upper && nz, side_n = upper || !nz;  // x >= 1, x + 1 >= 1
    const bool end = abs_x == abs_x_max;                      // x = MIN (lower half) or MAX (upper half)
    const i64 c0 = cdf_hw(c, k, side_x ? pm_lo : pm_hi, side_x, end && lower);
    return {x, c0, cdf_hw(c, k, side_n ? pm_hi : pm_lo, side_n, end && upper) - c0, false};
}

// ---- The decodability contract ------------------------------------------------------------------
// The exact inverse relies on T non-increasing, T = 0 from (P - 1) 2^vf on, and T(0) <= 2^(P-1) - E.
inline long contract_violations(const Config& c) {
    long bad = 0;
    const i64 lim = c.v_lim();
    i64 prev = tail(c, 0);
    bad += prev > c.half() - c.escape_slots;
    for (i64 a = 1; a < lim; ++a) {
        const i64 t = tail(c, a);
        bad += t > prev;
        bad += (a >= (i64(c.precision - 1) << c.v_frac_bits)) && t != 0;
        prev = t;
    }
    return bad;
}

// ---- Build identity -----------------------------------------------------------------------------
// config_id, FNV-1a over every field that defines the build (SPEC.md §4).
inline uint64_t config_id(const Config& c) {
    uint64_t h = 0xcbf29ce484222325ull;
    const auto mix = [&h](i64 v) {
        for (int i = 0; i < 8; ++i) {
            h ^= (uint64_t)(v >> (8 * i)) & 0xff;
            h *= 0x100000001b3ull;
        }
    };
    mix(0x6c617066);  // the Laplace exact-inverse form, floor everywhere, half-ulp Mitchell as wiring
    mix(CONTRACT_VERSION);
    for (i64 v : {i64(c.precision), i64(c.escape_slots), i64(c.symbol_width), i64(c.scale_exp_width),
                  i64(c.scale_man_width), i64(c.v_frac_bits), i64(c.symmetric_alphabet)})
        mix(v);
    for (i64 v : {i64(c.b_frac), i64(c.b_k)}) mix(v);
    return h;
}

}  // namespace pans1hw
