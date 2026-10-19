// SPDX-License-Identifier: Apache-2.0
// sweep: the rate cost of the hardware's arithmetic, and the coder's correctness, on the C++ reference.
// Specification: SPEC.md §5. Each row is a unit set, each column an octave of sigma.
// "build" is the build at the exact scale code. "build via b" takes the scale input b through the table.
// The other rows vary a's fraction bits and the scale mantissa at the same P, SW and alphabet.
// The gauss_ columns and their reports code discretized-Gaussian data with the Laplace model of each row.
// The rice_ columns and the last reports give a Golomb-Rice code at the best parameter per sigma on the Laplace data.
//
//   g++ -O2 -std=c++17 -fopenmp -DPANS_PRECISION=16 -DPANS_SYMBOL_WIDTH=16 ref/sweep.cpp -o build/p16s16/sweep
//   build/p16s16/sweep [csv]
#include <cstdio>
#include <string>
#include <vector>

#include "pans1_config.h"

using namespace pans1hw;

namespace {

// The continuous Laplace CDF with scale b.
double laplace_cdf(double t, double b) { return t < 0 ? 0.5 * std::exp(t / b) : 1.0 - 0.5 * std::exp(-t / b); }

// ---- Exact-math coder (the baseline) -----------------------------------------------------------------

struct Exact {
    i64 mp, x_min, x_max;
    double b;
    i64 cdf(i64 x) const { return (i64)std::floor((double)mp * laplace_cdf((double)x - 0.5, b)); }
    // The interval's ends, as interval_lo and interval_hi in pans1_ref.h.
    i64 lo(i64 x) const { return x == x_min ? 0 : cdf(x); }
    i64 hi(i64 x) const { return x == x_max ? mp : cdf(x + 1); }
};

// The probability of x under the discretized Laplace clipped to [x_min, x_max].
double prob(i64 x, double b, i64 x_min, i64 x_max) {
    const double a = (double)std::llabs(x);
    const double inner = x == x_min || x == x_max ? 0.0 : std::exp(-(a + 0.5) / b);  // the tail beyond x
    return x == 0 ? 1.0 - std::exp(-0.5 / b) : 0.5 * (std::exp(-(a - 0.5) / b) - inner);
}

// The probability of x under round(N(0, sigma^2)) clipped to [x_min, x_max], x's bin [x - 1/2, x + 1/2).
double prob_gauss(i64 x, double sigma, i64 x_min, i64 x_max) {
    const double r = 1.0 / (sigma * std::sqrt(2.0));
    const double a = (double)std::llabs(x);
    const double inner = x == x_min || x == x_max ? 0.0 : std::erfc((a + 0.5) * r);  // the tail beyond x
    return x == 0 ? std::erf(0.5 * r) : 0.5 * (std::erfc((a - 0.5) * r) - inner);
}

// A pmf on the symbols lo, lo + 1, ..
struct Pmf {
    i64 lo = 0;
    std::vector<double> p;
};

// The discretized Gaussian on the symbols within 40 sigma of 0, clipped to the alphabet. The rest underflows.
Pmf gauss_pmf(const Config& c, double sigma) {
    const i64 span = (i64)std::ceil(40.0 * sigma) + 2;
    Pmf g;
    g.lo = std::max(c.symbol_min(), -span);
    for (i64 x = g.lo; x <= std::min(c.symbol_max(), span); ++x)
        g.p.push_back(prob_gauss(x, sigma, c.symbol_min(), c.symbol_max()));
    return g;
}

// The coder's expected bits per symbol at scale code s on the pmf g, the escape mass in *escape.
double expected_bits(const Config& c, Scale s, const Pmf& g, double* escape = nullptr) {
    const double p_bits = c.precision;
    const double escape_bits = (p_bits - std::log2((double)c.escape_slots)) + p_bits;
    double bits = 0, esc = 0;
    for (size_t i = 0; i < g.p.size(); ++i) {
        const double p = g.p[i];
        if (!(p > 0)) continue;
        const EncOut e = encode(c, s, g.lo + (i64)i);
        if (e.escape) esc += p;
        bits += p * (e.escape ? escape_bits : p_bits - std::log2((double)e.freq));
    }
    if (escape) *escape = esc;
    return bits;
}

// The Kullback-Leibler divergence in bits of the clipped discretized Laplace at b from the pmf g.
double kl_laplace(const Config& c, const Pmf& g, double b) {
    double kl = 0;
    for (size_t i = 0; i < g.p.size(); ++i) {
        const double p = g.p[i], q = prob(g.lo + (i64)i, b, c.symbol_min(), c.symbol_max());
        if (p > 0) kl += q > 0 ? p * std::log2(p / q) : INFINITY;
    }
    return kl;
}

// The least kl_laplace over b in [b0 / 2, 2 b0], golden-section search on ln b. The b in *b_best.
double kl_laplace_best(const Config& c, const Pmf& g, double b0, double* b_best) {
    const double gr = 0.5 * (std::sqrt(5.0) - 1.0);
    double lo = std::log(0.5 * b0), hi = std::log(2.0 * b0);
    double x1 = hi - gr * (hi - lo), x2 = lo + gr * (hi - lo);
    double f1 = kl_laplace(c, g, std::exp(x1)), f2 = kl_laplace(c, g, std::exp(x2));
    for (int it = 0; it < 80; ++it) {
        if (f1 < f2) {
            hi = x2;
            x2 = x1;
            f2 = f1;
            x1 = hi - gr * (hi - lo);
            f1 = kl_laplace(c, g, std::exp(x1));
        } else {
            lo = x1;
            x1 = x2;
            f1 = f2;
            x2 = lo + gr * (hi - lo);
            f2 = kl_laplace(c, g, std::exp(x2));
        }
    }
    *b_best = std::exp(0.5 * (lo + hi));
    return kl_laplace(c, g, *b_best);
}

// ---- Golomb-Rice (the second baseline) ---------------------------------------------------------------

// The zigzag map of a signed symbol to an unsigned one: 0, -1, 1, -2, 2, .. to 0, 1, 2, 3, 4, ..
i64 zigzag(i64 x) { return x >= 0 ? 2 * x : -2 * x - 1; }

// The Rice code of u with parameter k: the quotient u >> k in unary (that many ones and a zero), then the k low bits
// of u. There is no escape. The alphabet is finite, and its largest symbols get the plain Rice code of their zigzag
// value, whatever its length.
double rice_length(i64 u, int k) { return double((u >> k) + 1 + k); }

// The expected Rice bits per symbol on the clipped discretized Laplace at b, over the symbols run_sigma takes, at the
// best k in [0, SW], the oracle parameter for this b. The least k of equal cost goes to *k_best. From k = SW on every
// quotient is 0, so a larger k only costs more.
double rice_best(const Config& c, double b, int* k_best) {
    const i64 span = (i64)std::ceil(40.0 * b) + 2;
    std::vector<double> bits(c.symbol_width + 1, 0.0);
    for (i64 x = std::max(c.symbol_min(), -span); x <= std::min(c.symbol_max(), span); ++x) {
        const double p = prob(x, b, c.symbol_min(), c.symbol_max());
        if (!(p > 0)) continue;
        for (int k = 0; k <= c.symbol_width; ++k) bits[k] += p * rice_length(zigzag(x), k);
    }
    *k_best = 0;
    for (int k = 1; k <= c.symbol_width; ++k)
        if (bits[k] < bits[*k_best]) *k_best = k;
    return bits[*k_best];
}

// The scale code's index, monotone in the slope k, and back.
int code_index(const Config& c, Scale s) { return int(s.exp << c.scale_man_width) + int(s.man); }
Scale code_of_index(const Config& c, int j) {
    return {unsigned(j >> c.scale_man_width), unsigned(j & ((1 << c.scale_man_width) - 1))};
}

// The b of a normal scale code, 1 / (k ln2), as laplace_b_of in pans_stream_ref.h.
double b_of_code(const Config& c, Scale s) {
    const Slope k = slope_of(c, s);
    return 1.0 / (std::ldexp((double)k.mant, k.exp2) * std::log(2.0));
}

// The best code on Gaussian data: the codes searched either side of the row's, 7 octaves of k at 3 mantissa bits.
constexpr int BEST_CODE_SPAN = 56;

struct Metrics {
    double escape = 0, bits = 0, bits_exact = 0, entropy = 0;
    long fail = 0;  // coded-symbol slots that do not decode back to the symbol
    // Gaussian data: escapes and bits at the row's code, bits at the best code and its b, the entropy, the KL at b
    // and at the KL-minimizing b.
    double g_escape = 0, g_bits = 0, g_bits_best = 0, g_b_best = 0, g_entropy = 0;
    double g_kl = 0, g_kl_best = 0, g_b_kl_best = 0;
    bool g_best_at_edge = false;  // the best code lies at the search window's edge
    // Golomb-Rice on the Laplace data: the bits per symbol at the best k, and that k.
    double rice_bits = 0;
    int rice_k = 0;
};

Metrics run_sigma(const Config& c, const DivMagic& dm, double sigma, bool via_b) {
    Metrics m;
    const double b = laplace_b(sigma);
    const Scale s = via_b ? scale_of_b(c, b_fixed_of(c, b)) : scale_from_sigma(c, sigma);
    const Exact ex{c.coded_slots(), c.symbol_min(), c.symbol_max(), b};
    const double p_bits = c.precision;
    const double escape_bits = (p_bits - std::log2((double)c.escape_slots)) + p_bits;  // the escape slot and P raw bits
    // The symbols within 40 b of 0, clipped to the alphabet. The rest holds under 2^-57 of the mass.
    const i64 span = (i64)std::ceil(40.0 * b) + 2;
    for (i64 x = std::max(c.symbol_min(), -span); x <= std::min(c.symbol_max(), span); ++x) {
        const double p = prob(x, b, c.symbol_min(), c.symbol_max());
        if (!(p > 0)) continue;
        m.entropy -= p * std::log2(p);
        const i64 fe = ex.hi(x) - ex.lo(x);
        m.bits_exact += p * (fe >= 1 ? p_bits - std::log2((double)fe) : escape_bits);
        const EncOut e = encode(c, s, x);
        if (e.escape) {
            m.escape += p;
            m.bits += p * escape_bits;
            continue;
        }
        m.bits += p * (p_bits - std::log2((double)e.freq));
        for (i64 slot = e.start; slot < e.start + e.freq; ++slot) {
            const DecOut d = decode_hw(c, dm, s, slot);
            if (d.escape || d.symbol != x || d.start != e.start || d.freq != e.freq) ++m.fail;
        }
    }
    // Gaussian data of standard deviation sigma at the same code s, and at the normal code that costs least.
    const Pmf g = gauss_pmf(c, sigma);
    for (double p : g.p)
        if (p > 0) m.g_entropy -= p * std::log2(p);
    m.g_bits = expected_bits(c, s, g, &m.g_escape);
    const int j0 = code_index(c, s);
    const int j_lo = std::max(1 << c.scale_man_width, j0 - BEST_CODE_SPAN);
    const int j_hi = std::min(int(c.scale_exp_inf() << c.scale_man_width) - 1, j0 + BEST_CODE_SPAN);
    m.g_bits_best = INFINITY;
    for (int j = j_lo; j <= j_hi; ++j) {
        const double v = expected_bits(c, code_of_index(c, j), g);
        if (v < m.g_bits_best) {
            m.g_bits_best = v;
            m.g_b_best = b_of_code(c, code_of_index(c, j));
            m.g_best_at_edge = j == j_lo || j == j_hi;
        }
    }
    m.g_kl = kl_laplace(c, g, b);
    m.g_kl_best = kl_laplace_best(c, g, b, &m.g_b_kl_best);
    m.rice_bits = rice_best(c, b, &m.rice_k);
    return m;
}

struct Named {
    std::string name;
    Config cfg;
    bool via_b = false;  // the code from b through the table
};

// A report's title and its header row of octave centers.
void print_header(const char* title, const std::vector<double>& centers) {
    std::printf("\n%s\n%-16s", title, "sigma:");
    for (double c0 : centers) std::printf("%16g", c0);
    std::printf("\n");
}

// A report's row: the mean / max of v(j) over each octave's nsub sigmas j, with one more decimal if fine.
template <class F>
void print_octaves(const std::string& name, int np, int nsub, bool fine, F v) {
    std::printf("%-16s", name.c_str());
    for (int pnt = 0; pnt < np; ++pnt) {
        double sum = 0, mx = -1e9;
        for (int k = 0; k < nsub; ++k) {
            const double x = v(pnt * nsub + k);
            sum += x;
            mx = std::max(mx, x);
        }
        std::printf(fine ? "%9.5f/%6.4f" : "%9.4f/%6.3f", sum / nsub, mx);
    }
    std::printf("\n");
}

}  // namespace

int main(int argc, char** argv) {
    const int P = PANS_PRECISION, SW = PANS_SYMBOL_WIDTH;  // the build's; make sweep-<cfg> passes them
    std::vector<Named> configs;
    const Config bc = build_config();
    configs.push_back({"build", bc});
    configs.push_back({"build via b", bc, true});
    for (int vf : {bc.v_frac_bits, bc.v_frac_bits + 2})
        configs.push_back({"vf" + std::to_string(vf), make_config(P, SW, vf, 3)});
    configs.push_back({"vf" + std::to_string(bc.v_frac_bits) + " man2", make_config(P, SW, bc.v_frac_bits, 2)});
    for (Named& n : configs) n.cfg.symmetric_alphabet = bc.symmetric_alphabet;  // the build's alphabet on every row

    // The points 1/4, 1, 4, .. up to a quarter of the alphabet, then every octave to half of it, 32 sigmas around each.
    std::vector<double> centers;
    for (double c0 = 0.25; c0 <= bc.symbol_max() / 4.0; c0 *= 4) centers.push_back(c0);
    for (double c0 = centers.back() * 2; c0 <= bc.symbol_max() / 2.0; c0 *= 2) centers.push_back(c0);
    const int nsub = 32;
    std::vector<double> sigmas;
    for (double c0 : centers)
        for (int i = 0; i < nsub; ++i) sigmas.push_back(c0 * std::exp2((i + 0.5) / nsub - 0.5));

    const int nc = (int)configs.size(), ns = (int)sigmas.size(), np = (int)centers.size();
    std::vector<Metrics> res(nc * ns);
    std::vector<long> contract(nc);
    std::vector<DivMagic> dms;
    for (const Named& n : configs) dms.push_back(div_magic(n.cfg));
    (void)scale_table(bc);  // built before the threads share it
#pragma omp parallel for schedule(dynamic)
    for (int i = 0; i < nc * ns + nc; ++i) {
        if (i < nc * ns) res[i] = run_sigma(configs[i / ns].cfg, dms[i / ns], sigmas[i % ns], configs[i / ns].via_b);
        else contract[i - nc * ns] = contract_violations(configs[i - nc * ns].cfg);
    }

    if (argc > 1) {
        FILE* f = std::fopen(argv[1], "w");
        std::fprintf(f, "config,sigma,escape,bits,bits_exact,entropy,fail,gauss_escape,gauss_bits,gauss_bits_best,"
                        "gauss_entropy,gauss_b_best,gauss_kl,gauss_kl_best,gauss_b_kl_best,rice_k,rice_bits\n");
        for (int i = 0; i < nc; ++i)
            for (int j = 0; j < ns; ++j) {
                const Metrics& m = res[i * ns + j];
                std::fprintf(f, "%s,%g,%.6e,%.8f,%.8f,%.8f,%ld,%.6e,%.8f,%.8f,%.8f,%.8g,%.8f,%.8f,%.8g,%d,%.8f\n",
                             configs[i].name.c_str(), sigmas[j], m.escape, m.bits, m.bits_exact, m.entropy, m.fail,
                             m.g_escape, m.g_bits, m.g_bits_best, m.g_entropy, m.g_b_best, m.g_kl, m.g_kl_best,
                             m.g_b_kl_best, m.rice_k, m.rice_bits);
            }
        std::fclose(f);
    }

    const char* what[3] = {"rate overhead vs the exact-math coder (%), mean / max over each octave",
                           "rate overhead vs entropy (%), mean over each octave", "escape probability (%), mean"};
    for (int w = 0; w < 3; ++w) {
        std::printf("\n%s\n%-16s", what[w], "sigma:");
        for (double c0 : centers) std::printf(w == 0 ? "%16g" : "%10g", c0);
        std::printf("%10s%8s\n", "contract", "fail");
        for (int i = 0; i < nc; ++i) {
            std::printf("%-16s", configs[i].name.c_str());
            long fail = 0;
            for (int j = 0; j < ns; ++j) fail += res[i * ns + j].fail;
            for (int pnt = 0; pnt < np; ++pnt) {
                double sum = 0, mx = -1e9;
                for (int k = 0; k < nsub; ++k) {
                    const Metrics& m = res[i * ns + pnt * nsub + k];
                    const double v = w == 0 ? 100.0 * (m.bits / m.bits_exact - 1.0)
                                   : w == 1 ? 100.0 * (m.bits / m.entropy - 1.0) : 100.0 * m.escape;
                    sum += v;
                    mx = std::max(mx, v);
                }
                if (w == 0) std::printf("%9.4f/%6.3f", sum / nsub, mx);
                else std::printf("%10.4f", sum / nsub);
            }
            std::printf("%10ld%8ld\n", contract[i], fail);
        }
    }
    // The table's cost, "build via b" against "build" (rows 1 and 0).
    print_header("the scale table's cost: build via b against build (%), mean / max over each octave", centers);
    print_octaves("build via b", np, nsub, false,
                  [&](int j) { return 100.0 * (res[1 * ns + j].bits / res[0 * ns + j].bits - 1.0); });

    // Gaussian data: the rate over the Gaussian's entropy at each row's code and at the best code near it.
    const char* g_what[4] = {
        "Gaussian data: rate overhead vs its entropy at the row's code (%), mean / max over each octave",
        "Gaussian data: rate overhead vs its entropy at the row's code (bits per symbol), mean / max over each octave",
        "Gaussian data: rate overhead vs its entropy at the best code (%), mean / max over each octave",
        "Gaussian data: rate overhead vs its entropy at the best code (bits per symbol), mean / max over each octave"};
    for (int w = 0; w < 4; ++w) {
        print_header(g_what[w], centers);
        for (int i = 0; i < nc; ++i)
            print_octaves(configs[i].name, np, nsub, w % 2 == 1, [&](int j) {
                const Metrics& m = res[i * ns + j];
                const double bits = w < 2 ? m.g_bits : m.g_bits_best;
                return w % 2 == 1 ? bits - m.g_entropy : 100.0 * (bits / m.g_entropy - 1.0);
            });
    }
    int edge = 0;
    for (const Metrics& m : res) edge += m.g_best_at_edge;
    std::printf("the best code: the least rate within %d codes of the row's; at the window's edge for %d of %d\n",
                BEST_CODE_SPAN, edge, nc * ns);
    // The entropy and the KL do not depend on the row's units (row 0).
    print_header("Gaussian data without the coder (bits): its entropy, the KL of the Laplace at b and at the best b",
                 centers);
    print_octaves("entropy", np, nsub, false, [&](int j) { return res[j].g_entropy; });
    print_octaves("KL at b", np, nsub, true, [&](int j) { return res[j].g_kl; });
    print_octaves("KL at best b", np, nsub, true, [&](int j) { return res[j].g_kl_best; });

    // Golomb-Rice at the best k per sigma beside the build (row 0), both over the Laplace data's entropy. The Rice
    // code does not depend on the row's units.
    for (int w = 0; w < 2; ++w) {
        print_header(w == 0 ? "Golomb-Rice at the best k per sigma and the build: rate overhead vs entropy (%), "
                              "mean / max over each octave"
                            : "Golomb-Rice at the best k per sigma and the build: rate overhead vs entropy "
                              "(bits per symbol), mean / max over each octave",
                     centers);
        print_octaves("build", np, nsub, w == 1, [&](int j) {
            return w == 1 ? res[j].bits - res[j].entropy : 100.0 * (res[j].bits / res[j].entropy - 1.0);
        });
        print_octaves("Golomb-Rice", np, nsub, w == 1, [&](int j) {
            return w == 1 ? res[j].rice_bits - res[j].entropy : 100.0 * (res[j].rice_bits / res[j].entropy - 1.0);
        });
    }
    print_header("the build's rate against Golomb-Rice at the best k per sigma (%), mean / max over each octave",
                 centers);
    print_octaves("build", np, nsub, false, [&](int j) { return 100.0 * (res[j].bits / res[j].rice_bits - 1.0); });
    print_header("Golomb-Rice: the best k, mean / max over each octave", centers);
    print_octaves("best k", np, nsub, false, [&](int j) { return (double)res[j].rice_k; });
    return 0;
}
