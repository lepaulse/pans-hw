// SPDX-License-Identifier: Apache-2.0
// power_stimulus: the workload and the corner set of the gate-level power simulation (make power-<cfg>).
// Specification: SPEC.md §6 for the corner set and §7 for the workload. The seed is fixed.
// The workload's kind (make's POWER_STIMULUS, SPEC.md §7.1): laplace (the default), σ log-uniform and a discretized
// Laplace symbol of that σ; static, the mid-alphabet symbol at the mid-range b for every transaction; random, symbols
// uniform over the alphabet with b uniform over its B_WIDTH bits. The decoder's slot is uniform inside the symbol's
// interval, or the escape slot; static's is the middle of the interval.
//
//   power_stimulus <count> <dir> [laplace|static|random]
//   <dir>/enc.hex: {exp, man, symbol}   <dir>/enc_expected.hex: {start, freq, escape}
//   <dir>/dec.hex: {exp, man, slot}     <dir>/dec_expected.hex: {symbol, start, freq, escape}
//   <dir>/enc_corner.hex, enc_corner_expected.hex, dec_corner.hex, dec_corner_expected.hex: the corner set
//   (hex, packed as the cores' ports; the expected outputs are the C++ reference's)
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <random>
#include <set>
#include <string>

#include "pans1_config.h"

using namespace pans1hw;

int main(int argc, char** argv) {
    const std::string kind = argc == 4 ? argv[3] : "laplace";
    if ((argc != 3 && argc != 4) || (kind != "laplace" && kind != "static" && kind != "random")) {
        std::fprintf(stderr, "usage: power_stimulus <count> <dir> [laplace|static|random]\n");
        return 2;
    }
    const long count = std::atol(argv[1]);
    const std::string dir = argv[2];
    const Config c = build_config();
    std::FILE* enc = std::fopen((dir + "/enc.hex").c_str(), "w");
    std::FILE* dec = std::fopen((dir + "/dec.hex").c_str(), "w");
    std::FILE* enc_expected = std::fopen((dir + "/enc_expected.hex").c_str(), "w");
    std::FILE* dec_expected = std::fopen((dir + "/dec_expected.hex").c_str(), "w");
    if (!enc || !dec || !enc_expected || !dec_expected) {
        std::perror("power_stimulus");
        return 1;
    }
    std::mt19937_64 rng(0x70616e73);  // fixed
    std::uniform_real_distribution<double> unit(0.0, 1.0);
    const double log_lo = std::log(0.25), log_hi = std::log(std::max(0.5, c.symbol_max() / 2.0));
    for (long i = 0; i < count; ++i) {
        i64 symbol = (c.symbol_min() + c.symbol_max()) / 2;  // static: the mid-alphabet symbol at the mid-range b
        Scale s = scale_of_b(c, (c.b_max() + 1) / 2);
        if (kind == "laplace") {
            const double sigma = std::exp(log_lo + (log_hi - log_lo) * unit(rng));
            const double b = laplace_b(sigma);
            double u = unit(rng);
            u = std::min(std::max(u, 1e-12), 1.0 - 1e-12);
            // Laplace(0, b) by its inverse CDF
            const double t = u < 0.5 ? b * std::log(2.0 * u) : -b * std::log(2.0 * (1.0 - u));
            symbol = std::clamp<i64>(std::llround(t), c.symbol_min(), c.symbol_max());
            s = scale_from_sigma(c, sigma);
        } else if (kind == "random") {
            symbol = c.symbol_min() + i64(rng() % uint64_t(c.symbol_max() - c.symbol_min() + 1));
            s = scale_of_b(c, i64(rng() % uint64_t(c.b_max() + 1)));
        }
        const EncOut e = encode(c, s, symbol);
        if (kind == "static" && e.escape) {
            std::fprintf(stderr, "power_stimulus: the static symbol escapes\n");
            return 1;
        }
        const i64 slot = kind == "static" ? e.start + e.freq / 2 : e.start + i64(unit(rng) * double(e.freq));
        const unsigned long long scale = (uint64_t(s.exp) << c.scale_man_width) | s.man;
        const unsigned long long sym_bits = uint64_t(symbol) & ((1ull << c.symbol_width) - 1);
        std::fprintf(enc, "%llx\n", (scale << c.symbol_width) | sym_bits);
        std::fprintf(dec, "%llx\n", (scale << c.precision) | uint64_t(slot));
        const DecOut d = decode(c, s, slot);
        const uint64_t p = uint64_t(c.precision);
        std::fprintf(enc_expected, "%llx\n", (unsigned long long)((uint64_t(e.start) << (p + 1)) | (uint64_t(e.freq) << 1) | e.escape));
        std::fprintf(dec_expected, "%llx\n",
                     (unsigned long long)((((uint64_t(d.symbol) & ((1ull << c.symbol_width) - 1)) << (2 * p + 1))
                                         | (uint64_t(d.start) << (p + 1)) | (uint64_t(d.freq) << 1)
                                         | uint64_t(d.escape))));
    }
    for (std::FILE* f : {enc, dec, enc_expected, dec_expected}) std::fclose(f);

    std::FILE* ec = std::fopen((dir + "/enc_corner.hex").c_str(), "w");
    std::FILE* ece = std::fopen((dir + "/enc_corner_expected.hex").c_str(), "w");
    std::FILE* dc = std::fopen((dir + "/dec_corner.hex").c_str(), "w");
    std::FILE* dce = std::fopen((dir + "/dec_corner_expected.hex").c_str(), "w");
    if (!ec || !ece || !dc || !dce) {
        std::perror("power_stimulus");
        return 1;
    }
    const long n_scales = 1L << (c.scale_exp_width + c.scale_man_width);
    const bool all_symbols = (n_scales << c.symbol_width) <= (1L << 17);  // every S-bit input
    const bool all_slots = (n_scales << c.precision) <= (1L << 17);
    std::set<i64> symbols, slots;
    for (i64 x : {c.symbol_input_min(), c.symbol_min(), c.symbol_min() + 1, i64(-1), i64(0), i64(1),
                  c.symbol_max() - 1, c.symbol_max()})
        symbols.insert(x);
    for (i64 q : {i64(0), i64(1), c.half() - 1, c.half(), c.half() + 1, c.coded_slots() - 2, c.coded_slots() - 1,
                  c.coded_slots(), c.slots() - 1})
        if (q >= 0 && q < c.slots()) slots.insert(q);
    if (all_symbols)
        for (i64 x = c.symbol_input_min(); x <= c.symbol_max(); ++x) symbols.insert(x);
    if (all_slots)
        for (i64 q = 0; q < c.slots(); ++q) slots.insert(q);
    const uint64_t p = uint64_t(c.precision);
    for (long code = 0; code < n_scales; ++code) {
        const Scale s{unsigned(code >> c.scale_man_width), unsigned(code & ((1 << c.scale_man_width) - 1))};
        const unsigned long long scale = (unsigned long long)code;
        for (i64 x : symbols) {
            const EncOut e = encode(c, s, x);
            std::fprintf(ec, "%llx\n", (scale << c.symbol_width) | (uint64_t(x) & ((1ull << c.symbol_width) - 1)));
            std::fprintf(ece, "%llx\n", (unsigned long long)((uint64_t(e.start) << (p + 1)) | (uint64_t(e.freq) << 1) | e.escape));
        }
        for (i64 q : slots) {
            const DecOut d = decode(c, s, q);
            std::fprintf(dc, "%llx\n", (scale << c.precision) | uint64_t(q));
            std::fprintf(dce, "%llx\n",
                         (unsigned long long)((((uint64_t(d.symbol) & ((1ull << c.symbol_width) - 1)) << (2 * p + 1))
                                             | (uint64_t(d.start) << (p + 1)) | (uint64_t(d.freq) << 1)
                                             | uint64_t(d.escape))));
        }
    }
    for (std::FILE* f : {ec, ece, dc, dce}) std::fclose(f);
    return 0;
}
