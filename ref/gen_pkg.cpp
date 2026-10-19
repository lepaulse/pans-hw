// SPDX-License-Identifier: Apache-2.0
// gen_pkg: writes the RTL package pans_pkg.sv of the hardware build (pans1_config.h) to stdout.
// It refuses, with exit status 1 and the reasons on stderr, a build the RTL cannot implement (SPEC.md §4).
// With --stream it also refuses a build the stream units cannot implement. The package is the same either way.
//
//   g++ -O2 -std=c++17 -DPANS_PRECISION=16 -DPANS_SYMBOL_WIDTH=16 -DPANS_ESCAPE_SLOTS=1 ref/gen_pkg.cpp -o gen_pkg
//   gen_pkg [--stream] > pans_pkg.sv
#include <algorithm>
#include <cstdio>
#include <string>
#include <vector>

#include "pans1_config.h"

using namespace pans1hw;

namespace {

// Every reason the RTL, or with `stream` the stream units, cannot implement this build.
std::vector<std::string> problems(const Config& c, bool stream) {
    std::vector<std::string> out;
    if (c.v_frac_bits > c.precision) out.push_back("V_FRAC_BITS must be <= PRECISION");
    if (tail_max(c) > c.half() - c.escape_slots)
        out.push_back("T(0) = " + std::to_string(tail_max(c)) + " exceeds 2^(P-1) - E: the half of a slot would not be its top bit");
    // >= 2 keeps the end symbols away from 0
    if (c.symbol_width < 2 || c.symbol_width > c.precision)
        out.push_back("SYMBOL_WIDTH must be in [2, PRECISION]");
    if (c.symbol_width < c.scale_man_width + 1)
        out.push_back("SYMBOL_WIDTH must be >= SCALE_MAN_WIDTH + 1 (the decoder's dividend width, pans1_ref.h mag_width)");
    if (c.escape_slots < 1) out.push_back("ESCAPE_SLOTS must be >= 1");
    if (c.coded_slots() < 2) out.push_back("M - ESCAPE_SLOTS must be >= 2");
    if (div_magic(c).shift < 0) out.push_back("no division shift below 62 is exact for every scale mantissa");
    if (stream && (c.escape_slots & (c.escape_slots - 1)) != 0)
        out.push_back("ESCAPE_SLOTS = " + std::to_string(c.escape_slots) +
                      " is not a power of two: the stream encoder's escape push (f = E) is a shift (STREAM.md)");
    if (c.b_frac < 0 || c.b_k < 0 || c.b_k > 8 || c.b_width() < 1 || c.b_width() > 48)
        out.push_back("the scale input: B_FRAC >= 0, B_K in [0, 8], B_WIDTH = SYMBOL_WIDTH - 1 + B_FRAC in [1, 48]");
    if (out.empty()) {
        const long bad = contract_violations(c);
        if (bad) out.push_back(std::to_string(bad) + " decodability-contract violations");
    }
    return out;
}

// The scale input's parameters and SCALE_TABLE, printed after the include that defines scale_t.
void print_scale_table(const Config& c) {
    const std::vector<Scale> t = build_scale_table(c);
    std::printf("  // ---- The scale input b (SPEC.md §1, §3; rtl/pans_scale_of_b.sv): the Laplace scale in symbol steps,\n"
                "  // unsigned fixed point, B_FRAC fraction bits on B_WIDTH = SYMBOL_WIDTH - 1 + B_FRAC\n"
                "  // bits. Its scale code is SCALE_TABLE[{lead, below}]: lead the position of b's leading one\n"
                "  // (B_LEAD_BITS bits; b = 0: lead = 0), below the B_K bits below it (zero-filled below bit 0). Entry\n"
                "  // j = scale_from_b at the geometric midpoint of its bucket, sqrt(lo hi) 2^-B_FRAC with\n"
                "  // lo = (2^B_K + below) 2^(lead - B_K), hi = lo + 2^(lead - B_K) (entry 0 holds b = 0 too: midpoint 0),\n"
                "  // a special code replaced by the nearest finite one (pans1_ref.h build_scale_table)\n");
    std::printf("  /* verilator lint_off UNUSEDPARAM */\n");
    std::printf("  localparam int     B_FRAC           = %d;\n", c.b_frac);
    std::printf("  localparam int     B_WIDTH          = %d;\n", c.b_width());
    std::printf("  localparam int     B_K              = %d;\n", c.b_k);
    std::printf("  localparam int     B_LEAD_BITS      = %d;\n", c.b_lead_bits());
    std::printf("  localparam int     SCALE_INDEX_BITS = %d;\n", c.scale_index_bits());
    std::printf("  localparam int     SCALE_TABLE_SIZE = %lld;\n", (long long)c.scale_table_size());
    std::printf("  localparam scale_t SCALE_TABLE [0:SCALE_TABLE_SIZE-1] = '{\n");
    const size_t row = size_t(1) << c.b_k, per_line = std::min<size_t>(row, 8);
    for (size_t j = 0; j < t.size(); ++j) {
        const int lead = int(j / row);
        if (j % row == 0)
            std::printf("    // lead = %d: b in [2^%d, 2^%d)\n", lead, lead - c.b_frac, lead + 1 - c.b_frac);
        if (j % per_line == 0) std::printf("    ");
        std::printf("{%d'd%u, %d'd%u}%s", c.scale_exp_width, t[j].exp, c.scale_man_width, t[j].man,
                    j + 1 == t.size() ? "" : ",");
        std::printf("%s", j % per_line == per_line - 1 || j + 1 == t.size() ? "\n" : " ");
    }
    std::printf("  };\n");
    std::printf("  /* verilator lint_on UNUSEDPARAM */\n\n");
}

}  // namespace

int main(int argc, char** argv) {
    const bool stream = argc == 2 && std::string(argv[1]) == "--stream";
    if (argc > 2 || (argc == 2 && !stream)) {
        std::fprintf(stderr, "usage: gen_pkg [--stream] > pans_pkg.sv\n");
        return 2;
    }
    const Config c = build_config();
    const std::vector<std::string> why = problems(c, stream);
    if (!why.empty()) {
        std::fprintf(stderr, "gen_pkg: refusing PRECISION=%d SYMBOL_WIDTH=%d ESCAPE_SLOTS=%d%s%s:\n", c.precision,
                     c.symbol_width, c.escape_slots, c.symmetric_alphabet ? " SYMMETRIC_ALPHABET=1" : "",
                     stream ? " for the stream units" : "");
        for (const std::string& w : why) std::fprintf(stderr, "  - %s\n", w.c_str());
        return 1;
    }
    std::printf(
        "// GENERATED by hw/ref/gen_pkg.cpp from hw/ref/pans1_config.h. Do not edit.\n"
        "// Parameters and division constants of the pANS hardware cores: Laplace, exact inverse (SPEC.md).\n"
        "package pans_pkg;\n"
        "  /* verilator lint_off UNUSEDPARAM */\n\n");
    std::printf("  // ---- Unit set: %s (ref/pans1_config.h)\n\n", c.units);
    std::printf("  // ---- Build identity: equals pans1hw::config_id() of the C++ reference built for this config\n");
    std::printf("  localparam logic [63:0] CONFIG_ID = 64'h%016llx;\n", (unsigned long long)config_id(c));
    std::printf("  // The coder's contract version (SPEC.md §4; pans1_ref.h CONTRACT_VERSION), hashed into CONFIG_ID\n");
    std::printf("  localparam int CONTRACT_VERSION = %d;\n\n", CONTRACT_VERSION);
    std::printf("  // ---- Coder\n");
    std::printf("  localparam int PRECISION       = %d;\n", c.precision);
    std::printf("  localparam int ESCAPE_SLOTS    = %d;\n", c.escape_slots);
    std::printf("  localparam int SYMBOL_WIDTH    = %d;\n", c.symbol_width);
    std::printf("  localparam int NUM_SLOTS       = %lld;\n", (long long)c.slots());
    std::printf("  localparam int CODED_SLOTS     = %lld;  // M' = M - E\n", (long long)c.coded_slots());
    std::printf("  // The alphabet [SYMBOL_MIN, SYMBOL_MAX]. SYMMETRIC_ALPHABET = 1: SYMBOL_MIN = -SYMBOL_MAX,\n"
                "  // and the S-bit input -2^(SYMBOL_WIDTH-1) lies outside it and escapes (PANS_SYMMETRIC_ALPHABET)\n");
    std::printf("  localparam int SYMMETRIC_ALPHABET = %d;\n", c.symmetric_alphabet ? 1 : 0);
    std::printf("  localparam int SYMBOL_MIN      = %lld;\n", (long long)c.symbol_min());
    std::printf("  localparam int SYMBOL_MAX      = %lld;\n\n", (long long)c.symbol_max());
    std::printf("  // ---- Scale code: the slope k = 1 / (b ln2) as an unsigned float; exponent 0 and all ones are the\n"
                "  // special (saturating) scales\n");
    std::printf("  localparam int SCALE_EXP_WIDTH = %d;\n", c.scale_exp_width);
    std::printf("  localparam int SCALE_MAN_WIDTH = %d;\n", c.scale_man_width);
    std::printf("  localparam int SCALE_BIAS      = %d;\n", c.scale_bias());
    std::printf("  localparam int SCALE_EXP_INF   = %u;\n\n", c.scale_exp_inf());
    std::printf("  // ---- Exponent domain a = k |x - 1/2|, V_FRAC_BITS fraction bits; T(a) = 0 from a = (P - 1) 2^vf on,\n"
                "  // so a >= V_SAT 2^vf is the saturated range (C = 0 or M'). T(0) = TAIL_MAX <= 2^(P-1) - E (checked)\n");
    std::printf("  localparam int V_FRAC_BITS     = %d;\n", c.v_frac_bits);
    std::printf("  localparam int V_SAT           = %d;\n", c.v_sat());
    std::printf("  localparam int TAIL_MAX        = %lld;\n\n", (long long)tail_max(c));
    std::printf("  // ---- The decoder's exact division by the scale\n");
    const DivMagic dm = div_magic(c);
    std::printf("  // floor((N + k_m) / (2 k_m)) = (N DIV_MAGIC[man] + 2^(DIV_SHIFT - 1) + DIV_COMP) >> DIV_SHIFT for 0 <= N <= DIV_N_MAX\n"
                "  // with the partial products of the DIV_DROP lowest columns left out of the product (pans1_ref.h div_magic:\n"
                "  // %ld of %d partial-product bits); N >= DIV_N_MAX + 1 = 2^MAG_WIDTH decodes beyond the alphabet for every mantissa\n",
                dm.pp_kept, mag_width(c) * dm.magic_width);
    std::printf("  localparam longint DIV_N_MAX = 64'sd%lld;\n", (long long)dm.n_max);
    std::printf("  localparam int DIV_SHIFT = %d;\n", dm.shift);
    std::printf("  localparam int DIV_DROP = %d;\n", dm.drop);
    std::printf("  localparam longint DIV_COMP = 64'sd%lld;\n", (long long)dm.comp);
    {
        const int w = dm.magic_width;
        std::printf("  localparam int DIV_MAGIC_WIDTH = %d;\n", w);
        std::printf("  localparam logic [%d:0] DIV_MAGIC [0:%d] = '{", w - 1, (int)dm.m.size() - 1);
        for (size_t i = 0; i < dm.m.size(); ++i) std::printf("%s%d'd%lld", i ? ", " : "", w, (long long)dm.m[i]);
        std::printf("};\n\n");
    }
    std::printf("  // ---- Stream (STREAM.md): W-bit words, W = 8 ceil(P/8) >= P, so a push or a pop moves at most one word;\n"
                "  // the state x in [L, L 2^W), L = M, of P + W bits, stored little-endian in STATE_BYTES bytes\n");
    std::printf("  localparam int WORD_WIDTH      = %d;\n", c.word_width());
    std::printf("  localparam int STATE_WIDTH     = %d;\n", c.state_width());
    std::printf("  localparam int STATE_BYTES     = %d;\n\n", c.state_bytes());
    std::printf("  /* verilator lint_on UNUSEDPARAM */\n\n");
    std::printf("`include \"pans_pkg_functions.svh\"\n\n");
    print_scale_table(c);
    std::printf("endpackage\n");
    return 0;
}
