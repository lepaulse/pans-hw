// SPDX-License-Identifier: Apache-2.0
// pans1_dpi.cpp: the C++ reference of the hardware build (pans1_config.h) as DPI-C functions.
#include "pans1_config.h"

using namespace pans1hw;

namespace {
const Config& config() {
    static const Config c = build_config();
    return c;
}
Scale scale_of(int exp, int man) { return Scale{(unsigned)exp, (unsigned)man}; }
}  // namespace

extern "C" {

// The build identity (pans1hw::config_id), to be compared with the RTL package's CONFIG_ID.
long long pans_ref_config_id() { return (long long)config_id(config()); }

// The scale code of a sigma. 0, inf and NaN are allowed.
void pans_ref_scale(double sigma, int* exp, int* man) {
    const Scale s = scale_from_sigma(config(), sigma);
    *exp = (int)s.exp;
    *man = (int)s.man;
}

// The scale input b: its widths, the wire value of a sigma, and the scale code of a b.
void pans_ref_b_widths(int* b_frac, int* b_width, int* b_k) {
    *b_frac = config().b_frac;
    *b_width = config().b_width();
    *b_k = config().b_k;
}

long long pans_ref_b_of_sigma(double sigma) { return (long long)b_fixed_of(config(), laplace_b(sigma)); }

void pans_ref_scale_of_b(long long b, int* exp, int* man) {
    const Scale s = (b >= 0 && b <= config().b_max()) ? scale_of_b(config(), (i64)b) : Scale{0, 0};
    *exp = (int)s.exp;
    *man = (int)s.man;
}

void pans_ref_encode(int exp, int man, int symbol, int* start, int* freq, int* escape) {
    const EncOut e = encode(config(), scale_of(exp, man), symbol);
    *start = (int)e.start;
    *freq = (int)e.freq;
    *escape = e.escape;
}

void pans_ref_decode(int exp, int man, int slot, int* symbol, int* start, int* freq, int* escape) {
    const DecOut d = decode(config(), scale_of(exp, man), slot);
    *symbol = (int)d.symbol;
    *start = (int)d.start;
    *freq = (int)d.freq;
    *escape = d.escape;
}

}  // extern "C"
