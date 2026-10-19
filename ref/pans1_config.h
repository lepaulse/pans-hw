// SPDX-License-Identifier: Apache-2.0
// pans1_config.h: the hardware build from the PANS_* macros, shared by the reference, the DPI models and gen_pkg.
// ci/matrix.toml passes the macros. Specification: SPEC.md §4.
#pragma once
#include "pans1_ref.h"

#ifndef PANS_PRECISION
#define PANS_PRECISION 8  // bring-up config; the used config is 16
#endif
#ifndef PANS_SYMBOL_WIDTH
#define PANS_SYMBOL_WIDTH 6  // bring-up config; the used config is 16
#endif
#ifndef PANS_ESCAPE_SLOTS
#define PANS_ESCAPE_SLOTS 1  // E: slots [M - E, M) are the escape
#endif
// The alphabet is every S-bit value at 0 and -(2^(S-1) - 1) .. 2^(S-1) - 1 at 1 (SPEC.md §1).
#ifndef PANS_SYMMETRIC_ALPHABET
#define PANS_SYMMETRIC_ALPHABET 0
#endif
#if PANS_SYMMETRIC_ALPHABET != 0 && PANS_SYMMETRIC_ALPHABET != 1
#error "PANS_SYMMETRIC_ALPHABET is 0 (every S-bit value) or 1 (-(2^(S-1) - 1) .. 2^(S-1) - 1)"
#endif

// B_FRAC and B_K are fixed by the contract (SPEC.md §1). The estimator macros of contract version 3 are refused.
#if defined(PANS_EST_FRAC) || defined(PANS_EST_SHIFT) || defined(PANS_EST_M0) || defined(PANS_EST_MFLOOR) || \
    defined(PANS_EST_K) || defined(PANS_EST_DELAY)
#error "PANS_EST_*: the scale estimator left the build at contract version 4 (the coder takes b; STREAM.md 1.7)"
#endif

namespace pans1hw {

// The Laplace unit set: VF = max(P - 6, 4) fraction bits of a, 3 mantissa bits of the scale code (SPEC.md §4, §5).
inline Config laplace_units(int precision, int symbol_width) {
    Config c = make_config(precision, symbol_width, std::max(precision - 6, 4), 3);
    c.escape_slots = PANS_ESCAPE_SLOTS;
    c.symmetric_alphabet = PANS_SYMMETRIC_ALPHABET != 0;
    c.units = "laplace";
    return c;
}

inline Config build_config() { return laplace_units(PANS_PRECISION, PANS_SYMBOL_WIDTH); }

}  // namespace pans1hw
