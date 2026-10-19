# pANS hardware coder: the cores

Specification version 0.8, 2026-10-17. Contract version 5.

This document specifies `pans_enc_core` and `pans_dec_core`, the two cores of the pANS hardware coder. pANS (parametric
ANS) computes each symbol's frequency from the parameters of its distribution. The cores compute a discretized-Laplace
model in integer arithmetic. They hold no frequency tables, neither a per-symbol frequency table nor a
cumulative-frequency table. The encoder core maps a symbol and a scale to the symbol's interval of the M slots. The
decoder core maps a slot back to its symbol and interval. The cores keep no state. The stream units add the ANS state,
the renormalization and the byte array ([STREAM.md](STREAM.md)). The scale of each symbol is an input. The probability
model that chooses it lies outside the coder. The scale table `SCALE_TABLE` converts that input, the Laplace scale b, to
the cores' scale code (§1.5). It is a constant of the build and normative data.

**Normative reference.** The bit-exact C++ model `ref/pans1_ref.h` defines the cores. `ref/pans1_config.h` builds it for
one configuration. The RTL functions in `rtl/pans_pkg_functions.svh` implement the model and carry the names of their
C++ counterparts. This document describes the model.

**Compatibility.** An encoder and a decoder are compatible only if they come from the same build, which `CONFIG_ID`
identifies (§4). A stream does not decode under another build. The stream is not compatible with the pANS 1.0 software
coder either. pANS 1.0 uses a Logistic C, a closed-form candidate and a ±1 select.

## 0. Vocabulary

The state and the stream use the terms of the ANS literature: Duda, Giesen's interleaved rANS, and pANS 1.0. pANS 1.0 is
the pANS software coder that this coder generalizes. §1 and §2 define the model's quantities. Hardware terms have their
standard meaning: valid/ready, FIFO, credit, reservation, bubble, pipeline stage, clock gate.

| Term | Meaning | In the code |
|---|---|---|
| P | probability bits. There are M = 2^P slots. M' = M - E of them are coded slots and E are escape slots | `PRECISION`, `NUM_SLOTS`, `CODED_SLOTS`, `ESCAPE_SLOTS` |
| slot | a value in [0, M). The decoder reads it from the state as x mod M (pANS 1.0's s) | `slot` |
| start c, frequency f | a symbol's interval [c, c + f) of the slots. c is its cumulative frequency and f its frequency (Giesen's start and freq) | ports `start`, `freq` |
| escape | a symbol sent raw: the escape interval [M', M), then the symbol's bits (§1.7, STREAM.md §1.4) | `escape` |
| state x | a lane's ANS state (STREAM.md §1). §1 and §2 describe stateless arithmetic, and there x is the symbol, the argument of C | `x` |
| L | the state's lower bound between symbols: x ∈ [L, L·2^W) with L = M. Every lane starts at L. The end check requires every lane back at L (pANS 1.0's L_LOW) | `STATE_L`, `state_L`, `at_L` |
| W | word bits. Renormalization moves W-bit words, base 2^W. The letter b denotes the Laplace scale (§1.2) | `WORD_WIDTH` |
| push, pop | one symbol into and out of a lane's state (pANS 1.0's push and dec_pop) | `stream_push`, `stream_pop` |
| renormalization: emit, refill | the encoder emits the state's low word before a push that would leave [L, L·2^W). The decoder refills a word after a pop that leaves x below L | `push_renorm`, `refill` |
| lane | one of N interleaved ANS states that share one word stream and one byte array (Giesen's interleaved streams). Lane j | `N_LANES`, `lane` |
| decoding step | one symbol of a lane through the decoder unit: the issue, the core, the multiply stage ((x >> P)·f) and the add-and-renormalize stage (the sum, the refills, the state write) | `pans_dec_stream` stages 0-6 |
| tag pipeline | the registers that carry a request's lane beside a core's fixed-latency pipeline. In the encoder unit they also carry the symbol | `q1_lane`, `q2_lane`, … |

The model's quantities, each defined once:

| Quantity | Defined in | In the code |
|---|---|---|
| alphabet [MIN, MAX], asymmetric or symmetric | §1.1 | `SYMBOL_MIN`, `SYMBOL_MAX` |
| Laplace scale b | §1.2 | |
| slope k, scale code {exp, man}, mantissa k_m | §1.2 | `scale_t`, `slope_of` |
| exponent product P(x), its magnitude \|P(x)\| | §1.3 | `pm` |
| exponent a(x) | §1.3 | `a_of_pm` |
| tail T(a) | §1.3 | |
| C(x) | §1.3 | `cdf`, `cdf_from_a` |
| lower half (x ≤ 0), upper half (x ≥ 1) | §1.3 | |
| special scales | §1.4 | |
| scale input b, the coder's table SCALE_TABLE[{lead, below}] | §1.5 | `scale_of_b`, `SCALE_TABLE` |
| end symbols, slots beyond the alphabet | §1.6 | `interval_lo`, `interval_hi` |
| escape rule | §1.7 | |
| threshold a+ | §2 | `a_threshold` |
| dividend R' | §2 | `dividend`, `rp` |
| quotient, its clamp to \|x\| | §2 | `q_divide`, `clamp_of`, `abs_x` |
| exponent magnitudes \|P(x)\|, \|P(x + 1)\| | §2 | `exponent_magnitudes` |

## 1. Coder

### 1.1 Symbols and alphabet

Symbols x are signed SW-bit values. There are M = 2^P slots. The M' = M - E slots [0, M') are coded slots. The E slots
[M', M) are escape slots.

The alphabet [MIN, MAX] has one of two forms:

| Form | MIN | MAX | Build option |
|---|---|---|---|
| asymmetric (default) | -2^(SW-1) | 2^(SW-1) - 1 | none |
| symmetric | -(2^(SW-1) - 1) | 2^(SW-1) - 1 | `PANS_SYMMETRIC_ALPHABET`, `symmetric_alphabet = true` in `ci/matrix.toml` |

A symmetric alphabet matches a quantizer with the output range ±(2^(SW-1) - 1). Such a quantizer never produces
-2^(SW-1). The symbol port still accepts -2^(SW-1). That input lies outside a symmetric alphabet and escapes at every
scale. The stream unit sends it raw as the SW bits x mod 2^SW and decodes it back.

In a symmetric alphabet MIN owns the slots below C(MIN + 1) that -2^(SW-1) owns in the asymmetric alphabet. So f(MIN) =
f(MAX) at every scale. At the widest scale code {1, 0}, p12s8sym gives f(-127) = 520 slots of 4,095, the same as f(127).
The asymmetric p12s8 gives -127 8 slots and -128 512 slots.

### 1.2 Model and scale code

The model is a discretized Laplace distribution: C(x) ≈ M'·F(x - ½), where F is the Laplace CDF of scale b. The cores
take the slope k = 1/(b·ln2) as their scale input. The base-2 logarithm of the density falls by k per symbol step. So
a = k·|x - ½| gives 2^-a = e^(-|x-½|/b).

With ρ = e^(-1/b), the discretized Laplace gives symbol 0 the probability 1 - ρ^(1/2) and a symbol x ≠ 0 the probability
ρ^(|x| - 1/2)·(1 - ρ)/2. Its variance is ρ^(1/2)·(1 + ρ)/(1 - ρ)^2. `laplace_b(σ)` is the b whose discrete variance is
σ². The continuous match b = σ/√2 is far off at small σ.

`scale_from_b` computes the code of k from b, rounded to nearest. `scale_from_sigma` applies `scale_from_b` to
`laplace_b(σ)`. The stream units take b itself and convert it to the code with the coder's table (§1.5).

The scale code holds k as an unsigned float {exp, man} (`scale_t`). exp has EW bits and man has 3 bits. The bias is
BIAS = 2^(EW-1) - 1. There are no subnormals. k = k_m·2^(exp - BIAS - 3), where k_m = 8 + man is the slope's mantissa
(`slope_of`).

### 1.3 Exponent, tail and C

The exponent product is P(x) = (2x - 1)·k_m. It always appears with its argument, because P alone denotes the
probability bits. Its magnitude is |P(x)| (`pm`). The exponent is a(x) = floor(|P(x)|·2^V_SHIFT) (`a_of_pm`), with
V_SHIFT = exp - BIAS - 4 + VF. a is unsigned and has VF fraction bits. It is floored in both halves. The lower half is x
≤ 0. The upper half is x ≥ 1.

The tail T(a) computes 2^-a with Mitchell's binary-logarithm approximation (1962), as wiring. Write a = a_hi·2^VF + a_lo
and φ = a_lo·2^-VF. Then

T(a) = floor((2^(VF+1) - 1 - a_lo)·2^(P-2) / 2^(VF + a_hi)) = floor(2^(P-1)·2^-a_hi·(1 - φ/2 - 2^-(VF+1))).

This is the classic Mitchell form 2^(P-1)·2^-m·(1 + r), with m = ceil(a) and r = m - a, evaluated at a + 1 ulp. The
fraction enters as ~a_lo. The shift amount is a_hi itself. The constant shift is wiring. It is a right shift, which
performs the floor, when P - 2 < VF. T needs one shifter and no table. T is non-increasing. T(0) = TAIL_MAX < 2^(P-1).
T = 0 for a_hi ≥ P - 1.

The tail lies within a factor [2^-(2^-VF), 1.0615·(1 + E/M')] of the exact Laplace tail, ± 1 slot. Three effects make up
the bound:

- a is floored to VF bits and T is evaluated at a + 1 ulp. So the rounding of a lies in (0, +1 ulp], a factor in
  [2^-ulp, 1), with ulp = 2^-VF.
- Mitchell's 1 - φ/2 for 2^-φ is at most 6.15% high.
- T uses 2^(P-1) for M'/2, a factor 1 + E/M'.

C(x) = T(a(x)) for x ≤ 0 and C(x) = M' - T(a(x)) for x ≥ 1 (`cdf_from_a`). Three properties follow:

- C is non-decreasing.
- C(x) + C(1 - x) = M' exactly. P(1 - x) = -P(x), so a(x) = a(1 - x) is one floor of the same |P|. The two halves are T
  and M' - T of the same T.
- C(0) ≤ 2^(P-1) ≤ C(1), because TAIL_MAX ≤ 2^(P-1) - E (§4).

C saturates for a ≥ V_LIM = P·2^VF and at the special scales (§1.4). It is then 0 for x ≤ 0 and M' for x ≥ 1. This is
also T's own limit.

### 1.4 Special scales

Two exponent values are special. exp = 0 underflows k and stands for σ = ∞ or NaN. exp all ones overflows k and stands
for σ = 0. The mantissa is ignored. At a special scale every C saturates. Symbol 0 owns [0, M'), and every other symbol
escapes. Coding symbol 0 this way is never worse in rate than escaping it. It needs no logic of its own, because it is
C's saturation.

The special scales are internal to the cores. The scale table holds none of them (§1.5). Only the cores' own scale input
reaches them, and the cores' tests use them.

### 1.5 The scale input b and the coder's table

The model supplies the Laplace scale b in symbol steps. The coder converts b to the cores' scale code. The model
function `scale_of_b` in `ref/pans1_ref.h` and the module `rtl/pans_scale_of_b.sv` perform the conversion (§3).

**Wire format.** b is unsigned fixed point. It has B_FRAC = 8 fraction bits on B_WIDTH = (SW - 1) + B_FRAC bits. B_WIDTH
is 15 at p12s8 and 23 at p16s16. An average of |x| kept with 8 fraction bits is a valid b as it is. From σ = 1 on, b's
step and the table together cost under 0.035% of rate (§5).

**Range.** b < 2^(SW-1). The mean |x| of an SW-bit alphabet never exceeds its half-width, so an estimator's average
stays below this bound. Beyond it a Laplace distribution is close to flat over the alphabet. A model with a larger b
saturates it to 2^B_WIDTH - 1.

**Index.** The table index is {lead, below}:

- lead is the position of b's leading one. It has B_LEAD_BITS = bits(B_WIDTH - 1) bits. b = 0 gives lead = 0, the index
  of b = 1, because the leading-one detect of 0 is 0.
- below is the B_K = 4 bits below the leading one. Bits below bit 0 are zero-filled. This applies to b < 2^B_K.

SCALE_INDEX_BITS = B_LEAD_BITS + B_K. The table has B_WIDTH·2^B_K entries of `scale_t`. That is 240 entries at p12s8 and
368 at p16s16.

**Entries.** Entry {lead, below} covers the b in [lo, hi)·2^-B_FRAC, with lo = (2^B_K + below)·2^(lead - B_K) and hi =
lo + 2^(lead - B_K). The entry is `scale_from_b` at the bucket's geometric midpoint √(lo·hi)·2^-B_FRAC. On a log scale
the midpoint lies at most half a bucket from every b in the bucket. An octave has 16 buckets, two per step of the scale
code. The model computes the midpoint as the double nearest its exact value. lo·hi is an exact integer below 2^53, the
square root is correctly rounded, and 2^-B_FRAC is an exact power of two.

An entry never holds a special code. `finite_scale` replaces a special code by the nearest finite code:

| Special code | Replaced by |
|---|---|
| {0, 0}, σ = ∞ | {1, 0}, the widest finite scale |
| {EXP_INF, 0}, σ = 0 | {EXP_INF - 1, 7}, the narrowest finite scale |

Index 0 also holds b = 0. Its bucket starts at 0, so its midpoint is 0. Therefore b = 0, b = 1 and b = 2^-8 map to the
narrowest finite code, never to a special code. That code saturates every C at every build, since k ≥ 240 and a(0) =
a(1) ≥ P. Symbol 0 then owns [0, M'), as at σ = 0.

The code's k is non-increasing as b grows. The top of the table depends on the build:

| Build | Widest scale of the code format | Top of the table |
|---|---|---|
| p12s8 | b ≈ 92 | the entries from b = 88 on are {1, 0}. Entries 232-239 (b ≥ 96) are {1, 0} by the clamp |
| p16s16 | | entries 360-367 are {1, 0} |
| p16s8, p12s10 (EW 5) | b up to 23,637 | the top entry is the code of b ≈ 2^(SW-1). Wider codes are not reached from b |
| p8s6, p4s4 (EW 4) | b up to 92, against b < 32 and b < 8 | as for p16s8 |

Where the top entry is the code of b ≈ 2^(SW-1), the wider codes are not reached from the m that an SW-bit estimator can
produce either (STREAM.md §1.7).

**The table is data.** `build_scale_table` makes it once. The model, `gen_pkg` and the tests share this one helper. An
implementation reads the table from `pans_pkg`, as `SCALE_TABLE` with `B_FRAC`, `B_WIDTH`, `B_K`, `B_LEAD_BITS`,
`SCALE_INDEX_BITS` and `SCALE_TABLE_SIZE`. It never recomputes the table from doubles, because one ulp can move a code.
§5 gives the table's rate cost against the exact code.

### 1.6 Intervals and the end symbols

Symbol x owns the interval [lo(x), hi(x)) with lo(x) = C(x) and hi(x) = C(x + 1). The alphabet's ends are the exception:
lo(MIN) = 0 and hi(MAX) = M' (`interval_lo`, `interval_hi`). MIN and MAX are the end symbols. The slots below C(MIN) and
the slots from C(MAX + 1) to M' - 1 are the slots beyond the alphabet. The end symbols own them, so no coded slot is
wasted. The intervals tile [0, M') in symbol order. Every slot below M' belongs to exactly one symbol.

The edge cases:

- At the special scales, MIN's interval [0, C(MIN + 1)) = [0, 0) and MAX's interval [C(MAX), M') = [M', M') are empty.
  Both end symbols escape, and symbol 0 keeps [0, M'). Since SW ≥ 2, MIN ≤ -1 and MAX ≥ 1, so neither end symbol is 0.
- Where C(MIN + 1) = 0, MIN's interval is empty and MIN escapes. No slot then lies beyond the alphabet on that side. The
  same holds for MAX where C(MAX) = M'.
- Where C(MIN) > 0, MIN owns the slots below C(MIN) and has freq ≥ 1. MIN is then coded even where [C(MIN), C(MIN + 1))
  is empty. The same holds for MAX where C(MAX + 1) < M'.

The end rule changes only the end symbols' intervals. Every inner symbol's interval is [C(x), C(x + 1)). The decoder's
symbol for each slot does not depend on the end rule. Contract version 1 had no end rule (§4). Under version 1 the slots
beyond the alphabet belonged to no symbol, and an end symbol with an empty [C(x), C(x + 1)) escaped.

`make inverse-<cfg>` counts, over all scale codes, the slots beyond the alphabet and the end symbols coded there whose
own [C(x), C(x + 1)) is empty. Every row is the same at contract versions 2 to 5. p12s8sym exists from version 3 on.

| Config | slots beyond the alphabet | at scale codes | end symbols coded only by the end rule |
|---|---|---|---|
| p4s4 | 335 | 37 of 128 | 59 |
| p8s6, p8s6e3 | 2,412 | 31 of 128 | 18 |
| p8s8 | 348 | 15 of 128 | 16 |
| p9s9 | 98 | 9 of 128 | 10 |
| p12s8 | 5,826 | 20 of 128 | 5 |
| p12s8sym | 5,866 | 20 of 128 | 10 |
| p12s9 | 858 | 12 of 128 | 7 |
| p12s10 | 158,946 | 68 of 256 | 78 |
| p16s8 | 3,593,752 | 88 of 256 | 30 |
| p16s16 | 93,782 | 24 of 256 | 24 |

At p16s8 the slots beyond the alphabet are 21% of all coded slots.

### 1.7 Encoder and decoder functions

**Encoder.** For a symbol x of the alphabet, (start, freq) = (lo(x), hi(x) - lo(x)) if freq ≥ 1. Otherwise x escapes
with (M', E). An input outside the alphabet has no interval and escapes. Only -2^(SW-1) of a symmetric alphabet is such
an input. No other condition causes an escape.

**Decoder.** A slot ≥ M' is an escape. The decoder returns symbol 0 with (M', E). For a slot below M' the decoder
returns the symbol whose interval holds the slot, and that interval, with freq ≥ 1. That symbol is the largest x in
[MIN, MAX] with lo(x) ≤ slot. Equivalently, it is the largest x with C(x) ≤ slot, or MIN if no such x exists. A slot
below M' never lies outside the interval decoded for it.

### 1.8 C and the end symbols in hardware

Both cores evaluate C with the same functions: `shift_of`, `a_of_pm` and `cdf_from_a`. So the encoder and the decoder
compute identical C.

The outer edge of an end symbol's interval is the saturated value of the C it replaces. That value is 0 in the lower
half and M' in the upper half. The hardware forces it through the saturation flag of `cdf_from_a`, in the stage where
each core knows the symbol:

| Core | Stage | Forced where |
|---|---|---|
| encoder | 1 | x = MIN, x = MAX |
| decoder | 3 | \|x\| = -MIN in the lower half, \|x\| = MAX in the upper half |

The decoder's clamp in stage 2 maps every slot beyond the alphabet to one of these two cases. The inverse itself (§2) is
unchanged by the end rule.

A symmetric alphabet changes two things. The decoder clamps the lower half at |x| = MAX, as the upper half.
`LOWER_WIDE` = 0 in `pans_pkg_functions.svh` selects this in `clamp_of`, in `exponent_magnitudes` and in the clamped |x|
of decoder stage 2. The encoder forces both CDFs of the input -2^(SW-1) = MIN - 1 to 0 in stage 1. Both lie in the lower
half, so the interval is [0, 0) and the input escapes.

An asymmetric build has the logic of contract version 2. Generic synthesis gives the same gates and depth for both cores
and for every function at p4s4, p8s8 and p16s16.

## 2. Exact-inverse decode (`decode_hw`)

The decoder core computes the decoder function of §1.7 in five steps of integer arithmetic.

1. **Threshold** (`a_threshold`). upper = slot[P-1]. z' = slot in the lower half and z' = M' - 1 - slot = ~slot - E in
   the upper half. The threshold is a+ = min{a : T(a) ≤ z'} = {P - 2 - msb(z'), ~w}. Here w is the VF bits below the
   leading one of z'. Ones fill w when msb(z') < VF. For z' = 0, a_hi = P - 1. The step is a leading-one detect, a shift
   and an inversion. The derivation: in the octave a_hi = n, T lies in [2^(P-2-n), 2^(P-1-n)). So the threshold lies in
   the octave n* = P - 2 - msb(z'). Within that octave T(a) ≤ z' iff a_lo ≥ ~w. The ones fill turns the floor of the
   shift into a strict compare.
2. **Dividend** (`dividend`, R' = `rp`). In the lower half, C(x) ≤ slot ⇔ |P(x)| > R'. In the upper half, C(x) ≤ slot
   ⇔ |P(x)| ≤ R'. R' = ceil(a+·2^-V_SHIFT) - 1. One right shifter computes it from A' = a+ - 1. A left shift fills ones.
   a+ = 0 and the special scales give R' = 0 and x = 0.
3. **Division** (`q_divide`). In both halves |x| = q = floor((R' + k_m) / (2k_m)). In the lower half this q is the
   smallest q with (2q + 1)·k_m > R'. In the upper half it is the largest q with (2q - 1)·k_m ≤ R'. The hardware
   computes q = (R'·DIV_MAGIC[man] + 2^(DIV_SHIFT-1) + DIV_COMP) >> DIV_SHIFT. It omits the partial products of the
   DIV_DROP lowest columns. The fractional part of (R' + k_m)/(2k_m) is a multiple of 1/(2k_m). So a perturbation in [0,
   1/(2k_m)) keeps the floor. Let DIV_MAGIC = ceil(2^DIV_SHIFT/(2k_m)) and e = 2k_m·DIV_MAGIC - 2^DIV_SHIFT. Let
   DIV_COMP = d_max, the largest sum of the dropped columns. The perturbation then lies in [0, (DIV_N_MAX·e +
   d_max·2k_m)/(2k_m·2^DIV_SHIFT)]. The division is exact iff DIV_N_MAX·e + d_max·2k_m < 2^DIV_SHIFT for every mantissa.
   `div_magic` picks the exact shift with the fewest partial-product bits.
4. **Clamp** (`clamp_of`, |x| = `abs_x`). q exceeds MAX in the upper half, or -MIN in the lower half, iff R' ≥
   (2|x|max + 1)·k_m. The hardware evaluates this compare beside the multiplier. The clamp also applies when R' has a
   bit at or above MAG_WIDTH, where 2^MAG_WIDTH ≥ 15·(2^SW + 1). The clamp gives MAX or MIN.
5. **Interval.** The exponent magnitudes |P(x)| and |P(x + 1)| = (2q ∓ 1)·k_m (`exponent_magnitudes`) come from R' and
   from rem = (R' + k_m) mod 2k_m. rem needs only the low K_WIDTH + 1 bits of R' and q. The halves follow from the
   slot's half and from q ≠ 0. Two C follow, as in the encoder. For an end symbol the outer C is forced to its half's
   limit (§1.8). The end symbols are |x| = -MIN in the lower half and |x| = MAX in the upper half.

**Why the inverse is exact.** Every step is monotone: the floor of |P|, T, and C(0) ≤ 2^(P-1) ≤ C(1). So {x : C(x) ≤
slot} is a prefix of the alphabet. Steps 1-4 find its end in integer arithmetic. There is no candidate, no ±1 correction
and no iteration. §6 checks the inverse exhaustively. The slots beyond the alphabet are lower-half slots below C(MIN) ≤
C(0) and upper-half slots from C(MAX + 1) ≥ C(1) on. The clamp gives them MIN and MAX, whose intervals hold them.

## 3. Interfaces and timing

| Core | In | Out | Latency |
|---|---|---|---|
| `pans_enc_core` | in_valid, in_scale, in_symbol (signed SW) | out_valid, out_start, out_freq (P bits), out_escape | 2 |
| `pans_dec_core` | in_valid, in_scale, in_slot (P bits) | out_valid, out_symbol (signed SW), out_start, out_freq, out_escape | 4 |

- in_scale is a `scale_t`, the packed {exp, man}.
- rst_n is synchronous and active low. It clears only the valid bits.
- The cores are valid-only. Each takes one request per cycle and answers in order after a fixed latency. There is no
  ready and no backpressure. out_valid is in_valid delayed by exactly 2 cycles in the encoder and 4 in the decoder. The
  stream unit paces the requests and takes each response in its out_valid cycle.
- Data registers have no reset and no enable. They load every cycle, so the outputs are meaningful only with out_valid.

**Pipeline stages.**

| Core | Stage | Work |
|---|---|---|
| encoder | 1 | \|P(x + 1)\| = (2xt + 1)·k_m with xt = x or ~x (one multiply), \|P(x)\| by one add, the end-symbol tests |
| encoder | 2 | two alignments, two C, the interval |
| decoder | 1 | the threshold, R', DIV_MAGIC[man] |
| decoder | 2 | the division multiplier and the clamp, the core's longest path |
| decoder | 3 | the magnitudes, the halves, the alignment, the end-symbol test |
| decoder | 4 | two C, the interval |

**Assertions.** SVA under `ifndef SYNTHESIS` check the following:

- encoder: a coded symbol has freq ≥ 1 and start + freq ≤ M';
- decoder: a coded slot has freq ≥ 1 and lies in [start, start + freq). The slot travels beside the pipeline for this
  check;
- both: an escape gives (M', E), and C(x + 1) ≥ C(x).

**Elaboration checks.** E ≥ 1, SW ≤ P, VF ≤ P, TAIL_MAX ≤ 2^(P-1) - E, the decoder's widths, CONTRACT_VERSION = 5, and
the alphabet's form: SYMBOL_MAX = 2^(SW-1) - 1 and SYMBOL_MIN = -2^(SW-1) + SYMMETRIC_ALPHABET. Contract versions 4 and
5 changed only the stream's scale input and its escape. The cores' logic is that of version 3.

### The scale lookup `pans_scale_of_b`

`rtl/pans_scale_of_b.sv` converts b (B_WIDTH bits) to a `scale_t`. The stream units use it. The cores do not. It is
combinational:

1. a leading-one detect over B_WIDTH bits;
2. the select of the B_K bits below the leading one. b is extended by B_K zeros, so the zero fill is wiring;
3. the read of `SCALE_TABLE`. Every index that steps 1 and 2 produce is below SCALE_TABLE_SIZE.

The lookup is bit-exact with `scale_of_b`. At p12s8 it is 186 generic gates with depth 20 on its own
(`ppa/synth_generic.sh`), and 198-206 gates inside the units (STREAM.md §4.2). The decoder unit registers the code into
the lane's scale register when it takes `scale_in`, off the issue path. The encoder unit registers it into an input
register in front of the core. That register adds one cycle of latency (STREAM.md §2.2).

### What the stream units add

The cores are stateless. The stream units hold the ANS state x and perform the renormalization and the word I/O.

- On an encoder escape, the stream unit first pushes the raw symbol as SW bits of the state. It emits one word iff x ≥
  2^(P + W - SW), then sets x to x·2^SW + raw. It then pushes (M', E).
- On a decoder escape, the stream unit pops the raw symbol next: x mod 2^SW, then x >> SW.
- Every slot below M' lies in its decoded interval, and every raw value has SW bits. So a corrupt stream shows only in
  the stream unit's own checks: the end check, the words and the header (STREAM.md §1.6).
- A lane's next slot depends on the current decode. So one lane can issue a decode only every 4 cycles plus the unit's
  own stages. N interleaved lanes (Giesen's interleaved streams) fill the pipeline.

STREAM.md defines the stream: the lanes, the byte array and the errors. Its model is `ref/pans_stream_ref.h`. STREAM.md
§2-§4 cover the stream units `pans_enc_stream` and `pans_dec_stream`: interfaces, verification and PPA.

## 4. Build parameters and identity

### Inputs and configs

A build is fixed by P, SW, E and the alphabet's form (§1.1). `ci/matrix.toml` lists the configs. The Makefile passes a
config as `-DPANS_PRECISION`, `-DPANS_SYMBOL_WIDTH` and `-DPANS_ESCAPE_SLOTS`, plus `-DPANS_SYMMETRIC_ALPHABET=1` where
`symmetric_alphabet = true`. A config's name gives P and SW: p12s8 has P = 12 and SW = 8. The suffix sym marks a
symmetric alphabet, and e3 marks E = 3. p12s8 is the asymmetric build. p12s8sym is the same build with the symmetric
alphabet. p8s6 is the default config of `make sim`. The configs with PPA numbers are p16s16, p12s8, p12s8sym, p9s9, p8s8
and p4s4 (§7).

| Config | P / SW / E | Alphabet | CONFIG_ID at contract version 5 |
|---|---|---|---|
| p16s16 | 16 / 16 / 1 | asymmetric | 64'h62b888d2932d5a52 |
| p16s8 | 16 / 8 / 1 | asymmetric | 64'h28cc894d53afff4a |
| p12s8 | 12 / 8 / 1 | asymmetric | 64'h43ac3fbee674055b |
| p12s8sym | 12 / 8 / 1 | symmetric | 64'h1d049683290a1afa |
| p12s9 | 12 / 9 / 1 | asymmetric | 64'h20fc73dd88fc2afa |
| p12s10 | 12 / 10 / 1 | asymmetric | 64'h7f6aa4ed840cfb98 |
| p9s9 | 9 / 9 / 1 | asymmetric | 64'h6108b2cda840e6dd |
| p8s8 | 8 / 8 / 1 | asymmetric | 64'hc172b97b9392eadd |
| p8s6 | 8 / 6 / 1 | asymmetric | 64'h407643399e4d4b13 |
| p8s6e3 | 8 / 6 / 3 | asymmetric | 64'h12ef723f5b5eca11 |
| p4s4 | 4 / 4 / 1 | asymmetric | 64'h9f2db9e5c64fa9dd |

### Derived parameters

`pans1_config.h` and `pans1_ref.h` derive the other parameters. `ref/gen_pkg.cpp` emits them into
`build/<cfg>/gen/pans_pkg.sv`.

| Parameter | Value |
|---|---|
| mantissa bits | 3 |
| VF | max(P - 6, 4) |
| EW | the smallest w with 2^(w-1) - 1 ≥ max(ceil(log2(2(P + 2))) + 2, SW - 2) |
| A_WIDTH | bits(P·2^VF - 1) |
| PM_WIDTH | bits((2^SW + 3)·15). With a symmetric alphabet bits((2^SW + 1)·15). Both are 12 at SW = 8 |
| MAG_WIDTH | SW + 4 |
| DIV_SHIFT, DIV_DROP, DIV_MAGIC | the division constants (§2, step 3) |

EW lets the exponent cover k = 1/(b·ln2) from σ ≈ 2^SW up to k > 2(P + 2). Beyond σ ≈ 2^SW an alphabet of 2^SW symbols
is close to uniform, and a raw symbol costs less than any code. At k > 2(P + 2) every a saturates. The largest normal
value of the format lies a factor 2 above that. The widths favor area. Each narrowing that saves area is used, because
the rate overhead stays small (§5). `ref/sweep.cpp` reports the rate of the build and of its width variants.

| Config | EW / BIAS | VF | TAIL_MAX | A / PM / MAG width | DIV_SHIFT / DROP / MAGIC bits |
|---|---|---|---|---|---|
| p16s16 | 5 / 15 | 10 | 32752 | 14 / 20 / 20 | 26 / 16 / 23 |
| p16s8 | 5 / 15 | 10 | 32752 | 14 / 12 / 12 | 18 / 9 / 15 |
| p12s10 | 5 / 15 | 6 | 2032 | 10 / 14 / 14 | 20 / 11 / 17 |
| p8s8 | 4 / 7 | 4 | 124 | 7 / 12 / 12 | 18 / 9 / 15 |
| p8s6, p8s6e3 | 4 / 7 | 4 | 124 | 7 / 10 / 10 | 15 / 6 / 12 |
| p4s4 | 4 / 7 | 4 | 7 | 6 / 9 / 8 | 14 / 6 / 11 |

`gen_pkg` emits SYMBOL_MIN, SYMBOL_MAX and SYMMETRIC_ALPHABET (0 or 1). A `-DPANS_SYMMETRIC_ALPHABET` other than 0 or 1
does not compile (`pans1_config.h`). For the scale input `gen_pkg` emits B_FRAC (8), B_WIDTH (SW + 7), B_K (4),
B_LEAD_BITS, SCALE_INDEX_BITS, SCALE_TABLE_SIZE and SCALE_TABLE (§1.5). The table follows the include of
`pans_pkg_functions.svh`, which defines `scale_t`.

| Config | B_WIDTH | B_LEAD_BITS / SCALE_INDEX_BITS | SCALE_TABLE_SIZE (of them an integer b reaches) |
|---|---|---|---|
| p4s4 | 11 | 4 / 8 | 176 (127) |
| p8s6, p8s6e3 | 13 | 4 / 8 | 208 (159) |
| p8s8, p12s8, p16s8 | 15 | 4 / 8 | 240 (191) |
| p9s9, p12s9 | 16 | 4 / 8 | 256 (207) |
| p12s10 | 17 | 5 / 9 | 272 (223) |
| p16s16 | 23 | 5 / 9 | 368 (319) |

An index {lead, below} with lead < B_K has an integer b only if the low B_K - lead bits of below are 0. So 49 entries of
every table are never read.

### Identity: `CONFIG_ID`

`CONFIG_ID` is the FNV-1a hash of these fields, in order:

1. the model tag;
2. the contract version, `CONTRACT_VERSION` = 5;
3. (P, E, SW, EW, 3, VF, the symmetric-alphabet flag);
4. the scale input's (B_FRAC, B_K).

`pans_pkg` also carries `CONTRACT_VERSION`. The cores, `pans_scale_of_b` and the stream units check it at elaboration.
The DPI model reports `CONFIG_ID`, and the testbench refuses a mismatch.

### Contract versions

Every contract version changes every `CONFIG_ID`.

| Version | What it defines |
|---|---|
| 1 | the cores' discretized-Laplace C (§1.3), the exact-inverse decoder (§2), the escape rule. Symbol x owns [C(x), C(x + 1)), so the slots beyond the alphabet belong to no symbol. An end symbol with an empty [C(x), C(x + 1)) escapes. The hash has no version field |
| 2 | the end symbols own the slots beyond the alphabet. MIN's interval starts at 0 and MAX's ends at M', so the intervals tile [0, M') (§1.6). The inner symbols' intervals and the decoder's symbol for every slot stay those of version 1 |
| 3 | the symmetric-alphabet option (§1.1). A scale table holds no special code, only the nearest finite code |
| 4 | the scale input b (B_FRAC 8 on SW - 1 + 8 bits) and the coder's table `SCALE_TABLE` (B_K 4), which converts b to the cores' code (§1.5). The special scales become internal to the cores. The scale estimator that versions up to 3 built in, with its parameters hashed, leaves the build (STREAM.md §1.7). `ref/pans1_config.h` refuses the old `-DPANS_EST_*` flags, and `ci/matrix.py` refuses the old `est_*` keys. The stream's byte format does not change |
| 5 | an escape's raw value is pushed as SW bits of the lane state, not as P bits. An escape costs log2(M/E) + SW bits, 20 at p12s8 instead of 24 (STREAM.md §1.4). The decoder unit's err_raw and err_interval flags are removed |

An array of one version does not decode under another. It must be re-encoded. At SW = P the arrays of versions 4 and 5
are identical byte for byte.

### Refused builds

`gen_pkg` refuses a build with exit status 1 and prints the reasons on stderr. It refuses:

- VF > P;
- TAIL_MAX > 2^(P-1) - E. So E ≤ 16 at P = 16, E ≤ 4 at P = 8 and E ≤ 1 at P = 4;
- SW outside [2, P], or SW < 4;
- E < 1, or M - E < 2;
- a build with no exact division shift below 62;
- a scale input outside B_FRAC ≥ 0, B_K in [0, 8] and B_WIDTH in [1, 48]. The contract's B_FRAC 8 and B_K 4 never reach
  this case;
- any violation of the decodability contract (`contract_violations`).

The matrix lists two refused builds: p3s2 (VF > P, SW < 4) and p8s9 (SW > P). `gen_pkg --stream` also refuses the builds
that the stream units cannot implement (STREAM.md §1.1).

## 5. Rate

`make sweep` runs the rate sweep `ref/sweep.cpp` at p16s16 and writes `ref/results/sweep_p16s16.csv`. `make sweep-<cfg>`
runs it for another config. The row `build` is the coder as built.

### Method

The sweep compares the coder with an exact-math coder of the same model.

- The exact-math coder computes C = floor(M'·F(x - ½)) in double precision at the unrounded scale.
- It uses the same intervals. The end symbols own the slots beyond the alphabet (§1.6).
- It uses the same escape rule. The CSVs charge an escape 2P - log2 E bits, the stream's cost up to contract version
  4. From version 5 on, an escape costs P + SW - log2 E bits, which is the same at P = SW. The sweep keeps the
  version-4 charge. At p12s8 an escape costs 4 bits less under version 5. Around σ = 32, with 0.2% escapes, that is
  about -0.008 bits per symbol.
- The symbols are discretized-Laplace values clipped to the alphabet. MIN and MAX take the mass beyond the alphabet, as
  a saturating quantizer gives it to them.
- The sweep takes 32 values of σ per octave. The scale code rounds σ to its grid. One σ per point would sample one
  rounding error, not its spread.

### Overhead at P = SW = 16

| σ (octave around) | 1/4 | 1 | 4 | 16 | 64 | 256 | 1024 | 4096 | 8192 |
|---|---|---|---|---|---|---|---|---|---|
| mean % | 0.126 | 0.067 | 0.326 | 0.403 | 0.349 | 0.290 | 0.516 | 53.1 | 68.7 |
| max % | 0.508 | 0.234 | 0.446 | 0.479 | 0.384 | 0.366 | 1.046 | 65.2 | 70.5 |
| escapes % | 0.0009 | 0.0018 | 0.0044 | 0.017 | 0.070 | 0.28 | 1.11 | 49 | 74 |

VF sets the σ range, one octave per fraction bit. VF = 10 holds to σ = 1024 and degrades above it, +9% at σ = 2048. With
VF = 12 the mean is 0.229% at σ = 1024 and 0.351% at σ = 4096. No decode fails.

### The end rule's effect

At P = SW = 16 the end rule (contract version 2, §1.6) changes nothing through σ = 1024. Little mass lies beyond the
alphabet there. At p12s8 (`sweep_p12s8.csv`, σ up to 45) the rule matters in the top octave. Around σ = 32 (σ 23-45) the
core codes 6.9606 bits per symbol with the version-2 intervals and 6.9921 with the version-1 intervals, a difference of
-0.0315. The exact-math coder's difference is -0.0447. Escapes fall from 0.283% to 0.205%. Both runs use the same
clipped symbols. The version-1 numbers come from a scratch build of the sweep. In the octave around σ = 16 the core's
rate does not change at any σ, and the exact-math coder's mean moves by -0.0004. Below that octave nothing moves.

The symmetric build p12s8sym (`sweep_p12s8sym.csv`) clips the symbols to -127..127. Around σ = 32:

| Around σ = 32 | p12s8 | p12s8sym |
|---|---|---|
| core, bits per symbol | 6.9606 | 6.9597 |
| exact-math coder, bits per symbol | 6.9306 | 6.9296 |
| entropy of the clipped source, bits per symbol | 6.9081 | 6.9074 |
| escapes | 0.205% | 0.201% |

Below that octave every mean of p12s8sym equals p12s8's to 4 decimals.

### The scale table's cost

The row `build via b` feeds the build as the stream does: b = round(laplace_b(σ)·2^8), converted by SCALE_TABLE. The
other rows use the exact code, `scale_from_sigma`, which is the code nearest the exact b. The table gives the rate of
`build via b` over the rate of `build`, as mean / max over each octave. It is the sweep's fourth report.

| σ (octave around) | 1/4 | 1 | 4 | 16 | 32 (p12s8) / 64 | 256 | 1024 |
|---|---|---|---|---|---|---|---|
| p12s8, p12s8sym (the same to these digits) | +0.076% / 1.23% | +0.006% / 0.23% | +0.007% / 0.12% | +0.008% / 0.20% | +0.032% / 0.43% | | |
| p16s16 | +0.100% / 1.10% | +0.023% / 0.28% | +0.005% / 0.21% | -0.004% / 0.10% | +0.002% / 0.04% | +0.010% / 0.17% | +0.017% / 0.65% |

At p16s16, beyond VF's range, the difference is -0.21% at σ 4096 and -0.09% at σ 8192. Around σ = 1/4, b is 37-60 units
of 2^-8, two units per bucket. There B_FRAC's step and the bucket move the code most. From σ = 1 on the table costs
under 0.035% in the mean. The escapes move with it. Around σ = 32 they rise from 0.205% to 0.217% at p12s8 and from
0.201% to 0.214% at p12s8sym.

### Model mismatch: Gaussian data

The `gauss_` columns of the CSVs code discretized-Gaussian data with each row's Laplace model. The symbols are
round(X), X ~ N(0, σ²), clipped to the alphabet. MIN and MAX take the mass beyond the alphabet, as for the Laplace
data. The probabilities are exact, from erfc over the bins [x - ½, x + ½). Escapes are charged as in the Method.

- `gauss_bits`, `gauss_escape`: the coder at the row's scale code, the code of the Laplace data, from
  b = laplace_b(σ).
- `gauss_bits_best`, `gauss_b_best`: the coder at the normal scale code of least rate within 56 codes of the row's
  code (7 octaves of k at 3 mantissa bits), and that code's b.
- `gauss_entropy`: the entropy of the clipped discretized Gaussian.
- `gauss_kl`, `gauss_kl_best`, `gauss_b_kl_best`: the Kullback-Leibler divergence (KL) of the clipped discretized
  Laplace from the clipped discretized Gaussian, from the exact probabilities and without the coder, at
  b = laplace_b(σ) and at the b that minimizes it.

The sweep prints their mean and max over each octave after the scale table's cost. `make sweep-p12s8 sweep-p16s16`
reproduces the tables below from the `build` rows. Each entry is the mean over an octave's 32 values of σ. The excess
is the rate over the Gaussian's entropy, at the row's code (matched) and at the best code. The last column gives the
excess on Laplace data over the Laplace's entropy, for comparison. The sweep has no octave around σ = 32 at p16s16.

| p12s8, σ (octave around) | Gaussian entropy, bits | excess at the matched code, % / bits | excess at the best code, % / bits | KL at b / at the best b, bits | Laplace data: excess, % / bits |
|---|---|---|---|---|---|
| 1 | 2.1090 | 3.007 / 0.0627 | 2.480 / 0.0516 | 0.0701 / 0.0562 | 0.198 / 0.0039 |
| 4 | 4.0511 | 1.766 / 0.0713 | 1.564 / 0.0631 | 0.0789 / 0.0680 | 0.409 / 0.0163 |
| 16 | 6.0473 | 1.380 / 0.0834 | 1.267 / 0.0766 | 0.0807 / 0.0697 | 0.805 / 0.0481 |
| 32 | 7.0437 | 1.330 / 0.0936 | 1.172 / 0.0824 | 0.0804 / 0.0694 | 0.768 / 0.0526 |

| p16s16, σ (octave around) | Gaussian entropy, bits | excess at the matched code, % / bits | excess at the best code, % / bits | KL at b / at the best b, bits | Laplace data: excess, % / bits |
|---|---|---|---|---|---|
| 1 | 2.1090 | 2.856 / 0.0594 | 2.362 / 0.0490 | 0.0701 / 0.0562 | 0.080 / 0.0016 |
| 4 | 4.0511 | 1.674 / 0.0676 | 1.487 / 0.0601 | 0.0789 / 0.0680 | 0.341 / 0.0135 |
| 16 | 6.0473 | 1.308 / 0.0790 | 1.207 / 0.0729 | 0.0807 / 0.0697 | 0.443 / 0.0263 |
| 64 | 8.0471 | 1.038 / 0.0835 | 0.959 / 0.0771 | 0.0808 / 0.0698 | 0.468 / 0.0372 |
| 256 | 10.0471 | 0.861 / 0.0864 | 0.790 / 0.0793 | 0.0808 / 0.0699 | 0.667 / 0.0665 |

For continuous densities, the KL of the Laplace of the same variance, b = σ/√2, from the Gaussian does not depend on
σ. It is 0.5 + 2/(√π ln 2) - ½ log2(2πe) = 0.0808 bits. The b that minimizes it is the Gaussian's mean |X|,
b = σ√(2/π), where it is 0.0699 bits. The sweep's laplace_b(σ) tends to σ/√2 as σ grows. The discrete measurement
converges to these values from σ ≈ 8 up. Around σ = 4 the KL is 0.0789 bits at b and 0.0680 at the best b, around
σ = 16 it is 0.0807 and 0.0697, and from σ = 64 to 4096 at p16s16 0.0808 and 0.0698-0.0699. From σ ≈ 3 up the
minimizing b is σ√(2/π) to within 0.5%. Where the alphabet clips the tails, the KL falls: to 0.0804 around σ = 32 at
p12s8 (0.0789 at σ = 45), and to 0.0805 around σ = 8192 at p16s16.

The coder's excess differs from the KL by the coder's own departure from the exact model, in either direction. At
p16s16 it is 0.0676 bits around σ = 4, below the KL, and 0.0864 bits around σ = 256, above it. In the tables' octaves
the best code saves 0.006-0.011 bits per symbol over the matched code. For continuous densities the saving is 0.0110
bits.

Around σ = 1/4 (σ 0.18-0.35) the variance match fails. Below σ ≈ 0.29 the rounded Gaussian's variance falls below σ²,
to 0.16σ² at σ = 0.18, so the matched code is too wide for it. The excess at the matched code is 8.68% in the mean and
47.5% at most at p12s8, most of it below σ = 0.25. That is at most 0.031 bits per symbol, as the entropy is 0.05-0.77
bits there. The best code recovers almost all of it: 0.347% (0.0011 bits) in the mean at p12s8, 0.209% (0.0010
bits) at p16s16.

### Baseline: adaptive Golomb-Rice

The `rice_` columns of the CSVs code the Laplace data of each σ with a Golomb-Rice code at the parameter, from 0 to SW,
that costs least at that σ: the zigzag map 0, -1, 1, -2, 2, … → 0, 1, 2, 3, 4, …, the quotient in unary and as many low
bits as the parameter, with no escape, so the largest symbols take the plain Rice code of their zigzag value.
`rice_k` is the parameter and `rice_bits` the expected bits per symbol. On these data an adaptive Rice coder, which
chooses its parameter from past symbols, spends at least `rice_bits`. The sweep prints both codes' excess over the
Laplace data's entropy, then the build's rate against Golomb-Rice's, after the Gaussian reports. `make sweep-p12s8
sweep-p16s16` reproduces the tables below from the `build` rows. Each entry is the mean or the max over an octave's 32
values of σ.

| p12s8, σ (octave around) | build: excess, mean % / bits | build: max % | Golomb-Rice: excess, mean % / bits | Golomb-Rice: max % | the build's rate against Golomb-Rice's, mean % |
|---|---|---|---|---|---|
| 1/4 | 0.472 / 0.0018 | 1.110 | 187.8 / 0.6857 | 344.5 | -62.50 |
| 1 | 0.198 / 0.0039 | 0.345 | 3.700 / 0.0702 | 8.297 | -3.35 |
| 4 | 0.409 / 0.0163 | 0.540 | 1.762 / 0.0680 | 3.976 | -1.32 |
| 16 | 0.805 / 0.0481 | 1.012 | 1.009 / 0.0592 | 2.160 | -0.20 |
| 32 | 0.768 / 0.0526 | 1.148 | 1.173 / 0.0811 | 2.265 | -0.40 |

| p16s16, σ (octave around) | build: excess, mean % / bits | build: max % | Golomb-Rice: excess, mean % / bits | Golomb-Rice: max % | the build's rate against Golomb-Rice's, mean % |
|---|---|---|---|---|---|
| 1/4 | 0.147 / 0.0005 | 0.528 | 187.8 / 0.6857 | 344.5 | -62.62 |
| 1 | 0.080 / 0.0016 | 0.242 | 3.700 / 0.0702 | 8.297 | -3.47 |
| 4 | 0.341 / 0.0135 | 0.466 | 1.762 / 0.0680 | 3.976 | -1.39 |
| 16 | 0.443 / 0.0263 | 0.513 | 1.006 / 0.0590 | 2.160 | -0.55 |
| 64 | 0.468 / 0.0372 | 0.500 | 0.743 / 0.0584 | 1.559 | -0.27 |
| 256 | 0.667 / 0.0665 | 0.837 | 0.592 / 0.0584 | 1.229 | +0.08 |
| 1024 | 1.777 / 0.2133 | 2.722 | 0.492 / 0.0584 | 1.015 | +1.28 |
| 4096 | 59.735 / 8.3580 | 74.040 | 0.425 / 0.0590 | 0.865 | +59.07 |
| 8192 | 81.727 / 12.1562 | 85.878 | 0.823 / 0.1228 | 1.991 | +80.24 |

Golomb-Rice spends at least one bit per symbol, so around σ = 1/4, where the entropy is 0.24-0.65 bits, its excess is
188%. From σ = 1 on its excess is 0.06-0.08 bits per symbol in each octave's mean (0.12 around σ = 8192 at p16s16,
where the alphabet clips), highest where its parameter steps. The build's excess grows with σ, to 0.053 bits around
σ = 32 at p12s8 and 0.21 around σ = 1024 at p16s16. The build spends 3.4% less than Golomb-Rice around σ = 1 at p12s8
(3.5% at p16s16). From σ = 16 to 256 the two lie within 0.6% of each other in each octave's mean, and at single values
of σ either is ahead. Around σ = 1024, the top octave of p16s16's σ range, the build spends 1.3% more, and beyond
2^VF its rate collapses.

## 6. Verification

`make ci` runs the smoke tier of `ci/matrix.toml` on every config. A check passes with 0 errors. A testbench passes only
if it also reaches every coverage item it requires.

### 6.1 Exhaustive check of the model (`ref/test_inverse.cpp`)

`make inverse-<cfg>` runs it for one config, and `make inverse` for every config of the matrix. Both CI tiers run it on
all 11 configs. It checks:

- the division against the exact floor, for every mantissa and every R' in [0, DIV_N_MAX];
- the clamp;
- the threshold against a bisection on T, for every z';
- `decode_hw` == `decode` for every scale code and every slot;
- every slot below M' lies in the interval decoded for it, with freq ≥ 1, and decodes to a symbol of the alphabet;
- every slot from M' on decodes to the escape;
- per scale code, the intervals tile [0, M'). MIN's starts at 0, MAX's ends at M', and each starts at the previous one's
  end. `encode` reports each interval, coded iff non-empty;
- the mirror. f(x) = f(-x) for MIN + 2 ≤ x ≤ MAX - 1, and f(MAX) = f(MIN) + f(MIN + 1). With a symmetric alphabet,
  f(x) = f(-x) for every x, the ends included, and the input -2^(SW-1) escapes at every scale code;
- the round trip of every coded slot;
- C(x) + C(1 - x) = M' for every pair, and C(0) ≤ 2^(P-1) ≤ C(1);
- the tails lie within [2^-(2^-VF), 1.0615·(1 + E/M')] ± 1 slot of the exact Laplace (§1.3).

It also reports the slots beyond the alphabet and the end symbols they make coded (§1.6).

### 6.2 The scale input b and its table

`ref/test_stream.cpp` checks the table on all 11 configs (`make stream-ref`):

- the index, the bucket midpoints and every entry against plain restatements of §1.5's definition. The check covers
  B_FRAC 0, 3, 8 and 12 × B_K 0-5 and 8, for every b below 2^16 and for random b above;
- no entry is a special code;
- the slope k is non-increasing as b grows;
- b = 0 gives the narrowest finite code;
- `scale_from_sigma` = `scale_from_b(laplace_b(σ))`;
- the package's B_* parameters and SCALE_TABLE, entry by entry. No EST_* parameter is left;
- the stream's round trips reach every table index that an integer b has (§4).

The decoder and encoder unit testbenches check an instance of the RTL lookup `pans_scale_of_b` against the model, at
every b below 2^16. Wider builds also get both ends of every index and a random b. The testbenches check the units' own
instances at every step. The decoder's issue reads the code. The encoder's accept is checked for lane, symbol and code.
Four mutations of the table and the lookup fail both unit testbenches (STREAM.md §3).

### 6.3 Simulation of the cores (`make sim-<cfg>`)

`tb/simple/tb_pans_cores.sv` drives both cores and compares them with the reference through DPI. It checks:

- the `CONFIG_ID`;
- per scale, every SW-bit input and its round trip through the decoder. An input below a symmetric alphabet must escape;
- per scale, every slot. Its symbol must lie in the alphabet;
- per scale, a back-to-back stream, idle every 7th cycle. out_valid must equal in_valid delayed by 2 and 4 cycles, and
  the responses must come in order.

The default scales are σ = 0, ∞ and 2^-4 to 2^8 in quarter octaves. `build/<cfg>/sim/Vtb_pans_cores +ALL_SCALES` runs
every scale code.

Three further targets complete the core checks. `make lint` runs Verilator -Wall on both cores of every config. `make
refuse` passes iff the generator refuses every `[refused]` build. `make guard` passes iff the testbench rejects a C++
reference built for another config.

### 6.4 Gate level (`make power-<cfg>`)

The routed netlist is simulated at zero delay against the reference. It runs a corner set and then the power workload
(§7.1). The corner set has every scale code with the symbols {MIN, MIN + 1, -1, 0, 1, MAX - 1, MAX} and the slots {0, 1,
2^(P-1) ± 1, 2^(P-1), M' - 2, M' - 1, M', M - 1}. With a symmetric alphabet it adds the input -2^(SW-1). Where every
SW-bit input or every slot fits in 2^17 transactions, the corner set has all of them instead. At p12s8 and p12s8sym the
encoder's corner set therefore has every SW-bit input, -128 among them. A pass requires no mismatch.

| Config | Contract version | Result |
|---|---|---|
| p16s16 | 5 | no mismatch |
| p4s4 | 5 | no mismatch |
| p8s8 | 5 | no mismatch |
| p9s9 | 5 | no mismatch |
| p12s8 | 5 | no mismatch |
| p12s8sym | 5 | no mismatch |

### 6.5 Mutations

Each mutation was built and run once, then reverted.

| Mutation | Config, contract version | Fails |
|---|---|---|
| `decode_hw` without MAX's forced end | p12s8, 2 | `test_inverse`, on decode mismatches |
| the whole model at version 1 (the intervals and `decode_hw`) | p12s8, 2 | `test_inverse`, on slots outside their interval, tiling errors and mirror errors |
| decoder RTL without MAX's forced end | p12s8, 2 | `sim-p12s8`, first on the assertion `a_slot_in_interval` |
| encoder RTL without MIN's forced end | p12s8, 2 | `sim-p12s8`, on the reference compare |
| `decode_hw` with the lower half clamped at \|x\| = 128 | p12s8sym, 3 | `test_inverse`, on decode mismatches |
| `encode` without the alphabet test | p12s8sym, 3 | `test_inverse`, on -128 coded |
| decoder RTL: the clamped \|x\| of stage 2, `clamp_of` or `exponent_magnitudes` with the asymmetric lower bound | p12s8sym, 3 | `sim-p12s8sym`, on `a_slot_in_interval`, `a_monotone` or the reference compare |
| encoder RTL without either forced CDF of -128 | p12s8sym, 3 | `sim-p12s8sym`, on `a_slot_in_interval`, `a_monotone` or the reference compare |

STREAM.md §3 lists the mutations of the scale table and the lookup.

### 6.6 Not covered

No formal proof covers the current RTL. The exhaustive check covers the model. The stream units rely on the simulations
of STREAM.md §3.

## 7. PPA

### 7.1 Method

`make asic-<cfg> power-<cfg>` runs the cores through ORFS `26Q4-137-g9c431405a` on Nangate45 and reports post-route
numbers. Area excludes tap cells. It includes the input registers of the wrapper `ppa/pans_cores_timed.sv`.

Energy is power × target period, at one transaction per cycle. Power comes from the zero-delay activity of 2000
transactions, so it contains no glitches. The workload draws σ log-uniform in [1/4, MAX/2] and symbols from a
discretized Laplace of that σ (`laplace_b`). The scale is `scale_from_sigma`. One seed of 2000 transactions is used. An
earlier Logistic workload gave the same numbers within 2%. `POWER_STIMULUS=static` or `random` replaces this workload by
a constant or a uniformly random one of the same length (§7.6); the results go to `power_static/` or `power_random/`.

`ci/matrix.toml` sets the target periods with `asic_period_ns`: 5.0 for p16s16 and 3.0 for p4s4, p8s8, p9s9, p12s8,
p12s8sym and p12s9. §7.2 quotes every one of them but p12s9. At this tree `make asic-<cfg> power-<cfg>` reproduces every
row of the current design to the last digit: the version-5 rows of §7.2, §7.5 (with `ASIC_PLATFORM=asap7`) and §7.6
(with `POWER_STIMULUS`). The older rows show the differences between versions (§7.3).

Nangate45 is an academic 45 nm library. The numbers compare designs within this flow, not with a commercial process.
`ASIC_PLATFORM=asap7` runs the same flow on the ASAP7 kit (§7.5).

### 7.2 Results

Every row is post-route on Nangate45 with ORFS `26Q4-137-g9c431405a` at the row's target period. The energies are on the
workload of §7.1.

| Config (build), target, contract version | Encoder: area / Fmax / energy per symbol | Decoder: area / Fmax / energy per symbol |
|---|---|---|
| p12s8 (asymmetric alphabet), 3 ns, version 5 (2026-10-18) | 1,593 µm² / 709 MHz / 3.00 pJ | 3,506 µm² / 762 MHz / 6.16 pJ |
| p12s8sym (symmetric alphabet), 3 ns, version 5 (2026-10-18) | 1,553 µm² / 765 MHz / 3.00 pJ | 3,593 µm² / 735 MHz / 6.26 pJ |
| p16s16, 5 ns, version 5 (2026-10-18) | 2,782 µm² / 605 MHz / 5.29 pJ | 6,005 µm² / 651 MHz / 9.84 pJ |
| p4s4, 3 ns, version 5 (2026-10-18) | 689 µm² / 1,272 MHz / 1.23 pJ | 2,089 µm² / 836 MHz / 3.54 pJ |
| p8s8, 3 ns, version 5 (2026-10-18) | 1,287 µm² / 893 MHz / 2.46 pJ | 3,025 µm² / 720 MHz / 5.25 pJ |
| p9s9, 3 ns, version 5 (2026-10-18) | 1,503 µm² / 884 MHz / 2.82 pJ | 3,642 µm² / 762 MHz / 6.00 pJ |
| p12s8sym (symmetric alphabet), 3 ns, version 3. The cores of versions 4 and 5 are unchanged; a rerun at version 5 on 2026-10-17 gave the same numbers to the last digit | 1,553 µm² / 764 MHz / 3.00 pJ | 3,592 µm² / 738 MHz / 6.25 pJ |
| p12s8 (asymmetric alphabet), 3 ns, version 2 | 1,593 µm² / 711 MHz / 3.00 pJ | 3,510 µm² / 766 MHz / 6.19 pJ |
| p12s8 (asymmetric alphabet), 3 ns, version 1 | 1,469 µm² / 692 MHz / 2.82 pJ | 3,360 µm² / 725 MHz / 5.91 pJ |
| p16s16, 5 ns, version 1 | 2,540 µm² / 571 MHz / 4.7 pJ | 5,988 µm² / 592 MHz / 9.9 pJ |
| p4s4, 3 ns, version 1 | 645 µm² / 1,280 MHz / 1.2 pJ | 2,095 µm² / 851 MHz / 3.5 pJ |

### 7.3 Differences between contract versions

The ORFS and energy rows below are post-route on Nangate45 with ORFS `26Q4-137-g9c431405a` at 3 ns. The cores' energies
are on the workload of §7.1, the stream units' on the EMA workloads of STREAM.md §4.1.

**Version 1 → 2 at p12s8.** The end rule adds two equality compares and a flop to the encoder's stage 1. It adds two
compares to the decoder's stage 3. Both rows come from a clean `build/`. The flow is deterministic, so a rerun
reproduces every number.

| p12s8, version 1 → 2 | Encoder | Decoder |
|---|---|---|
| generic gates (`make ppa-p12s8`) | 775 → 778 | 2,176 → 2,185 |
| generic depth | 42 → 42 | 45 → 44, the multiplier stage |
| ORFS area | +124 µm² (+8.4%) | +150 µm² (+4.5%) |
| ORFS Fmax | +19 MHz | +40 MHz |
| energy per symbol | +0.19 pJ | +0.28 pJ |
| the same compares written bit by bit (not in the tree) | 1,684 µm² / 734 MHz | 3,432 µm² / 736 MHz |
| inside the stream units (STREAM.md §4.2) | +114 µm² (+1.1%) and one flop | +60 µm² (+0.5%) |

The longest path does not change. Most of the ORFS difference comes from the mapping, not from the logic. The bit-by-bit
form, x[S-1] && !(|x[S-2:0]) and the like, computes the same function. So at this size structurally equivalent RTL moves
the cores by ±6% in area. The stream units' noise band is ±3% (STREAM.md §4.1).

**Version 2 → 3: p12s8 → p12s8sym.** The symmetric alphabet adds the compare for -128 to the encoder. In the decoder the
lower half's clamp becomes the upper half's. Both rows come from a clean `build/` on the same day. The version-2 row
reran to the last digit.

| p12s8 (version 2) → p12s8sym (version 3) | Encoder | Decoder |
|---|---|---|
| generic gates | 778 → 797 | 2,185 → 2,107 |
| generic depth | 42 → 41 | 44 → 44 |
| generic gates of `clamp_of` / `exponent_magnitudes` | | 82 → 50 / 257 → 221 |
| ORFS area | -40 µm² (-2.5%) | +82 µm² (+2.3%) |
| ORFS Fmax | +53 MHz | -28 MHz |
| energy per symbol | ±0.00 pJ | +0.06 pJ (+1.0%) |
| the stream unit (STREAM.md §4.2) | +51 µm² (+0.5%), +1.4% energy | -352 µm² (-3.0%), -4.4% energy |

The ORFS differences lie inside the ±6% of structurally equivalent RTL. An asymmetric build's cores synthesize to the
same generic gates as at version 2 (§1.8).

**Versions 4 and 5** change no core logic. Only the constant of the elaboration check changes. Version 4 adds the scale
lookup `pans_scale_of_b` (186 generic gates, depth 20, on its own at p12s8) and the encoder unit's input register.
Version 5 changes the raw push in the stream units. STREAM.md §4.2 gives their cost.

Both builds were rerun at version 5 on 2026-10-17 from a clean `build/` with `make asic-<cfg> power-<cfg> ppa-<cfg>`.
p12s8 reproduces the version-2 row's areas and energies to the last digit, and its encoder's Fmax. Its decoder reaches
770 MHz against 766. Its generic gates are version 2's: 778 (depth 42) and 2,185 (depth 44). p12s8sym reproduces its
version-3 row to the last digit, with 797 / 41 and 2,107 / 44 generic gates. Both have 0 netlist mismatches, in the
corners and in the workload. The commits since then rename and comment and leave every generic gate count as it was;
on 2026-10-18 the same targets gave the version-5 rows of §7.2, within 1% in Fmax and 0.6% in energy of these.

### 7.4 Measured alternatives in the cores' arithmetic

The RTL uses the first form in each row. The gate counts are generic gates at p16s16.

| Function | Form in the RTL | Alternative | Alternative's cost |
|---|---|---|---|
| `clamp_of` | compares of R''s top K_WIDTH bits with k_m and k_m - 1, and of its low SW bits | a generic 20-bit compare against the computed bound | +250 gates |
| `exponent_magnitudes` | hi = R' + (2k_m - rem), in parallel with lo = R' - rem | hi = lo + 2k_m | 67 gates fewer, but one more carry chain in series at the end of the stage, about 0.25 ns at Nangate45 |
| the clamp's lo | a subtraction | the concatenation {k_m - 1, -(3 - 2·w)·k_m mod 2^SW}, with w = 1 in the lower half of an asymmetric alphabet and 0 otherwise | 16 gates fewer, but wrong for k_m = 0 and for 3·K_MAX ≥ 2^SW (p4s4) |
| DIV_MAGIC[man] | selected in the decoder's stage 1 and registered | selected at the head of the multiplier's path | the select and the mantissa's fanout add 0.6 ns to that path |

### 7.5 ASAP7

`make asic-p12s8 power-p12s8 ASIC_PLATFORM=asap7` runs the cores through the flow of §7.1 on ASAP7, a predictive 7 nm
FinFET process design kit from Arizona State University (Clark et al. 2016). ASAP7 is not a manufacturable process. Its
numbers compare designs at a 7 nm-class node within this flow, not with a commercial 7 nm process. The results go to
`build/p12s8/asic_asap7/` and `build/p12s8/power_asap7/`, with every ORFS log.

**Method.** Only the platform and the period differ from §7.1:

- **Platform and corner.** ORFS `26Q4-137-g9c431405a`, platform `asap7`, typical corner. `ppa/asic/config.mk.in` sets
  `CORNER = TC`, because ORFS's default for asap7 is the fast corner (FF, 0.77 V). The libraries are the RVT NLDM
  libraries at TT, characterized at 0.7 V and 25 °C (operating conditions `PVT_0P7V_25C`). The same five files serve
  synthesis, place and route, the cell models of the gate-level simulation and OpenSTA's power, all in
  `/OpenROAD-flow-scripts/flow/platforms/asap7/lib/NLDM/` of the image: `asap7sc7p5t_AO_RVT_TT_nldm_211120.lib.gz`,
  `asap7sc7p5t_INVBUF_RVT_TT_nldm_220122.lib.gz`, `asap7sc7p5t_OA_RVT_TT_nldm_211120.lib.gz`,
  `asap7sc7p5t_SEQ_RVT_TT_nldm_220123.lib` and `asap7sc7p5t_SIMPLE_RVT_TT_nldm_211120.lib.gz`.
- **Period.** 1.0 ns, `asic_period_ns_asap7` of p12s8. Every block closes there: the cores with +0.217 and +0.170 ns of
  slack, the stream units with +0.054 and +0.040 ns on their worst flop paths (STREAM.md §4.6). The target is tighter
  than Nangate45's 3 ns, where the cores keep about +1.6 ns.
- **Settings.** `CORE_UTILIZATION` 40 and the adder map off, as on Nangate45. Everything else is ORFS's default for
  asap7: placement density 0.60, routing on M2-M7, and its dont-use list. That list excludes every fractional-drive
  cell (`*xp*`, `*x1p*`, such as NAND2xp33), the scan flops and the integrated clock gates.
- **Geometry and wires.** Areas are at the kit's drawn dimensions (the 1x LEF, 7.5-track rows 0.27 µm high). The routed
  parasitics come from OpenRCX with ORFS's ASAP7 extraction rules, as on Nangate45 with its own.
- **Workload and checks.** As in §7.1 and §6.4. Energy is power × 1 ns.

**Results** (2026-10-18, contract version 5). Nangate45 is the version-5 row of §7.2 at 3 ns. The ratios are Nangate45
over ASAP7 for area and energy, and ASAP7 over Nangate45 for Fmax.

| p12s8 | Encoder core | Decoder core |
|---|---|---|
| area, the wrapper's input registers included | 129 µm² | 273 µm² |
| flops, the wrapper's included | 75 | 167 |
| worst slack at 1 ns, Fmax | +0.217 ns, 1,277 MHz | +0.170 ns, 1,204 MHz |
| power, energy per symbol | 0.50 mW, 0.50 pJ | 1.11 mW, 1.11 pJ |
| netlist check: workload, corners | 0 mismatches in 2,000, 0 in 32,768 | 0 in 2,000, 0 in 1,024 |
| Nangate45, 3 ns: area / Fmax / energy | 1,593 µm² / 709 MHz / 3.00 pJ | 3,506 µm² / 762 MHz / 6.16 pJ |
| ratios: area / Fmax / energy | 12.3× / 1.80× / 5.97× | 12.9× / 1.58× / 5.55× |

Both cores' worst paths run register to register inside the core: the encoder's from stage 1's `q1_shift` to the
output register `out_start`, the decoder's from stage 2's `q2_q_lo` to stage 3's `q3_a_x`. The Fmax is ORFS's
1 / (target - worst slack) at the 1 ns target. It is not a maximum frequency of the design, and the two platforms ran at
different timing pressure.

Leakage is under 0.3 µW in both cores: ASAP7's typical libraries leak almost nothing at 25 °C, so the energy is
switching and internal power. On Nangate45 leakage is about 4% of the energy at 3 ns.

**Beyond 7 nm.** Foundry disclosures give three steps from the 7 nm class to the 2 nm class: N7 → N5 about 1.84× logic
density and 30% less power; N5 → N3E about 1.6× and more than 30%; N3E → N2 1.2× for logic and 25-30% less power at the
same speed. Applied to the ASAP7 decoder unit of STREAM.md §4.6 (866 µm², 2.05 pJ per symbol decode) they give about
250 µm² and 0.7 pJ. That is arithmetic on a predictive kit's result, an order of magnitude, not a measurement. Sources:
N7 → N5, G. Yeap et al. (TSMC), "5nm CMOS Production Technology Platform featuring full-fledged EUV, and High Mobility
Channel FinFETs with densest 0.021µm² SRAM cells for Mobile SoC and High Performance Computing Applications," IEDM 2019
([TSMC research](https://research.tsmc.com/page/logic/7.html), [IEEE Xplore](https://ieeexplore.ieee.org/document/8993577)).
N5 → N3E, TSMC, ["Optimized for Both Smartphones and HPC
Applications"](https://www.tsmc.com/english/dedicatedFoundry/technology/platform_smartphone_tech_advancedTech), accessed
2026-10-18; the page does not say at which speed the power is saved. N3E → N2, TSMC's 2 nm statements as quoted by
A. Shilov, ["TSMC begins quietly volume production of 2nm-class chips
…"](https://www.tomshardware.com/tech-industry/semiconductors/tsmc-begins-quietly-volume-production-of-2nm-class-chips-first-gaa-transistor-for-tsmc-claims-up-to-15-percent-improvement-at-iso-power),
Tom's Hardware, 29 December 2025, a secondary source.

### 7.6 Workload sensitivity

`make power-p12s8 POWER_STIMULUS=static` and `POWER_STIMULUS=random`, with `ASIC_PLATFORM=asap7` for ASAP7, rerun the
cores' gate-level power on the routed netlists of §7.2 and §7.5 with two other workloads of 2,000 transactions. static
repeats one transaction: the mid-alphabet symbol 0 at the mid-range b = 2^14, the scale code the coder's for that b
(`scale_of_b`), the decoder's slot the middle of the symbol's interval. Nothing escapes: it is the activity floor.
random draws the symbols uniform over the alphabet and b uniform over its 15 bits, the decoder's slot uniform inside the
interval, escapes included: the ceiling. Every netlist check, workload and corners, has 0 mismatches. The percentages
are against the Laplace workload of §7.1, run on the same day from the same netlists (2026-10-18, contract version 5).

| p12s8, pJ per symbol decode / encode | escapes in 2,000 | decoder core, Nangate45 3 ns | decoder core, ASAP7 1 ns | encoder core, Nangate45 | encoder core, ASAP7 |
|---|---|---|---|---|---|
| static | 0 | 1.99 (-67.7%) | 0.43 (-61.0%) | 0.92 (-69.4%) | 0.16 (-68.4%) |
| Laplace (§7.1) | 4 | 6.16 | 1.11 | 3.00 | 0.50 |
| random | 167 | 6.34 (+2.9%) | 1.15 (+3.3%) | 2.94 (-2.3%) | 0.51 (+0.4%) |

The uniformly random workload costs within 3.3% of the Laplace one on both platforms, with about 40 times its escapes.
A core that sees the same transaction every cycle spends 31-39% of it: only the clock tree and the flops' clock pins
still switch.
