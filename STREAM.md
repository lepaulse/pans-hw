# pANS stream: N interleaved lanes over the cores

Stream specification version 0.8, 2026-10-17. Contract version 5.

This document specifies the stream of the pANS hardware coder ([SPEC.md](SPEC.md)). N interleaved lanes of ANS state run
over the cores. All lanes share one word stream and one byte array. Two stream units, `pans_enc_stream` and
`pans_dec_stream`, encode and decode the stream. The scale of every symbol, the scale input b, is an input. The
probability model that chooses b lies outside the coder. §1.7 gives example sources, among them an adaptive one that
hardware can implement.

**Contents.** §1 defines the stream. §2 describes the units `rtl/pans_dec_stream.sv` and `rtl/pans_enc_stream.sv`. §3
covers their verification on every config that the stream build accepts. §4 gives their PPA at p12s8 and p12s8sym, N =
8, 3 ns, on Nangate45. The PPA ledgers `rtl/PPA_DEC_STREAM.md` and `rtl/PPA_ENC_STREAM.md` record every optimization
candidate measured on the units.

**Normative reference.** The bit-exact C++ model `ref/pans_stream_ref.h` defines the stream. It builds on the cores'
model, `encode` and `decode` in `ref/pans1_ref.h` (SPEC.md). Its self-test is `ref/test_stream.cpp`. `make
stream-ref-<cfg>` runs the self-test for one config, and `make stream-ref` for every config. The testbenches reach the
model through the DPI `ref/pans_stream_dpi.cpp`, whose header gives the interface.

**Relation to pANS 1.0.** pANS 1.0 is the pANS software coder, with 16-bit words and a 32-bit state. The per-lane
functions of this stream are pANS 1.0's `push` and `dec_pop`, generalized from P = W = 16 to the build. The interval of
a symbol is the cores', not pANS 1.0's Logistic C and candidate. `ref/test_stream.cpp` §4 carries a literal
transcription of pANS 1.0's per-lane functions and checks the model against it at P = W = S = 16.

**Names.** S is the symbol width, SYMBOL_WIDTH, written SW in SPEC.md. p12s8 is the 12/8 build with the asymmetric
alphabet. p12s8sym is the same build with the symmetric alphabet. Every measurement names the config it was made on.

## 1. Stream definition

### 1.1 Parameters and widths

The build supplies P, S, E and the alphabet (`ci/matrix.toml`, SPEC.md §1.1). The alphabet is every S-bit value, or,
with `symmetric_alphabet`, the range -(2^(S-1) - 1) .. 2^(S-1) - 1. With the symmetric alphabet the cores escape the
S-bit input -2^(S-1). N is the number of lanes, the units' parameter `N_LANES`, default 8. M = 2^P and M' = M - E.

`Config::word_width`, `state_width` and `state_bytes` in `ref/pans1_ref.h` derive the widths. `ref/gen_pkg.cpp` emits
them into `pans_pkg` as `WORD_WIDTH`, `STATE_WIDTH` and `STATE_BYTES`.

- W = 8·ceil(P/8). Words are whole bytes, and W ≥ P. So a push or a pop moves at most one word. This holds for the
  escape's raw push and pop too, since S ≤ P ≤ W (§1.4).
- STATE_WIDTH = P + W. Between symbols the state x lies in [L, L·2^W), with L = M (pANS 1.0's L_LOW, SPEC.md §0).
- STATE_BYTES = ceil(STATE_WIDTH / 8), the size of a state stored in the array.

| Config | W | STATE_WIDTH | STATE_BYTES |
|---|---|---|---|
| p4s4 | 8 | 12 | 2 |
| p8s6, p8s8, p8s6e3 | 8 | 16 | 2 |
| p9s9 | 16 | 25 | 4 |
| p12s8, p12s9, p12s10 | 16 | 28 | 4 |
| p16s8, p16s16 | 16 | 32 | 4 |

`gen_pkg --stream` refuses a build that the units cannot implement. The units need E to be a power of two, because the
encoder's escape push with f = E is a shift. So p8s6e3 is refused for the units. The model itself takes any E, and
`test_stream` passes on p8s6e3.

The widths do not enter `CONFIG_ID`. They are functions of P, which the hash already contains. N is a parameter of the
units, not of the build. The scale input's B_FRAC and B_K (§1.7) are hashed. A scale estimator is not part of the build.

### 1.2 Symbol numbering

Symbol i, the global index with 0 ≤ i < n, belongs to lane i mod N at position t = i div N. n can be any count. Lanes j
< n mod N hold one symbol more than the others. With n < N some lanes hold no symbol, and their state stays L. The
decoder outputs the symbols in the order i = 0, 1, …, n - 1.

### 1.3 Orders: the interleaved-ANS invariant

The decoder takes i = 0, 1, …, n - 1. The encoder pushes i = n - 1, …, 0, the exact reverse, with every lane starting
from x = L. Four properties follow:

1. Per lane, the decoder meets the symbols in the reverse order of their pushes, as a stack.
2. Decode step i undoes push i. It starts in the state that push i left and ends in the state that push i found.
3. Decode step i reads exactly the words that push i emitted, last emitted first. A pop refills iff its push emitted: x'
   < L after the pop iff x ≥ f·2^W before the push. For an escape's raw value the condition is x ≥ 2^(P + W - S) (§1.4).
4. So the words, listed last emitted first, are in the decoder's consumption order. One word stream serves all lanes.
   There are no per-lane counts or offsets, and the decoder never needs to know which lane a word belongs to.

`test_stream` checks properties 2 and 3 at every step, with both states in [L, L·2^W). It checks property 4 on the
array. It also checks that each lane's share of the words equals that lane encoded alone. The checks run for N ∈ {1, 2,
8, 16}, with unequal lane lengths and empty lanes.

The interleaving is Giesen's interleaved rANS. Lanes that each kept a stream of their own would push t ascending and
decode t descending. Giesen's construction reverses both orders, so one word stream serves every lane. This stream's
layout follows from it: one byte array with the lane states as its header (§1.5), and the decoder's output in symbol
order.

### 1.4 Per lane

```
push(c, f):   if (f < M && x >= f << W) { emit(x mod 2^W); x >>= W; }        // pANS 1.0's push; f < M always here
              x = (x / f) * M + c + (x mod f)
push_raw(v):  if (x >= 2^(P + W - S)) { emit(x mod 2^W); x >>= W; }           // stream_push_raw
              x = x * 2^S + v                                                 // v < 2^S: S bits of the state
encode(sym):  (c, f, esc) = core(scale, sym)                                  // pans1_ref.h encode
              coded: push(c, f);  escaped: push_raw(raw); push(M', E)         // raw = sym mod 2^S (S <= P)

pop(s, c, f): x = ((x >> P) * f + (s - c)) mod 2^STATE_WIDTH                  // pANS 1.0's dec_pop
              if (x < L) x = (x << W) | next_word()
pop_raw():    v = x mod 2^S; x = x >> S                                       // stream_pop_raw
              if (x < L) x = (x << W) | next_word()
decode():     s = x mod M; (sym, c, f, esc) = core(scale, s)                  // pans1_ref.h decode
              esc (s >= M'):  pop(s, M', E); v = pop_raw()                   // two pops, a refill each
                              symbol = v sign-extended from S bits
              coded:          pop(s, c, f); symbol = sym                    // s in [c, c + f): SPEC.md §1.6
```

**The raw push.** An escape's raw value has S bits. The raw push stores it as S bits of the state, one of 2^S values of
its own. It is the standard ANS push of a uniform symbol with frequency 1 of 2^S, and only the escape rule, which
escapes a symbol whose interval is empty at its scale (SPEC.md §1.7), is this coder's own. Pushed as push(raw, 1), one
slot of all M, it would spend P - S bits on nothing. An escape therefore costs log2(M/E) + S bits, not log2(M/E) + P. At
p12s8 that is 20 bits instead of 24. Contract versions 1-4 used push(raw, 1) (SPEC.md §4).

The bound 2^(P + W - S) keeps the state in [L, L·2^W) with at most one word, for every x in that range:

- Without the word, x < 2^(P + W - S). So x·2^S + v ≤ 2^(P + W) - 2^S + 2^S - 1 < L·2^W. And x ≥ L gives x·2^S + v ≥ L.
- With the word, x >> W lies in [2^(P - S), 2^P), because x < 2^(P + W). So x·2^S + v lies in [L, 2^(P + S)). That range
  is below L·2^W, since S ≤ W.

The decoder undoes the raw push. (x·2^S + v) >> S is the state that the push found. That state is ≥ L iff the push
emitted nothing. Otherwise it is x >> W < L, and the refill brings the emitted word back.

Two special cases:

- At S = W, which implies S = P = W (p8s8, p16s16), the bound is L and every raw push emits.
- At S = P the bound is 2^W = 1 << W, and x·2^S + v = x·M + v. push_raw is then push(raw, 1) and pop_raw is pop(v, v,
  1), the rule of contract versions 1-4 and of pANS 1.0. So at S = P (p16s16, p8s8, p9s9, p4s4) every array is identical
  byte for byte to version 4's. Only `CONFIG_ID` differs.

In the encoder unit the bound is an OR of x's top S bits, and x·2^S + v is a concatenation. In the decoder unit the raw
pop is a shift.

**Symbols and raw values.** The stream takes every S-bit value as a symbol (`symbol_valid`). With a symmetric alphabet
the core gives the input -2^(S-1) no interval, so it escapes at every scale. Its raw value 2^(S-1) decodes back to
-2^(S-1). The decoder's core never returns -2^(S-1) for a coded slot. At S = 16 the conversion raw = sym mod 2^S and the
sign extension of v are pANS 1.0's casts (uint16_t)(int16_t)sym and (int16_t)(uint16_t)v.

**Arithmetic.** On every stream that an encoder produces, nothing wraps. The mod 2^STATE_WIDTH and the guard f < M are
pANS 1.0's 32-bit unsigned arithmetic at P = W = 16. They make corrupt input decode deterministically (§1.6).

**Words per symbol.** A coded symbol moves 0 or 1 words. An escape moves 0, 1 or 2 words. Two words in one escape
require W + log2 E < P + S. The raw push emits only from x ≥ 2^(P + W - S) and leaves x·2^S + raw < 2^(P + S). That
value must reach E·2^W for the escape push to emit too. At S = P = W with E = 1 every escape moves exactly two words,
and the raw value is one of them, verbatim. At p4s4, where W = P + S, every escape moves exactly one word.

**Where the raw value lands.** Where the escape push emits, its word is the low W bits of x·2^S + raw. The raw value
sits in the word's low S bits. Where the escape push does not emit, the raw value's bits from log2 E up sit at the
state's bits P and up. Its low log2 E bits sit at the bottom, in M' + r. The lane's next push that emits takes the
state's low W bits. Where P < W < P + S - log2 E (p9s9, p12s8, p12s9, p12s10), that word holds W - P of the raw value's
bits, its low W - P bits where E = 1. The state holds the rest, and the raw value crosses a word boundary. `test_stream`
checks that word and counts such escapes (§1.9).

### 1.5 The byte array

The array is little-endian throughout:

| Bytes | Content |
|---|---|
| `[0, N·STATE_BYTES)` | the encoder's final states, lane 0 first, STATE_BYTES each |
| then W/8 bytes per word | the words in the decoder's consumption order, which is the encoder's emission order reversed |

Length = N·STATE_BYTES + (W/8)·words. The encoder emits words in emission order. The host that stores them writes them
reversed. The units never buffer a stream. The decoder needs (n, n_bytes) besides the array.

### 1.6 Decoding: errors and the end check

- **Header.** Each state is loaded modulo 2^STATE_WIDTH. A stored state outside [L, 2^STATE_WIDTH) sets **err_header**.
  That covers a state below L and a state with a bit set above STATE_WIDTH. No encoder writes such a state.
- **Reads beyond the array.** A read of a header byte or of a word past the last whole word gives 0 and sets
  **err_words**. pANS 1.0's prefetched word is likewise 0 once the words are used up. A trailing partial word is never
  read.
- **No interval error and no raw-value error.** The intervals tile [0, M') (SPEC.md §1.6). So on any input every slot
  below M' lies inside the interval decoded for it. The decoder unit asserts this property in simulation (`a_interval`).
  An escape's raw value v has exactly S bits (§1.4), so it is never out of range. The only flags are err_words and
  err_header. Contract version 1 raised a flag err_interval on the slots beyond the alphabet. From version 2 on that
  flag could not fire. Before version 5 a flag err_raw caught v ≥ 2^S, possible only where S < P. Version 5 removed both
  flags (SPEC.md §4).
- **Errors are sticky.** Decoding always runs to the n-th symbol with the arithmetic of §1.4.
- **ok** = no error flag ∧ every lane back at L ∧ n_bytes = N·STATE_BYTES + (W/8)·words_consumed. So every byte is
  consumed, and no word and no byte is left over. This is pANS 1.0's `dec_ok` (x = 2^16, every word read once) plus the
  flags and the byte count.
- **ok = 1 is not a checksum.** A corrupted array that is itself an encoding of other symbols decodes with ok = 1.
  `test_stream` counts such collisions. It proves each one by re-encoding the decoded symbols back to the corrupted
  array exactly. Most collisions are a corrupted escape raw value. The raw pop takes v's S bits, and the state goes on
  from the bits above them. So a flipped raw bit changes that symbol and nothing else. `test_stream` and the decoder
  testbenches flip one on purpose and require ok = 1 with only that symbol changed. At P = 8 and P = 4 the end check
  itself is short enough to collide sometimes. The format has no checksum. Integrity beyond the end check belongs to the
  system that stores and transports the array.
- **Agreement with pANS 1.0.** Errors and ok agree with pANS 1.0's. At P = W = S = 16 and N = 1, the transcription
  decodes flipped and truncated arrays to the same symbols and state, with the same end check (`test_stream` §4).

### 1.7 The scale input b and the coder's table

Every symbol's scale is the **scale input b** (SPEC.md §1.5). b is the Laplace scale that the model produces, in symbol
steps. It is unsigned fixed point with B_FRAC = 8 fraction bits on B_WIDTH = S - 1 + B_FRAC bits: 15 at p12s8 and 23 at
p16s16. The model produces b. The coder handles everything from b inward.

Both units convert b into the cores' scale code with **the coder's table**, `SCALE_TABLE[{lead, below}]`. lead is the
position of b's leading one, with b = 0 giving lead = 0. below is the B_K = 4 bits below it, zero-filled below bit 0.
The RTL is `rtl/pans_scale_of_b.sv` and the model is `scale_of_b`. The decoder unit converts b when it takes `scale_in`
and stores the code in the lane's scale register (§2.1). The encoder unit converts it into its input register (§2.2).
The model converts b per symbol in `encode_stream` and `stream_decode_next`.

b = 0 maps to the narrowest finite code. Symbol 0 then takes [0, M') and every other symbol escapes, as at σ = 0. A b
beyond the format's widest scale maps to the widest finite code {1, 0}. The special codes stay internal to the cores.
SPEC.md §1.5 and §4 give the table's definition, its sizes and the rule for b = 0. SPEC.md §5 gives its rate cost
against the exact code. The table is data. An implementation reads it from `pans_pkg` (`SCALE_TABLE`) and never
recomputes it from doubles, because one ulp can move a code. An array made under another contract version does not
decode and must be re-encoded (SPEC.md §4).

**The consumer's loop.** The decoder gets the b for (lane, t) from the consumer. The consumer computes it from the
lane's decoded symbols. The model's interface is `ScaleSource`. `scale(lane, t)` returns b, a `ScaleB`. `observe(lane,
t, symbol)` passes a decoded symbol. `history_needed(t)` states the source's delay. Its rules:

- The b for (j, t) may depend only on lane j's symbols at positions < `history_needed(t)` ≤ t.
- The source is asked once per (j, t), with t ascending.
- The answer must not depend on when it is asked. A consumer that is behind supplies b ahead.

The encoder's b are the host's replay of the same source over the symbols in decode order (`replay_scales`).

**Sources in the model.** Each source produces b:

- constant;
- a list in global order. The list holds random b, log-uniform over b's octaves with both ends of its range, 0 and
  2^B_WIDTH - 1, or directed values;
- `EmaScaleSource`, an integer moving average of |symbol| per lane. It updates m ← m + floor(((|symbol| << 8) - m) / 8),
  starting from σ = 4's mean |symbol|, m = 724. Its output is b = max(m, 45), where 45 is σ = 1/4's mean |symbol|. The
  units' tests use it as their first consumer, and the power workloads use it;
- `IntScaleSource`, the example integer scale estimator below. Hardware can implement this form.

The EMA and INT sources take a delay d. The b for t uses the lane's symbols before t - d. The tests use d = 0, 1 and 2.

**The example integer scale estimator** (`IntScaleSource`, `ref/pans_stream_ref.h`). It is an adaptive consumer, defined
so that an RTL estimator can be verified against it bit for bit. It is not part of the build. It emits b. Its parameters
are constructor arguments (`EstParams`), and none of them is in `CONFIG_ID`. Every run-time operation is on integers.
The parameters are FRAC, SHIFT, M0, MFLOOR and DELAY. Each lane keeps a state m, an unsigned integer of S - 1 + FRAC
bits. m holds |symbol| in units of 2^-FRAC steps.

| Step | Definition | Widths and rules |
|---|---|---|
| start | m = M0 | 0 ≤ M0 < 2^(S-1+FRAC) |
| update, per symbol of the lane | m ← m + (((\|symbol\| << FRAC) - m) >>> SHIFT) | the difference is signed (S + FRAC + 1 bits). >>> is the arithmetic shift, a floor division by 2^SHIFT. SHIFT ≥ 1 |
| output | b = max(m, MFLOOR) as B_FRAC fixed point: m' itself at FRAC = B_FRAC = 8, else m' << (8 - FRAC) or m' >> (FRAC - 8) | a Laplace's b is its mean \|symbol\|. m < 2^(S-1+FRAC) gives b < 2^B_WIDTH, and at FRAC 8 m's width is B_WIDTH. MFLOOR ≥ 0 acts on the output. m itself is not clamped |
| delay | the b for (lane, t) uses the lane's symbols at positions < t - DELAY | `history_needed(t)` = max(0, t - DELAY) |

m never leaves [0, 2^(S-1+FRAC)). The target |symbol| << FRAC is at most 2^(S-1+FRAC). A step toward a larger target,
with difference d > 0, is floor(d / 2^SHIFT) < d, so m stays below the target. A step down is at least floor(-m /
2^SHIFT) ≥ -m. The floor division rounds toward lower m. From below, m stops up to 2^SHIFT - 1 units short of a
constant |symbol|. At the example parameters that is 15/256 of a step. From above, a run of zeros takes m down by at
least one unit per symbol, to 0.

**The example parameters** are the defaults of `EstParams`, returned by `IntScaleSource::example(cfg[, delay])`:

| Parameter | Value | Meaning |
|---|---|---|
| FRAC | 8 | m is in b's fixed point, so b = m' |
| SHIFT | 4 | the newest symbol weighs 1/16 |
| M0 | 5793 | σ 32's mean \|symbol\|: a wide start that the lane's first symbols narrow. Where σ 32 lies beyond the alphabet (S = 4: p4s4), M0 is 724 (σ 4) |
| MFLOOR | 1 | σ 0.0055, the least nonzero b. A run of zeros takes b down to the table's narrowest codes |
| DELAY | 1 | a consumer one symbol behind |

These are example values, chosen on int8 convolutional-network weights. Other data needs its own choice. `stream_tool
--adaptive` measures a parameter set on files (§1.10). The DPI's kind 3 is this set at the job's delay.

M0 and MFLOOR are integers in m's units. `IntScaleSource::m_of_sigma` gives m = round(σ/√2·2^FRAC), a Laplace's
mean |symbol| for a σ. At FRAC 8, σ 32 is 5793, σ 0.25 is 45, and m = 1 is σ 0.0055. `IntScaleSource(cfg, params)` takes
any set. `IntScaleSource::problems` names what makes a set unfit as an estimator for the build. A fit set has S - 1 +
FRAC in [2, 48], FRAC ≥ 0, SHIFT in [1, S - 1 + FRAC], and M0 and MFLOOR in [0, 2^(S-1+FRAC)).

**Contract versions up to 3** built an estimator of this form into the build. Its parameters were hashed into
`CONFIG_ID`: `PANS_EST_*`, the matrix's est_* keys, and `EST_*` in `pans_pkg`. It had a table of its own, `EST_TABLE`.
Its index was {lead, below} on m' = max(m, MFLOOR), with K = 2 bits below the leading one, 4 buckets per octave of m.
Its entry was `scale_from_sigma`(√2·√(lo·hi)·2^-FRAC), with a special code replaced by the nearest finite one. From
version 4 on, the coder's table is the only table. It has B_K = 4, 16 buckets per octave of b, and entries
`scale_from_b` at the bucket's midpoint. `ref/pans1_config.h` refuses `PANS_EST_*`, and `ci/matrix.py` refuses the est_*
keys. The two conversions agree within 0.5% from b = 2 on. The ratio laplace_b(√2·m) / m is 0.995 at m = 2, 0.981 at 1,
0.925 at 0.35, 1.016 at 0.177 and 1.89 at 0.05. So version 3's b was up to 7.5% smaller between 0.18 and 2, which gave
narrower codes. Below 0.18 it was larger, which gave wider codes. SPEC.md §5 gives the coder's table's rate cost against
the exact code.

### 1.8 What the model reports

- **Encoder** (`encode_stream`): the array, the final states, the words in emission order, n_escapes, and per symbol in
  push order (i, lane, t, symbol, the scale input b and its scale code, start, freq, escape, words emitted, state
  after).
- **Decoder** (`decode_stream`): the symbols and escape flags in order, ok, err_words, err_header, words_consumed, words
  in the array, n_escapes, final states, and per step (lane, t, b and its scale code, slot, start, freq, escape, raw v,
  symbol, words read, state after, the step's error bit). The step's error bit marks a read past the array.

### 1.9 Verification of the model

`make stream-ref` runs `ref/test_stream.cpp` on all 11 matrix configs. It covers:

- **Round trips** with N ∈ {1, 2, 8, 16} and n ∈ {0, 1, N - 1, N, N + 1, …, 10^5}. The sources are constant, list, EMA
  and INT, each producing b. EMA and INT run at delays 0-2. Their b are recomputed from their definitions at every step,
  and every step's code must be the table's code for its b.
- **Escape storms** on all lanes and on one lane, with b = 0 and 1, the narrowest code. Streams with f = 1 only and with
  f = M' only.
- **The escape's raw push** (§1.4). `stream_push_raw` and `stream_pop_raw` run on single states: B - 2, B - 1, B and B +
  1 with B = 2^(P + W - S), the range's ends, and random states. Where [L, L·2^W) holds at most 2^16 states (p4s4,
  p8s*), they run on every state, and where S ≤ 10 with every raw value next to B. The state must stay in [L, L·2^W).
  The push must emit one word iff x ≥ B and must give x·2^S + v exactly. The pop must give back v, the word and x. At
  S = P both must equal push(v, 1) and pop(v, v, 1).
- **Bound jobs.** `JobGen::bound_job` builds streams whose escapes read exactly B and B - 1. The model finds each lane's
  path there from x = L. It computes the target state's predecessors backward and confirms each candidate by pushing it
  forward onto the target.
- **Raw values across words.** In every round trip, an escape whose escape push emitted nothing must have its raw value
  in the next word its lane emits, bit for bit. Escapes whose raw value crosses a word boundary are counted.
- **Corrupt arrays** (§1.6). Each must raise its flags. Each trace must equal the clean trace until the corruption is
  read. Among them, a flipped escape raw value must decode with ok = 1 as a proven collision, with only that symbol
  changed. The slots just beyond the alphabet, C(MIN) - 1 and C(MAX + 1), are among the corrupt arrays and must decode
  inside MIN's and MAX's intervals.
- **The pANS 1.0 transcription** at p16s16 (`ref/test_stream.cpp` §4). Words, state, symbols, escapes and the end check
  must be identical, on clean and corrupt arrays. The transcription's scale is the table's code. pANS 1.0 codes a symbol
  directly iff f ≥ 1 and the candidates at c0 and c0 + f - 1 both lie within 1 of the symbol. The transcription replaces
  this condition by the core's f ≥ 1. At S = P the raw push is pANS 1.0's push(raw, 1).
- **The DPI**, driven as a testbench drives it. b is asked for as early as the source allows. The jobs are clean,
  corrupted and directed, of every kind. Every push and every step carries b and its code.
- **The coder's scale table** (§1.7, SPEC.md §1.5, §6). Index, bucket midpoints and entries are checked against plain
  restatements at B_FRAC 0-12 × B_K 0-8, for every b below 2^16 and random b above. No entry is a special code. The
  slope k is non-increasing in b. b = 0 gives the narrowest code. `finite_scale` is checked on every scale code, and
  `scale_from_sigma` = `scale_from_b(laplace_b)`.
- **The package.** B_*, `SCALE_TABLE`, `CONTRACT_VERSION`, `SYMBOL_MIN`, `SYMBOL_MAX` and `SYMMETRIC_ALPHABET` are
  parsed from `build/<cfg>/gen/pans_pkg.sv` and checked against the model's. No `EST_*` parameter may be left.
- **The example integer estimator** (§1.7). Its update and its b are checked against plain restatements at FRAC 0-12.
  Its parameter bounds are checked. Round trips run over a FRAC × SHIFT sweep. Each parameter, moved alone, must change
  the b.
- **`stream_tool`**. Its rows are checked against the model (§1.10).
- **The symmetric alphabet** (p12s8sym). The input -2^(S-1) appears at every table index that an integer b reaches,
  between random symbols. Each must escape with raw value 2^(S-1) and decode back.

A run fails if it never reaches one of these coverage items:

- escapes, f = 1, f = M', b = 0, the table's last index, and every table index that an integer b has;
- a zero-filled index (0 < b < 2^B_K) and empty lanes;
- two-word steps, where W + log2 E < P + S;
- each error flag, words and header;
- escapes reading B and, where B - 1 ≥ L and S < W, B - 1;
- a raw value crossing a word boundary, where P < W < P + S - log2 E;
- the estimator's floor, and m moving both ways;
- with a symmetric alphabet, an input outside the alphabet.

A coded slot outside the interval decoded for it fails the run, in any trace, corrupt arrays included.

### 1.10 Files of symbols: `stream_tool`

`make stream-tool-<cfg>` builds `build/<cfg>/stream_tool` (`ref/stream_tool.cpp`). The tool puts files of symbols
through the model with a chosen scale source and reports where the bits go:

```
build/p12s8/stream_tool [--lanes N] [--width 8|16] [--const B | --b-list | --adaptive [--frac F] [--shift S]
                        [--m0 M | --start-sigma X] [--mfloor M | --floor-sigma X] [--delay D]] file... > rows.csv
```

**Input.** Each file holds one stream's symbols in decode order (§1.2). The symbols are int8 (`--width 8`) or int16
little-endian (`--width 16`), two's complement. The default width is 8 where S ≤ 8, else 16.

**Scale source.** One of three:

- a constant b: `--const B`, with B an integer and b = B·2^-B_FRAC symbol steps;
- a list of b per symbol from a second file: `--b-list`. The operands then come in pairs, a symbol file and its b file.
  The b file is text, one integer b per symbol;
- the example adaptive estimator `IntScaleSource`: `--adaptive`, the default. Its parameters default to the example set
  (§1.7). `--start-sigma` and `--floor-sigma` convert σ to m by `m_of_sigma` at the FRAC in use. A FRAC other than the
  example's needs both, because m's units change.

**Processing.** Per file the tool computes the encoder's scale inputs b by `replay_scales` of the source, each through
the coder's table (§1.7). It runs `encode_stream` at N lanes (default 8) and `decode_stream` with a fresh source. The
decoded symbols must equal the file's, with ok = 1. With a symmetric alphabet an input -2^(S-1) escapes, counts in
n_escapes and round-trips.

**Errors.** A file that cannot be read, that holds a value outside the build's S-bit inputs, that has a b list of
another length or a b beyond B_WIDTH bits, or that fails the round trip is reported on stderr and gets no row. The exit
status is then 1. Bad options give exit status 2.

**Output.** The build, the source and its parameters go to stderr. stdout is CSV, one row per file, with the columns
`file, n, lanes, bytes, header_bytes, words, n_escapes, ideal_coded_bits, escape_bits, ideal_total_bits, actual_bits,
actual_bps, ideal_bps`:

- ideal_coded_bits = Σ log2(M / f) over the coded symbols, with f from the encoder's trace. It is the model's cost.
  Compare it with the stream's entropy;
- escape_bits = n_escapes · (log2(M / E) + S): the escape slots, then the raw value's S bits (§1.4). It is the escapes'
  cost;
- ideal_total_bits is their sum, and actual_bits = 8 · bytes;
- actual_bits - ideal_total_bits is the coder's overhead. It consists of the header's N final states and ANS's rounding.
  Each final state carries log2(x / L) < W bits of the payload;
- actual_bps and ideal_bps are the two per-symbol rates.

The CSV header defines every column. `test_stream` checks every source's rows against `encode_stream`: the example
estimator and another set given as options, constant b at 0, 1 and 2^B_WIDTH - 1, b lists of random b, and both file
widths. It also checks the failures.

**Example.** `make stream-ref-p12s8` writes the self-test's `drift.bin` to `build/p12s8/stream_tool_check/`. It holds
6,000 symbols from the model's job generator. Per lane they follow a discretized Laplace whose σ moves by up to an
octave every 64 symbols, with 2% uniform outliers. At p12s8 the example estimator codes it in 3,694 bytes, 4.925 bits
per symbol, against an ideal 4.886, with 75 escapes. One constant b = 4 (`--const 1024`) takes 4,620 bytes, 6.160 bits
per symbol, with 525 escapes.

## 2. The units

`pans_dec_stream` and `pans_enc_stream` each instantiate their core unchanged (SPEC.md §3) and keep its conventions:

- `rst_n` is synchronous and active low. It clears control only: phases, valid, busy, fresh and scale bits, FIFO
  pointers and counts, credit, reservations and the job flags.
- Data registers have no reset. They load every cycle, or when written: the lane states, the job's counters and the FIFO
  entries.
- The arithmetic is in single-exit functions with every local assigned (`rtl/pans_dec_stream_functions.svh`,
  `rtl/pans_enc_stream_functions.svh`). Each function names its `pans_stream_ref.h` counterpart.
- SVA sit under `ifndef SYNTHESIS`. Elaboration checks cover every parameter.
- Every channel is valid/ready. A transfer is valid && ready at a rising edge.

`gen_pkg --stream` refuses the builds that the units cannot implement: E not a power of two, as in p8s6e3. The encoder
unit also refuses such a build at elaboration. With `CLOCK_GATE` = 1, the default of both units, the register groups
written only now and then sit behind clock gates (§2.3). `CLOCK_GATE` = 0 gives enable flops with the same function,
cycle for cycle (§3). The widths in §2.1 and §2.2 are those at p12s8, N = 8.

Every parameter is a make variable on every unit target: lint, the simulations, the round trip, generic gates, ORFS and
power. `N_LANES` applies to both units. `DEC_<parameter>` and `ENC_<parameter>` set one unit's parameter. Each defaults
to the unit's default (Makefile header). A parameter set builds in its own directory: `build/<cfg>/dec_stream/<tag>/`,
`enc_stream/<tag>/` and `stream_rt/<tag>/`. At the defaults the tags are `n8_b4_f8_o7` and `n8_d2_o8`.

### 2.1 `pans_dec_stream`

| Parameter | Default | Limits (elaboration checks) | |
|---|---|---|---|
| `N_LANES` | 8 | ≥ 1 | lanes |
| `DATA_BYTES` | 4 | a multiple of STATE_BYTES (whole states per beat) | `data_in` beat |
| `FIFO_WORDS` | 8 | ≥ 4 and > DATA_BYTES / (W/8) | word FIFO entries. 9 or 10 remove the issue's waits for words (below) |
| `OUT_DEPTH` | 7 | ≥ 1. ≥ 7 for one symbol per cycle (6 steps in flight + 1) | output FIFO entries |
| `COUNT_WIDTH` | 32 | ≤ 32, and holds the header's and a beat's byte counts | the count ports and the job counters |
| `CLOCK_GATE` | 1 | | 1: 35 clock gates at the defaults (below). 0: enable flops |

LANE_WIDTH = ceil(log2 N), or 1 at N = 1. POS_WIDTH = the bits of (2^COUNT_WIDTH - 2) div N, 29 at N = 8.

| Channel | Fields (bits) | Rule |
|---|---|---|
| `job_in` (in) | `n_sym` (COUNT_WIDTH = 32), `n_bytes` (32) | taken only when idle, after the previous job's `done_out` |
| `data_in` (in) | `data` (8·DATA_BYTES = 32), byte k at bits [8k+7:8k] | the array from byte 0, exactly ceil(n_bytes / DATA_BYTES) beats. The last beat may be partial, and bytes beyond n_bytes are ignored. Never ready for a beat beyond the array. Beats that the decode does not need are taken and dropped at the end |
| `scale_in` (in) | `lane` (3), `b` (B_WIDTH = 15, the scale input, §1.7) | ready iff that lane holds no scale and has a symbol left that needs one. A scale of the next job waits for its `job_in`. One per symbol per lane, the lane's first before its first symbol. b goes through `pans_scale_of_b` as it is taken, and the lane's scale register holds the code (EW + 3 = 7 bits). This is the unit's only input-to-output path: ready depends on `scale_in_lane` |
| `sym_out` (out) | `lane` (3), `t` (29), `symbol` (signed S = 8), `escape` (1) | decode order, global index i = t·N + lane |
| `done_out` (out) | `ok`, `n_escapes` (32), `words_consumed` (32), `err_words`, `err_header` | after the last symbol has left `sym_out` and the array has been taken |

| Stage | Work, ending in a register |
|---|---|
| scale_in | b → the scale code by `pans_scale_of_b`: a leading-one detect, a 4-bit select, `SCALE_TABLE`. The code goes into the lane's scale register (7 flops, behind the lane's scale clock gate), off the issue path. The issue reads the code |
| header | the first N·STATE_BYTES bytes into x[N], DATA_BYTES / STATE_BYTES states per beat, each checked (`err_header`). A header beat wholly beyond the array reads as zeros (`err_words`) and is not requested |
| word FIFO | the array's remaining whole words, up to DATA_BYTES / (W/8) per beat. The read port shows the two head words and pops 0, 1 or 2 per cycle |
| 0 issue | strict round robin over the lanes (below). The core gets (scale[j], slot = x[j] mod M) through mux trees. A tag pipeline carries the lane beside the core. It does not carry t: the output regenerates (lane, t) from a counter pair, and the symbols leave in decode order |
| 1-4 | `pans_dec_core`: (symbol, start c, freq f, escape). Stage 4 also reads x[j] into a register |
| 5 multiply | (x[j] >> P)·f from that register, on the W × P multiplier (16 × 12). s - c, which lies in [0, f) on any input (§1.6, asserted as `a_interval`) |
| 6 add and renormalize | the sum mod 2^STATE_WIDTH and refill 1. For an escape: v = x mod 2^S, the raw pop x >> S and refill 2 (§1.4). Refill 2 takes the second head word if refill 1 took the first. x[j] is written with its at-L flag (x = L) and its second-word flag (an escape from this x reads two words). The step pops its words, frees the lane's scale slot and writes {symbol, escape} into the output FIFO. The pop count is registered for `words_consumed` a cycle later |

The reads off the issue path use AND-OR over a one-hot of their index register: stage 4's x[j], the FIFO's head words
and the output entry. The issue's own reads stay mux trees: the slot, the scale, and the scale and second-word flags
(rtl/PPA_DEC_STREAM.md §5.2).

**Issue.** Lane j issues iff all of these hold:

- it has a symbol left;
- its scale is present;
- the FIFO holds the step's words beyond those reserved by the steps in flight, or every whole word of the array is
  already in the FIFO. A coded slot needs 1 word. For a slot ≥ M', the core's escape test, known at issue, the step
  needs 2 words if the lane's second-word flag is set and 1 otherwise;
- an output credit is left. Credit = free output entries minus steps in flight.

Otherwise nothing issues, and the round-robin pointer holds. "A symbol left" and "the array has bytes left" are
registered flags beside their 32-bit counters. So neither the issue nor a clock gate's enable waits for a 32-bit
compare. Issue to x[j] written takes 6 cycles. So for N ≥ 6 strict round robin keeps a lane with a pop in flight from
issuing, which is asserted. Below N = 6 a busy bit per lane does this.

**Escapes** take no extra cycle. The add-and-renormalize stage does both pops and both refills. Two words occur only
where W + log2 E < P + S. The issue knows it from x: an escape reads two words iff x' = (x >> P)·E + (s - M') < 2^(P +
S - W) (`escape_second_word`). The symbol is v sign-extended from S bits.

**Errors** are sticky per job. Decoding runs to the n-th symbol with the model's arithmetic. A pop beyond the FIFO's
words, when the array has none left, reads 0 and sets `err_words`. So corrupt input cannot hang the unit. `ok` is §1.6's
end check: the at-L flags, and the byte count from two flags, no byte left over and no whole word dropped. A new job
starts clean, with the FIFO flushed and the flags cleared.

**Clock gates** (`CLOCK_GATE` = 1). The gated groups and their enables:

| Group | Flops per gate | Enable | Gates at N = 8 |
|---|---|---|---|
| a lane's state, at-L and second-word flags | 30 | the header step for the lane, or stage 6's registered lane | 8 |
| a lane's scale | 7 | `scale_in`'s transfer to the lane | 8 |
| an output entry | 9 | stage 6's registered valid and the write pointer | 7 |
| a word FIFO entry | 16 | the beat's transfer, its byte-present bits, the write pointer | 8 |
| `n_escapes`, `words_consumed`, the output's (lane, t), the array's bytes left | | one gate each | 4 |

That is 8 + 8 + 7 + 8 + 4 = 35 gates at the defaults. Some groups are not gated. The symbols left and the round robin
have the issue condition as their enable. The FIFO, credit and reservation counts and pointers, the flags, and the
per-cycle pipeline registers are not gated either. Gating the per-cycle registers by their stage's valid (G3) is an
option for a host that leaves the unit idle (§4.3).

**The consumer's loop.** A symbol is on `sym_out` 6 cycles after its issue, with the output FIFO empty. Its lane's scale
slot is free from that cycle. The lane's next turn comes N issues after its last one.

Throughput, measured with `make sim-dec-stream-p12s8sym N_LANES=<N> DEC_FIFO_WORDS=<F> DEC_OUT_DEPTH=<O>`, seed 1, at
contract version 3. The runs use the testbench's throughput jobs: EMA delay 0, 16,384 symbols, no stalls. The figures
are symbols per cycle from the first to the last symbol.

| N | FIFO_WORDS | OUT_DEPTH | scale 1 cycle after the symbol | 2 cycles after | issue bubbles at 1 cycle |
|---|---|---|---|---|---|
| 8 | 8 | 7 | 0.9939 | 0.8884 | 101 for FIFO words |
| 8 | 9 | 7 | 0.9999 | 0.8889 | 2 for FIFO words |
| 8 | 10 | 7 | 1.0000 | 0.8889 | 1 for FIFO words |
| 8 | 8 | 8 | 0.9939 | 0.8884 | 101 for FIFO words |
| 8 | 8 | 6 | 0.8571 | 0.8571 | 2,701 for output credit, 31 for FIFO words |
| 16 | 8 | 7 | 0.9919 | 0.9921 | 134 for FIFO words |
| 16 | 10 | 7 | 1.0000 | 1.0000 | 1 for FIFO words |

"1 cycle after" means the scale is on `scale_in` in the cycle after the symbol's transfer. Every further cycle of
consumer latency costs one cycle per round of N symbols, while it exceeds the loop's slack. At N = 8 and 2 cycles that
gives 8/9.

The bubbles at FIFO_WORDS = 8 are structural. With six coded steps in flight the issue needs 7 words. A beat of two
words is taken only at 6 or fewer (rtl/PPA_DEC_STREAM.md §1). 9 or 10 entries remove the bubbles. The default is 8, the
smallest value. §4.3 gives the cost of 9 and 10.

The same jobs at later contract versions, at the defaults, seed 1 unless stated:

| Contract version, config | scale 1 cycle after the symbol | 2 cycles after | issue bubbles for FIFO words at 1 cycle |
|---|---|---|---|
| 3, p12s8sym (the table above) | 0.9939 | 0.8884 | 101 |
| 4, p12s8sym | 0.9934 | 0.8884 | 110 |
| 5, p12s8sym | 0.9967 (seed 2: 0.9959) | 0.8886 | 55 |
| 5, p12s8 | 0.9967 (seed 2: 0.9960) | 0.8886 | 56 |

At version 4 the EMA source gives b (§1.7). The throughput barely moves, because the scale input's lookup sits beside
the loop, not in it. At version 5 escapes move fewer words (§1.4), so fewer issues wait for words.

### 2.2 `pans_enc_stream`

| Parameter | Default | Limits (elaboration checks) | |
|---|---|---|---|
| `N_LANES` | 8 | ≥ 1 | lanes |
| `DIV_BITS` | 2 | 1 ≤ DIV_BITS ≤ W | quotient bits per divider stage. D = ceil((W - DIV_FIRST_BITS) / DIV_BITS) stages, and the last one takes the rest |
| `DIV_STEP_BITS` | 1 | divides DIV_BITS and W | quotient bits per restoring step within a stage (radix 2^DIV_STEP_BITS) |
| `DIV_FIRST_BITS` | = DIV_STEP_BITS | a multiple of DIV_STEP_BITS, 0 ≤ DIV_FIRST_BITS < W | quotient bits taken in the renorm stage |
| `OUT_DEPTH` | 8 | even, ≥ 2 (an escape's two words, one per FIFO bank) | word FIFO entries |
| `COUNT_WIDTH` | 32 | none. Jobs stay below 2^COUNT_WIDTH symbols | `n_sym`, `n_escapes` and their counters. `n_words` has one bit more |
| `CLOCK_GATE` | 1 | | 1: N + 1 clock gates (below). 0: enable flops |

At p12s8 and the defaults D = 8. Two accepts of one lane are ISSUE_DISTANCE = D + 1 cycles apart. ISSUE_DISTANCE is a
localparam that the testbench reads. So the unit takes min(1, N / (D + 1)) symbols per cycle.

| Channel | Fields (bits) | Rule |
|---|---|---|
| `job_in` (in) | `n_sym` (COUNT_WIDTH = 32) | taken when idle, between jobs. One setup cycle follows |
| `sym_in` (in) | `lane` (3), `symbol` (signed S = 8), `b` (B_WIDTH = 15, the scale input, §1.7) | push order, global index i = n - 1, ..., 0. Taken into the input register (below), with b turned into the scale code on the way. When the core takes the beat, `lane` is checked against the unit's own counter, (n - 1) mod N counting down. A mismatch sets `err_order`, and the unit goes on with its own lane. So its words and states are the model's for the symbols and scale inputs sent |
| `word_out` (out) | `word` (W = 16) | emission order, one word per beat |
| `done_out` (out) | `lane` (3), `last`, `state` (STATE_WIDTH = 28), `n_words` (33), `n_escapes` (32), `err_order` | N beats, lane 0 first, with `last` on lane N - 1. Each beat carries a lane's final state. The counts and `err_order` are the job's, on every beat. The beats follow the job's last word on `word_out` |

No output depends combinationally on an input. No input drives the core directly. `sym_in` ends in the input register,
through the scale lookup, and the core's stage 1 starts from registers.

| Stage | Work, ending in a register |
|---|---|
| setup | the first push's lane (n - 1) mod N: a constant-modulus tree over n's bits plus one modular add. At N = 8 it is the low bits |
| 0 input | the `sym_in` beat {lane field, symbol, the code of b by `pans_scale_of_b`} into the input register (19 flops at p12s8). The register holds the beat until the core takes it (the accept, below), and takes the next beat in the cycle it empties. `sym_in` is ready iff the job runs, symbols remain and the register is empty or empties now, all from registers. The stage adds one cycle of latency. The throughput is the accept's |
| 1-2 | `pans_enc_core`: (scale, symbol) -> (c, f, escape). A tag pipeline {lane, symbol} runs beside it |
| 3 renorm | read x[lane]. A lane not yet written in the job reads L, through a fresh bit per lane instead of a load of L. Coded: emit the low word iff x >> W ≥ f, then shift. Escaped: the raw push (§1.4) emits the low word iff x ≥ 2^(P + W - S), an OR of x's top S bits, and forms x·2^S + raw as wiring. push(M', E) completes in the same stage, since a division by E = 2^e is wiring. The finished state rides the divider as a division by 1. So every symbol takes one path, and the lane states have one write port. 0, 1 or 2 words go into the FIFO, in push order. The stage also computes the first DIV_FIRST_BITS quotient bits |
| 4 .. 2 + D | divider stages 1 to D - 1: exact restoring division, DIV_BITS quotient bits each, as DIV_BITS / DIV_STEP_BITS cascaded restoring steps |
| 3 + D | divider stage D: the remaining quotient bits. Its last step is fused with the assembly x = q·M + c + r, one P-bit add per candidate. The result is written into x[lane] |

**Accept.** The accept moves the input register's beat into the core. A lane takes a symbol only when no push of the
lane in flight can still change the state that its renorm stage reads. A busy bit runs from the accept until D cycles
later. By then the older push's write is certain to land before the next push's read. This is asserted on every read
(`a_lane_in_order`). So the state that a push reads is its predecessor's, for any N and divider depth. The accept also
requires room in the FIFO. The FIFO's count plus two words for every symbol between the accept and the renorm stage, the
new one included, must fit OUT_DEPTH. So the FIFO never overflows. Escapes take no extra cycle. `done_out` reads the
states through the renorm stage's read port, since the pipeline is empty by then.

**The FIFO** holds OUT_DEPTH words in two banks, even and odd entries. The two words of a cycle go to consecutive
entries. So each bank has one write port with one data mux. The FIFO counts `n_words` at its output.

**Clock gates** (`CLOCK_GATE` = 1). One gate per lane state (28 flops), enabled by the last divider stage's registered
valid and lane. One gate for the escape count, enabled by the job start or a registered escape. That is N + 1 = 9 gates
at N = 8. The FIFO entries keep enable flops. Their write enables come out of the renorm stage's longest path. As gate
enables they had +0.44 ns of slack at the latch, against +1.34 ns register to register (rtl/PPA_ENC_STREAM.md R4).

Throughput, measured with `make sim-enc-stream-p12s8 ENC_DIV_BITS=<b> ENC_DIV_FIRST_BITS=<f> ENC_OUT_DEPTH=<o>
N_LANES=<N>`, seed 1. The runs used the units after their optimization rounds, at contract version 1, when the LIST jobs
drew scale codes, special ones among them. The jobs are the testbench's throughput jobs: 4,000 symbols per source, no
stalls. The figures are symbols per cycle. At contract version 5 the default row gives 0.889 for each source.

| N | DIV_BITS / DIV_FIRST_BITS | OUT_DEPTH | D | D + 1 | CONST | EMA | LIST (6% special scales) |
|---|---|---|---|---|---|---|---|
| 8 | 1 / 1 | 8 | 15 | 16 | 0.501 | 0.501 | 0.500 |
| 8 | 2 / 1 (default) | 8 | 8 | 9 | 0.889 | 0.889 | 0.887 |
| 8 | 2 / 0 | 8 | 8 | 9 | 0.889 | 0.889 | 0.887 |
| 8 | 2 / 2 | 8 | 7 | 8 | 1.000 | 1.000 | 0.987 |
| 8 | 3 / 1 | 8 | 5 | 6 | 1.000 | 1.000 | 0.984 |
| 8 | 4 / 1 | 8 | 4 | 5 | 1.000 | 1.000 | 0.988 |
| 8 | 4 / 4 | 8 | 3 | 4 | 1.000 | 1.000 | 0.984 |
| 8 | 2 / 1 | 6 | 8 | 9 | 0.858 | 0.834 | 0.798 |
| 8 | 3 / 1 (COUNT_WIDTH 24) | 6 | 5 | 6 | 0.899 | 0.877 | 0.795 |
| 16 | 2 / 1 | 8 | 8 | 9 | 1.000 | 1.000 | 0.986 |

Where the lane distance allows full rate, values below 1.000 come from the FIFO credit, which reserves two words per
symbol on its way to the renorm stage. Escapes (LIST) and OUT_DEPTH = 6 lower the rate further.

### 2.3 The clock gate (`rtl/pans_stream_cg.sv`)

One module, `pans_stream_cg`, serves both units and any further gate. A latch captures the enable while the clock is
low. An AND combines the latched enable with the clock. So the gated clock rises exactly in the cycles where the enable
was 1, and it cannot glitch. The flops behind the gate are plain flops. They need no hold mux. When unclocked they draw
neither their clock pin's power nor their share of the clock tree.

Nangate45 has integrated clock-gating cells, but yosys infers none. ORFS maps the module to DLL_X1 + AND2_X1. A
technology with an ICG cell substitutes that cell for the module. OpenSTA times the enable into the latch against the
clock's falling edge, which is the latch's opening, half a period after the launch. Time borrowing into the low phase is
reported separately. OpenSTA times the latch's output into the AND as a clock-gating check.

**Rule.** A gate's enable comes straight from registers, or from registers through a few gates. The measurements behind
the rule:

- The encoder's FIFO write enables come from the renorm stage's longest path. As gate enables they had +0.44 ns at the
  latch against +1.34 ns register to register. The FIFO is therefore not gated (§2.2).
- The decoder's word-FIFO enables had 0.93 ns of logic through two 32-bit counter tests. With those tests turned into
  registered flags, the enables have +0.75 ns at the latch. The latch's own period limit, 1.50 ns, then equals the
  register paths' (rtl/PPA_DEC_STREAM.md §6, G2 and G2b).

**Enables from the ports.** Some enables include a handshake at the unit's ports:

- the decoder's scale gates (`scale_in` valid and lane);
- the decoder's word gates (`data_in` valid);
- the decoder's counter gates (`job_in` valid, `sym_out` ready);
- the encoder's escape count (`job_in` valid).

The half-period check at the latch then covers the host's logic in front of those ports too. The PPA flow drives every
port from a register (§4). A host that drives these ports through logic must time those paths, or build the units with
`CLOCK_GATE` = 0.

**How the flow reports it.** ORFS's Fmax is 1 / (period - the worst register-to-register slack). Its worst slack also
counts a gate's enable at the latch. With gates these are two different paths. The decoder's round 3 reports +0.752 ns,
a word gate's enable at its latch, and 649 MHz from its register paths. The worst register path there ends in the
add-and-renormalize stage's at-L flags, at +1.460 ns. The encoder's round 2 reports +1.279 ns, the escape count's
enable, and 619 MHz, from the renorm stage at +1.385 ns. The stage report (§4.1) gives both kinds of path, per group,
for every run.

### 2.4 What the host does

- **Encoding.** The scale inputs b come from the consumer's model. The host replays that model over the symbols in
  decode order (§1.7, `replay_scales`). Per job the host sends `job_in` {n}, then `sym_in` {lane, symbol, b} in push
  order, global index i = n - 1 down to 0, lane i mod N. It takes `word_out` as it comes and then `done_out`'s N states.
  The array (§1.5) is the N states, lane 0 first and STATE_BYTES each, followed by the words reversed. Word k of
  `word_out`, counting from 0, goes to byte N·STATE_BYTES + (W/8)·(n_words - 1 - k). Everything is little-endian.
  n_words is known only at `done_out`. So the host either buffers the words, or writes them downward from the end of a
  region and puts the header in front. The unit never buffers a stream. `tb_pans_stream_rt` assembles the array exactly
  this way (§3).
- **Decoding.** The host sends `job_in` {n, n_bytes}. The end check needs n_bytes to confirm that every byte is
  consumed. It sends the array on `data_in` from byte 0. On `scale_in` it sends the b of every (lane, t), a lane's first
  before its first symbol, then each as soon as the consumer has it. It takes `sym_out` in order, and finally
  `done_out.ok`. Neither the host nor the consumer needs a table, because the coder's table is in the unit.
- **Ports into clock-gate enables** (§2.3). They come from registers, or are timed with the unit, or the unit is built
  with `CLOCK_GATE` = 0.
- **Throughput at N = 8.** The decoder delivers one symbol per cycle when the b of a lane's next symbol is on `scale_in`
  in the cycle after the lane's symbol left `sym_out`. That gives 0.9967 symbols per cycle at the default FIFO_WORDS = 8
  (contract version 5), 0.9999 at 9 and 1.0000 at 10. At contract version 3, whose escapes moved more words, the default
  gave 0.9939. One cycle later the rate is 8/9 (0.8884 to 0.8889). Each further cycle costs another cycle per round
  (§2.1). At N = 16 two cycles of consumer latency cost nothing measurable: 0.9921, and 1.0000 at FIFO_WORDS = 10.
  FIFO_WORDS 9 or 10 cost area (§4.3). The encoder takes 0.889 symbols per cycle at the default DIV_BITS = 2, and 1 from
  DIV_BITS = 3 (N = 8, W = 16, §2.2). §4.3 gives the area and energy of its options.

## 3. Verification

`make ci` runs the stream checks of the smoke tier on every config (CI, below). A simulation passes with 0 errors and
every coverage item it requires reached.

| Check | Target | Covers |
|---|---|---|
| model self-test | `stream-ref-<cfg>` | §1.9. Also the DPI against the model, every push beat and push step included (`pans_stream_job_enc_step`, with b and its code), the coder's scale table against its definition and the package (§1.7), the example integer estimator against its definition, `stream_tool` against the model (§1.10), the raw push on single states and the bound jobs (§1.4) |
| decoder against the model | `sim-dec-stream-<cfg>` | `tb/stream/tb_pans_dec_stream.sv` (below) |
| encoder against the model | `sim-enc-stream-<cfg>` | `tb/stream/tb_pans_enc_stream.sv` (below) |
| RTL round trip | `sim-stream-rt-<cfg>` | `tb/stream/tb_pans_stream_rt.sv` (below) |
| lint | `lint-dec-stream-<cfg>`, `lint-enc-stream-<cfg>` | Verilator -Wall, clean |
| refusal | `stream-refuse-p8s6e3` | `gen_pkg --stream` refuses E = 3 and logs its reason |
| gate level | `power-stream-<cfg>`, run at p12s8 and p12s8sym | the routed netlists against the model (§4.1): the `+CHECK` jobs (corners, corrupt arrays, random stalls) and every power workload |

**The decoder testbench** (`tb/stream/tb_pans_dec_stream.sv`) checks:

- first, the scale lookup, an instance of `pans_scale_of_b`, against the model at every b below 2^16. Above 2^16 it
  checks both ends of each table index and a random b;
- every `sym_out` and `done_out` field;
- white box, every step against the model's trace: the issue (the code that b resolved to, the slot), the core's
  interval, the state, the words and the error bits.

The consumer is the model's EMA or INT source (§1.7). It is fed with the unit's own symbols and puts its b on
`scale_in`. The jobs are random, corrupt and directed, under three stall profiles. The INT jobs run last, so the other
jobs keep their seeds. Then come the raw push's bound job and a job with more words appended than the word FIFO holds.
The corrupt jobs reach drained beats only by chance. The appended-words job reaches that corner for certain. The
testbench requires its coverage corners, among them b = 0 and the table's last index. With a symmetric alphabet it
requires an escaped -2^(S-1).

**The encoder testbench** (`tb/stream/tb_pans_enc_stream.sv`) checks:

- the scale lookup, as the decoder's testbench does;
- every word, final state and count;
- every push at three points: where it enters the core (its lane, its symbol and the code of its b, from the input
  register), where it leaves the renorm stage (interval, escape, words), and where it writes its state.

The jobs are random and directed, under five stall profiles. They include a wrong lane field, a reset in mid-job, and
the first lane over the whole count range. The raw push's bound job has escapes reading 2^(P + W - S) and one below it.
The testbench requires its coverage corners, both of those reads among them. With a symmetric alphabet it requires
inputs -2^(S-1), each escaped.

**Clock gates in simulation.** The simulations run the units with their clock gates: `CLOCK_GATE` = 1, with the latch
and the AND as written, evaluated by Verilator. The testbenches change inputs away from the rising edge. `CLOCK_GATE` =
0 gives the same function, cycle for cycle. At p12s8, `sim-dec-stream` and `sim-stream-rt` with `DEC_CLOCK_GATE=0`, and
`sim-enc-stream` and `sim-stream-rt` with `ENC_CLOCK_GATE=0`, report the same checks and cycles as the gated runs
(contract version 5). The same holds on p12s8sym at contract versions 3, 4 and 5.

**The round trip** (`tb/stream/tb_pans_stream_rt.sv`) runs both units at once. `pans_enc_stream` encodes each job. The
testbench assembles its words and `done_out` states into the array exactly as §1.5 says. `pans_dec_stream` decodes the
array back. The round trip passes iff all of these hold:

- the decoded symbols are the job's, in order;
- `done_out` says `ok`, with every flag clear;
- the decoder consumed the encoder's word count and found its escape count.

Nothing is compared with the model's array, words, states or trace. The model only makes the stimulus, the symbols and
the scale inputs b that the host gives the encoder, and acts as the decoder's consumer. Both units share the scale
lookup, so an error in it round-trips. The unit testbenches check the lookup.

The round trip runs these jobs:

- three stall profiles × n in {0, 1, N - 1, N, N + 1, random to 3,000} × the sources CONST, EMA at delays 0-2, and LIST.
  The stall profiles are gaps on every input channel, ready low on every output channel, and consumer latency 1 to 6
  cycles;
- escape storms on every lane and on one lane;
- f = M' only, with b = 0;
- the raw push's bound job, with escapes reading 2^(P + W - S) and one below it.

The round trip requires these coverage items: escapes, two-word pushes and pops (W + log2 E < P + S), empty lanes, n =
0, and with a symmetric alphabet a -2^(S-1) back as an escape. Reversing the words wrongly in the testbench fails the
round trip. So does a decoder whose escape takes its second refill from the wrong head word. Both were tried once and
reverted.

**CI.** `ci/matrix.toml` defines a smoke tier and a full tier, and `make ci` runs the smoke tier. On each of the 10
stream configs both tiers run `lint-dec-stream`, `sim-dec-stream`, `lint-enc-stream`, `sim-enc-stream` and
`sim-stream-rt` at the default parameters. Each simulation runs with seeds 1, 2 and 3 and must pass all three. p8s6e3
(`stream = "refused"`) gets one `stream-refuse` job instead. `stream-ref` runs on every config, after the config's `pkg`
job, because it reads the package's `SCALE_TABLE`.

**Parameter variants outside CI.** These parameter sets passed lint, the unit simulation and the round trip once, at
p12s8 unless stated, with `make lint-<unit>-stream-p12s8 sim-<unit>-stream-p12s8 sim-stream-rt-p12s8
<VARIABLE>=<value>`:

| When | Decoder | Encoder | Round trip |
|---|---|---|---|
| after the optimization rounds (contract version 1) | FIFO_WORDS 9, 10; OUT_DEPTH 6, 8; COUNT_WIDTH 24; CLOCK_GATE 0; N_LANES 16 with FIFO_WORDS 8 and 10 | DIV_BITS 1, 3, 4; DIV_FIRST_BITS 0 and 2 (DIV_BITS 2), 4 (DIV_BITS 4); DIV_STEP_BITS 2 (DIV_BITS 4); OUT_DEPTH 6; DIV_BITS 3 with OUT_DEPTH 6 and COUNT_WIDTH 24; CLOCK_GATE 0; N_LANES 16 | both units at COUNT_WIDTH 24; the encoder at DIV_BITS 3, OUT_DEPTH 6, CLOCK_GATE 0 |
| during the rounds (rtl/PPA_DEC_STREAM.md §4, §5.5, §6.1; rtl/PPA_ENC_STREAM.md §3) | all configs; FIFO_WORDS up to 16; OUT_DEPTH 16 | all configs; N_LANES 1, 3, 16; DIV_BITS 16; DIV_STEP_BITS 4; p16s16 at N = 16; p4s4 at DIV_BITS 1 | |
| before the rounds | N_LANES 1, 2, 3, 5, 16; DATA_BYTES 8 with FIFO_WORDS 8 and 5; FIFO_WORDS 4 | N_LANES 1, 2, 3, 5, 16 | N_LANES 1, 2, 3, 5, 16; DATA_BYTES 8 with DIV_BITS 4 and both output FIFOs at their minimum (decoder OUT_DEPTH 1, encoder 2); N = 3 with FIFO_WORDS 4 and DIV_BITS 1 |

At the minimum depths the unit testbenches' own coverage demands failed, while every functional check passed. At decoder
OUT_DEPTH = 1 the output credit always runs out first, so no issue bubble ever waits for a scale. At encoder OUT_DEPTH =
2 or 3 the FIFO credit refuses every beat that a busy lane would refuse. The encoder testbench waives that demand below
OUT_DEPTH 6 (rtl/PPA_ENC_STREAM.md F).

**Mutations.** Each mutation is a textual change of the RTL, built and run once and then reverted.

On the units before the optimization rounds:

- Decoder. Each of its 14 assertions fires on a mutation aimed at it. The assertions cover issue and pop order, the lane
  in flight, the state before and after a step, the words per step, the FIFO's pop, dry and room conditions, the credit,
  the interval, the escape prediction, the tag pipeline and the scales used. A round robin that skips its last lane
  deadlocks into the watchdog. Three planted bugs that no assertion sees fail the comparison with the model: an escape's
  second refill always from the first head word, ok without the byte count, and the raw symbol's sign bit dropped.
- Encoder. Each of its 14 assertions fires on a mutation aimed at it. Three planted bugs fail the comparison: the raw
  value sign-extended, an escape's two words swapped, and the first-lane tree missing its last level at N = 3.

The assertions added by the optimization rounds have not been mutation-tested: decoder `a_at_L` and `a_out_order`,
encoder `a_assemble_fused`, `a_lane_in_order`, `a_release_busy` and `a_remainder_last`. The encoder's `a_busy_write` was
removed with the early lane release.

The scale table and the lookup (contract version 4, p12s8sym, seed 1). Each of these fails `sim-dec-stream` and
`sim-enc-stream`, in the lookup check and in the job checks. `sim-stream-rt` passes each one, as it must, because a
lookup common to both units round-trips.

| Mutation | Also fails |
|---|---|
| a wrong `SCALE_TABLE` entry in the package: entry 201, {2, 6} → {2, 5}, b in [25, 26) | `stream-ref`, on the package against the model |
| `pans_scale_of_b`'s leading-one detect misses b's top bit | |
| the select of below one bit too high, so the leading one itself lands in below | |
| the zero fill below bit 0 made a ones fill | |

The encoder's assertion `a_input_in_job`, which requires the input register to be empty outside a job, has not been
mutation-tested.

The raw push (contract version 5, p12s8sym, seed 1):

| Mutation | Fails |
|---|---|
| encoder: emit iff x > 2^(P + W - S), the bound off by one upward | `sim-enc-stream` in its first random jobs, on the white-box renorm check (an escape from x = 2^(P + W - S) leaves the renorm stage with 0 words, the model 1) and on `a_one_word_per_push`; `sim-stream-rt` on `a_one_word_per_push` |
| encoder: emit iff x ≥ 2^(P + W - S) - 1, the bound off by one downward | `sim-enc-stream` on the word compare, in a random LIST job; `sim-stream-rt` on the bound job's decode |
| encoder: the OR over the top S + 1 bits, the bound halved | `sim-enc-stream` on the word compare |
| encoder: the OR over the top S - 1 bits, the bound doubled | `sim-enc-stream` on `a_one_word_per_push` |
| decoder: the raw pop shifts by P instead of S | `sim-dec-stream` at the add-and-renormalize stage (state and words against the model); `sim-stream-rt` on `a_words_per_step` (a step read 2 words, reserved 1) |

Random jobs reach both states next to the bound, because they escape often enough at the narrow codes. The bound jobs
reach them for certain.

**Not covered.** No formal proof covers the units. The clock gates' timing is not simulated: the gate-level simulation
is also at zero delay, and §4's STA covers the timing. The throughput figures come from the testbenches' EMA jobs
only.

## 4. PPA (p12s8 and p12s8sym, N = 8; Nangate45 at 3 ns, ASAP7 at 1 ns)

Every area, Fmax, slack and energy in §4.1-§4.5 comes from ORFS `26Q4-137-g9c431405a` on Nangate45 at a 3 ns target,
and every energy from the EMA workloads of §4.1. Nangate45 is an academic 45 nm library. The numbers compare designs
within this flow, not with a commercial process. §4.6 gives p12s8 on ASAP7, a predictive 7 nm kit, at a 1 ns target.

### 4.1 The flow

`make asic-stream-p12s8 power-stream-p12s8` runs both units. `asic-dec-stream-`, `asic-enc-stream-`, `power-dec-stream-`
and `power-enc-stream-p12s8` run one unit. `make -j2` runs the two units at once, in minutes each. Every parameter is a
make variable (§2). The flow is ORFS `26Q4-137-g9c431405a` on Nangate45 with a 3 ns target, the `asic_period_ns` of
p12s8, and the cores' flow settings (SPEC.md §7).

**The timing wrapper.** Each unit runs inside `ppa/pans_stream_timed.sv`, which takes and forwards every unit parameter.
Every channel passes a two-entry register slice. So every port is a register's port, and every timed path runs register
to register. The slices keep one transfer per cycle and add a cycle of latency each.

**Groups.** Synthesis keeps the unit's module (`SYNTH_KEEP_MODULES`). Its cells, and the buffers placed on its nets,
keep the prefix `u_dut/`. Area and power are summed per group: the unit, the clock tree (`clkbuf*`, `clkload*`, which
also clocks the slices) and the wrapper (the slices and the port buffers). The groups add up to the design. Every number
below is the unit's own cells unless it says otherwise.

**Reports.** Each run writes ORFS's totals to `build/p12s8/<unit>_stream/<tag>/asic/metrics.json`, and two further
reports:

- **the stage report** (`stages.rpt`, from `ppa/asic/stream_stages.tcl`). It gives the worst setup slack into each
  pipeline stage's registers, the gates' enables at their latches (`cg_enable`), their gating checks (`cg_check`), and
  the design's worst slack. The decoder's stages are: issue -> core stage 1, core stages 2-4, stage 4's state read, the
  multiply stage, the add-and-renormalize stage's state, at-L flags, word FIFO and output FIFO, the tag pipeline, the
  issue control, the scales (the end of `scale_in`'s lookup) and the counters. The encoder's stages are: the input
  register (the end of `sym_in`'s lookup), core stages 1-2, the tag pipeline, the renorm stage, the divider stages, the
  lane write, the FIFO and the counters;
- **the power by register group** (`power/metrics.json`, `unit_groups` per workload, from `ppa/asic/power_stream.tcl`
  with `ppa/asic/stream_regs.tcl`). It splits the unit's flops by register, the gates' latches, and the combinational
  cells as one lump. The groups add up to the unit.

**Power.** A zero-delay gate-level simulation of the routed netlist (`ppa/power_tb_stream.sv`) gives the activity, so
the numbers contain no glitches. The simulation first runs `+CHECK`: jobs that reach the corners, corrupt arrays and
random stalls. Then it runs the workloads: 2 EMA jobs × 4,096 symbols from the stream model, a slowly drifting
discretized Laplace per lane with 2% uniform outliers. Every consumer is always ready, and each job is checked against
the model. OpenSTA then computes power from that activity and the routed parasitics.

The decoder's consumer puts each scale on `scale_in` in the cycle after the wrapper hands out the symbol it needs.
Through the two slices the unit then sees the scale three cycles after its symbol. That gives 0.797 symbols per cycle at
EMA delay 0. At delay 1 a lane's scale is ready a round early, and the unit reaches 0.990.

Energy per symbol = power × 3 ns × cycles / symbols, over the simulated window from the reset release to the last beat.
Idle cycles are included. `POWER_P_IDLE=<%>` holds the producer back on that share of the cycles: the encoder's `sym_in`
and the decoder's `scale_in`. The results then go to `power_idle<%>/`. `POWER_STIMULUS=static` or `random` replaces the
EMA jobs by a constant or a uniformly random job of the same length (§4.7); the results go to `power_static/` or
`power_random/`.

**Determinism.** The flow is deterministic. A rerun reproduces every number and the routed netlist byte for byte. The
encoder's round-2 netlist at this flow's head is byte-identical to its round's run.

**Flow noise.** Small RTL changes that leave the function and the critical structure alone still move ABC's mapping and
the placement. Between structurally equivalent netlists the spread is ±3% in area and energy and ±7% in Fmax. Examples:

- encoder R4c, the same function with its pipeline registers declared in one generate block: +3.0% area, +3.8% energy,
  567 against 619 MHz;
- decoder FIFO_WORDS 9 and 10, one entry apart: 5% apart in power;
- the decoder's issue path: 1.31-1.60 ns of slack across identical issue RTL.

A difference inside these bands, from one sample, is not significant. The ledgers therefore judge a change on the stages
it touches.

### 4.2 Results

#### Current design (contract version 5)

At this tree `make asic-stream-<cfg> power-stream-<cfg> ppa-stream-<cfg>` reproduces every column below to the last
digit, for p12s8 and p12s8sym (run 2026-10-18; the flow is deterministic). The units' own logic is the same in both
builds, because the alphabet lives in the cores (SPEC.md §1.8, §7).

| | decoder p12s8sym | decoder p12s8 | encoder p12s8sym | encoder p12s8 |
|---|---|---|---|---|
| unit area | 11,516 µm² | 11,400 (-116, -1.0%) | 10,571 | 10,286 (-285, -2.7%) |
| flops + gate latches | 952 + 35 | 952 + 35 | 1,036 + 9 | 1,036 + 9 |
| reported worst slack, Fmax | +0.733 ns (a gate enable), 614 MHz | +0.719 (a gate enable), 643 | +1.244 (the renorm stage), 570 | +1.152 (the renorm stage), 541 |
| the core's stages: slack | s2 +1.463, s3 +1.591, s4 +1.859 | s2 +1.474, s3 +1.681, s4 +1.854 | s1 +1.883, s2 +1.494 | s1 +1.867, s2 +1.505 |
| symbols per cycle (power workload; decoder: EMA delay 1) | 0.990 | 0.990 | 0.885 | 0.885 |
| unit, pJ/symbol (decoder: EMA delay 1 / 0) | 12.08 / 13.52 | 12.02 / 13.46 | 16.12 | 15.70 (-2.6%) |
| unit + clock tree, pJ/symbol | 17.27 / 19.83 | 17.25 / 19.82 | 22.37 | 21.93 |
| netlist check, workloads | 0 mismatches | 0 | 0 | 0 |
| generic gates, flat; depth | 8,245; 58 | 8,326; 57 | 5,169; 52 (DIV_BITS 2) | 5,142; 52 |

Every difference between the builds lies inside the noise band of §4.1 (±3% in area and energy, ±7% in Fmax). The
encoder's Fmax (-5.0%) comes from its renorm stage (+1.244 → +1.152 ns), which the alphabet does not touch. The cores'
generic gates are each build's own (SPEC.md §7): the decoder core 2,107 / 2,185 and the encoder core 797 / 778. The
version-5 columns of 2026-10-17, before commits that rename and comment and leave every generic gate count as it was,
differ from these by up to 2.5% in Fmax and 0.6% in energy.

#### The optimization rounds (2026-10-15, contract version 1, p12s8)

`pans_dec_stream` went through three rounds (rtl/PPA_DEC_STREAM.md §0-§6). Each column is `make asic-stream-p12s8
power-stream-p12s8` on the column's RTL with the unit's defaults. OUT_DEPTH is 8 before round 1 and 7 from round 1 on.
The round-3 column is the decoder round's own run (`make asic-dec-stream-p12s8 power-dec-stream-p12s8`). Its decoder
differs from the final one only in the gate module's name and in the wrapper forwarding the same parameter values.

| | before | round 1 | round 2 | round 3 |
|---|---|---|---|---|
| unit area | 16,485 µm² | 12,852 | 12,240 | 11,605 (-30%) |
| flops (+ gate latches) | 1,427 | 962 | 956 | 955 + 35 |
| register stages' slack: issue / multiply stage / core stage 2 | - | 1.60 / 1.56 / 1.60 ns | 1.42 / 1.57 / 1.56 | 1.49 / 1.51 / 1.58 |
| reported worst slack, Fmax | +1.124 ns, 533 MHz | +1.563, 696 | +1.416, 631 | +0.752 (a gate enable), 649 |
| unit power, pJ/symbol (EMA delay 1, 0.987 symbols per cycle) | 6.00 mW, 18.3 | 4.97, 15.12 | 4.72, 14.36 | 3.99, 12.13 (-34%) |
| unit + clock tree, pJ/symbol | 27.0 | 21.86 | 20.86 | 17.26 (-36%) |
| clock tree power | 2.83 mW | 2.22 | 2.17 | 1.68 |
| unit / unit + clock tree at EMA delay 0 (0.797 symbols per cycle) | 21.2 / 31.8 | 17.23 / 25.57 | 16.41 / 24.46 | 13.59 / 19.79 |
| netlist check, workloads | 0 mismatches | 0 | 0 | 0 |
| generic gates, flat (`make ppa-dec-stream-p12s8`) | 9,266 | 7,515 | 7,835 | 8,199 |

- Round 1 cut what the unit stored and carried. t is regenerated at the output. The state is read once. The end check
  works without two 32-bit counters. The output FIFO is one entry shorter. An escape reserves the words it reads.
- Round 2 made the reads off the issue path AND-OR over one-hot selects.
- Round 3 put 35 clock gates on the groups written only now and then (§2.1).

The before column's Fmax is set by its issue stage. From round 1 on the register stages sit at 1.4-1.6 ns of slack
(625-715 MHz). Which of them ORFS reports moves with the placement, the issue path by ±0.15 ns (§4.1). Generic gates
count an enable flop as one cell and the AND-OR reads as more gates than a mux tree. So rounds 2 and 3 show up in cells,
not in generic gates.

`pans_enc_stream` went through two rounds (rtl/PPA_ENC_STREAM.md §1-§5). Each column is `make asic-stream-p12s8
power-stream-p12s8` on the column's RTL with the unit's defaults. The final flow (`make asic-enc-stream-p12s8
power-enc-stream-p12s8`) reproduced the round-2 column, with a routed netlist byte-identical to the round's.

| | before | round 1 | round 2 |
|---|---|---|---|
| unit area | 12,200 µm² | 10,414 | 9,917 (-19%) |
| flops (+ gate latches) | 1,058 | 1,016 | 1,016 + 9 |
| renorm stage's slack | - | - | 1.385 ns |
| reported worst slack, Fmax | +1.517 ns, 674 MHz | +1.387, 620 | +1.279 (a gate enable), 619 |
| symbols per cycle (power workload) | 0.665 | 0.885 | 0.885 |
| unit power, pJ/symbol | 4.67 mW, 21.1 | 4.75, 16.09 | 4.40, 14.91 (-29%) |
| unit + clock tree, pJ/symbol | 31.0 | 22.87 | 21.00 (-32%) |
| clock tree power | 2.21 mW | 2.00 | 1.80 |
| netlist check, workload | 0 mismatches | 0 | 0 |
| generic gates, flat (`make ppa-enc-stream-p12s8`, DIV_BITS 2) | 6,794 | 4,760 | 4,872 |

- Round 1, on generic gates, made the divider radix 2 in cascaded steps (DIV_STEP_BITS). It fused the assembly into the
  last stage and released a lane early, which cut the issue distance from 11 to 9. It replaced the lane-state load by
  fresh bits, banked the FIFO, and took the first quotient bit in the renorm stage (DIV_FIRST_BITS).
- Round 2, on ORFS, put the lane states and the escape count behind clock gates and moved the read-lane select off the
  longest path.

The gates raise the generic gate count, which is 4,663 at `ENC_CLOCK_GATE=0`. An enable flop counts as one generic cell,
hold mux included. A clock gate's latch and AND count as extra cells.

**Stage report, encoder round 2.** From the final flow's `build/p12s8/enc_stream/n8_d2_o8/asic/stages.rpt` on the
round's RTL, slack in ns:

| core stage 1 | core stage 2 | tag pipeline | renorm stage | divider stages | lane write | FIFO entries | FIFO control | counters | gate enables at their latches | gating checks |
|---|---|---|---|---|---|---|---|---|---|---|
| +1.945 | +1.580 | +2.173 | +1.385 | +1.657 | +1.886 | +1.524 | +1.651 | +1.755 | +1.279 | +1.293 |

The renorm stage's path runs q2_lane -> the state read -> the first quotient step.

**Stage report, decoder round 3.** From the round's run, read with this flow's stage report, slack in ns:

| issue -> core stage 1 | core stages 2 / 3 / 4 | stage 4's read | multiply stage | add-and-renormalize: state / at-L flags / word FIFO / output FIFO | gate enables at their latches | gating checks |
|---|---|---|---|---|---|---|
| +1.489 | +1.575 / +1.663 / +1.908 | +2.517 | +1.510 | +1.666 / +1.460 / +1.726 / +1.901 | +0.752 | +1.286 |

#### Contract version 2

Contract version 2 lets the end symbols own the slots beyond the alphabet (SPEC.md §1.6). It adds two equality compares
to the decoder core's stage 3, and two compares and a flop to the encoder core's stage 1. Before is version 1, which
reproduces the decoder's round-3 and the encoder's round-2 columns above to the last digit. After is version 2. Both are
p12s8, each from a clean `build/` with `make asic-dec-stream-p12s8 power-dec-stream-p12s8` and `make
asic-enc-stream-p12s8 power-enc-stream-p12s8`.

| | decoder before | decoder after | encoder before | encoder after |
|---|---|---|---|---|
| unit area | 11,605 µm² | 11,665 (+60, +0.5%) | 9,917 | 10,031 (+114, +1.1%) |
| flops + gate latches | 955 + 35 | 955 + 35 | 1,016 + 9 | 1,017 + 9 |
| reported worst slack, Fmax | +0.752 ns (a gate enable), 649 MHz | +0.818 (a gate enable), 664 | +1.279 (a gate enable), 619 | +1.264 (a gate enable), 613 |
| the core's stages: slack | s2 +1.575, s3 +1.663, s4 +1.908 | s2 +1.537, s3 +1.584, s4 +1.907 | s1 +1.945, s2 +1.580 | s1 +1.954, s2 +1.616 |
| unit, pJ/symbol (decoder: EMA delay 1 / 0) | 12.13 / 13.59 | 12.42 / 14.00 | 14.91 | 15.20 (+1.9%) |
| unit + clock tree, pJ/symbol | 17.26 / 19.79 | 17.40 / 20.03 | 21.00 | 21.15 |
| netlist check, workloads | 0 mismatches | 0 | 0 | 0 |

Every difference lies inside the noise band of §4.1. The decoder's energy at EMA delay 0, +3.0% (+2.4% at delay 1), sits
at its edge. The decoder core's stage 3, where the compares sit, lost 0.08 ns of slack and does not limit the unit.
SPEC.md §7 gives the cores alone and their generic gates.

#### Contract version 3

Contract version 3 adds the symmetric alphabet, built as p12s8sym (SPEC.md §1.1). In the decoder core the lower half's
clamp becomes the upper half's. The encoder core's stage 1 gains a compare for -128. The table rule is data. Before is
p12s8 at version 2, which reproduces the version-2 columns above to the last digit. After is p12s8sym at version 3. Both
come from a clean `build/` on the same day.

| | decoder before | decoder after | encoder before | encoder after |
|---|---|---|---|---|
| unit area | 11,665 µm² | 11,313 (-352, -3.0%) | 10,031 | 10,082 (+51, +0.5%) |
| flops + gate latches | 955 + 35 | 955 + 35 | 1,017 + 9 | 1,017 + 9 |
| reported worst slack, Fmax | +0.818 ns (a gate enable), 664 MHz | +0.793 (a gate enable), 644 | +1.264 (a gate enable), 613 | +1.288 (a gate check), 631 |
| the core's stages: slack | s2 +1.537, s3 +1.584, s4 +1.907 | s2 +1.574, s3 +1.611, s4 +1.867 | s1 +1.954, s2 +1.616 | s1 +1.896, s2 +1.633 |
| unit, pJ/symbol (decoder: EMA delay 1 / 0) | 12.42 / 14.00 | 11.87 / 13.40 (-4.4%) | 15.20 | 15.41 (+1.4%) |
| unit + clock tree, pJ/symbol | 17.40 / 20.03 | 16.93 / 19.52 | 21.15 | 21.49 |
| netlist check, workloads | 0 mismatches | 0 | 0 | 0 |

The decoder's difference sits at the edge of the noise band of §4.1. Its logic did shrink: the core's generic gates go
from 2,185 to 2,107. The core alone in its wrapper grew 2.3% (SPEC.md §7).

#### Contract version 4

Contract version 4 adds the scale input b (§1.7). `pans_scale_of_b` sits between `scale_in` and the decoder's lane scale
registers, and in front of the encoder's new input register. The cores are unchanged and were not rerun. Both runs are
p12s8sym. Before is the version-3 numbers above. After is version 4, from a clean `build/`, with `make
asic-dec-stream-p12s8sym power-dec-stream-p12s8sym asic-enc-stream-p12s8sym power-enc-stream-p12s8sym` and `make
ppa-dec-stream-p12s8sym ppa-enc-stream-p12s8sym`. The workloads are the same jobs, except that the EMA source now gives
b (§1.7). So their scale codes follow the table, not `scale_from_sigma` per m.

| | decoder before | decoder after | encoder before | encoder after |
|---|---|---|---|---|
| unit area | 11,313 µm² | 11,498 (+185, +1.6%) | 10,082 | 10,154 (+72, +0.7%) |
| flops + gate latches | 955 + 35 | 955 + 35 | 1,017 + 9 | 1,036 + 9 (the input register's 19) |
| reported worst slack, Fmax | +0.793 ns (a gate enable), 644 MHz | +0.725 (a gate enable), 616 | +1.288 (a gate check), 631 | +1.294 (a gate enable), 592 |
| the core's stages: slack | s2 +1.574, s3 +1.611, s4 +1.867 | s2 +1.522, s3 +1.624, s4 +1.913 | s1 +1.896, s2 +1.633 | s1 +1.872, s2 +1.628 |
| the lookup's path (port register → `pans_scale_of_b` → register): slack | | +2.135 (into the lane scale registers; the `scale` group +1.808, at `scale_more_q`) | | +2.137 (`input`) |
| unit, pJ/symbol (decoder: EMA delay 1 / 0) | 11.87 / 13.40 | 12.21 / 13.70 (+2.9% / +2.2%) | 15.41 | 15.39 (-0.1%) |
| unit + clock tree, pJ/symbol | 16.93 / 19.52 | 17.33 / 19.90 | 21.49 | 21.53 |
| netlist check, workloads | 0 mismatches | 0 | 0 | 0 |
| generic gates, flat; depth | 8,142; 60 | 8,308 (+166); 60 | 4,855; 52 (DIV_BITS 2) | 5,151 (+296); 52 |
| the lookup, `pans_scale_of_b` (generic gates, hierarchy kept) | | 206 | | 198 |

The lookup ends in registers with ample slack in both units: 2.14 ns at 3 ns in each. In the decoder the lookup's
arrival from the port register is 1.05 ns. On its own the lookup is 186 gates with depth 20 (SPEC.md §3). Every number
that moved lies inside the noise band of §4.1. The Fmax each unit reports comes from a register path that the change did
not touch: in the decoder the add-and-renormalize stage's at-L flags at +1.378 ns, in the encoder the renorm stage at
+1.311. That path moves with the placement. Version 3's worst register paths were at +1.447 and +1.415 ns.

The encoder's input register costs 0.085 mW (`input_reg`, 19 flops). Its unit energy is unchanged within 0.1%. The
decoder adds no flops, because the lane scale registers hold the 7-bit code as before. Its +185 µm² are the lookup's
logic and placement. How much of its +2-3% energy comes from the lookup and how much from the workload's changed codes
is not separated.

The table is periodic in lead. At p12s8 every entry that an integer b reaches is exp = 15 - lead - below[3], clamped to
[1, 14], with the mantissa a function of below alone. A lookup built that way, with a subtract, a 16-entry mantissa ROM
and the clamps, synthesizes to 197 generic gates with depth 21. That scratch RTL equals `SCALE_TABLE` at all 32,768 b.
The table read is 186 gates with depth 20. So ABC finds the structure itself, and the RTL keeps the table read.

#### Contract version 5

Contract version 5 pushes an escape's raw value as S bits (§1.4). The encoder's renorm stage pushes it with an OR of the
state's top S bits and a concatenation. Version 4 had push(raw, 1)'s compare of the top P bits instead. The decoder's
add-and-renormalize stage shifts by S and lost the raw check. The multiply stage lost the interval check. The decoder
unit lost three flops: q5_err_interval, err_interval_q and err_raw_q. The cores are unchanged and were not rerun. Both
runs are p12s8sym. Before is the version-4 numbers above. An encoder rerun from an export of the version-4 tree
reproduces them to the last digit: 10,154.5 µm², 592.1 MHz, 15.388 and 21.527 pJ. After is version 5, the same targets
from a clean `build/`. The workloads are the same jobs. Their escapes move fewer words.

| | decoder before | decoder after | encoder before | encoder after |
|---|---|---|---|---|
| unit area | 11,498 µm² | 11,528 (+30, +0.3%) | 10,154 | 10,571 (+416, +4.1%) |
| flops + gate latches | 955 + 35 | 952 + 35 | 1,036 + 9 | 1,036 + 9 |
| reported worst slack, Fmax | +0.725 ns (a gate enable), 616 MHz | +0.782 (a gate enable), 630 | +1.294 (a gate enable), 592 | +1.240 (the renorm stage), 568 |
| the stages the change touches: slack | the add-and-renormalize stage's at-L flags +1.378 (the others not recorded) | the multiply stage +1.510; the add-and-renormalize stage: state +1.618, at-L flags +1.413, word FIFO +1.579, output FIFO +1.680 | renorm +1.311, FIFO entries +1.469 | renorm +1.240, FIFO entries +1.420 |
| the core's stages: slack | s2 +1.522, s3 +1.624, s4 +1.913 | s2 +1.477, s3 +1.584, s4 +1.867 | s1 +1.872, s2 +1.628 | s1 +1.884, s2 +1.495 |
| symbols per cycle (power workload; decoder: EMA delay 1) | 0.987 | 0.990 | 0.885 | 0.885 |
| unit, pJ/symbol (decoder: EMA delay 1 / 0) | 12.21 / 13.70 | 12.07 / 13.51 (-1.1% / -1.4%) | 15.39 | 16.13 (+4.8%) |
| unit + clock tree, pJ/symbol | 17.33 / 19.90 | 17.21 / 19.75 | 21.53 | 22.37 (+3.9%) |
| netlist check, workloads | 0 mismatches | 0 | 0 | 0 |
| generic gates, flat; depth | 8,308; 60 | 8,245 (-63); 58 | 5,151; 52 (DIV_BITS 2) | 5,169 (+18); 52 |

The decoder moves inside the noise band of §4.1. Its logic is smaller: -63 generic gates, the unit's own module 5,956 →
5,910, depth 60 → 58. Its workload is faster, with fewer words per escape and fewer waits for words.

The encoder's area and energy move just outside the band, +4.1% and +4.8%. Its Fmax moves inside it, -4%. Every added
cell is combinational: `unit_comb` goes from 4,463 to 4,871 cells and from 2.10 to 2.28 mW. The flops and their power
are unchanged. The added cells are already present after ORFS's synthesis, before placement. The unit after yosys and
ABC goes from 5,866 to 6,317 cells and from 10,464 to 10,903 µm². The generic flow sees only +18 gates. At DIV_BITS 1
and 4 the generic counts are 5,639 → 5,613 and 4,963 → 4,964. The renorm stage holds the escape's two pushes and is the
unit's critical path (+1.311 → +1.240 ns). Version 4's raw push left four constant-zero bits in the escape's state,
because raw was zero-extended to P bits. In version 5 every bit of the state is live. ABC's timing-driven mapping grows
there by more than the generic count shows. rtl/PPA_ENC_STREAM.md §5 lists an untried simplification of the escape path.

### 4.3 Options (measured, not the defaults)

**Encoder**, on the round-2 design (rtl/PPA_ENC_STREAM.md §4). Each row is `make asic-enc-stream-p12s8
power-enc-stream-p12s8 <VARIABLES>`. Every netlist check has 0 mismatches.

| Variables | unit area | flops, latches included | worst slack / Fmax | symbols per cycle | pJ/symbol unit / + clock tree |
|---|---|---|---|---|---|
| (the defaults: DIV_BITS 2) | 9,917 µm² | 1,025 | +1.279 / 619 MHz | 0.885 | 14.91 / 21.00 |
| `ENC_DIV_BITS=3` | 10,021 | 857 | +1.257 / 576 | 0.995 | 13.96 / 18.64 |
| `ENC_DIV_BITS=4 ENC_DIV_FIRST_BITS=4` (three divider stages) | 9,505 | 745 | +0.347 / 377 | 0.996 | 13.18 / 17.36 |
| `ENC_OUT_DEPTH=6` | 9,572 | 992 | +1.276 / 615 | 0.832 | 14.61 / 20.80 |
| `ENC_DIV_BITS=3 ENC_OUT_DEPTH=6 ENC_COUNT_WIDTH=24` | 9,113 | 800 | +1.155 / 542 | 0.849 | 13.62 / 18.49 |

Where area and energy count more than the clock, the last two rows are the designs. Three divider stages suit a clock of
377 MHz or less. Otherwise DIV_BITS 3 with OUT_DEPTH 6 and COUNT_WIDTH 24 fits, for n below 2^24.

One further option is not in the tree. It exists as scratch RTL (rtl/PPA_ENC_STREAM.md R5): the divider's pipeline
registers gated by the valid of the stage feeding them. It costs +3.7% area and is a wash at full rate. With a producer
idle 50% / 80% of the time it gives 17.29 / 24.06 pJ per symbol (unit), against the default's 19.36 / 32.01. `make
power-enc-stream-p12s8 POWER_P_IDLE=50` reproduces the default's 19.36 at 0.497 symbols per cycle, 30.12 with the clock
tree.

**Decoder** (rtl/PPA_DEC_STREAM.md §5.3, §5.4, §6). The FIFO_WORDS rows are on the round-2 design, before the clock
gates.

| Variant | unit area | worst slack / Fmax | pJ/symbol unit, EMA delay 1 | symbols per cycle, EMA delay 1 | |
|---|---|---|---|---|---|
| round 2, FIFO_WORDS 8 | 12,240 µm² | +1.416 / 631 MHz | 14.36 | 0.987 | the round-2 default |
| `DEC_FIFO_WORDS=9` | 12,803 | +1.463 / 651 | 14.60 | 0.995 | |
| `DEC_FIFO_WORDS=10` | 12,410 | +1.497 / 665 | 13.93 | 0.995 | |
| one pop stage (pop1; not in the tree) | 12,754 | +0.752 / 445 | 14.80 | 0.995 | issue -> state written in 5 cycles. At EMA delay 0 it reaches 0.885 instead of 0.797, and the consumer may answer two cycles after the symbol |
| no stage-4 state register (not in the tree) | 12,109 | +1.297 / 587 | 14.14 | 0.987 | |
| round 3 + G3: the per-cycle pipeline registers gated (not in the tree) | 11,640 | +0.776 / 624 | 12.23 | 0.987 | at EMA delay 0 13.70 against round 3's 13.59, a wash. It suits a host that idles the unit |

FIFO_WORDS 9 or 10 removes the issue's waits for words (§2.1), for +170 to +563 µm². The energy per symbol moves inside
the noise. COUNT_WIDTH 24 (n and n_bytes below 2^24) is measured in generic gates only: -294 gates and -40 flops on the
round-3 design (`make ppa-dec-stream-p12s8 DEC_COUNT_WIDTH=24`). On the round-1 design it was -203 and -48
(rtl/PPA_DEC_STREAM.md §1, K).

### 4.4 Where the energy goes

A clocked flop costs about 1.3-1.5 µW of clock-pin internal power whether or not it toggles. It also costs 1.5-1.6 µW of
clock tree at 3 ns. That is 2.8-3.3 µW standing, by the two ledgers' estimates, or about 0.01 pJ per symbol per flop at
0.987 symbols per cycle. Flop count and clock gating are therefore the energy lever. Gating the data of a register that
is clocked anyway is not (encoder R1). The combinational logic, the work itself, switches on real data every cycle.

**Decoder, round 3** (this flow's split of its round's run, EMA delay 1). The unit's 3.99 mW split into:

| Group | mW |
|---|---|
| combinational (6,087 cells: the core's logic, the multiplier, the muxes) | 2.24 |
| the core's 143 flops | 0.59 |
| the per-cycle tag pipeline (q4, q5, q6) | 0.41 |
| the counters | 0.24 |
| the control | 0.18 |
| the gated lane states | 0.16 |
| the 35 gate latches | 0.07 |
| the gated output entries | 0.045 |
| the gated scales | 0.035 |
| the gated word FIFO | 0.025 |

The clock tree adds 1.68 mW. It also clocks the wrapper's 442 flops.

**Encoder, round 2** (the final flow's run). The unit's 4.40 mW split into:

| Group | mW |
|---|---|
| combinational (the divider's restoring steps and the renorm stage) | 2.03 |
| the divider's pipeline registers (224 dividend, 96 f, 96 c, 24 lane, 8 valid flops) | 1.51 |
| the core's flops | 0.19 |
| the counters | 0.18 |
| the FIFO | 0.20 |
| the gated lane states | 0.14 |
| the tag pipeline | 0.09 |

The clock tree adds 1.80 mW. What remains is the arithmetic and the registers that every stage needs every cycle. The
encoder's divider stage count (DIV_BITS) is its remaining lever (§4.3).

### 4.5 Every candidate

The ledgers give each candidate with its numbers, whether the design kept it, and why:

| Unit, round | Ledger sections | Kept | Not kept, or an option |
|---|---|---|---|
| decoder, round 1 (generic gates) | rtl/PPA_DEC_STREAM.md §1-§3 | A ((lane, t) regenerated), D (the state read once), B and B2 (the pop count registered, FIFO bookkeeping from candidates), E (the end check without two counters), F+G (one write mux per beat slot, at-L flags), OUT_DEPTH 7, I and I2 (an escape reserves its words), K (COUNT_WIDTH) | not kept: C and C2 (s - c in the multiplier), and the list of rtl/PPA_DEC_STREAM.md §3 (carry-save multiply, prefetched slot, shift-register FIFO, head-word prefetch, a beat counter, another pop stage) |
| decoder, round 2 (ORFS) | rtl/PPA_DEC_STREAM.md §5 | the AND-OR reads off the issue path | options or not kept: the full AND-OR, C, C + mux2, one pop stage, no stage-4 register, FIFO_WORDS 9 and 10. Not run, with reasons: rtl/PPA_DEC_STREAM.md §5.5 |
| decoder, round 3 (clock gates) | rtl/PPA_DEC_STREAM.md §6 | G1 and G2b (35 gates). G2 was reworked into G2b | option: G3. Not gated, with reasons: rtl/PPA_DEC_STREAM.md §6.1 |
| encoder, round 1 (generic gates) | rtl/PPA_ENC_STREAM.md §1-§3 | A (DIV_STEP_BITS), B (fused assembly), C (early lane release), D (fresh bits), E (COUNT_WIDTH, words counted at the FIFO output), F (banked FIFO), H (DIV_FIRST_BITS) | not kept: non-restoring steps, G (c per lane), I (escape bypass), J (a shared multi-cycle divider), K (f per lane), L (3f carried) |
| encoder, round 2 (ORFS) | rtl/PPA_ENC_STREAM.md §4, §5 | R2 (the read-lane select), R4b (CLOCK_GATE: lane states and escape count) | not kept: R1 (entry hold), R4 (FIFO gates), R4c, R6 (one-hot lane select), ABC_AREA. Option: R5. Not tried: rtl/PPA_ENC_STREAM.md §5 |

### 4.6 ASAP7

`make asic-stream-p12s8 power-stream-p12s8 ASIC_PLATFORM=asap7` runs both units on ASAP7 at its typical corner with a
1.0 ns target, `asic_period_ns_asap7` of p12s8. SPEC.md §7.5 gives the kit, the five TT libraries (0.7 V, 25 °C), the
settings and what differs from Nangate45. The wrapper, the groups, the reports and the workloads are those of §4.1, with
the unit defaults. The results go to `asic_asap7/` and `power_asap7/` beside `asic/` and `power/`, with every ORFS log.

Both columns are p12s8 at contract version 5, run on 2026-10-18. Nangate45 is the p12s8 column of §4.2 at 3 ns. The
ratios are Nangate45 over ASAP7 for area and energy, and ASAP7 over Nangate45 for Fmax.

| p12s8, N = 8, 1 ns | decoder | encoder |
|---|---|---|
| unit area | 866 µm² | 798 µm² |
| flops + gate latches | 952 + 35 | 1,036 + 9 |
| clock tree, wrapper (its flops) | 87 µm², 292 µm² (454) | 52 µm², 226 µm² (352) |
| reported worst slack | 0.000 ns (a gate enable, time borrowed at its latch) | +0.054 ns (the renorm stage) |
| worst flop path, Fmax | +0.040 ns (issue -> core stage 1), 1,041 MHz | +0.054 ns, 1,057 MHz |
| the core's stages: slack | s2 +0.085, s3 +0.146, s4 +0.304 | s1 +0.221, s2 +0.067 |
| symbols per cycle (power workload; decoder: EMA delay 1 / 0) | 0.990 / 0.797 | 0.885 |
| unit, pJ/symbol | 2.05 / 2.28 | 2.85 |
| unit + clock tree, pJ/symbol | 3.48 / 4.00 | 4.39 |
| netlist check (+CHECK), workloads | 0 mismatches in 20,624 checks; 0 | 0 in 1,825; 0 |
| Nangate45, 3 ns: unit area / Fmax / unit pJ / with clock tree | 11,400 µm² / 643 MHz / 12.02 / 17.25 | 10,286 µm² / 541 MHz / 15.70 / 21.93 |
| ratios: unit area / Fmax / unit energy / with clock tree | 13.2× / 1.62× / 5.88× / 4.95× | 12.9× / 1.95× / 5.51× / 5.00× |

**Timing.** Every path of both units meets the 1 ns target; the total negative slack is 0. The decoder's reported
worst slack, 0.000 ns, is time borrowing at a latch: the enable of a clock gate (`g_word[2]`) borrows 6 ps of the
492 ps its latch allows, and the gating check behind the latch keeps +0.360 ns. ORFS's Fmax counts the worst flop path,
+0.040 ns. The slowest stages keep 40-90 ps: the decoder's issue -> core stage 1, counters, at-L flags, multiply stage,
core stage 2 and word FIFO, the encoder's renorm stage, core stage 2 and FIFO. On Nangate45 the same stages keep
1.15-1.8 ns of 3 ns. The Fmax is ORFS's 1 / (target - worst flop slack) at this target, not a maximum frequency of the
units.

**Energy.** The units' own cells spend 5.5-5.9× less energy per symbol than on Nangate45, their clock tree 3.6×
(decoder) and 4.1× (encoder) less. The clock tree's share therefore grows: with it, the units spend 4.95× and 5.00×
less.
Leakage is 1.0-1.3 µW per design, unit, clock tree and wrapper together. On Nangate45 it is 4-6% of the units' power.

### 4.7 Workload sensitivity

`make power-dec-stream-p12s8 POWER_STIMULUS=static` and `POWER_STIMULUS=random`, with `ASIC_PLATFORM=asap7` for ASAP7,
rerun the decoder unit's gate-level power on the routed netlists of §4.2 and §4.6 with two other workloads. Each is two
LIST jobs of 4,096 symbols, as long as the EMA workload. static codes the mid-alphabet symbol 0 at the mid-range b =
2^14 for every symbol, so nothing escapes: the activity floor. random draws the symbols uniform over the alphabet and b
uniform over its 15 bits, escapes included: the ceiling. Their scales are known ahead, so the decoder runs each once, as
at EMA delay 1. The netlist check (`+CHECK`) and every workload have 0 mismatches. The percentages are against the EMA
workload at delay 1 (§4.1), run on the same day from the same netlists (2026-10-18, contract version 5).

| p12s8 `pans_dec_stream`, N = 8 | symbols per cycle | escapes in 8,192 | Nangate45, 3 ns: unit / with clock tree, pJ per symbol decode | ASAP7, 1 ns: unit / with clock tree |
|---|---|---|---|---|
| static | 0.995 | 0 | 5.63 (-53.1%) / 10.86 (-37.0%) | 0.97 (-52.6%) / 2.41 (-30.8%) |
| EMA, delay 1 | 0.990 | 142 | 12.02 / 17.25 | 2.05 / 3.48 |
| random | 0.967 | 719 | 12.00 (-0.2%) / 17.39 (+0.8%) | 2.05 (+0.1%) / 3.54 (+1.6%) |

The uniformly random job costs within 2% of the EMA workload on both platforms, with five times its escapes and 2.4%
fewer symbols per cycle. The static job halves the unit's own energy: its lane states still change with every symbol
decode and its words still arrive. The clock tree's power hardly depends on the data (1.73-1.74 mW on Nangate45,
1.42-1.44 mW on ASAP7, in all three), so with the clock tree static saves 37% on Nangate45 and 31% on ASAP7.
