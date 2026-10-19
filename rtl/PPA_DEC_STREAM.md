# pans_dec_stream: PPA ledger

This ledger records every optimization candidate measured on the decoder stream unit `pans_dec_stream`, with its numbers
and its verdict. The measurements ran in three rounds, at contract version 1, on p12s8 with N_LANES = 8 and the unit's
defaults unless a row says otherwise. STREAM.md §4.2 gives the numbers of the current design, and STREAM.md §4.5
summarizes this ledger.

| Round | Date | Flow | Sections |
|---|---|---|---|
| 1 | 2026-10-15 | generic gates | §0-§4 |
| 2 | 2026-10-15 | ORFS, Nangate45, 3 ns | §5 |
| 3 | 2026-10-15 | ORFS, Nangate45, 3 ns: clock gates | §6 |

**How to read a row.** A row names a change, its result measured against the design as it stood, and a verdict. "Kept"
means the design includes the change. "Not kept" means it does not, for the stated reason. "Option" means a parameter
value or a variant that trades one metric for another. Its numbers are recorded, and the default does not use it. The
kept changes are cumulative. Each row's base is the design with every kept change above it.

## Methods

**Generic gates (round 1).** `ppa/synth_generic.sh` runs yosys with slang, flattened, with a pinned ABC script (`make
ppa-dec-stream-p12s8`). Gate counts compare only within this flow, never as area. "Depth" is the number of gates on the
longest register-to-register path (`ltp -noff`). Per-stage depths come from the same netlist with the fan-in cone of
each register group cut at flops. A scratch yosys script does this: `select w:<reg> %ci:+[Q] %ci %ci*:-<flop types>; ltp
-noff`. The flop count includes every `$_DFF*` cell, the core's ~140 among them. In the cores a flop costs 4-5 generic
gate-equivalents of area. So round 1 judges a change on gates + ~4.5 × flops. Wide muxes synthesize better than their
gate count suggests. ABC's result moves by ±3 levels and a few tens of gates between equivalent netlists. This ledger
calls differences inside that band noise.

**Throughput.** `make sim-dec-stream-p12s8`, seed 1, runs the testbench's throughput jobs: 16,384 EMA-scaled symbols, no
stalls. In (a) the consumer's scale arrives one cycle after the symbol's transfer. In (b) it arrives two cycles after.

**ORFS (rounds 2 and 3).** The flow is the stream units' PPA flow (STREAM.md §4.1). The unit runs inside its timing
wrapper, with the unit's cells kept apart (`u_dut/`). Power comes from a zero-delay gate-level simulation of the routed
netlist on 2 EMA jobs × 4,096 symbols. The energy figures are the unit's own cells at EMA delay 1, 0.987 symbols per
cycle. "Unit + clock" adds the whole clock tree, which also clocks the wrapper's 442 flops. `make
build/p12s8/dec_stream/<tag>/{asic,power}/metrics.json` runs one variant. Every variant is one ORFS run against the same
flow. The flow is deterministic, and round 2 reproduced the round-1 result to the µm². The flow's stage report
(`ppa/asic/stream_stages.tcl`) and its power groups (`ppa/asic/stream_regs.tcl`) grew out of round 2's scratch scripts
`stage_timing.tcl` and `power_split.tcl`.

**The rule of rounds 2 and 3.** A change is kept iff it lowers area or energy at equal or better Fmax. A change that
trades Fmax for area is recorded as an option.

## 0. Baseline (the unit before the rounds, 2026-10-15)

| gates, flat | of them the core | flops | of them the unit | logic | depth | throughput (a) | throughput (b) |
|---|---|---|---|---|---|---|---|
| 9,266 | 2,194 | 1,427 | 1,287 | 7,839 | 68 | 0.9885 (192 bubbles in 16,384 symbols for want of FIFO words) | 0.8880 |

Flops of the unit by register, from an RTL inventory (1,279 + a few control bits):

| Register | Flops | Note |
|---|---|---|
| x_q[8] | 224 | the lane states: inherent |
| scale_q[8], scale_present, scale_more | 72 | inherent (scale_t is 7 bits at p12s8) |
| tag pipeline q1..q4 {lane 3, t 29, slot 12} | 176 | t and slot carried beside the core |
| q5 {lane, t, prod 28, d 13, escape, symbol 8, err} | 83 | |
| output FIFO 8 x {lane 3, t 29, symbol 8, escape 1} | 328 | t and lane stored per entry |
| word FIFO 8 x 16 + pointers + count | 138 | |
| six 32-bit counters (n_bytes, sym_left, in_bytes_left, in_words_left, words_consumed, n_escapes) | 192 | |
| rr_lane, rr_t | 32 | |
| reserved, credit, of pointers/count, hdr_beat, phase, errors, ok | ~30 | |

The baseline holds t (29 bits) in 14 places: rr_t, 4 tag pipeline stages, q5 and 8 FIFO entries, 406 flops in all. It
holds the slot in 4 tag pipeline stages, 48 flops. Two 32-bit counters, n_bytes and in_words_left, serve only the end
check and the word count.

Depth per stage of the baseline (gates, cone cut at flops):

| Path ends at | Depth | What |
|---|---|---|
| words_consumed_q | 68 | the add-and-renormalize stage: 28-bit sum -> refill tests -> pop count -> 32-bit add |
| wf_count_q / wf_rd_q | 63 / 59 | the add-and-renormalize stage: the same pops into the FIFO bookkeeping |
| q5_prod | 58 | the multiply stage: 8:1 x read mux -> 16 x 12 multiply |
| err_words_q | 56 | the add-and-renormalize stage: s6_over |
| x_q[*] | 52 | the add-and-renormalize stage: the state write |
| in_words_left_q | 52 | in_wr_n (data_in_ready <- FIFO room / drain) -> 32-bit decrement |
| u_core.q1_* | 49 | issue 8:1 slot mux -> the core's stage 1 |
| u_core.q2_* | 46 | the core's stage 2 (its own longest; 42 when synthesized alone) |
| of_mem_q[*] | 45 | the add-and-renormalize stage: the symbol into the output FIFO |
| u_core.q3_*, out_* | 29, 31 | |

Micro-measurements, from scratch wrappers in the same flow:

| Block | Gates | Depth |
|---|---|---|
| 16 x 12 multiply | 1,102 | 52 |
| the same behind an 8:1 28-bit mux | 1,345 | 60 |
| multiply-add (x_hi f + d) | 1,234 | 56 |
| the 28-bit add with both refills | 398 | 49 |
| the refills alone | 231 | 13 |
| the core's q_divide (its multiplier) | 778 | 38 |

## 1. Candidates, in the order tried

Gates, flops and depth after each change: flat, p12s8, defaults.

| # | Change | Gates | Flops | Depth | Verdict |
|---|---|---|---|---|---|
| 0 | baseline | 9,266 | 1,427 | 68 | |
| A | (lane, t) regenerated at the output from a counter pair. t dropped from rr, the tag pipeline, q5 and the output FIFO. lane dropped from the FIFO | 8,526 | 1,029 | 63 | kept: -740 gates, -398 flops |
| D | x[j] read once, in stage 4 into q4_x (lane-only tag pipeline). The slot and the multiply stage's read mux go | 8,597 | 1,012 | 67 | kept: -17 flops for +71 gates. The slot is read twice, at issue and in stage 4. The alternative, a 16-bit x_hi register beside the slot's tag pipeline, is +16 flops. The multiply stage goes from 58 to 53 |
| C | the add s - c folded into the multiply stage as a multiply-add, the refills alone in the add-and-renormalize stage | 8,565 | 999 | 63 | not kept: the multiply stage reaches 63, because the 13-bit s - c subtract sits in series before the multiplier's tree. The standalone MAC was 56 |
| C2 | C with s and c as separate operands of the multiplier's tree | 8,678 | 999 | 61 | not kept: +113 gates for 2 levels |
| B | the pop count registered (q6_pops). words_consumed adds it a cycle later | 8,577 | 1,015 | 62 | kept: the words_consumed path goes from 67 to 25. The FIFO bookkeeping is then the longest path |
| B2 | the word FIFO's pointer and count after a pop selected among candidates from the registers. A dry FIFO ends empty (rd = wr, count = written) instead of taking a clamped pop count | 8,571 | 1,018 | 57 | kept: wf_count 62 -> 57, wf_rd 58 -> 54, err_words 55 -> 53 |
| E | the end check and the word count without n_bytes and in_words_left: bytes_exact and word_dropped flags. "Every word in the FIFO" = in_bytes_left < W/8. in_wr_n from the beat mask's byte-present bits, with one shared high-bits test | 7,913 | 956 | 59 | kept: -658 gates, -62 flops. The 52-deep decrement path goes |
| F+G | one state write mux per header beat slot, not per lane. at_L[j] = (x == L) decided at the write, and the end check ANDs the bits | 7,539 | 964 | 58 | kept: -374 gates, +8 flops |
| OUT_DEPTH | default 8 -> 7 | 7,496 | 953 | 58 | kept: the same throughput, since 7 = 6 steps in flight + 1. 6 falls to 0.857 (credit bubbles). 16 gains nothing (7,752 / 1,041) |
| I | an escape reserves the words it will read: 2 iff escape_second_word(x) (x' = (x >> P) E + s mod E < 2^(2P - W)), else 1. Released from the step's own x | 7,577 | 954 | 59 | kept: throughput (a) 0.9885 -> 0.9939, (b) 0.8880 -> 0.8884, for +81 gates |
| I2 | escape_second_word decided at the lane write into second_word_q[j], like at_L. The issue mux reads one bit | 7,515 | 962 | 59 | kept: -62 gates, +8 flops. The issue path goes from 26 to 22 |
| K | COUNT_WIDTH a parameter, default 32 (the ports) | 7,515 | 962 | 59 | kept, with no change at 32. At 24: 7,312 / 914 / 53, and the simulation passes with -GCOUNT_WIDTH=24 |

FIFO_WORDS, measured at F+G with OUT_DEPTH 8. Throughput (a) and (b), and the bubbles for FIFO words in (a):

| FIFO_WORDS | Gates | Flops | (a) | (b) | Bubbles |
|---|---|---|---|---|---|
| 8 (default) | 7,539 | 964 | 0.9885 | 0.8880 | 192 |
| 9 | 7,718 | 983 | 0.9995 | 0.8889 | 10 |
| 10 | 7,794 | 999 | 1.0000 | 0.8889 | 1 |
| 12 | 8,030 | 1,031 | 1.0000 | 0.8889 | 1 |
| 16 | 8,312 | 1,096 | 1.0000 | 0.8889 | 1 |

The bubbles at 8 are structural. With six coded steps in flight the issue needs count ≥ 7. A beat of 2 words is accepted
only at count ≤ 6. So the count cycles 8 -> 7 -> 6 (a bubble, a beat) -> 8.

No stall-free reservation tighter than one word per coded step exists. Whether a coded step pops depends on f. On
corrupt input f = 0, a slot outside every interval, pops for any x. A step that reserved 0 words could then find its
word still in the array, and the unit would diverge from the model.

The default is 8. 9 or 10 buy 1.1-1.15% throughput for 2.4-3.4% more gates, and both are a parameter value away. With I
the default has fewer bubbles (0.9939).

## 2. Result of round 1

| | gates, flat | of them the core | flops | of them the unit | logic | depth | throughput (a) | throughput (b) |
|---|---|---|---|---|---|---|---|---|
| round-1 design | 7,515 | 2,194 (unchanged) | 962 | ~820 | 6,553 | 59 | 0.9939 | 0.8884 |
| against the baseline | -1,751 (-19%) | | -465 (-33%) | | | 68 -> 59 | | |

Counting a flop as 4.5 gate-equivalents, the area falls by 3,840 of ~15,700, -24%. Throughput is from seed 1.

Depth per stage of the round-1 design:

| Path ends at | Depth | What |
|---|---|---|
| at_L_q | 59 | the add-and-renormalize stage: 28-bit add -> refills -> state mux -> == L |
| wf_count_q / wf_rd_q | 56 / 54 | the add-and-renormalize stage: the pops select the FIFO's count and pointer |
| q5_* (the multiply stage) | 53 | q4_x register -> 16 x 12 multiply (no mux ahead of it) |
| x_q[*] | 53 | the add-and-renormalize stage: the state write |
| err_words_q | 52 | the add-and-renormalize stage: s6_over from the refills and the count flags |
| u_core.q1_* | 49 | issue 8:1 slot mux (12 bits) -> the core's stage 1 |
| u_core.q2_* | 47 | the core's stage 2 |
| of_mem_q[*] | 44 | the add-and-renormalize stage: the symbol |
| words_consumed_q | 33 | the registered pop count into the 32-bit add |
| q1_valid (issue) | 22 | |

The unit's longest paths are the multiply stage at 53 and the add-and-renormalize stage at 52-59. They lie within 4-10
levels of the core's stage 1 as seen in the unit (49) and within 6-12 of its stage 2 (47). The baseline's longest paths
lay 10-19 levels beyond.

Generic depth cannot rank these paths in real cells. ABC's generic multiplier, maccmap's CSA tree with a 28-bit final
adder, is 52 levels for 16 x 12 and 38 for the core's q_divide. Real cells with timing-driven mapping compress both
differently. ORFS at 3 ns shows whether the multiply stage's multiplier or the add-and-renormalize stage's add and
refills set Fmax below the core's (§5). Both are a single stage's arithmetic. The cores' p16s16 decoder, with a
comparable multiplier in its stage 2, reaches 592 MHz at its 5 ns target at contract version 1 (SPEC.md §7.2).

## 3. Not taken in round 1, with the reason

- **Carry-save split of the multiply.** The multiply stage would hold the CSA tree only, as sum and carry vectors. The
  add-and-renormalize stage would do the final 3:2 add and the refills. That balances the stages at ~35 / ~55 instead of
  53 / 59, for +28 flops and a hand-written tree. Not needed at a 3 ns target. It pays only if ORFS shows the multiply
  stage setting Fmax.
- **A prefetched slot register at issue.** The issue's 8:1 mux adds ~7 levels to the core's stage 1: 49 against 42
  alone. A 12-bit register loaded a cycle ahead, plus a bypass from the add-and-renormalize stage's write, removes it
  for +12 flops. It would not shorten the unit's longest path, the add-and-renormalize stage. Not taken.
- **One-hot lane selects and AND-OR read muxes.** Generic gates count a MUX2 as one gate and AND-OR as two per input, so
  this flow cannot rank them. Left to ORFS, should the muxes show up there. They are 12 + 28 bits of x, 7 of scale, 2 x
  16 of FIFO words and 9 of the output entry. Round 2 measured them (§5.2).
- **A shift-register word FIFO.** It has no read muxes, but every entry moves by 0, 1 or 2 per cycle and takes a write.
  Each bit needs ~4 MUX2, against ~2 for the circular FIFO's read and write. More gates, and all entries toggle. Not
  built.
- **Head-word prefetch registers** for the two-word read port. The same muxes move to the refill of the registers, and
  32 flops are added. The read port is not on the longest path. Not built.
- **A beat counter instead of in_bytes_left.** It saves the saturating 32-bit compare, ~30 gates. It needs n_bytes mod
  DATA_BYTES at job_in, which for a DATA_BYTES that is not a power of two is a constant-modulus tree. The parameter
  allows 12. Not worth the generality cost.
- **Credit derived from of_count and the valids.** 3 flops and an adder against a popcount on the issue condition: a
  wash.
- **A stall-free reservation tighter than one word per coded step.** Impossible with bit-exactness on corrupt input
  (§1).
- **Another pipeline stage in the pop**, to cut the multiply stage or the add-and-renormalize stage in two. The loop
  from issue to x written would take 7 cycles. At N_LANES = 8 the consumer's scale would then have to arrive in the same
  cycle as the symbol, or throughput drops to 8/9. That breaks the interface's N = 8 timing. Round 2 measured the
  opposite change, one pop stage fewer (pop1, §5.4).

## 4. Verification of the round-1 design

`make lint-dec-stream-<cfg>` is clean, and `make sim-dec-stream-<cfg>` passes, on every stream config: p8s6, p16s16,
p8s8, p12s9, p9s9, p16s8, p12s10, p4s4 and p12s8. p8s6e3 is refused for the stream units. At p12s8 the simulation also
passes with each of these parameter values: FIFO_WORDS 9, 10, 12 and 16, OUT_DEPTH 6, 7 and 16, and COUNT_WIDTH 24
(`make sim-dec-stream-p12s8 DEC_FIFO_WORDS=9` and so on).

Testbench changes in this round:

- The issue check no longer reads the removed rr_t_q. t is checked at sym_out against the model.
- The watchdog message prints in_bytes_left instead of the removed in_words_left.
- The OUT_DEPTH default follows the unit's, 7.
- COUNT_WIDTH is a testbench parameter, passed to the unit.

New SVA in the unit:

- `a_out_order` checks the output's (lane, t) against a shadow count.
- `a_at_L` checks the at-L flags against the states at ST_FIN.
- `a_words_per_step` now bounds the pops by the step's reservation.

## 5. Round 2: ORFS (Nangate45, 3 ns, p12s8, N = 8)

### 5.1 Starting point: the round-1 result, reproduced

| | unit area | flops (seq. area) | worst slack | Fmax | critical path | unit power | pJ/symbol unit / unit + clock |
|---|---|---|---|---|---|---|---|
| round-1 result | 12,852 µm² | 962 (4,350) | +1.563 ns | 696 MHz | q4_x -> q5_prod (the multiply stage's 16 x 12 multiply) | 4.97 mW | 15.12 / 21.86 |
| for scale: the unit before round 1 | 16,485 | 1,427 (6,454) | +1.124 | 533 | the issue stage | 6.00 | 18.3 / 27.0 |
| `pans_dec_core` alone | 3,360 | | | 725 | | | 5.9 per transaction |

**Where the unit's 4.97 mW go.** A scratch Tcl script groups the unit's cells by register, flops by name. Every
combinational cell of the unit falls into one lump, since ABC renames them. EMA delay 1:

| Group | Cells | mW | of which internal / switching |
|---|---|---|---|
| combinational, unit + core | 6,832 | 2.644 | 1.156 / 1.296 |
| core's flops | 143 | 0.589 | 0.374 / 0.203 |
| tag pipeline (q1..q4 lane, q4_x, q5_*, q6) | 103 | 0.398 | 0.276 / 0.114 |
| x_q | 224 | 0.367 | 0.342 / 0.007 |
| counters (sym_left, in_bytes_left, words_consumed, n_escapes, out_t) | 162 | 0.347 | 0.328 / 0.007 |
| control (pointers, counts, flags, phase) | 83 | 0.260 | 0.191 / 0.063 |
| word FIFO | 128 | 0.173 | 0.161 / 0.002 |
| output FIFO | 63 | 0.106 | 0.100 / 0.002 |
| scale_q | 56 | 0.087 | 0.082 / 0.001 |
| clock tree (outside the unit; 1,404 flops in all) | 418 | 2.217 | 1.060 / 1.147 |

A flop costs ~1.5 µW of internal power whether or not it toggles. The counters' 162 flops burn 0.33 mW at near-zero
activity. x_q's 224 flops, each written one cycle in eight, burn 0.34 mW. Each flop also costs ~1.6 µW of clock tree.
That is ~3.3 µW ≈ 0.010 pJ/symbol per flop at 0.987 symbols per cycle. The unit's 962 flops are therefore ~3.2 mW of the
7.2 mW "unit + clock", 44%. The combinational 2.64 mW, 37%, is the core's logic, the multiplier and the muxes, switching
on real data every cycle. The lever for energy is the flop count. The lever for the combinational part is activity,
which at one symbol per cycle is the work itself.

### 5.2 Candidates

| Variant | unit area | flops | slack / Fmax | unit mW | pJ/sym unit | sym/cycle | Verdict |
|---|---|---|---|---|---|---|---|
| FIFO_WORDS 9 (`n8_b4_f9_o7`) | 13,060 (+208) | 981 | +1.514 / 673 | 5.00 | 15.09 | 0.995 | option: +1.6% area and -3% Fmax for +0.8% throughput at equal energy per symbol |
| mux: all 8:1 reads as AND-OR over a one-hot decoded from the index (issue: slot, scale, present, busy, second_word; stage 4: x; the add-and-renormalize stage: the two FIFO head words; the output entry) | 12,172 (-680) | 953 | +1.421 / 633 | 4.65 | 14.16 | 0.987 | not kept as is: -5.3% area and -6.3% energy, but the issue read's decode and AND-OR into the core's stage 1 became the critical path (rr_lane -> u_core.q1_rm). See mux2 |
| C: the add s - c folded into the multiply stage (multiply-add), the refills alone in the add-and-renormalize stage (-13 flops: q5_x1 for q5_prod + q5_d) | 12,769 (-83) | 949 | +1.313 / 593 | 4.97 | 15.12 | 0.987 | not kept: equal energy, -0.6% area, Fmax -15%. The reported critical path is the issue stage into the core's stage 1, which C does not touch (per-stage timing below) |
| mux2: AND-OR for the reads off the issue path only (the stage-4 x read, the FIFO head words, the output entry). The issue reads stay a mux tree | 12,240 (-612) | 956 | +1.416 / 631 | 4.72 | 14.36 | 0.987 | kept: -4.8% area, -5.0% energy. The stages it touches keep their slack. The Fmax figure reflects the issue stage's placement variance (next table) |
| C + mux2 | 12,328 (-524) | 943 | +1.340 / 602 | 4.72 | 14.35 | 0.987 | not kept: against mux2 alone +88 µm², inside the flow's ±100 µm² area noise, at equal energy and -13 flops. The multiply stage's slack falls from 1.57 to 1.44 ns. The split into a multiply stage and an add-and-renormalize stage stays, and C is recorded as neutral in cells |

**Per-stage worst slack** in ns at a 3 ns period, from scratch STA on each routed result (`report_checks -to` the
stage's registers):

| Stage (path into) | base | mux | C | mux2 |
|---|---|---|---|---|
| issue -> core stage 1 (q1_*) | 1.60 | 1.42 | 1.31 | 1.42 |
| core stage 2 / 3 / 4 | 1.60 / 1.67 / 1.90 | 1.56 / 1.62 / 1.90 | 1.57 / 1.76 / 1.89 | 1.56 / 1.62 / 1.77 |
| stage 4: q4_x (the state read) | 2.21 | 2.50 | 2.33 | 2.51 |
| the multiply stage (q5_*) | 1.56 | 1.47 | 1.51 | 1.57 |
| the add-and-renormalize stage: x_q / at_L_q / word FIFO | 1.68 / 1.60 / 1.65 | 1.71 / 1.49 / 1.66 | 1.95 / 1.87 / 1.74 | 1.69 / 1.51 / 1.68 |

The issue stage's slack moves between 1.31 and 1.60 ns across designs whose issue RTL is identical: base, C and mux2.
The high-fanout `rr_lane` path's placement varies with the rest of the netlist. That variance alone produced every "Fmax
drop" in the candidates table, because ORFS reports the single worst path. So a change is judged on the stages it
touches. The unit's Fmax is set by the multiply stage and the core's stages 1-2, at ~1.45-1.6 ns of slack (≈ 690-700
MHz), with the issue path's ±0.15 ns on top.

In cells the multiply-add (C) costs the multiply stage nothing measurable, 1.51 against 1.56 ns. It gives the
add-and-renormalize stage 0.2-0.3 ns, where the generic flow had put the MAC 10 levels deeper. So C was combined with
mux2 and measured again (the C + mux2 row): it is neutral in cells and not kept. The full AND-OR variant's issue path
(1.42 ns, starting at rr_lane's decode) cannot be told apart from the variance. mux2 keeps the mux tree there. The
remaining -68 µm² / -0.2 pJ of the full AND-OR are an option.

### 5.3 FIFO_WORDS on the mux2 design, for the default

| FIFO_WORDS | unit area | flops | slack / Fmax | unit mW | pJ/sym unit | sym/cycle (delay 1) |
|---|---|---|---|---|---|---|
| 8 (default) | 12,240 | 956 | +1.416 / 631 | 4.72 | 14.36 | 0.987 |
| 9 | 12,803 (+563) | 974 | +1.463 / 651 | 4.84 | 14.60 | 0.995 |
| 10 | 12,410 (+170) | 990 | +1.497 / 665 | 4.62 | 13.93 | 0.995 |

9 and 10 remove the FIFO bubbles, 0.987 -> 0.995 at EMA delay 1 and 1.000 steady in the testbench, for +170 to +563 µm².
The energy per symbol moves by -3% to +2%, which is the flow's noise. F9 and F10 are the same design but for one FIFO
entry, and they differ by 5% in unit power (4.84 against 4.62 mW). So power differences below ±3% do not decide
anything. The AND-OR change lies outside that band: the two mux-tree samples sit at 4.97 and 5.00 mW, the five AND-OR
samples at 4.62-4.84. The default stays 8, the smallest. pop1 (§5.4) removes the bubbles in another way.

### 5.4 Options that trade Fmax

| Variant | unit area | flops | slack / Fmax | unit mW | pJ/sym unit | sym/cycle delay 0 / 1 | Note |
|---|---|---|---|---|---|---|---|
| mux2 (the kept design) | 12,240 | 956 | +1.416 / 631 | 4.72 | 14.36 | 0.797 / 0.987 | |
| pop1: one pop stage (multiply-add, refills and the writes in stage 5; POP_LOOP 5; on C + mux2) | 12,754 (+514) | 900 (-56) | +0.752 / 445 | 4.91 | 14.80 | 0.885 / 0.995 | the pop stage is the critical path (q4_x -> at_L_q, 2.25 ns). Timing repair upsizes it beyond the 56 flops saved. It buys the loop: issue -> x written in 5 cycles. At N = 8 the consumer's scale may then arrive two cycles after the symbol instead of one: 0.797 -> 0.885 through the wrapper's slices, and in the testbench's throughput (b) 0.888 -> 0.9999. With four steps in flight the FIFO_WORDS = 8 reservation no longer bubbles (0.9999 in (a)). At the 3 ns target it still closes, with 0.75 ns to spare |
| no q4_x: the multiply stage reads x[q4_lane] itself (AND-OR) instead of the stage-4 register (on mux2) | 12,109 (-131) | 928 (-28) | +1.297 / 587 | 4.65 | 14.14 | 0.797 / 0.987 | the multiply stage's slack falls from 1.57 to 1.33 ns, because the read sits in front of the multiplier. The multiply stage then limits the unit, below the core. Area -1% and energy -1.5% lie inside the flow's noise. An option for a design that accepts ~10% less Fmax for 28 flops |

### 5.5 Result of round 2, and what was not taken

Kept from this round: the AND-OR reads off the issue path (mux2). The decoder unit at p12s8, N = 8, 3 ns, Nangate45,
inside its timing wrapper:

| | before round 1 | after round 1 | after round 2 |
|---|---|---|---|
| unit area / flops | 16,485 µm² / 1,427 | 12,852 / 962 | 12,240 / 956 |
| worst slack, Fmax (the issue path's placement variance ±0.15 ns) | +1.124 ns, 533 MHz | +1.563, 696 | +1.416, 631 |
| stage-limited: multiply stage / core stage 2 slack | | 1.56 / 1.60 | 1.57 / 1.56 |
| unit power, pJ/symbol (EMA delay 1) | 6.00 mW, 18.3 | 4.97, 15.12 | 4.72, 14.36 |
| unit + clock tree pJ/symbol | 27.0 | 21.86 | 20.86 |
| symbols per cycle (delay 0 / 1, through the wrapper) | 0.797 / 0.982 | 0.797 / 0.987 | 0.797 / 0.987 |
| generic gates | 9,266 | 7,515 | 7,835 (AND-OR counts more; not comparable) |

Not taken in round 2, and why:

- **Energy: gating the per-cycle registers (q4_x, q5_*, 70 flops) with valid.** Round 2 assumed that Nangate45 has no
  enable flop and that ORFS infers no clock gating. An enable is then a MUX2 per bit (+130 µm²), and the saving appears
  only in bubbles, 1-2% of cycles at full rate. Not run. Round 3 (§6) measured clock gates, G3 among them.
- **Counters** (162 flops, 0.33 mW at near-zero activity). Their cost is the clock, not the arithmetic. The levers are
  the width (COUNT_WIDTH 24: -40 flops, an option because of the ports) and clock gating.
- **at-L and second-word flags back to comparators.** -16 flops against ~+100 µm² of comparators that toggle 1/8 of the
  time. Below the flow's noise either way. Not run.
- **Prefetched slot and scale registers for the issue stage.** ~+40 flops, with bypasses from the add-and-renormalize
  stage and the header. They would take the 8:1 muxes off the core's stage-1 path and remove the issue path's ±0.15 ns
  variance. The Fmax gain is at most ~10%, at +130 µW. Outside the round's rule, since area and energy rise.
- **Carry-save split of the multiply stage** (+28 flops). In cells the multiply stage does not limit the unit: 1.44-1.57
  ns of slack beside the core's own 1.44-1.60. Nothing to gain at 3 ns.
- **Full AND-OR, the issue reads included.** -68 µm² and -0.2 pJ more than mux2, with the one-hot decode at the start of
  the issue path. Not distinguishable from the variance with one sample each. An option (its row in §5.2).
- **FIFO_WORDS 9 and 10, COUNT_WIDTH 24, pop1.** Options, with their numbers above.

Verification of the round-2 design: `make lint-dec-stream-<cfg>` is clean, and `make sim-dec-stream-<cfg>` and `make
sim-stream-rt-<cfg>`, the RTL round trip through `pans_enc_stream`, pass on every stream config: p8s6, p16s16, p8s8,
p12s9, p9s9, p16s8, p12s10, p4s4 and p12s8. Every ORFS variant above also passed the power flow's gate-level netlist
check against the model and both workloads, with no mismatch. The option variants pop1 and no q4_x passed lint and the
simulation at p12s8 before their ORFS runs. They are not in the tree.

## 6. Round 3: clock gates (ORFS, Nangate45, 3 ns, p12s8, N = 8)

§5.5 rests on the assumption that no clock gating is available. Nangate45 does have integrated clock-gating cells. yosys
does not infer them, but a behavioural gate maps to DLL_X1 + AND2_X1. The gate is an enable latched while the clock is
low and ANDed with the clock. This round used `pans_dec_stream_cg`, and the encoder used `pans_enc_stream_cg`. Both are
now the shared `pans_stream_cg` of `rtl/pans_stream_cg.sv`. OpenSTA times the gate's enable as a clock-gating check with
half a period.

A gated flop is a plain flop, with no hold mux. Unclocked, it draws neither its clock-pin internal power nor its share
of the clock tree. A clocked flop costs ~2.8 µW standing by the encoder's measurement, and ~3.3 µW by §5.1's split. The
encoder also gives the rule for enables: a gate's enable must come from registers through little logic, because the
check gets half a period. The parameter `CLOCK_GATE` (default 1; 0 restores the enable flops) has the encoder's
semantics.

The gates, added step by step, each step on top of the previous one:

- **G1.** Each lane's state with its at-L and second-word flags: 30 flops, enabled by the header step for the lane or
  the registered lane decode of stage 6. Each lane's scale: 7 flops, enabled by scale_in's transfer to the lane. Each
  output FIFO entry: 9 flops, enabled by q5_valid and the registered write pointer. n_escapes, enabled by the job start
  or a registered escape. words_consumed, enabled by the job start or q6_valid. The output's (lane, t), enabled by the
  job start or a symbol out. in_bytes_left, enabled by reset, the job start or a beat. That is 8 + 8 + 7 + 4 = 27 gates.
  Not gated: sym_left and rr_lane, whose enable is the issue condition, the longest control path. The FIFO, credit and
  reservation counts, which change most cycles with enables from the issue. The flags and small control.
- **G2.** The word FIFO entries, 16 flops each, 8 more gates. The enable is the entry's write select through in_wr_n:
  the beat's transfer, the byte-present bits and the write pointer. That is a few gates from registers and from the
  data_in slice's valid.
- **G2b.** G2 plus two flags kept beside the counters: in_more_q = in_bytes_left != 0 and sym_done_q = sym_left == 0.
  data_in_ready, the issue, the drain and the word gates' enables then read a register.
- **G3.** G2b plus the per-cycle pipeline registers q4_{lane,x} and q5_* behind 2 gates, enabled by q3_valid and
  q4_valid.

| Variant | unit area | flops (+ gate latches) | reported slack / Fmax | unit mW | pJ/sym unit / unit + clock | clock tree mW | Verdict |
|---|---|---|---|---|---|---|---|
| round-2 result (mux2, enable flops) | 12,240 | 956 | +1.416 / 631 | 4.72 | 14.36 / 20.86 | 2.17 | reference |
| G1 (27 gates) | 11,959 (-2.3%) | 953 + 27 | +0.807 (a gating check, below) / 616 | 4.26 | 12.95 / 18.45 (-9.8% / -11.6%) | 1.81 | kept (below) |
| G2 (35 gates: + the word FIFO entries) | 11,525 (-5.8%) | 953 + 35 | +0.574 (a gating check) / 581 | 3.95 | 12.01 / 16.99 (-16.4% / -18.6%) | 1.64 | reworked as G2b. The word gates' enable path (sym_left == 0 and in_bytes_left != 0 through in_beat and in_wr_n) is 0.93 ns. Its half-period check would cap the period at ~1.86 ns, below the register paths (1.35-1.40 ns of slack here). The add-and-renormalize stage's FIFO-count path also lost 0.37 ns, as in_wr_n's fanout grew by 8 gates |
| G2b | 11,605 (-5.2%) | 955 + 35 | +0.752 / 649 | 3.99 | 12.13 / 17.26 (-15.5% / -17.3%) | 1.68 | kept. Register stages: 1.49 (issue), 1.51 (the multiply stage), 1.58-1.91 (core), 1.73 (the add-and-renormalize stage). The word gates' check has +0.75 ns, from 0.75 ns of enable logic. That is a 1.50 ns period limit, equal to the register paths' own 1.49-1.54. The two flags also took the 32-bit tests off the issue condition |
| G3 | 11,640 (+35) | 955 + 37 | +0.776 / 624 | 4.02 | 12.23 / 17.16. At EMA delay 0 (20% bubbles) 13.70 / 19.57, against G2b's 13.59 / 19.79 | 1.62 | not the default: a wash at both workloads, inside the ±3% noise, as the encoder's R5. An option for a host that idles the unit. The encoder measured -16% / -29% at 50% / 80% idle with the same kind of gates |

**G1's timing.** The register stages keep their slack: issue -> core stage 1 1.38 ns, the core 1.55 / 1.70 / 1.86, the
multiply stage 1.47, the add-and-renormalize stage 1.65. So Fmax is equal. ORFS's "worst slack" of +0.807 is a
clock-gating check. A gate's enable must settle before the clock's falling edge, so it has half a period less the
latch's setup. The worst enable is the lane gate's header-step term (in_bytes_left != 0 -> in_hdr_step -> we), 0.69 ns,
with +0.81 ns of slack at 3 ns. The scale gate follows (scale_in's lane through the two flag reads, +1.11), then the
output entries (+1.21). A half-period check with 0.69 ns of logic limits the period at 1.39 ns. The register paths have
1.4-1.6 ns of slack. So the gates do not cap the unit's clock. If they ever did, the header-step term could come from a
registered header write one-hot.

**G1's power** (fine split, EMA delay 1; the gated groups now sit under one name):

| Group | before G1 | G1 |
|---|---|---|
| the lane states, scales, output entries and flags | 0.82 mW | 0.44 |
| the counters (in_bytes_left, n_escapes, out_t gated; sym_left not) | 0.35 | 0.23 |
| the combinational cells (the hold muxes are gone) | 2.64 | 2.45 |
| the clock tree | 2.22 | 1.81 |
| the word FIFO (G2's target) | 0.17 | 0.17 |
| the core's flops | 0.58 | 0.58 |
| the per-cycle tag pipeline | 0.39 | 0.39 |
| unit | 4.72 | 4.26 |

### 6.1 Result of round 3

Kept: G1 and G2b, with `CLOCK_GATE` default 1. The decoder unit at p12s8, N = 8, 3 ns, Nangate45, inside its timing
wrapper, through the three rounds:

| | before round 1 | round 1 | round 2 | round 3 |
|---|---|---|---|---|
| unit area | 16,485 µm² | 12,852 | 12,240 | 11,605 (-30% in all) |
| flops (+ gate latches) | 1,427 | 962 | 956 | 955 + 35 |
| register stages' slack: issue / multiply stage / core 2 | - / - / - | 1.60 / 1.56 / 1.60 | 1.42 / 1.57 / 1.56 | 1.49 / 1.51 / 1.58 |
| reported worst slack, Fmax | +1.124, 533 MHz | +1.563, 696 | +1.416, 631 | +0.752 (a gating check), 649 |
| unit power, pJ/symbol (EMA delay 1, 0.987 sym/cycle) | 6.00 mW, 18.3 | 4.97, 15.12 | 4.72, 14.36 | 3.99, 12.13 (-34% in all) |
| unit + clock tree pJ/symbol | 27.0 | 21.86 | 20.86 | 17.26 |
| clock tree | 2.83 mW | 2.22 | 2.17 | 1.68 |
| netlist check / workloads | 0 mismatches | 0 | 0 | 0 |

Not gated, and why:

- **sym_left and rr_lane** (35 flops). Their enable is the issue condition, the longest control path. A registered copy
  (q1_valid) would lag the count by a cycle and let one symbol too many issue, unless the compares moved too.
- **The FIFO, credit and reservation counts and pointers, scale_present/more, the error flags, hdr_beat, phase** (~70
  flops). They are written most cycles at full rate, with enables from the issue or q5_valid. Or they have too few flops
  per distinct enable for a gate to pay: a latch and an AND are ~5 µm² and carry their own clock pin.
- **The core's 143 flops.** They belong to the core, not to this unit.
- **The per-cycle pipeline registers** (q4_x, q5_*: 70 flops) behind their stage valids (G3). Measured above as an
  option for a consumer that leaves bubbles. At the power workload's 1-20% bubbles it is a wash, as the encoder's R5. So
  it is not the default.

Verification of the round-3 design: `make lint-dec-stream-<cfg>` is clean, and `make sim-dec-stream-<cfg>` and `make
sim-stream-rt-<cfg>` pass on every stream config: p8s6, p16s16, p8s8, p12s9, p9s9, p16s8, p12s10, p4s4 and p12s8. At
p12s8, `DEC_CLOCK_GATE=0` passes for G1 and G2b. The ORFS power flow's netlist check and both workloads show no mismatch
for G1, G2 and G2b.
