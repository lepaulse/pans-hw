# pans_enc_stream: PPA ledger

This ledger records every optimization candidate measured on the encoder stream unit `pans_enc_stream`, with its numbers
and its verdict. The unit is `rtl/pans_enc_stream.sv` with `rtl/pans_enc_stream_functions.svh`. It is bit-exact with
`ref/pans_stream_ref.h` (`make sim-enc-stream-<cfg>`). The measurements ran in two rounds, at contract version 1.
STREAM.md §4.2 gives the numbers of the current design, and STREAM.md §4.5 summarizes this ledger.

| Round | Date | Flow | Sections |
|---|---|---|---|
| 1 | 2026-10-15 | generic gates | §1-§3 |
| 2 | 2026-10-15 | ORFS, Nangate45, 3 ns | §4-§5 |

**Scope of the rounds.** The ports and their semantics, the stream format (STREAM.md §1) and the core `pans_enc_core`
stay fixed. Everything inside the unit may change, and so may the parameter defaults. A changed default is documented.

**How to read a row.** A row or a lettered section names a change, its result measured against the design as it stood,
and a verdict. "Kept" means the design includes the change. "Dropped" means it does not, for the stated reason. "Option"
means a parameter value or a variant that trades one metric for another. Its numbers are recorded, and the default does
not use it. The kept changes are cumulative. Each section's base is the design with every kept change before it, as its
heading says ("after B").

## Methods

**Generic gates (round 1).** `ppa/synth_generic.sh` runs yosys and ABC with the cells AND, NAND, OR, NOR, XOR, XNOR and
MUX, and counts every flop as one cell. Depth is the number of gates on the longest register-to-register path (`ltp
-noff`). The numbers are at p12s8 (P = 12, W = 16, STATE_WIDTH = 28), N_LANES = 8 and OUT_DEPTH = 8 unless stated.

Generic gates mislead in three ways. A flop is 4-5 gate equivalents of standard-cell area: a Nangate45 DFF_X1 is 4.5
NAND2. So the tables list flops beside gates. A wide mux maps to AOI or mux cells at less than its MUX2 count. A ripple
chain's depth is what ABC's area mapping leaves, and timing-driven mapping (ORFS at 3 ns) shortens chains at some area
cost. ORFS at 3 ns on Nangate45 gives the final numbers (§4).

**ORFS (round 2).** §4 describes the flow.

## 1. Baseline (the unit before the rounds, 2026-10-15)

`make ppa-enc-stream-p12s8`:

| DIV_BITS | gates | flops | logic | depth | loop (cycles) | throughput at N = 8 |
|---|---|---|---|---|---|---|
| 1 | 5,571 | 1,506 | 4,065 | 42 | 20 | 0.40 |
| 2 | 6,794 | 1,058 | 5,736 | 42 | 12 | 0.67 |
| 4 | 12,208 | 834 | 11,374 | 48 | 8 | 1.00 |
| core alone | 775 | 58 | 717 | 42 | | |

Area map at DIV_BITS = 2, from scratch wrappers around the unit's own functions and blocks in the same flow. The sum,
6,768, matches the unit within 0.4%.

| Block | gates | flops | depth | notes |
|---|---|---|---|---|
| pans_enc_core | 775 | 58 | 42 | the longest path of the unit (stage 2 of the core) |
| renorm stage, comb (push_renorm x3, escape path, muxes) | 314 | 0 | 20 | push_renorm alone 100 / depth 16 |
| divider: 8 stages x div_stage(DIV_BITS 2) | 3,072 | 0 | 31 per stage | one stage 384; DIV_BITS 1: 99 / 25; DIV_BITS 4: 2,110 / 41 |
| divider pipeline registers: 9 x {lane 3, div 28, f 12, c 12} + 9 valid | 0 | 504 | | carried through every stage |
| assemble | 56 | 0 | 22 | the P-bit add c + r |
| lanes: 8 x 28 state flops with init + write, 2 read muxes (renorm, done_out), busy | 1,075 | 232 | 6 | 843 logic gates: 2 x 196 read mux, 224 init gates (sync set to L = M), decode |
| output FIFO (OUT_DEPTH 8, two words in, one out) | 578 | 141 | 9 | OUT_DEPTH 6: 391 / 108; 4: 259 / 73 |
| job counters (job_rem, cnt_words, cnt_escapes, exp_lane, done_lane, err_order), COUNT_WIDTH 32 | 403 | 104 | 35 | COUNT_WIDTH 24: 296 / 80 |
| first_lane (N = 8: the low bits minus 1) | 4 | 0 | 2 | N = 3: 196 / 19 |
| tag pipeline q1/q2 {lane, symbol}, valids, control | ~90 | ~30 | | |

At DIV_BITS 2 the divider, 3,072 gates + 504 flops, is 53% of the gates. The lane array is the next block (1,075), then
the core (775), the FIFO (578) and the counters (403).

Divider step costs, measured in isolation (one combinational stage of cascaded restoring steps, p12s8):

| steps per stage | radix 2 (DIV_STEP_BITS 1): gates / depth | radix 4: gates / depth | radix 16 |
|---|---|---|---|
| 1 | 99 / 25 | 384 / 31 | 2,110 / 41 |
| 2 | 181 / 51 | 733 / 59 | |
| 3 | 265 / 76 | | |
| 4 | 360 / 101 | 1,348 / 113 | |
| 8 | 729 / 197 | | |
| 16 | 1,509 / 400 | | |

Per quotient bit, radix 2 costs 90-99 gates, radix 4 168-192 and radix 16 527. The radix-4 stage's only advantage is
depth per quotient bit, 15.5 against 25 levels. It needs three subtractors and a 3f adder for two bits, where two
cascaded radix-2 steps use two subtractors. Non-restoring radix 2 (signed remainder, add or subtract by the sign) costs
115 / 196 / 280 / 363 gates for 1-4 steps, at depth 29 / 52 / 77 / 102. It also needs a final correction of 82 gates and
a 13-bit remainder. It loses against restoring division at every step count: the saved restore mux is paid back by the
conditional negation.

## 2. Candidates

Each section gives the change, its measured difference against the design as it stood, and the verdict with its reason.
Full unit numbers are at p12s8, N = 8, OUT_DEPTH 8 unless stated.

### A. Divider radix as its own parameter: DIV_STEP_BITS, cascaded radix-2 steps per stage (kept)

Before: one restoring step of DIV_BITS quotient bits per stage, radix 2^DIV_BITS. After: a stage cascades DIV_BITS /
DIV_STEP_BITS steps of radix 2^DIV_STEP_BITS (`div_step`, `div_stage`). DIV_BITS no longer has to divide W. There are
ceil(W / DIV_BITS) stages, and the last one takes the rest. The baseline points reproduce exactly, since the function is
the same:

| DIV_BITS / DIV_STEP_BITS | gates | flops | depth | stages | loop | note |
|---|---|---|---|---|---|---|
| 1 / 1 | 5,571 | 1,506 | 42 | 16 | 20 | = baseline DIV_BITS 1 |
| 2 / 1 | 5,152 | 1,058 | 52 | 8 | 12 | baseline DIV_BITS 2 was 6,794: -1,642 gates (-24%) at equal flops and loop |
| 3 / 1 | 5,021 | 946 | 77 | 6 | 10 | stages 3,3,3,3,3,1 |
| 4 / 1 | 4,902 | 834 | 101 | 4 | 8 | baseline DIV_BITS 4 was 12,208 |
| 8 / 1 | 4,782 | 722 | 201 | 2 | 6 | |
| 16 / 1 | 4,742 | 666 | 388 | 1 | 5 | a combinational divider |
| 2 / 2 | 6,794 | 1,058 | 42 | 8 | 12 | = baseline DIV_BITS 2 |
| 4 / 2 | 6,430 | 834 | 60 | 4 | 8 | two radix-4 steps per stage |
| 8 / 2 | 6,060 | 722 | 115 | 2 | 6 | |
| 4 / 4 | 12,208 | 834 | 48 | 4 | 8 | = baseline DIV_BITS 4 |

Radix 2 wins at every stage count. The whole divider's logic is then ~1,500 gates whatever the stage count. The stage
count buys only flops (56 per stage) and loop cycles, against depth (25 levels per quotient bit). The default stays
DIV_BITS = 2 with DIV_STEP_BITS = 1. Its depth is 52, so the divider becomes the longest path, 10 levels above the
core's 42. The 3 ns ORFS run (§4) shows whether DIV_BITS 3 (depth 77, 6 stages) or 4 (depth 101) still closes, since
timing-driven mapping shortens these ripple chains.

### B. Assembly merged into the last divider stage (kept, fused form)

Before (after A): the last divider stage registers {lane, div, f, c}. A separate assemble stage forms x = {q, c + r} and
writes the lane. The loop is 4 + DIV_STAGES. After: the last divider stage writes the lane itself. Two forms were
measured (gates / flops / depth):

| form | DIV_BITS 1 | 2 | 3 | 4 | 4 / radix 4 |
|---|---|---|---|---|---|
| after A (separate assemble) | 5,571 / 1,506 fl / d42 | 5,152 / 1,058 / d52 | 5,021 / 946 / d77 | 4,902 / 834 / d101 | 6,430 / 834 / d60 |
| plain merge: c + r after the last step's restore mux | 5,633 / 1,462 / d43 | 5,155 / 1,014 / d61 | 4,997 / 902 / d77 | 5,013 / 790 / d105 | |
| fused: c + (t - m f) for every candidate m in parallel, one-hot select | 5,678 / 1,462 / d42 | 5,196 / 1,014 / d52 | 5,102 / 902 / d77 | 5,031 / 790 / d100 | 6,764 / 790 / d61 |

Kept: the fused form (`div_assemble`, `div_stage_last`). At DIV_BITS 2 it saves 44 flops, one register set. The loop
goes from 12 to 11 cycles, and the throughput at N = 8 from 0.667 to 0.728. The depth stays at 52. The plain merge put
the P-bit add in series with the step's borrow chain, which gave 61. The fused form costs +44 gates, the second P-bit
adder. Flop-weighted (4.5 per flop) it goes from 8,855 to 8,745.

At DIV_BITS 3 the last stage has one step. There the plain merge is the cheaper form (4,997 against 5,102) at equal
depth. The fused form serves the default, and this note records the alternative instead of a parameter. At radix 4 the
fused step needs four adders (+334 gates), and radix 4 loses anyway (A). The check `a_assemble_fused` compares the fused
result with the unfused step and assembly on every write.

### C. Early lane release: busy cleared when the write is certain to precede the next read (kept)

Before (after B): busy[lane] runs from the accept to the write. So a lane's two pushes are 3 + DIV_STAGES cycles apart,
although the second push reads the state only at its renorm stage, two cycles after its accept. After: busy is cleared
when the push is in slot DIV_STAGES - 1. The pipeline is in order and fixed-latency, so the write lands the cycle before
the lane's next push reads. The issue distance per lane becomes DIV_STAGES + 1 cycles, exposed as `ISSUE_DISTANCE`. The
testbench reads it for its throughput bound and its busy-coverage check. pipe_empty is the OR of the valid bits instead
of busy == 0. `a_lane_in_order` checks on every renorm read that no older push of the lane is still in the divider.

| DIV_BITS | gates (before -> after) | flops | depth | issue distance (before -> after) | throughput at N = 8 (measured) |
|---|---|---|---|---|---|
| 2 | 5,196 -> 5,241 | 1,014 | 52 | 11 -> 9 | 0.728 -> 0.889 |
| 3 | 5,102 -> 5,096 | 902 | 77 | 9 -> 7 | 0.889 -> 1.000 |

At DIV_BITS 2 the change costs +45 gates, within ABC's run-to-run restructuring. The decode moved from the write stage
to the release slot, and the empty test from 8 busy bits to 10 valid bits. The gain is +22% throughput at N = 8. At
DIV_BITS 3, N = 8 saturates.

Not taken further: a bypass of the write into the renorm read, forwarding sa_x when sa_lane == q2_lane. It would cut the
distance to DIV_STAGES, at the cost of a 28-bit 2:1 mux and a compare in the renorm stage's path: ~35 gates and +2
levels on the unit's second-longest path. It pays only where N_LANES = DIV_STAGES exactly (N = 8, DIV_BITS 2). The
stage-count parameter covers that case, since DIV_BITS 3 saturates N = 8 already.

### I. Escape bypass: the escape's finished state written through a second lane write port (dropped)

Before: the escape's finished state x2 rides the divider as a division by 1. The renorm stage muxes dividend, f and c by
core_escape: 28 + 12 + 12 = 52 MUX2, with no other cost. The escape takes the full latency, but no throughput depends on
it, since the lane's issue distance is the same either way. The alternative was measured as a block: the lane array with
a second write port (per lane a 28-bit 2:1 data mux and a second enable decode, the busy clear from either port). It
costs 1,759 gates against 1,075 for the one-port array (+684), to remove 52 muxes. Dropped on the block numbers. It
would also need a second release path and an ordering rule between the two write ports.

### J. A multi-cycle divider shared across lanes instead of the pipeline (dropped by arithmetic)

An iterative divider of one stage's logic (181 gates at 2 radix-2 steps), with its 56 data flops and a 56-bit load mux,
serves one push every DIV_STAGES cycles. K of them give K / DIV_STAGES symbols per cycle. The pipeline gives min(1,
N_LANES / (DIV_STAGES + 1)) with DIV_STAGES copies of the stage logic and DIV_STAGES register sets. At N = 8 and
DIV_STAGES = 8 (p12s8, DIV_BITS 2) the pipeline's 0.889 symbols/cycle need 8 iterative units: 8 x (181 + 56 + 56) =
2,344, against the pipeline's 8 x (181 + 56) = 1,896. That is before the scheduler, the out-of-order lane writes and the
busy bookkeeping. A pipeline is the unrolled iteration without load muxes. At every N and stage count it is the smaller
of the two for the same throughput, so no variant was built.

### K. f or c per lane instead of carried through the divider (c: measured in G below; f: by arithmetic)

Every divider stage reads f. A per-lane store needs N x P flops and a P-bit N:1 read mux per stage: N P + DIV_STAGES x P
(N - 1) MUX2, that is 96 flops + 672 muxes at N = 8, P = 12 and 8 stages. Carrying f costs 8 x 12 = 96 flops. The store
never wins. c is read once, at the last stage (see G).

### D. Lane states: fresh bits instead of a synchronous load of L, one read port shared with done_out (kept)

Before (after C): a job loads L = M into all N x 28 state flops. That is a sync set with enable. In generic cells it
hides inside `$_SDFFCE_` flops. In Nangate45 it is an AND/OR gate per bit, 224 at N = 8, which the gate count never
showed. The array has two read muxes, 28-bit N:1 each: the renorm stage's by q2_lane and done_out's by done_lane.

After: a fresh bit per lane, set by the job and cleared by the lane's first write. The read port returns L for a fresh
lane. done_out reads through the renorm stage's port. The pipeline is empty in ST_DONE, so the port's lane is done_lane
then. The read path gains a 3-bit 2:1 mux ahead of the N:1 mux and a 28-bit 2:1 mux behind it. That adds 2 levels to the
renorm stage, 24 -> 26, which is not the longest path.

| DIV_BITS | gates (before -> after) | flops | depth | N = 16: gates / flops |
|---|---|---|---|---|
| 2 | 5,241 -> 5,060 | 1,014 -> 1,019 | 52 | 5,852 / 1,271 |
| 3 | 5,096 -> 4,909 | 902 -> 907 | 76 | |

The change saves 181 generic gates, the second read mux, plus the 224 load gates that only the standard-cell mapping
would have shown. It adds N fresh flops. The 224 lane flops are now plain enable flops (`$_DFFE_`). In Nangate45 each is
a DFF and a hold mux, as before, minus the set gating.

### F. Output FIFO in two banks, even and odd entries, with one write port each (kept)

Before (after D): one array of OUT_DEPTH words with two writes per cycle. Every entry has its own word_a / word_b data
mux (16 MUX2 per entry) behind its enable. After: the two words of a cycle go to consecutive entries. With an even
OUT_DEPTH they always fall into different banks. Each bank has one write port, whose data mux (word_a or word_b, by the
write pointer's parity) all its entries share. The read side selects the bank by the read pointer's parity. The
pointers, the count, the credit and the `fifo_count` that the testbench probes are unchanged. OUT_DEPTH must be even,
which an elaboration check enforces. The stated minimum stays 2.

| | blocks alone (OUT_DEPTH 8 / 6) | unit, DIV_BITS 2 (gates / flops / depth) | unit, DIV_BITS 3 |
|---|---|---|---|
| before | 578 / 391 | 5,060 / 1,019 / 52 | 4,909 / 907 / 76 |
| after | 361 / 283 | 4,845 / 1,018 / 52 | 4,667 / 906 / 77 |

The change saves 215 gates at the default.

OUT_DEPTH itself: 8 -> 6 gives 4,766 gates / 985 flops (-79 gates, -33 flops) at a measured throughput of 0.858 instead
of 0.889 symbols/cycle. The credit stalls while two symbols are between the accept and the renorm stage. 4 gives 4,673 /
950 at a measured 0.623. The default stays 8. OUT_DEPTH = 6 is the documented area option. The testbench's two-word
coverage items ("the FIFO at OUT_DEPTH - 1 or more", "full") still occur at OUT_DEPTH 6 and 8. The testbench waives its
busy-coverage expectation below OUT_DEPTH 6, where the credit alone paces the lanes. That is a testbench limitation that
predates this change: at OUT_DEPTH 2 the baseline failed it too.

### E. COUNT_WIDTH as a parameter (default 32, 24 shown); n_words counted at the FIFO output (kept)

COUNT_WIDTH was a localparam (32). It is now a parameter with the default unchanged, so the ports keep their widths. The
testbench takes it too. The word count moves from the renorm stage, +0..2 per cycle through a registered copy of
s3_push, to the FIFO's pop, +1. Every word has left the FIFO before done_out, so the count is the same. The 2-bit addend
and its two flops go (block alone: 103 -> 97 gates).

| | DIV_BITS 2: gates / flops | DIV_BITS 3 |
|---|---|---|
| before (F) | 4,845 / 1,018 | 4,667 / 906 |
| after, COUNT_WIDTH 32 | 4,816 / 1,016 | 4,670 / 904 |
| after, COUNT_WIDTH 24 | 4,735 / 992 | |

COUNT_WIDTH 24 saves 81 gates and 24 flops: job_rem, cnt_words and cnt_escapes shrink by 8 bits each, with their
incrementers. It requires jobs below 2^24 symbols. The testbench's first-lane test covers the count range of whatever
width is built, and passes at 24. The default stays 32. A generic unit cannot assume a bound on n, and the port widths
are set by the integration.

### G. c stored per lane instead of carried through the divider (dropped)

c is used once, at the last stage. Measured as blocks (p12s8, P = 12): the carried c is DIV_STAGES x 12 plain flops and
nothing else, 96 cells at 8 stages and 192 at 16. The per-lane store is N x 12 enable flops, written at the renorm stage
by q2_lane and read at the last stage by sa_lane. It costs 271 cells at N = 8 (96 flops + 175 gates of write decode and
read mux) and 575 at N = 16. At the default (8 stages, N = 8) the store costs 271 against 96. Flop-weighted, with 4.5
per flop and the enable flop a DFF plus a hold mux, that is 607 against 432. Only at 16 stages (DIV_BITS 1) with N = 8
does the store win in area, 607 against 864. DIV_BITS 1 is itself the weakest point of A. Dropped. The same arithmetic
rules out storing f or the lane index (K).

### H. DIV_FIRST_BITS: the renorm stage takes the first quotient bits (kept, default 1)

The renorm stage's logic is 22 levels deep: the read mux, the fresh select, the P-bit compare, the escape path and the
output muxes, 741 gates as a block. A radix-2 step is 25 levels. So one step fits beside the renorm within a 2-step
stage's 52. The parameter DIV_FIRST_BITS, a multiple of DIV_STEP_BITS below W, takes that many quotient bits in the
renorm stage. The divider stages share the remaining W - DIV_FIRST_BITS bits.

| DIV_BITS / DIV_FIRST_BITS | gates | flops | depth | divider stages | issue distance | throughput at N = 8 |
|---|---|---|---|---|---|---|
| 2 / 0 (E) | 4,816 | 1,016 | 52 | 8 (2,2,2,2,2,2,2,2) | 9 | 0.889 |
| 2 / 1 | 4,760 | 1,016 | 52 | 8 (2,2,2,2,2,2,2,1) | 9 | 0.889 |
| 2 / 2 | 4,712 | 960 | 69 | 7 | 8 | 1.000 |
| 3 / 0 | 4,670 | 904 | 76 | 6 (3,3,3,3,3,1) | 7 | 1.000 |
| 3 / 1 | 4,602 | 848 | 77 | 5 (3,3,3,3,3) | 6 | 1.000 |
| 4 / 0 | 4,635 | 792 | 99 | 4 | 5 | 1.000 |
| 4 / 4 | 4,515 | 736 | 118 | 3 | 4 | 1.000 |
| 5 / 1 | 4,611 | 736 | 123 | 3 (5,5,5) | 4 | 1.000 |

The default is DIV_FIRST_BITS = 1 with DIV_BITS = 2: -56 gates at equal flops and depth. The last stage then has one
step and the fused assembly, the cheaper and balanced form noted in B. The renorm stage goes from 26 to ~48 levels, the
second-longest path after the 52 of a 2-step stage. Where the clock allows 3-step stages, DIV_BITS 3 with DIV_FIRST_BITS
1 is the design: 5 stages, 4,602 gates, 848 flops, N = 8 saturated.

### L. 3f carried through the pipeline instead of formed per radix-4 stage (dropped)

L matters only at DIV_STEP_BITS 2. A radix-4 step with 3f supplied costs 352 gates at depth 29, against 384 / 31 with
its own 3f adder (block, p12s8). That saves 32 gates per stage, for a 14-bit register per stage. At 8 stages it is -256
gates and +112 flops, about +250 gate equivalents net. Dropped, and radix 4 is itself the loser of A.

The default is written DIV_FIRST_BITS = DIV_STEP_BITS, one step of whatever radix. So a radix-4 or radix-16 build needs
no second parameter, and the elaboration check requires a multiple of DIV_STEP_BITS. With the earlier fixed default of
1, the DIV_STEP_BITS 2 and 4 variants were refused at elaboration, as the check intends.

## 3. Result of round 1

The round-1 design has the defaults N_LANES 8, DIV_BITS 2, DIV_STEP_BITS 1, DIV_FIRST_BITS 1, OUT_DEPTH 8 and
COUNT_WIDTH 32. The numbers come from `make ppa-enc-stream-p12s8` and the scratch sweep, at p12s8, N = 8, OUT_DEPTH 8:

| DIV_BITS | baseline gates / flops / depth / throughput | final gates / flops / depth / throughput | divider stages (final) |
|---|---|---|---|
| 1 | 5,571 / 1,506 / 42 / 0.40 | 5,227 / 1,408 / 47 / 0.50 | 15 |
| 2 (default) | 6,794 / 1,058 / 42 / 0.67 | 4,760 / 1,016 / 52 / 0.89 | 8 |
| 3 | - | 4,602 / 848 / 77 / 1.00 | 5 |
| 4 | 12,208 / 834 / 48 / 1.00 | 4,570 / 792 / 102 / 1.00 | 4 (4,4,4,3); with DIV_FIRST_BITS 4: 4,515 / 736 / 118, 3 stages |
| core alone | 775 / 58 / 42 | unchanged | |

At the default the round saves 30% of the gates and 4% of the flops, and gains 33% throughput. Flop-weighted (4.5 per
flop) the unit goes from 11,555 to 8,316 (-28%), and the baseline's count does not even include its 224 lane-load gates.
The unit minus the core is 3,985 gates / 958 flops. The longest path is the divider's 2-step stage (52), then the renorm
stage with its first step (~48), then the core (42).

Other configs and N, on the final design with DIV_BITS 2:

| Config, N | gates / flops / depth | note |
|---|---|---|
| p16s16 | 6,232 / 1,185 / 67 | W = P = 16: a wider remainder |
| p8s8 | 2,705 / 544 / 42 | W = 8: 4 divider stages, distance 5 |
| p4s4 | 1,950 / 441 / 32 | |
| p12s8, N = 4 | 4,230 / 884 | |
| p12s8, N = 16 | 5,616 / 1,268 | each lane is 28 state flops plus its share of muxes and decode, ~175 gate equivalents |

Where the stage count goes for a given clock follows from the table of H. At 3 ns on Nangate45 the cores' stage of 42
levels ran at 1.75 ns (SPEC.md §7, p16s16), so ~70 levels fit. DIV_BITS 2 (52) is safe. DIV_BITS 3 (77) is the candidate
to try in ORFS first, since timing-driven mapping shortens ripple chains. It saves 158 gates and 168 flops and saturates
N = 8. Where area comes first, the choice is DIV_BITS 3 or 4 with DIV_FIRST_BITS, OUT_DEPTH 6, COUNT_WIDTH 24 if n <
2^24 is known, and N as small as the consumer's lane count allows.

Kept: A (radix-2 cascaded steps, DIV_STEP_BITS), B (assembly fused into the last stage), C (early lane release), D
(fresh bits, one read port), E (COUNT_WIDTH parameter, words counted at the FIFO output), F (banked FIFO), H
(DIV_FIRST_BITS). Dropped with numbers: non-restoring steps (§1), G (c per lane), I (escape bypass), J (shared
multi-cycle divider), K (f per lane), L (3f carried), and the plain (unfused) merge of B at radix 2. Documented options,
not defaults: OUT_DEPTH 6 and 4, COUNT_WIDTH 24.

Not tried in round 1, and why:

- a write-to-read bypass in the renorm stage (C). It saves one cycle of distance for ~35 gates on the second-longest
  path, and the stage-count parameter covers N = 8;
- the renorm stage's escape path: three push_renorm and two 28-bit muxes, ~100 gates, under 2% of the unit;
- first_lane for N not a power of two: 196 gates at N = 3 for the 32-bit mod-N tree. A serial reduction over extra setup
  cycles would change the job's latency, not its ports, and N = 8 costs 4 gates;
- carry-save or SRT division. It is exact after a final resolve, but its per-step logic is not smaller than a restoring
  radix-2 step (99 gates). Its gain is depth, which the stage parameters already trade;
- anything in the core or the stream format.

Verification of the round-1 design: `make lint-enc-stream-<cfg>` and `make sim-enc-stream-<cfg>` pass on p12s8, p16s16,
p8s6, p8s8, p12s9, p9s9, p16s8, p12s10 and p4s4, with 0 errors and every coverage item met. At p12s8 they also pass with
N_LANES 1, 3 and 16, DIV_BITS 1, 3, 4 and 16, OUT_DEPTH 6, COUNT_WIDTH 24, DIV_STEP_BITS 2 and 4 (DIV_BITS 4),
DIV_FIRST_BITS 0 (DIV_BITS 2) and 4 (DIV_BITS 4), and DIV_BITS 3 with DIV_FIRST_BITS 1. They pass at p16s16 with N_LANES
16 and at p4s4 with DIV_BITS 1. The testbench's busy-coverage expectation is waived below OUT_DEPTH 6, where the credit
paces the lanes. The baseline failed it at OUT_DEPTH 2 as well.

## 4. Round 2: ORFS (Nangate45, 3 ns, p12s8, N = 8, OUT_DEPTH 8)

The unit runs inside its timing wrapper. The numbers are for the unit's own cells unless stated.

**Flow.** The stream PPA flow, `make asic-stream-p12s8` and `power-stream-p12s8` (STREAM.md §4.1), with
`ppa/pans_stream_timed.sv`, `ppa/asic/*.tcl` and `ppa/power_tb_stream.sv`. The workload is two EMA jobs of 4096 symbols,
with producers never idle and consumers always ready. Power comes from zero-delay gate-level activity on the routed
netlist and OpenSTA with the routed parasitics. The round ran a scratch copy of the flow, because the Makefile then
passed only N_LANES, DIV_BITS and OUT_DEPTH, and the round needed to drive DIV_STEP_BITS, DIV_FIRST_BITS and
COUNT_WIDTH. The scratch flow gives the Makefile flow's numbers for the merged design to the digit. The flow's power
groups (`ppa/asic/stream_regs.tcl`) took over this round's scratch script `power_groups.tcl`. Energy = unit power x
period x cycles / symbols. "Fmax" is the post-route worst slack turned into a period.

The Makefile now passes DIV_STEP_BITS, DIV_FIRST_BITS, COUNT_WIDTH and CLOCK_GATE as `ENC_DIV_STEP_BITS`,
`ENC_DIV_FIRST_BITS`, `ENC_COUNT_WIDTH` and `ENC_CLOCK_GATE`, and the power testbench's producer-idle percentage
(+P_IDLE) as `POWER_P_IDLE`. So every option of this round reproduces without the scratch flow.

### Baseline: the round-1 design

| | area µm² | flops | slack ns / Fmax MHz | sym/cycle | unit mW | unit pJ/sym | unit + clock tree pJ/sym |
|---|---|---|---|---|---|---|---|
| DIV_BITS 2 | 10,414 | 1,016 | +1.387 / 620 | 0.885 | 4.75 | 16.09 | 22.87 |
| DIV_BITS 3 (Makefile flow) | 10,791 | 848 | +1.188 / 552 | 0.995 | 5.19 | 15.6 | |
| core alone (Makefile flow) | 1,469 | 58 | 692 MHz | | | 2.8 | |

**Where the unit's 4.75 mW go.** OpenSTA reports power per instance, grouped here by register name. The combinational
cells are flattened and nameless, so they form one group.

| Group | Flops | mW |
|---|---|---|
| divider pipeline registers | 440 | 1.52 |
| of them qd_div | 224 | 0.74 |
| of them qd_f | 96 | 0.40 |
| of them qd_c | 96 | 0.28 |
| of them qd_lane | 24 | 0.08 |
| of them the valids | | 0.02 |
| unit comb (4,675 cells: the divider's subtract/select cascades, the renorm stage, the read muxes, the core's comb) | | 2.11 |
| lane_x (enable flops) | 224 | 0.36 |
| counters | 104 | 0.24 |
| core flops | 56 | 0.19 |
| FIFO | 128 | 0.17 + 0.03 control |
| tag pipeline | | 0.08 |
| busy/fresh | | 0.04 |

The clock tree (370 buffers) draws 2.00 mW for 1,352 flops, the wrapper's 336 included. That is ~1.5 µW per flop on top
of the flop's own ~1.3 µW of clock-pin internal power. A flop costs ~2.8 µW before it toggles. A toggling pipeline flop
costs ~3.3 + 1.5 µW. The unit's 13 pJ beyond the core are therefore 440 pipeline flops and their logic (~45%), the 224
lane flops, and the clock they all draw.

### Noise floor of the flow

The flow is deterministic: the same RTL gives the Makefile flow's numbers to the digit. Small RTL changes still move
ABC's mapping and the placement. Between variants that cannot touch the critical path (COUNT_WIDTH 24, the hold muxes of
R1 below) the unit's Fmax moved over 569-662 MHz, the comb cell count over 4,675-4,847 and its power over 1.99-2.14 mW.
The working noise floor of this round is ±1.5% in area, ±2.5% in energy and ±7% in Fmax. R4c revises it to ±3% for area
and energy. A change is kept only when it clears the floor, or when it is free and structurally sound, such as a mux
removed from the longest path.

### R1. Hold the divider pipeline's entry register when the renorm slot is invalid (dropped)

The idea: a bubble then repeats the previous push through every stage. So no pipeline flop or divider gate toggles in a
bubble, for 55 hold muxes instead of 440. Measured: unit 10,355 µm², 15.54 pJ/sym at 0.885 sym/cycle (-3.4%), Fmax 569
MHz (-8%). The hold mux sits at the end of the renorm stage. ORFS's post-route path shows that stage as the unit's
longest path: st -> lane read mux -> fresh select -> compare -> first step -> qd_div[0].

R1 combined with R2 below (R1a): 10,534 µm², 16.13 pJ, 632 MHz. With DIV_FIRST_BITS 0: 10,563, 16.04, 662 MHz.

Why so little: in a bubble the baseline's entry data already repeats. The core's inputs are the wrapper slice's held
registers, and q2_lane changes only on an accept. Only a write to the lane being read changes the stale value.

Producer idle sweep, from a scratch copy of the power testbench with +P_IDLE and consumers always ready. Unit pJ/sym,
baseline -> R1a:

| producer idle | baseline | R1a |
|---|---|---|
| 0% | 16.10 | 16.13 |
| 50% | 21.41 | 20.86 (-2.5%) |
| 80% | 37.28 | 36.20 (-3%) |

Below the noise at the workload, 3% at best at low utilization, +1% area: dropped.

The idle sweep also shows the energy floor. At 80% idle the unit still draws 2.49 mW, 0.67 of it combinational and the
rest flops, and the clock tree draws 2.0 mW regardless. A clocked flop costs ~1.3 µW of clock-pin internal power plus
~1.5 µW of clock tree at 333 MHz. That standing cost is the floor: 1,016 flops ≈ 2.8 mW ≈ 9.5 pJ/sym of the 22.9 unit +
clock tree. Flop count and clock gating (R4) are the energy lever. Data gating is not.

### R2. done_out reads through q2_lane, so the read-lane mux leaves the longest path (kept, numbers below)

rd_lane was (st == ST_DONE) ? done_lane : q2_lane, in front of the 8:1 state read mux, on the longest path. Now q2_lane
itself loads done_lane's next value from the cycle the job is through, when the tag pipeline is idle. rd_lane = q2_lane.
The mux moves to a 3-bit register input. No function change (the simulations pass) and no area.

### Parameters in ORFS (R1a RTL, that is R2 + the dropped hold; every netlist check 0 mismatches)

| variant | unit µm² | flops | slack ns / Fmax MHz | sym/cycle | unit pJ/sym | unit + clock pJ/sym |
|---|---|---|---|---|---|---|
| DIV_BITS 2 (reference) | 10,534 | 1,016 | +1.419 / 632 | 0.885 | 16.13 | 22.96 |
| DIV_BITS 2, DIV_FIRST_BITS 0 | 10,563 | 1,016 | +1.489 / 662 | 0.885 | 16.04 | 22.91 |
| DIV_BITS 3 | 10,464 | 848 | +1.091 / 524 | 0.995 | 15.08 | 20.51 |
| DIV_BITS 4, DIV_FIRST_BITS 4 (3 stages) | 10,132 | 736 | +0.274 / 367 | 0.996 | 14.43 | 19.40 |
| OUT_DEPTH 6 | 10,014 | 983 | +1.415 / 631 | 0.832 | 15.78 | 22.80 |
| COUNT_WIDTH 24 | 10,356 | 992 | +1.271 / 578 | 0.885 | 15.95 | 22.44 |
| ABC_AREA = 1 (a flow parameter of the Makefile flow), DIV_BITS 2 | 10,845 | 1,016 | +1.838 / 860 | 0.885 | 17.00 | 23.90 |
| ABC_AREA = 1, DIV_BITS 3 | 10,320 | 848 | +1.703 / 771 | 0.995 | 15.47 | 20.89 |

1. **Stage count.** Each stage removed saves ~56 flops (~0.16 mW standing) and one cycle of lane distance. But ORFS maps
   the deeper stage with larger cells. DIV_BITS 3 gives -0.7% area, -6.5% unit energy (-10.7% with the clock tree) and
   +12% throughput at N = 8, for -17% Fmax, still with +1.09 ns of slack at 3 ns. DIV_BITS 4 / FIRST 4, three stages,
   gives -3.8% area and -10.5% / -15.5% energy, for Fmax 367 MHz (+0.27 ns slack). Both are options under the round's
   rule, since they trade Fmax. At a 3 ns clock the three-stage divider is the energy design.
2. **OUT_DEPTH 6.** -5% area, -2% energy, -6% throughput: an option.
3. **COUNT_WIDTH 24.** -1.7% area and -1% energy, inside the noise for Fmax. It needs n < 2^24: an option.
4. **ABC_AREA.** ORFS's "area" ABC script gives more cells here (6,426 against 4,769 comb) and more energy, at a much
   higher Fmax. It is not an area script for this design, and the Makefile does not use it.

### R2 measured

| | unit µm² | slack / Fmax | unit pJ/sym | unit + clock | comb cells |
|---|---|---|---|---|---|
| baseline | 10,414 | +1.387 / 620 | 16.09 | 22.87 | 4,675 |
| R2 (q2_lane carries the read lane) | 10,347 | +1.344 / 604 | 15.99 | 22.86 | 4,530 |

Inside the noise everywhere, with one level fewer on the longest path. That path still runs from q2_lane through the
read mux, the fresh select, the compare and the first step to qd_div[0]. Kept as a structural simplification.

### R4. Clock gates on the register groups written only now and then (kept for the lane states and the escape count)

Nangate45 has integrated clock-gating cells (CLKGATE_X1..X8, CLKGATETST_X1..X8 in the liberty). yosys does not infer
them. ORFS maps a behavioural gate, an enable latched while the clock is low and ANDed with the clock, to DLL_X1 +
AND2_X1. OpenSTA then times it as a clock-gating check. The round's gate was `pans_enc_stream_cg`. It is now
`pans_stream_cg` in `rtl/pans_stream_cg.sv`, shared by both units. The gated flops become plain flops with no hold mux.
A flop that is not clocked draws neither its clock-pin internal power nor its share of the clock tree.

The first variant, R4, gates the lane states (one gate per lane), every FIFO entry (16 gates) and the escape counter.
The second variant, R4b, gates the lane states and the escape count only. The FIFO keeps its enable flops.

| | unit µm² | flops | worst slack / Fmax | unit pJ/sym | unit + clock | lane_x µW | FIFO µW | counters µW | clock tree mW |
|---|---|---|---|---|---|---|---|---|---|
| R2 (enable flops) | 10,347 | 1,016 | +1.344 / 604 | 15.99 | 22.86 | 360 | 171 | 239 | 2.02 |
| R4, 17 gates | 9,676 | 1,031 (17 latches counted) | +0.436 / 637 | 14.61 | 20.04 | 142 | 58 | 175 | 1.60 |
| R4b, 9 gates (lanes + escape count) | 9,917 | 1,025 | +1.279 / 619 | 14.91 | 21.00 | 142 | 171 | 175 | 1.80 |

**R4.** -6.5% area, as 224 + 128 hold muxes go for 17 latch + AND pairs. -8.6% unit energy and -12.3% with the clock
tree. Netlist check 0 mismatches. But the worst setup path moved. A gate's enable must settle before the clock's falling
edge, half a period. The FIFO's write enables come out of the renorm stage's longest path (s3_push: lane read mux, fresh
select, the compare, the escape path). The path q2_lane -> g_fifo[1].u_cg0 latch has +0.44 ns of slack, against +1.34 ns
register to register. So the FIFO gates would cap the clock near 2.1 ns, where the design otherwise reaches 1.65 ns. The
lane gates' enables (sa_write, sa_lane) and the escape counter's (q2_valid, core_escape) come from registers. They are
short paths, with +1.30 ns of slack on the gating check.

**R4b against R2.** -4.2% area, -6.8% unit energy, -8.1% with the clock tree. Fmax is unchanged: 619 against 604 / 620.
The gating checks have +1.28 ns of slack, and the register-to-register path is the same renorm path as before. Netlist
check 0 mismatches. Kept, as the parameter `CLOCK_GATE`. Its default is 1, and 0 gives the enable flops back for a flow
without latches. The gate is the textbook latch and AND. A technology mapping that prefers its CLKGATE cell substitutes
that cell for the one module. The rule for any further gate: its enable must come straight out of registers, because it
gets only half a period.

### The parameters with R4b (options; every netlist check 0 mismatches)

| variant | unit µm² | flops | slack ns / Fmax MHz | sym/cycle | unit pJ/sym | unit + clock pJ/sym |
|---|---|---|---|---|---|---|
| DIV_BITS 2 (the default, R4b) | 9,917 | 1,025 | +1.279 / 619 | 0.885 | 14.91 | 21.00 |
| DIV_BITS 3 | 10,021 | 857 | +1.257 / 576 | 0.995 | 13.96 | 18.64 |
| DIV_BITS 4, DIV_FIRST_BITS 4 (3 stages) | 9,505 | 745 | +0.347 / 377 | 0.996 | 13.18 | 17.36 |
| OUT_DEPTH 6 | 9,572 | 992 | +1.276 / 615 | 0.832 | 14.61 | 20.80 |
| DIV_BITS 3, OUT_DEPTH 6, COUNT_WIDTH 24 | 9,113 | 800 | +1.155 / 542 | 0.849 | 13.62 | 18.49 |

The flop counts include the gate latches. At a 3 ns clock every row meets timing. Where area and energy come first, the
last two rows are the designs. Three divider stages suit a clock of 377 MHz or less: -4.2% area, -11.6% unit energy and
-17% with the clock tree against the default. Otherwise DIV_BITS 3 with OUT_DEPTH 6 and COUNT_WIDTH 24 gives -8.1% area
and -8.7% / -12% energy, at 542 MHz and 0.85 symbols/cycle.

### R5. The divider's register sets gated by the valid of the slot feeding them (not the default, not in the tree)

R5 adds eight gates. Their enables are q2_valid and the qd_valid bits, all registers, so the gating checks have slack. A
bubble leaves a set unclocked. At the power workload (0.885 sym/cycle, 11.5% bubbles) the unit is 10,286 µm², +3.7%
against R4b. ORFS mapped the gated sets with sync-set flops and more combinational logic: 4,721 against 4,377 cells. It
has 1,033 flops, slack +1.231 ns / 612 MHz, 15.12 pJ/sym (+1.4%) and 20.50 with the clock tree (-2.4%, the tree shrinks
to 289 buffers). That is a wash at full load, for +3.7% area. R5 pays with a host that does not keep the encoder fed.
Producer idle sweep, consumers always ready, unit / unit + clock tree pJ/sym:

| producer idle | R4b | R5 |
|---|---|---|
| 0% (0.885 sym/cycle) | 14.91 / 21.00 | 15.12 / 20.50 |
| 50% (0.497) | 19.36 / 30.1 | 17.29 / 25.4 (-11% / -16%) |
| 80% (0.200) | 32.01 / 58.7 | 24.06 / 41.4 (-25% / -29%) |

R5 is not the default, by the round's rule of equal area at equal Fmax. It is not a parameter in the RTL either. Gating
the sets needs each set's flops declared in its own generate block, since they have different clocks. That restructuring
alone, without any gate, maps 3% worse in ORFS (R4c below). This entry records R5 for an encoder whose host paces the
symbols: eight gates on q2_valid / qd_valid, the sets as plain flops behind them, -16% / -29% energy at 50% / 80% idle.

### R4c. The R4b function with the pipeline's register sets restructured into one generate block (reverted)

R4c changes code structure only. Each set's flops are local to a generate block, and the qd_* arrays are assigned from
them, the form R5 needs. It gives 10,215 µm², 15.47 pJ/sym and 567 MHz, against R4b's 9,917 / 14.91 / 619 for the same
function: +3% area, +3.8% energy, 4,619 against 4,377 comb cells. The flow's sensitivity to the netlist order is that
large. The R4b structure stays, and the noise floor above is revised to ±3% for area and energy.

### R6. One-hot select for the lane state read (dropped by generic synthesis)

A one-hot select register (N flops, loaded beside q2_lane) with an AND-OR read, against the binary 3-bit select and its
mux tree, both with the fresh select, as blocks: binary 469 gates (3 flops) at depth 7, one-hot 466 gates (8 flops) at
depth 7. Equal depth and five more flops leave nothing for ORFS to resolve under a ±3% / ±7% noise floor. Dropped.

## 5. Result of round 2

The round-2 design has the round-1 defaults plus CLOCK_GATE = 1. ORFS Nangate45 3 ns, p12s8, N = 8, OUT_DEPTH 8:

| | unit µm² | flops | slack / Fmax | sym/cycle | unit pJ/sym | unit + clock tree pJ/sym |
|---|---|---|---|---|---|---|
| round-1 design (the baseline of both the Makefile flow and the scratch flow) | 10,414 | 1,016 | +1.387 / 620 | 0.885 | 16.09 | 22.87 |
| round 2 (R2 + R4b) | 9,917 | 1,025 | +1.279 / 619 | 0.885 | 14.91 | 21.00 |

The round gives -4.8% area, -7.3% unit energy and -8.2% with the clock tree, with Fmax unchanged and 0 netlist
mismatches. The simulations pass on all 9 stream configs and on 16 parameter variants.

Options measured on this design (tables above):

- DIV_BITS 3: 13.96 / 18.64 pJ, 576 MHz, N = 8 saturated;
- three divider stages: 13.18 / 17.36 pJ, 377 MHz;
- OUT_DEPTH 6: -3.5% area, 0.832 sym/cycle;
- the area-first combination DIV_BITS 3 + OUT_DEPTH 6 + COUNT_WIDTH 24: 9,113 µm², 13.62 / 18.49 pJ, 542 MHz, 0.849
  sym/cycle;
- the pipeline gates of R5, for a host that idles the encoder: -16% / -29% energy at 50% / 80% idle, +3.7% area.

Dropped with numbers: R1 (entry hold), R6 (one-hot select), ABC_AREA (more cells and energy here), and R5 and R4c as
defaults.

**Where the remaining energy is** (R4b, 4.40 mW unit + 1.80 mW clock tree at 0.885 sym/cycle):

| Part | mW |
|---|---|
| the divider's comb (the 16 restoring steps and the renorm stage, toggling on every push: the arithmetic itself) | 2.0 |
| the 440 pipeline flops, plus their clock-tree share (the stage count is the lever) | 1.4 |
| the lane array (gated) | 0.14 |
| the FIFO | 0.17 |
| the counters | 0.17 |
| the core (not this unit's) | 0.19 |
| the clock tree, for 1,361 flops including the wrapper's 336 | 1.80 |

**Not tried, and why:**

- delaying the FIFO's write by a register stage, so that its entries could be gated with registered enables. The 34
  flops of pipeline would cost about what the FIFO gates saved, 113 µW: a wash, even before the added latency;
- gating job_rem or cnt_words. They change on 88-89% of the cycles;
- an ICG cell instantiated by name. That is technology-specific. The behavioural gate maps to DLL + AND2 and times as a
  gating check, which is the portable form;
- anything in the core;
- a shorter escape path in the renorm stage. The encoder's states always lie in [L, L·2^W), which is asserted. So where
  P + S ≥ W + log2 E, as at p12s8, an escape whose raw push emits nothing always emits its escape word. The escape path
  could use that. Contract version 5 made the renorm stage the unit's critical path (STREAM.md §4.2).
