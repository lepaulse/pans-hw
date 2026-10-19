# pANS hardware coder

The pANS hardware coder is an asymmetric numeral system (ANS) entropy coder whose decoder inverts a parametric
cumulative distribution function (CDF) exactly, in closed form. pANS (parametric ANS) computes each symbol's frequency
from the parameters of its distribution instead of reading it from a table, so the distribution may change with every
symbol. Here each symbol is coded against a discretized Laplace distribution with a scale predicted per symbol, and the
decoder recovers the slot's symbol with a threshold, a shift and one division. There is no frequency table, no search
and no correction step.

The decoder core is about 2,200 gates and 3,500 µm² on Nangate45, runs at 760 MHz, decodes one symbol per cycle and
spends 6 pJ per symbol. On the ASAP7 predictive 7 nm kit it is 273 µm² and 1.1 pJ per symbol decode, and the eight-lane
decoder unit around it, with the ANS state, the renormalization and the word FIFO, is 866 µm² and 2.0 pJ. The coder's
rate lies within a few tenths of a percent of its model's ideal cost (SPEC.md §5). The probability model supplies one
number per symbol, the scale, and nothing else.

The software pANS coder is published at https://github.com/lepaulse/pans. This repository holds two cores that map a
symbol to its interval and a slot back to its symbol, two stream units that run N interleaved lanes of ANS state around
the cores and read or write one byte array, the bit-exact C++ models that define them, and the verification and PPA
flow.

## Features and limits

- **No frequency tables.** `pans_enc_core` and `pans_dec_core` compute the discretized-Laplace model in integer
  arithmetic. The only constants are the scale lookup `SCALE_TABLE` (240 × 7 bits at p12s8), which converts the scale
  input b to the cores' scale code and is normative data of the build, and eight 15-bit division constants. The tail
  2^-a uses Mitchell's binary-logarithm approximation (1962) and reduces to wiring; the division is exact, by a
  truncated magic multiply.
- **Exact inverse.** The decoder core inverts the coder's integer CDF exactly: a threshold, a shift and a division give
  the slot's symbol. No candidate, no correction step, no search. Encoder and decoder agree by the integer definition,
  checked exhaustively over every (scale, slot) (SPEC.md §6.1).
- **Rate.** The coder spends a few tenths of a percent above its model's entropy (SPEC.md §5). On Gaussian data the
  Laplace model costs about 0.08 bits per symbol above the Gaussian's entropy, the divergence between the two shapes:
  about 3% at σ = 1, 1 to 2% for σ from 4 to 64, under 1% beyond. Golomb-Rice, the usual hardware coder for
  Laplace-shaped data, cannot spend less than one bit per symbol: around σ = 1/4 it spends 2.8× this coder's rate, at
  σ = 1 about 2% more, and from σ ≈ 4 to 256 the two tie within 1% (SPEC.md §5).
- **σ range is a hard limit.** Beyond about 2^VF the excess rises to tens of percent; the Configurations table gives
  each build's usable range.
- **One symbol per cycle per core.** The cores are valid-only pipelines of 2 stages (encoder) and 4 stages (decoder)
  ([SPEC.md](SPEC.md)).
- **Stream units.** `pans_enc_stream` and `pans_dec_stream` keep N interleaved lanes of ANS state around a core. All
  lanes share one word stream in one byte array, and the channels are valid/ready. At N = 8 the decoder unit decodes
  one symbol per cycle when the consumer returns a lane's next scale one cycle after the lane's symbol leaves; two
  cycles give 8 symbols in 9 ([STREAM.md](STREAM.md) §2.1).
- **Scale input.** Every symbol carries a scale input b, the Laplace scale predicted by its model, with 8 fraction bits.
  How b is predicted is the model's business. The stream model ships example scale sources, one of them an integer
  moving average of |symbol| that hardware can implement.
- **Builds.** A build is fixed by its precision P (M = 2^P slots), its symbol width SW, its escape slots E and an
  optional symmetric alphabet. The package generator refuses builds the RTL cannot support.
- **Compatibility.** An encoder and a decoder are compatible only within one build, identified by `CONFIG_ID`. The
  stream is not compatible with the software pANS coders.
- **Normative models.** The bit-exact C++ models `ref/pans1_ref.h` (the cores) and `ref/pans_stream_ref.h` (the stream)
  define the coder. The testbenches check the RTL against them through DPI.
- **Not covered.** No formal proof covers the RTL.

## Measurements

Post-route results from OpenROAD-flow-scripts (ORFS), image `openroad/orfs:26Q4-137-g9c431405a`, on Nangate45 at a
3 ns target and on ASAP7 at 1 ns. Energy comes from zero-delay gate-level activity on a Laplace workload (SPEC.md §7.1,
STREAM.md §4.1). Fmax is 1 / (target − worst slack). Nangate45 is an academic 45 nm library and ASAP7 a predictive 7 nm
kit at its typical corner (TT, 0.7 V, 25 °C); both compare designs within this flow, not against a commercial process.

| p12s8 (M = 2^12, int8 symbols) | platform, target | area | Fmax | energy per symbol | throughput |
|---|---|---|---|---|---|
| `pans_enc_core` | Nangate45, 3 ns | 1,593 µm² | 709 MHz | 3.00 pJ | 1 per cycle |
| | ASAP7, 1 ns | 129 µm² | 1,277 MHz | 0.50 pJ | |
| `pans_dec_core` | Nangate45, 3 ns | 3,506 µm² | 762 MHz | 6.16 pJ | 1 per cycle |
| | ASAP7, 1 ns | 273 µm² | 1,204 MHz | 1.11 pJ | |
| `pans_dec_stream`, N = 8 | Nangate45, 3 ns | 11,400 µm² | 643 MHz | 12.02 pJ (17.25 with its clock tree) | 0.99 per cycle |
| | ASAP7, 1 ns | 866 µm² | 1,041 MHz | 2.05 pJ (3.48) | |
| `pans_enc_stream`, N = 8 | Nangate45, 3 ns | 10,286 µm² | 541 MHz | 15.70 pJ (21.93) | 0.89 per cycle (1 from DIV_BITS 3) |
| | ASAP7, 1 ns | 798 µm² | 1,057 MHz | 2.85 pJ (4.39) | |

Scaling the ASAP7 unit by the foundry-disclosed steps from 7 nm to the 2 nm class (1.84×, 1.6× and 1.2× in logic
density, about 30% less power per step at equal speed) puts it near 250 µm² and 0.7 pJ per symbol decode. That is
arithmetic on a predictive kit, an order of magnitude, not a measurement; SPEC.md §7.5 lists the sources.

SPEC.md §7 and STREAM.md §4 give further builds, the symmetric-alphabet build p12s8sym, the design steps and the
measured alternatives.

## Configurations

A build is a point (P, SW, E) plus the alphabet option. The package generator derives every width from it and refuses
what the RTL cannot support. The same RTL covers 4-bit to 16-bit symbols and 2^4 to 2^16 slots; the verification matrix
runs eleven builds on every commit.

| Build | Precision M | Alphabet | σ range | Excess over the model's entropy, mean / worst octave | Core area, encoder / decoder | Fmax, encoder / decoder | Energy per symbol, encoder / decoder |
|---|---|---|---|---|---|---|---|
| p4s4 | 2^4 = 16 | int4, 16 symbols | 1/4 to 2 | 20.93% / 47.92% | 689 / 2,089 µm² | 1,272 / 836 MHz | 1.23 / 3.54 pJ |
| p8s8 | 2^8 = 256 | int8, 256 symbols | 1/4 to 16 | 2.46% / 4.76% | 1,287 / 3,025 µm² | 893 / 720 MHz | 2.46 / 5.25 pJ |
| p9s9 | 2^9 = 512 | int9, 512 symbols | 1/4 to 16 | 1.56% / 3.17% | 1,503 / 3,642 µm² | 884 / 762 MHz | 2.82 / 6.00 pJ |
| p12s8 | 2^12 = 4,096 | int8, 256 symbols | 1/4 to 32 | 0.53% / 0.81% | 1,593 / 3,506 µm² | 709 / 762 MHz | 3.00 / 6.16 pJ |
| p12s8sym | 2^12 = 4,096 | int8, -127..127 | 1/4 to 32 | 0.53% / 0.81% | 1,553 / 3,593 µm² | 765 / 735 MHz | 3.00 / 6.26 pJ |
| p16s16 | 2^16 = 65,536 | int16, 65,536 symbols | 1/4 to 1024 | 0.56% / 1.78% | 2,782 / 6,005 µm² | 605 / 651 MHz | 5.29 / 9.84 pJ |

Nangate45, post-route, 3 ns targets except p16s16 at 5 ns (SPEC.md §7). The excess is the rate above the entropy of the
discretized Laplace the data follows, from the rate sweep (`ref/results/sweep_<build>.csv`, 32 values of σ per octave,
SPEC.md §5): the mean over the σ range and the mean of its worst octave. The range covers the octaves centered at or
below 2^VF (SPEC.md §4); beyond it the rate collapses, to 13% around σ = 32 at p8s8 and to 60% around σ = 4096 at
p16s16. At p4s4 the 16 slots set most of the excess: an exact-math coder at the same precision spends 73% above the
entropy around σ = 1/4, against the build's 48%.

The decoder core grows from about 2,100 to about 6,000 µm² while the alphabet grows from 16 to 65,536 symbols, because
it holds no table. The remaining builds of the matrix (p8s6, p8s6e3, p12s9, p12s10, p16s8) are verified on every commit;
`make ppa-<cfg>` gives any build's generic gates.

## Requirements

| Tool | Version | For |
|---|---|---|
| GNU make | 4.4 or later (`.NOTINTERMEDIATE`) | every target |
| Verilator | 5.052 | lint, every simulation |
| yosys with the slang frontend | the OSS CAD Suite build of 2026-10-14 (Yosys 0.70 with the slang plugin) | generic gate counts (`make ppa-*`) |
| g++ | C++17 and OpenMP (15.2 used) | the models, the self-tests, the DPI |
| Python 3 | 3.11 or later, for `tomllib`. The standard library only (3.13 used) | the matrix, the CI runner, the metrics scripts |
| docker | the image `openroad/orfs:26Q4-137-g9c431405a` | ORFS and power (`make asic-*`, `power-*`) |

The Makefile, `ci/run.py` and `ppa/synth_generic.sh` run `verilator`, `yosys`, `g++`, `python3` and `docker` from the
PATH. The make variables `VERILATOR`, `YOSYS`, `CXX` and `PYTHON` override the first four, and `ORFS_IMAGE` the image.
A specific installation is selected by setting one of them on the command line or in the environment:

```
make VERILATOR=/path/to/verilator ci
make YOSYS=/path/to/oss-cad-suite/bin/yosys ppa-p12s8
```

`make ci` requires Verilator, g++ and Python, and runs in a tree without git as well. The post-commit hook installed
by `make ci-hooks` is optional.

## Quick start

```
make ci                                    # the smoke tier of the verification matrix on every config
make sim-dec-stream-p12s8 SEEDS="1 2 3"    # the decoder unit against the stream model, three seeds
make sweep-p12s8                           # rate and escapes over σ -> ref/results/sweep_p12s8.csv
make asic-p12s8 power-p12s8                # both cores through ORFS (docker) and their gate-level power
make asic-p12s8 power-p12s8 ASIC_PLATFORM=asap7    # the same on ASAP7, into build/p12s8/asic_asap7/, power_asap7/
```

A simulation passes when it prints `PASSED: <n> checks, 0 errors`. `make ci` writes
`ci/runs/<time>-<commit>/summary.md`. `make asic-stream-p12s8 power-stream-p12s8` runs both stream units through ORFS
and gate-level power.

`make stream-tool-p12s8` builds `build/p12s8/stream_tool`. The tool encodes and decodes files of symbols through the
stream model and reports where the bits go (STREAM.md §1.10). This makes it possible to measure what a given scale
source costs on given data:

```
build/p12s8/stream_tool --const 1024 data.bin       # one scale b = 4 for every symbol
build/p12s8/stream_tool --b-list data.bin data.b    # a b per symbol from a text file
build/p12s8/stream_tool data.bin                    # the example adaptive estimator
```

## Layout

| Path | What |
|---|---|
| `rtl/` | the cores, the stream units, the scale lookup `pans_scale_of_b`, the clock gate; the PPA ledgers |
| `ref/` | the C++ models (`pans1_ref.h`, `pans_stream_ref.h`, `pans1_config.h`), the package generator `gen_pkg.cpp`, the self-tests `test_inverse.cpp` and `test_stream.cpp`, the DPI (`pans1_dpi.cpp`, `pans_stream_dpi.cpp`), the rate sweep `sweep.cpp` with `results/`, `stream_tool.cpp` |
| `tb/simple/` | the cores' testbench: every input and slot per scale, against the model |
| `tb/stream/` | the stream units' testbenches and the RTL round trip from the encoder unit to the decoder unit |
| `ppa/` | generic synthesis, the ORFS configuration and scripts, the timing wrappers, the gate-level power testbenches |
| `ci/` | the verification matrix `matrix.toml`, its loader, the CI runner `run.py`, a post-commit hook |

A build writes everything it produces to `build/<cfg>/`. The RTL package `pans_pkg.sv` is generated there for each
config by `ref/gen_pkg.cpp`, which refuses builds that the RTL cannot support, with the reasons, and stamps `CONFIG_ID`.

## Targets

The targets below take a config `<cfg>` of `ci/matrix.toml`. The Makefile's header lists every target and variable.

- Cores: `pkg-`, `lint-`, `sim-`, `inverse-` (the exhaustive check of the decoder algorithm), `ppa-` (generic gates),
  `asic-` (ORFS), `power-<cfg>`. The `asic-` and `power-` targets, the stream units' too, take `ASIC_PLATFORM=nangate45`
  (the default) or `asap7`.
- Stream: `stream-ref-` (the model's self-test), `lint-`, `sim-dec-stream-`, `sim-enc-stream-`, `sim-stream-rt-` (the
  round trip), `ppa-`, `asic-`, `power-stream-<cfg>`, `stream-tool-<cfg>`. Every unit parameter is a make variable
  (`N_LANES`, `DEC_FIFO_WORDS`, `ENC_DIV_BITS`, ...). The simulations take `SEEDS="1 2 3"` and `PLUSARGS`.
- Every config: `make lint`, `inverse`, `stream-ref`, `refuse`, `stream-refuse`, `guard`. `make ci` runs the smoke tier,
  `make matrix` shows the matrix as a table, and `make sweep-<cfg>` runs the rate sweep.

## Adding a config

Add a section to `ci/matrix.toml`:

```
[config.p10s8]
precision    = 10
symbol_width = 8
escape_slots = 1
why          = "M = 2^10 over int8 symbols"
# optional: symmetric_alphabet = true, asic_period_ns = 3.0, stream = "refused"
```

`make matrix` shows the new config. `make pkg-p10s8` generates its package, or prints why the generator refuses the
build (SPEC.md §4 lists the limits). `make ci` runs every job of the smoke tier on it. A build that the generator is
required to refuse goes under `[refused.<name>]`, and a build that the stream units cannot implement is marked
`stream = "refused"`.

## Documentation

- [SPEC.md](SPEC.md): the cores. It covers the model, the intervals, the escape, the alphabet options, the interfaces,
  the build parameters and identity, the rate, the verification and the PPA.
- [STREAM.md](STREAM.md): the stream. It covers the stream definition, the stream units, their verification and their
  PPA.
- `rtl/PPA_DEC_STREAM.md` and `rtl/PPA_ENC_STREAM.md`: the PPA ledgers of the stream units. They record every
  optimization candidate measured on the units, with its numbers and whether the design kept it.

## Verification

- Cores: SPEC.md §6. It covers the exhaustive `test_inverse`, the directed simulation, the gate-level check and the
  mutation tests.
- Stream: STREAM.md §1.9 and §3. §1.9 covers the model's self-test, including a literal transcription of pANS 1.0's
  per-lane functions. §3 covers the unit testbenches, the RTL round trip, the CI jobs, the parameter variants and the
  mutation tests.
- CI: `ci/run.py --tier smoke` runs a tier as a parallel job graph. `ci/run.py --status` lists the runs. `make ci-hooks`
  installs an optional post-commit hook that runs the smoke tier on each commit in `build/ci/worktree`.

## License

Apache License 2.0 ([LICENSE](LICENSE)). Copyright 2026 Lars Erik Songe Paulsen.
