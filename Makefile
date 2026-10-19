# SPDX-License-Identifier: Apache-2.0
# pANS hardware coder: the cores (SPEC.md) and the stream units (STREAM.md), per config of ci/matrix.toml.
# A config builds into build/<cfg>/ only. Its package is build/<cfg>/gen/pans_pkg.sv, never in rtl/.
#
# Cores
#   make pkg-<cfg>               generate the package (the generator refuses unsupported builds)
#   make lint-<cfg>              Verilator -Wall on both cores
#   make sim-<cfg>               directed encode/decode test against the C++ reference (tb/simple)
#   make inverse-<cfg>           exhaustive check of the decoder algorithm and of C on the C++ reference
#   make ppa-<cfg>               generic synthesis of both cores and of each function: gates, depth, operator widths
#   make asic-<cfg>              both cores through ORFS at the config's target period: area, slack, fmax
#   make power-<cfg>             power of the routed cores from a gate-level simulation of a fixed workload
#                                (POWER_STIMULUS=static|random: another workload, into power_<stimulus>/)
#   make refuse-<cfg>            for a [refused] config: passes iff the generator refuses it
#   make guard                   a C++ reference built for another config must be rejected by the testbench
# ASIC platform, on every asic- and power- target (cores and stream units): ASIC_PLATFORM=nangate45 (the default, at
#   asic_period_ns of ci/matrix.toml) or ASIC_PLATFORM=asap7 (ASAP7 at its typical corner, RVT TT 0.7 V 25 C, at
#   asic_period_ns_asap7), which builds into asic_asap7/ and power_asap7/ beside asic/ and power/:
#   make asic-p12s8 power-p12s8 asic-stream-p12s8 power-stream-p12s8 ASIC_PLATFORM=asap7
# Stream (STREAM.md). A unit target stops first if gen_pkg --stream refuses the config (p8s6e3).
#   make stream-ref-<cfg>        the stream model's self-test (ref/test_stream.cpp, STREAM.md §1.9)
#   make stream-tool-<cfg>       builds build/<cfg>/stream_tool (ref/stream_tool.cpp, STREAM.md §1.10)
#   make lint-dec-stream-<cfg>   Verilator -Wall on pans_dec_stream (lint-enc-stream-<cfg>: pans_enc_stream)
#   make sim-dec-stream-<cfg>    pans_dec_stream against the stream model (tb/stream/tb_pans_dec_stream.sv, DPI)
#   make sim-enc-stream-<cfg>    pans_enc_stream against the stream model (tb/stream/tb_pans_enc_stream.sv, DPI)
#   make sim-stream-rt-<cfg>     RTL round trip, encoder to array to decoder (tb/stream/tb_pans_stream_rt.sv)
#   make ppa-dec-stream-<cfg>    generic gates of pans_dec_stream, flat and per module (ppa-enc-stream-<cfg>: of
#                                pans_enc_stream at each DIV_BITS of ENC_PPA_DIV_BITS, default "1 2 4")
#   make asic-stream-<cfg>       both units in their timing wrappers through ORFS: area by group, slack, fmax and
#                                the worst slack per stage (one unit: asic-dec-stream-<cfg>, asic-enc-stream-<cfg>)
#   make power-stream-<cfg>      power of the routed units from EMA jobs, also by register group (one unit:
#                                power-dec-stream-<cfg>, power-enc-stream-<cfg>; POWER_P_IDLE=<%>: power_idle<%>/;
#                                POWER_STIMULUS=static|random: power_<stimulus>/)
#   make lint-stream-<cfg> | sim-stream-<cfg> | ppa-stream-<cfg>   both units (sim: with the round trip)
#   make stream-refuse-<cfg>     for a config marked stream = "refused": passes iff gen_pkg --stream refuses it
#   Unit parameters, Verilator / yosys / ORFS -G overrides on every unit target (each set builds in its own
#   directory): N_LANES (both units; default 8); decoder DEC_DATA_BYTES (4), DEC_FIFO_WORDS (8), DEC_OUT_DEPTH (7),
#   DEC_COUNT_WIDTH (32), DEC_CLOCK_GATE (1); encoder ENC_DIV_BITS (2), ENC_DIV_STEP_BITS (1), ENC_DIV_FIRST_BITS
#   (= ENC_DIV_STEP_BITS), ENC_OUT_DEPTH (8), ENC_COUNT_WIDTH (32), ENC_CLOCK_GATE (1). STREAM.md §2: their limits.
#   Simulations: SEEDS="<n> <n> ..." (or SEED=<n>; default 1), one run per seed, the first failure stops;
#   PLUSARGS="+..." goes to every run.
# Every config
#   make lint | sim | refuse | inverse | stream-ref | stream-refuse   (sim: CFG, default p8s6)
#   make ci                      the smoke tier through the CI runner (ci/run.py); make ci-hooks installs the hook
#   make sweep-<cfg>             rate and escapes over sigma on Laplace data, and the rate on Gaussian data
#                                (ref/sweep.cpp) -> ref/results/sweep_<cfg>.csv
#   make sweep                   the rate sweep at p16s16 (SPEC.md §5)
#   make matrix                  show the matrix

VERILATOR ?= verilator
YOSYS     ?= yosys
export YOSYS
CXX       ?= g++
PYTHON    ?= python3
BUILD     := build
JOBS      ?= 8
CFG       ?= p8s6

RTL_CORES  := rtl/pans_enc_core.sv rtl/pans_dec_core.sv
RTL_INC    := rtl/pans_pkg_functions.svh
REF        := ref/pans1_ref.h ref/pans1_config.h
STREAM_REF := ref/pans_stream_ref.h ref/pans_stream_dpi.cpp
SMOKE_TB   := tb/simple/tb_pans_cores.sv

.PHONY: all pkg lint sim refuse inverse stream-ref stream-refuse guard ci ci-hooks sweep matrix clean
.SECONDARY:
all: lint sim

# ---- The matrix: config lists and per-config -D flags (CFG_DEFS_<cfg>)
include $(BUILD)/matrix.mk
$(BUILD)/matrix.mk: ci/matrix.toml ci/matrix.py
	@mkdir -p $(@D)
	$(PYTHON) ci/matrix.py make > $@.tmp && mv $@.tmp $@

defs = $(or $(CFG_DEFS_$1),$(error config '$1' is not in ci/matrix.toml))

# The config's -D flags as a file, rewritten only when they change: a matrix edit rebuilds exactly the configs
# it touches. While the file is older than matrix.mk, every make runs this recipe again, also several at once
# (the CI runner's parallel jobs of one config): hence a temporary file per process. The files are prerequisites of
# .NOTINTERMEDIATE (GNU make 4.4), so make never takes them for intermediate files: .SECONDARY marks every file
# intermediate, and so does a pattern rule that reaches a file named nowhere else. For an intermediate prerequisite
# make compares the target with that file's own prerequisites, so every target older than matrix.mk was remade
# although the flags did not change: gen_pkg, while other jobs ran it. matrix.mk is listed too, so that the list is
# never empty (before matrix.mk exists): an empty one means every file, which make refuses beside .SECONDARY. The
# recipe also runs under make -n (+), so that a dry run sees the file unchanged.
$(BUILD)/%/defs: $(BUILD)/matrix.mk
	+@mkdir -p $(@D)
	+@tmp=$@.tmp.$$$$; echo '$(call defs,$*)' > $$tmp; if cmp -s $$tmp $@; then rm $$tmp; else mv $$tmp $@; fi
.NOTINTERMEDIATE: $(BUILD)/matrix.mk $(foreach c,$(CONFIGS) $(REFUSED),$(BUILD)/$c/defs)

# ---- Package (its scale table SCALE_TABLE is the model's: ref/pans1_ref.h build_scale_table)
$(BUILD)/%/gen_pkg: ref/gen_pkg.cpp $(REF) $(BUILD)/%/defs
	$(CXX) -O2 -std=c++17 -Wall -Wextra -Iref $(call defs,$*) $< -o $@

$(BUILD)/%/gen/pans_pkg.sv: $(BUILD)/%/gen_pkg
	@mkdir -p $(@D)
	$< > $@.tmp && mv $@.tmp $@

pkg-%: $(BUILD)/%/gen/pans_pkg.sv ;

refuse-%: $(BUILD)/%/gen_pkg
	@if $< > /dev/null; then echo "FAIL: the generator accepted $*, which ci/matrix.toml lists as refused"; exit 1; \
	 else echo "PASS: $* refused as expected"; fi

# ---- Lint
$(BUILD)/%/lint.ok: $(BUILD)/%/gen/pans_pkg.sv $(RTL_CORES) $(RTL_INC)
	$(VERILATOR) --lint-only -Wall -Irtl --top-module pans_enc_core $< rtl/pans_enc_core.sv
	$(VERILATOR) --lint-only -Wall -Irtl --top-module pans_dec_core $< rtl/pans_dec_core.sv
	touch $@

lint-%: $(BUILD)/%/lint.ok ;

# ---- Directed encode/decode test against the C++ reference (DPI)
# A simulator's C++ (the DPI and the model behind it) is compiled with the config's -D flags, which Verilator's own
# incremental build does not track: when they change (build/<cfg>/defs newer than the simulator), the object
# directory is emptied first, so that no object of the old flags is linked (a CONFIG_ID mismatch otherwise).
fresh_objects = $(if $(filter %/defs,$?),rm -rf $(@D) && )

$(BUILD)/%/sim/Vtb_pans_cores: $(BUILD)/%/gen/pans_pkg.sv $(RTL_CORES) $(RTL_INC) $(REF) ref/pans1_dpi.cpp $(SMOKE_TB) \
                              $(BUILD)/%/defs
	$(fresh_objects)$(VERILATOR) --binary --timing --assert -j $(JOBS) -Irtl -CFLAGS "-std=c++17 -I$(CURDIR)/ref $(call defs,$*)" \
	  --top-module tb_pans_cores -Mdir $(@D) -o Vtb_pans_cores \
	  $< $(RTL_CORES) $(SMOKE_TB) $(CURDIR)/ref/pans1_dpi.cpp

sim-%: $(BUILD)/%/sim/Vtb_pans_cores
	$<

# ---- Exhaustive check on the C++ reference (ref/test_inverse.cpp): decode_hw against decode for every scale code
# and slot, the round trip of every coded slot, the division, clamp and threshold, and C itself (SPEC.md §6).
# Built with the config's -D flags and -Werror; JOBS OpenMP threads. Exit status 0 iff every check has 0 errors.
$(BUILD)/%/test_inverse: ref/test_inverse.cpp $(REF) $(BUILD)/%/defs
	$(CXX) -O2 -std=c++17 -fopenmp -Wall -Wextra -Werror -Iref $(call defs,$*) $< -o $@

inverse-%: $(BUILD)/%/test_inverse
	OMP_NUM_THREADS=$(JOBS) $<
	@echo "PASS: test_inverse $*"

# ---- PPA: generic synthesis (yosys + slang, SYNTHESIS defined, so no assertions) of both cores and of each
# function they call, shift_of aside, on its own (ppa/pans_functions_wrapped.sv). The ABC script is pinned
# (ppa/abc_medium.script): gate counts are comparable only within one script, never as area.
PPA_FUNCTIONS := slope_of a_of_pm cdf_from_a a_threshold dividend q_divide clamp_of exponent_magnitudes
$(BUILD)/%/ppa/metrics.json: $(BUILD)/%/gen/pans_pkg.sv $(RTL_CORES) $(RTL_INC) ppa/pans_functions_wrapped.sv \
                             ppa/synth_generic.sh ppa/abc_medium.script ppa/metrics.py
	@for core in enc dec; do \
	  ppa/synth_generic.sh $(@D) $$core pans_$${core}_core $< rtl/pans_$${core}_core.sv || exit 1; done
	@for fn in $(PPA_FUNCTIONS); do \
	  ppa/synth_generic.sh $(@D) fn_$$fn fn_$$fn $< ppa/pans_functions_wrapped.sv || exit 1; done
	$(PYTHON) ppa/metrics.py $(@D) > $@.tmp 2> /dev/null && mv $@.tmp $@

ppa-%: $(BUILD)/%/ppa/metrics.json
	@$(PYTHON) ppa/metrics.py $(<D) > /dev/null

# ---- ASIC PPA through OpenROAD-flow-scripts (pinned image), place and route at the config's target period, each
# design inside its timing wrapper (inputs and outputs registered). ASIC_PLATFORM picks the platform: nangate45 (the
# default; the period is asic_period_ns in ci/matrix.toml) or asap7 (asic_period_ns_asap7), at its typical corner.
# A platform other than nangate45 builds into directories with its name as suffix (asic_asap7/, power_asap7/), so the
# platforms never overwrite each other; nangate45 keeps asic/ and power/.
ORFS_IMAGE    ?= openroad/orfs:26Q4-137-g9c431405a
ORFS_DOCKER    = docker run --rm --user $$(id -u):$$(id -g) -v $(CURDIR):/work
ORFS_RUN       = $(ORFS_DOCKER) $(ORFS_IMAGE)
OPENROAD       = /OpenROAD-flow-scripts/tools/install/OpenROAD/bin/openroad -exit -no_splash
ASIC_PLATFORM ?= nangate45
PLATFORMS     := /OpenROAD-flow-scripts/flow/platforms
NANGATE45_LIB := $(PLATFORMS)/nangate45/lib/NangateOpenCellLibrary_typical.lib
# ASAP7's typical corner, the libraries ORFS reads with CORNER = TC (ppa/asic/config.mk.in): RVT, TT, NLDM, 0.7 V, 25 C
ASAP7_LIBS    := $(addprefix $(PLATFORMS)/asap7/lib/NLDM/asap7sc7p5t_,AO_RVT_TT_nldm_211120.lib.gz \
                   INVBUF_RVT_TT_nldm_220122.lib.gz OA_RVT_TT_nldm_211120.lib.gz SEQ_RVT_TT_nldm_220123.lib \
                   SIMPLE_RVT_TT_nldm_211120.lib.gz)
# Per platform: the liberty files of the cell models and of OpenSTA, and their time unit, which the SDC and ORFS's
# reports use (ci/matrix.toml gives the period in ns)
ASIC_LIBERTY_nangate45   := $(NANGATE45_LIB)
ASIC_LIBERTY_asap7       := $(ASAP7_LIBS)
ASIC_TIME_UNIT_nangate45 := ns
ASIC_TIME_UNIT_asap7     := ps
ifeq ($(ASIC_LIBERTY_$(ASIC_PLATFORM)),)
$(error ASIC_PLATFORM is '$(ASIC_PLATFORM)': the platforms are nangate45 and asap7)
endif
ASIC_LIBERTY  := $(ASIC_LIBERTY_$(ASIC_PLATFORM))
ASIC_SUFFIX   := $(if $(filter-out nangate45,$(ASIC_PLATFORM)),_$(ASIC_PLATFORM))
ASIC_DIR      := asic$(ASIC_SUFFIX)
ASIC_CELLS    := $(BUILD)/asic_cells/$(ASIC_PLATFORM)_cells.v
ASIC_FLOW     := ppa/asic/config.mk.in ppa/asic/constraint.sdc $(BUILD)/matrix.mk
asic_period    = $(or $(ASIC_PERIOD$(ASIC_SUFFIX)_$1),$\
                   $(error config '$1' has no asic_period_ns$(ASIC_SUFFIX) in ci/matrix.toml))
# the period in the platform's time unit, for config.mk and the SDC
orfs_period    = $(if $(filter ps,$(ASIC_TIME_UNIT_$(ASIC_PLATFORM))),$(call period_ps,$1),$(call asic_period,$1))

# One ORFS run: $1 its directory (WORK_HOME), $2 the top, $3 the RTL, $4 the top's parameters (NAME=VALUE ...), $5
# the modules kept through synthesis, $6 the config (its target period). ppa/asic/config.mk.in filled in becomes
# $1/config.mk; the log is $1/flow.log, the period $1/period_ns (in ns).
orfs_flow = rm -rf $1 && mkdir -p $1 && \
  sed -e "s|@TOP@|$2|" -e "s|@FILES@|$(addprefix /work/,$3)|" -e "s|@PARAMS@|$(subst =, ,$4)|" -e "s|@KEEP@|$5|" \
      -e "s|@PLATFORM@|$(ASIC_PLATFORM)|" -e "s|@PERIOD@|$(call orfs_period,$6)|" -e "s|@WORK@|/work/$1|" \
      ppa/asic/config.mk.in > $1/config.mk && \
  echo '$(call asic_period,$6)' > $1/period_ns && \
  $(ORFS_DOCKER) -w /OpenROAD-flow-scripts/flow $(ORFS_IMAGE) make DESIGN_CONFIG=/work/$1/config.mk > $1/flow.log 2>&1 \
  || { echo "ORFS failed for $2; see $1/flow.log"; exit 1; }

$(BUILD)/%/$(ASIC_DIR)/metrics.json: $(BUILD)/%/gen/pans_pkg.sv $(RTL_CORES) $(RTL_INC) ppa/pans_cores_timed.sv \
                                     $(ASIC_FLOW) ppa/asic_metrics.py
	@for core in enc dec; do \
	  $(call orfs_flow,$(@D)/$$core,pans_$${core}_core_timed,$< $(RTL_CORES) ppa/pans_cores_timed.sv,,,$*); \
	done
	@echo '$(call asic_period,$*)' > $(@D)/period_ns
	$(PYTHON) ppa/asic_metrics.py $(@D) > $@.tmp 2> /dev/null && mv $@.tmp $@

asic-%: $(BUILD)/%/$(ASIC_DIR)/metrics.json
	@$(PYTHON) ppa/asic_metrics.py $(<D) > /dev/null

# ---- Power: zero-delay gate-level simulation of each core's post-route netlist on a fixed workload
# (ppa/power_stimulus.cpp), self-checked against the C++ reference, then OpenSTA with that activity and the routed
# parasitics (ppa/asic/power.tcl). Cell models come from the liberty's cell functions (every library of the platform).
# POWER_STIMULUS picks the workload of every power target, the cores' and the stream units': laplace (the default,
# SPEC.md §7.1, STREAM.md §4.1), static (the mid-alphabet symbol at the mid-range b throughout: the activity floor) or
# random (symbols uniform over the alphabet, b uniform over its B_WIDTH bits, escapes included: the ceiling). Away from
# laplace the results go to power_<stimulus>/ beside power/ (with ASIC_PLATFORM's suffix: power_static_asap7/).
POWER_COUNT    ?= 2000
POWER_STIMULUS ?= laplace
ifneq ($(filter-out laplace static random,$(POWER_STIMULUS)),)
$(error POWER_STIMULUS is '$(POWER_STIMULUS)': laplace, static or random)
endif
POWER_SUFFIX := $(if $(filter-out laplace,$(POWER_STIMULUS)),_$(POWER_STIMULUS))
POWER_CORES  := power$(POWER_SUFFIX)$(ASIC_SUFFIX)
period_ps    = $(shell $(PYTHON) -c "print(round($(call asic_period,$1) * 1000))")
# Verilator on a routed netlist: $1 top (the testbench), $2 its -G parameters, $3 sources; into $(@D)/obj_$4, log
# $(@D)/$4.verilate.log
verilate_netlist = rm -rf $(@D)/obj_$4 && \
  $(VERILATOR) --binary --timing --trace-vcd --trace-underscore -O1 -j $(JOBS) -Wno-fatal -Wno-lint -Wno-style \
    --timescale 1ns/1ps -Irtl --top-module $1 -GPERIOD_PS=$(call period_ps,$*) $(addprefix -G,$2) \
    -Mdir $(@D)/obj_$4 -o Vpower_tb $3 > $(@D)/$4.verilate.log 2>&1 \
  || { echo "verilator failed: $(@D)/$4.verilate.log"; exit 1; }

$(ASIC_CELLS):
	@mkdir -p $(@D)
	$(ORFS_RUN) yosys -q -p "$(foreach l,$(ASIC_LIBERTY),read_liberty -ignore_miss_func $l;) write_verilog -noattr /work/$@"

$(BUILD)/%/$(POWER_CORES)/metrics.json: $(BUILD)/%/$(ASIC_DIR)/metrics.json $(ASIC_CELLS) $(REF) \
                                        ppa/power_stimulus.cpp ppa/power_tb.sv ppa/asic/power.tcl ppa/power_metrics.py
	@mkdir -p $(@D)
	$(CXX) -O2 -std=c++17 -Wall -Wextra -Iref $(call defs,$*) ppa/power_stimulus.cpp -o $(@D)/power_stimulus
	$(@D)/power_stimulus $(POWER_COUNT) $(@D) $(POWER_STIMULUS)
	@for core in enc dec; do \
	  results=$(BUILD)/$*/$(ASIC_DIR)/$$core/results/$(ASIC_PLATFORM)/pans_$${core}_core_timed/base; \
	  $(call verilate_netlist,power_tb_$$core,,$(BUILD)/$*/gen/pans_pkg.sv $(ASIC_CELLS) \
	    $$results/6_final.v ppa/power_tb.sv,$$core); \
	  $(@D)/obj_$$core/Vpower_tb +STIMULUS=$(@D)/$${core}_corner.hex +EXPECTED=$(@D)/$${core}_corner_expected.hex \
	    +COUNT=$$(wc -l < $(@D)/$${core}_corner.hex) > $(@D)/$$core.corner.log 2>&1 \
	    || { cat $(@D)/$$core.corner.log; exit 1; }; \
	  $(@D)/obj_$$core/Vpower_tb +STIMULUS=$(@D)/$$core.hex +EXPECTED=$(@D)/$${core}_expected.hex +COUNT=$(POWER_COUNT) \
	    +VCD=$(@D)/$$core.vcd > $(@D)/$$core.sim.log 2>&1 || { cat $(@D)/$$core.sim.log; exit 1; }; \
	  $(ORFS_RUN) env LIBERTY="$(ASIC_LIBERTY)" RESULTS=/work/$$results VCD=/work/$(@D)/$$core.vcd \
	    VCD_SCOPE=power_tb_$$core/dut $(OPENROAD) /work/ppa/asic/power.tcl > $(@D)/$$core.power.rpt 2>&1 \
	    || { tail $(@D)/$$core.power.rpt; exit 1; }; \
	  rm -f $(@D)/$$core.vcd; \
	done
	$(PYTHON) ppa/power_metrics.py $(@D) $(call asic_period,$*) $(POWER_STIMULUS) > $@.tmp 2> /dev/null && mv $@.tmp $@

power-%: $(BUILD)/%/$(POWER_CORES)/metrics.json
	@$(PYTHON) ppa/power_metrics.py $(<D) $(call asic_period,$*) $(POWER_STIMULUS) > /dev/null

# ---- Guard: the bring-up package with a C++ reference built for p8s8 must fail with both IDs printed
$(BUILD)/guard/Vtb_pans_cores: $(BUILD)/p8s6/gen/pans_pkg.sv $(RTL_CORES) $(RTL_INC) $(REF) ref/pans1_dpi.cpp $(SMOKE_TB) \
                               $(BUILD)/p8s8/defs
	$(fresh_objects)$(VERILATOR) --binary --timing --assert -j $(JOBS) -Irtl -CFLAGS "-std=c++17 -I$(CURDIR)/ref $(call defs,p8s8)" \
	  --top-module tb_pans_cores -Mdir $(@D) -o Vtb_pans_cores \
	  $< $(RTL_CORES) $(SMOKE_TB) $(CURDIR)/ref/pans1_dpi.cpp

guard: $(BUILD)/guard/Vtb_pans_cores
	@if $< > $(BUILD)/guard/run.log 2>&1; then echo "FAIL: mismatched C++ reference was not rejected"; exit 1; fi
	@if grep -q "config mismatch" $(BUILD)/guard/run.log; then echo "PASS: mismatch rejected:"; grep "config mismatch" $(BUILD)/guard/run.log; \
	 else echo "FAIL: run failed without reporting the config mismatch"; cat $(BUILD)/guard/run.log; exit 1; fi

# ==== Stream (STREAM.md) ===========================================================================================

# ---- The stream model's self-test (ref/test_stream.cpp): round trips with N in {1, 2, 8, 16} lanes, the
# interleaving orders step by step, corrupt arrays, at P = W = S = 16 the pANS 1.0 transcription, the DPI
# (ref/pans_stream_dpi.cpp, linked in) driven as a testbench drives it, the coder's scale table against its definition
# and the package's B_* and SCALE_TABLE (--pkg), the example integer scale estimator against its definition, and
# stream_tool's rows against the model (--tool). Built with the config's -D flags and -Werror. Exit status 0 iff every check passes and every coverage corner was reached.
# The model takes any build, so this runs on every config (p8s6e3 included: its package is the cores', which
# gen_pkg accepts; only --stream refuses it).
$(BUILD)/%/test_stream: ref/test_stream.cpp $(STREAM_REF) $(REF) $(BUILD)/%/defs
	$(CXX) -O2 -std=c++17 -Wall -Wextra -Werror -Iref $(call defs,$*) $< ref/pans_stream_dpi.cpp -o $@

stream-ref-%: $(BUILD)/%/test_stream $(BUILD)/%/gen/pans_pkg.sv $(BUILD)/%/stream_tool
	$< --pkg $(BUILD)/$*/gen/pans_pkg.sv --tool $(BUILD)/$*/stream_tool
	@echo "PASS: test_stream $*"

# ---- The stream tool (ref/stream_tool.cpp, whose header gives the invocation)
$(BUILD)/%/stream_tool: ref/stream_tool.cpp ref/pans_stream_ref.h $(REF) $(BUILD)/%/defs
	$(CXX) -O2 -std=c++17 -Wall -Wextra -Werror -Iref $(call defs,$*) $< -o $@

stream-tool-%: $(BUILD)/%/stream_tool ;

# ---- The units' parameters and build directories (one per parameter set). Each variable is the unit parameter of
# the same name (DEC_ / ENC_: one unit's), at the unit's default; STREAM.md §2.1 and §2.2 give the limits. Every
# target of a unit builds with all of them (lint, sims, the round trip, generic gates, ORFS, power). The directory
# tag names N_LANES, DATA_BYTES (b), FIFO_WORDS (f), OUT_DEPTH (o) and DIV_BITS (d) always; a knob added later
# appears only away from its default, as _<letter><value> (s DIV_STEP_BITS, f DIV_FIRST_BITS against
# DIV_STEP_BITS, c COUNT_WIDTH, g CLOCK_GATE), so the default builds keep the directories the ledgers name
# (dec_stream/n8_b4_f8_o7, enc_stream/n8_d2_o8).
N_LANES            ?= 8
DEC_DATA_BYTES     ?= 4
DEC_FIFO_WORDS     ?= 8
DEC_OUT_DEPTH      ?= 7
DEC_COUNT_WIDTH    ?= 32
DEC_CLOCK_GATE     ?= 1
ENC_DIV_BITS       ?= 2
ENC_DIV_STEP_BITS  ?= 1
ENC_DIV_FIRST_BITS ?= $(ENC_DIV_STEP_BITS)
ENC_OUT_DEPTH      ?= 8
ENC_COUNT_WIDTH    ?= 32
ENC_CLOCK_GATE     ?= 1
ENC_PPA_DIV_BITS   ?= 1 2 4
SEED               ?= 1
SEEDS              ?= $(SEED)
PLUSARGS           ?=

knob = $(if $(filter-out $2,$1),_$3$1)
DEC_PARAMS  := N_LANES=$(N_LANES) DATA_BYTES=$(DEC_DATA_BYTES) FIFO_WORDS=$(DEC_FIFO_WORDS) \
               OUT_DEPTH=$(DEC_OUT_DEPTH) COUNT_WIDTH=$(DEC_COUNT_WIDTH) CLOCK_GATE=$(DEC_CLOCK_GATE)
ENC_PARAMS  := N_LANES=$(N_LANES) DIV_BITS=$(ENC_DIV_BITS) DIV_STEP_BITS=$(ENC_DIV_STEP_BITS) \
               DIV_FIRST_BITS=$(ENC_DIV_FIRST_BITS) OUT_DEPTH=$(ENC_OUT_DEPTH) COUNT_WIDTH=$(ENC_COUNT_WIDTH) \
               CLOCK_GATE=$(ENC_CLOCK_GATE)
RT_PARAMS   := N_LANES=$(N_LANES) DATA_BYTES=$(DEC_DATA_BYTES) FIFO_WORDS=$(DEC_FIFO_WORDS) \
               DEC_OUT_DEPTH=$(DEC_OUT_DEPTH) DEC_COUNT_WIDTH=$(DEC_COUNT_WIDTH) DEC_CLOCK_GATE=$(DEC_CLOCK_GATE) \
               DIV_BITS=$(ENC_DIV_BITS) \
               DIV_STEP_BITS=$(ENC_DIV_STEP_BITS) DIV_FIRST_BITS=$(ENC_DIV_FIRST_BITS) \
               ENC_OUT_DEPTH=$(ENC_OUT_DEPTH) ENC_COUNT_WIDTH=$(ENC_COUNT_WIDTH) ENC_CLOCK_GATE=$(ENC_CLOCK_GATE)
DEC_KNOBS   := $(call knob,$(DEC_COUNT_WIDTH),32,c)$(call knob,$(DEC_CLOCK_GATE),1,g)
ENC_KNOBS   := $(call knob,$(ENC_DIV_STEP_BITS),1,s)$(call knob,$(ENC_DIV_FIRST_BITS),$(ENC_DIV_STEP_BITS),f)$\
               $(call knob,$(ENC_COUNT_WIDTH),32,c)$(call knob,$(ENC_CLOCK_GATE),1,g)
DEC_TAG     := n$(N_LANES)_b$(DEC_DATA_BYTES)_f$(DEC_FIFO_WORDS)_o$(DEC_OUT_DEPTH)$(DEC_KNOBS)
ENC_TAG     := n$(N_LANES)_d$(ENC_DIV_BITS)_o$(ENC_OUT_DEPTH)$(ENC_KNOBS)
DEC_DIR     := dec_stream/$(DEC_TAG)
ENC_DIR     := enc_stream/$(ENC_TAG)
RT_DIR      := stream_rt/$(DEC_TAG)_d$(ENC_DIV_BITS)_o$(ENC_OUT_DEPTH)$(ENC_KNOBS)
ENC_PPA_DIR := enc_stream/ppa_n$(N_LANES)_o$(ENC_OUT_DEPTH)$(ENC_KNOBS)

STREAM_CG  := rtl/pans_stream_cg.sv
SCALE_OF_B := rtl/pans_scale_of_b.sv
DEC_RTL    := rtl/pans_dec_core.sv $(STREAM_CG) $(SCALE_OF_B) rtl/pans_dec_stream.sv
ENC_RTL    := rtl/pans_enc_core.sv $(STREAM_CG) $(SCALE_OF_B) rtl/pans_enc_stream.sv
RT_RTL     := $(sort $(DEC_RTL) $(ENC_RTL))
DEC_INC    := $(RTL_INC) rtl/pans_dec_stream_functions.svh
ENC_INC    := $(RTL_INC) rtl/pans_enc_stream_functions.svh
STREAM_DPI := ref/pans_stream_dpi.cpp ref/pans1_dpi.cpp
DPI_DEPS   := $(STREAM_REF) $(REF) ref/pans1_dpi.cpp

# ---- The build check: gen_pkg --stream accepts the config, or every unit target stops here with its reasons. Each
# unit target runs it first, in its own recipe, and no file records it: the CI runner's parallel unit jobs of one
# config write no file in common.
stream_check = @$(BUILD)/$*/gen_pkg --stream > /dev/null || \
  { echo "FAIL: gen_pkg --stream refuses $*: no stream units for this build"; exit 1; }

stream-refuse-%: $(BUILD)/%/gen_pkg
	@if $< --stream > /dev/null; then \
	   echo "FAIL: gen_pkg --stream accepted $*, a build the stream units support (stream-refuse expects a refusal)"; \
	   exit 1; fi
	@echo "PASS: gen_pkg --stream refuses $*, as ci/matrix.toml says (stream = \"refused\")"

# ---- Lint: Verilator -Wall on each unit with its core, at the unit parameters
$(BUILD)/%/$(DEC_DIR)/lint.ok: $(BUILD)/%/gen_pkg $(BUILD)/%/gen/pans_pkg.sv $(DEC_RTL) $(DEC_INC)
	$(stream_check)
	@mkdir -p $(@D)
	$(VERILATOR) --lint-only -Wall -Irtl --top-module pans_dec_stream $(addprefix -G,$(DEC_PARAMS)) \
	  $(BUILD)/$*/gen/pans_pkg.sv $(DEC_RTL)
	touch $@

$(BUILD)/%/$(ENC_DIR)/lint.ok: $(BUILD)/%/gen_pkg $(BUILD)/%/gen/pans_pkg.sv $(ENC_RTL) $(ENC_INC)
	$(stream_check)
	@mkdir -p $(@D)
	$(VERILATOR) --lint-only -Wall -Irtl --top-module pans_enc_stream $(addprefix -G,$(ENC_PARAMS)) \
	  $(BUILD)/$*/gen/pans_pkg.sv $(ENC_RTL)
	touch $@

lint-dec-stream-%: $(BUILD)/%/$(DEC_DIR)/lint.ok ;
lint-enc-stream-%: $(BUILD)/%/$(ENC_DIR)/lint.ok ;

# ---- Simulations: the testbenches with the stream model's DPI (ref/pans_stream_dpi.cpp; ref/pans1_dpi.cpp for the
# build identity), built with the config's -D flags. stream_tb: $1 the testbench top, $2 its parameters, $3 the RTL
# and the testbench. run_seeds: the simulator $1 once per seed of SEEDS, with PLUSARGS; the first failure stops.
# Each run ends with "PASSED: <n> checks, 0 errors" or fails.
stream_tb = $(fresh_objects)mkdir -p $(@D) && $(VERILATOR) --binary --timing --assert -j $(JOBS) -Irtl $(addprefix -G,$2) \
  -CFLAGS "-std=c++17 -O2 -I$(CURDIR)/ref $(call defs,$*)" --top-module $1 -Mdir $(@D) -o $(@F) \
  $(BUILD)/$*/gen/pans_pkg.sv $3 $(addprefix $(CURDIR)/,$(STREAM_DPI))
run_seeds = @for s in $(SEEDS); do echo "$1 +SEED=$$s $(PLUSARGS)"; $1 +SEED=$$s $(PLUSARGS) || exit 1; done

$(BUILD)/%/$(DEC_DIR)/sim/Vtb_pans_dec_stream: $(BUILD)/%/gen_pkg $(BUILD)/%/gen/pans_pkg.sv $(DEC_RTL) $(DEC_INC) \
                                               tb/stream/tb_pans_dec_stream.sv $(DPI_DEPS) $(BUILD)/%/defs
	$(stream_check)
	$(call stream_tb,tb_pans_dec_stream,$(DEC_PARAMS),$(DEC_RTL) tb/stream/tb_pans_dec_stream.sv)

$(BUILD)/%/$(ENC_DIR)/sim/Vtb_pans_enc_stream: $(BUILD)/%/gen_pkg $(BUILD)/%/gen/pans_pkg.sv $(ENC_RTL) $(ENC_INC) \
                                               tb/stream/tb_pans_enc_stream.sv $(DPI_DEPS) $(BUILD)/%/defs
	$(stream_check)
	$(call stream_tb,tb_pans_enc_stream,$(ENC_PARAMS),$(ENC_RTL) tb/stream/tb_pans_enc_stream.sv)

$(BUILD)/%/$(RT_DIR)/Vtb_pans_stream_rt: $(BUILD)/%/gen_pkg $(BUILD)/%/gen/pans_pkg.sv $(DEC_RTL) $(DEC_INC) \
                                         $(ENC_RTL) $(ENC_INC) tb/stream/tb_pans_stream_rt.sv $(DPI_DEPS) \
                                         $(BUILD)/%/defs
	$(stream_check)
	$(call stream_tb,tb_pans_stream_rt,$(RT_PARAMS),$(RT_RTL) tb/stream/tb_pans_stream_rt.sv)

sim-dec-stream-%: $(BUILD)/%/$(DEC_DIR)/sim/Vtb_pans_dec_stream
	$(call run_seeds,$<)

sim-enc-stream-%: $(BUILD)/%/$(ENC_DIR)/sim/Vtb_pans_enc_stream
	$(call run_seeds,$<)

sim-stream-rt-%: $(BUILD)/%/$(RT_DIR)/Vtb_pans_stream_rt
	$(call run_seeds,$<)

# ---- Generic gates (ppa/synth_generic.sh, the pinned ABC script; SYNTH_HIER=1 adds the gates per module, each
# module synthesized on its own): pans_dec_stream at its parameters; pans_enc_stream at each DIV_BITS of
# ENC_PPA_DIV_BITS (enc_stream_d<DIV_BITS>), its other parameters as set. Per module: the unit's own logic, its
# core, the clock gate and the scale lookup (pans_scale_of_b: SCALE_TABLE).
$(BUILD)/%/$(DEC_DIR)/ppa/metrics.json: $(BUILD)/%/gen_pkg $(BUILD)/%/gen/pans_pkg.sv $(DEC_RTL) $(DEC_INC) \
                                        ppa/synth_generic.sh ppa/abc_medium.script ppa/metrics.py
	$(stream_check)
	SYNTH_HIER=1 ppa/synth_generic.sh $(@D) dec_stream pans_dec_stream $(BUILD)/$*/gen/pans_pkg.sv $(DEC_RTL) \
	  $(foreach p,$(DEC_PARAMS),-G $p)
	$(PYTHON) ppa/metrics.py $(@D) > $@.tmp 2> /dev/null && mv $@.tmp $@

$(BUILD)/%/$(ENC_PPA_DIR)/metrics.json: $(BUILD)/%/gen_pkg $(BUILD)/%/gen/pans_pkg.sv $(ENC_RTL) $(ENC_INC) \
                                        ppa/synth_generic.sh ppa/abc_medium.script ppa/metrics.py
	$(stream_check)
	@for d in $(ENC_PPA_DIV_BITS); do \
	  SYNTH_HIER=1 ppa/synth_generic.sh $(@D) enc_stream_d$$d pans_enc_stream $(BUILD)/$*/gen/pans_pkg.sv $(ENC_RTL) \
	    $(foreach p,$(filter-out DIV_BITS=%,$(ENC_PARAMS)),-G $p) -G DIV_BITS=$$d || exit 1; done
	$(PYTHON) ppa/metrics.py $(@D) > $@.tmp 2> /dev/null && mv $@.tmp $@

ppa-dec-stream-%: $(BUILD)/%/$(DEC_DIR)/ppa/metrics.json
	@$(PYTHON) ppa/metrics.py $(<D) > /dev/null

ppa-enc-stream-%: $(BUILD)/%/$(ENC_PPA_DIR)/metrics.json
	@$(PYTHON) ppa/metrics.py $(<D) > /dev/null

# ---- ASIC PPA: each unit inside its timing wrapper (ppa/pans_stream_timed.sv: a register slice on every channel)
# through ORFS at the config's target period, the unit's module kept through synthesis so that its cells keep the
# prefix u_dut/: the area split (ppa/asic/stream_area.tcl) gives the unit, the clock tree and the wrapper apart, and
# the stage report (ppa/asic/stream_stages.tcl) the worst path into each of the unit's register groups
# (ppa/asic/stream_regs.tcl), so that a change is judged on the stages it touches (rtl/PPA_DEC_STREAM.md §5.2).
STREAM_ASIC_RTL := rtl/pans_enc_core.sv rtl/pans_dec_core.sv $(STREAM_CG) $(SCALE_OF_B) rtl/pans_enc_stream.sv \
                   rtl/pans_dec_stream.sv ppa/pans_stream_timed.sv
STREAM_ASIC_DEPS = $(BUILD)/%/gen_pkg $(BUILD)/%/gen/pans_pkg.sv $(STREAM_ASIC_RTL) $(DEC_INC) $(ENC_INC) \
                   $(ASIC_FLOW) ppa/asic/stream_area.tcl ppa/asic/stream_groups.tcl ppa/asic/stream_regs.tcl \
                   ppa/asic/stream_stages.tcl ppa/asic_metrics.py

# $1 dec or enc, $2 its parameters
asic_stream = \
  $(call orfs_flow,$(@D),pans_$1_stream_timed,$(BUILD)/$*/gen/pans_pkg.sv $(STREAM_ASIC_RTL),$2,pans_$1_stream,$*) && \
  $(ORFS_RUN) env RESULTS=/work/$(@D)/results/$(ASIC_PLATFORM)/pans_$1_stream_timed/base \
    $(OPENROAD) /work/ppa/asic/stream_area.tcl > $(@D)/area.rpt 2>&1 || { tail $(@D)/area.rpt; exit 1; }; \
  $(ORFS_RUN) env UNIT=$1 LIBERTY="$(ASIC_LIBERTY)" \
    RESULTS=/work/$(@D)/results/$(ASIC_PLATFORM)/pans_$1_stream_timed/base \
    $(OPENROAD) /work/ppa/asic/stream_stages.tcl > $(@D)/stages.rpt 2>&1 || { tail $(@D)/stages.rpt; exit 1; }; \
  $(PYTHON) ppa/asic_metrics.py --stream $1 $(@D) > $@.tmp 2> /dev/null && mv $@.tmp $@

$(BUILD)/%/$(DEC_DIR)/$(ASIC_DIR)/metrics.json: $(STREAM_ASIC_DEPS)
	$(stream_check)
	@$(call asic_stream,dec,$(DEC_PARAMS))

$(BUILD)/%/$(ENC_DIR)/$(ASIC_DIR)/metrics.json: $(STREAM_ASIC_DEPS)
	$(stream_check)
	@$(call asic_stream,enc,$(ENC_PARAMS))

asic-dec-stream-%: $(BUILD)/%/$(DEC_DIR)/$(ASIC_DIR)/metrics.json
	@$(PYTHON) ppa/asic_metrics.py --stream dec $(<D) > /dev/null

asic-enc-stream-%: $(BUILD)/%/$(ENC_DIR)/$(ASIC_DIR)/metrics.json
	@$(PYTHON) ppa/asic_metrics.py --stream enc $(<D) > /dev/null

asic-stream-%: asic-dec-stream-% asic-enc-stream-% ;

# ---- Power: a zero-delay gate-level simulation of each routed unit (ppa/power_tb_stream.sv, the stream model's DPI):
# first +CHECK, jobs that reach the corners with random stalls, then the workloads (POWER_JOBS EMA jobs of POWER_N
# symbols, every consumer always ready; the decoder's at each EMA delay of POWER_DEC_DELAYS: 0, its scale one cycle
# after the symbol it needs; 1, the consumer a symbol ahead), each checked against the model; then OpenSTA with each
# workload's activity and the routed parasitics (ppa/asic/power_stream.tcl: the total, the unit / clock tree /
# wrapper split, and the unit's cells by register group of ppa/asic/stream_regs.tcl). POWER_P_IDLE (percent, default
# 0) holds the producer back on that share of the cycles in the workloads: the encoder's sym_in, the decoder's
# scale_in; away from 0 the results go to power_idle<P>/ beside power/ (with ASIC_PLATFORM's suffix: power_asap7/).
# POWER_STIMULUS (above) static or random replaces the EMA jobs by LIST jobs of the same length
# (ppa/power_tb_stream.sv), whose scales are known ahead: the decoder runs them once, as at EMA delay 1.
POWER_JOBS       ?= 2
POWER_N          ?= 4096
POWER_DEC_DELAYS ?= 0 1
POWER_P_IDLE     ?= 0
POWER_DIR        := power$(call knob,$(POWER_P_IDLE),0,idle)$(POWER_SUFFIX)$(ASIC_SUFFIX)

# $1 dec or enc, $2 its parameters, $3 the EMA delays of its workloads
power_stream = mkdir -p $(@D) && \
  $(call verilate_netlist,power_tb_$1_stream,$2,$(BUILD)/$*/gen/pans_pkg.sv $(ASIC_CELLS) \
    $(<D)/results/$(ASIC_PLATFORM)/pans_$1_stream_timed/base/6_final.v ppa/power_tb_stream.sv \
    $(addprefix $(CURDIR)/,$(STREAM_DPI)) -CFLAGS "-std=c++17 -O2 -I$(CURDIR)/ref $(call defs,$*)",$1) && \
  { $(@D)/obj_$1/Vpower_tb +CHECK > $(@D)/check.log 2>&1 || { tail $(@D)/check.log; exit 1; }; } && \
  rm -f $(@D)/sim_d*.log $(@D)/power_d*.rpt && \
  for d in $3; do \
    $(@D)/obj_$1/Vpower_tb +JOBS=$(POWER_JOBS) +N=$(POWER_N) +DELAY=$$d +P_IDLE=$(POWER_P_IDLE) \
      +STIMULUS=$(POWER_STIMULUS) +VCD=$(@D)/$1.vcd > $(@D)/sim_d$$d.log 2>&1 || { tail $(@D)/sim_d$$d.log; exit 1; }; \
    $(ORFS_RUN) env UNIT=$1 LIBERTY="$(ASIC_LIBERTY)" \
      RESULTS=/work/$(<D)/results/$(ASIC_PLATFORM)/pans_$1_stream_timed/base \
      VCD=/work/$(@D)/$1.vcd VCD_SCOPE=power_tb_$1_stream/dut $(OPENROAD) /work/ppa/asic/power_stream.tcl \
      > $(@D)/power_d$$d.rpt 2>&1 || { tail $(@D)/power_d$$d.rpt; exit 1; }; \
    rm -f $(@D)/$1.vcd; \
  done && \
  $(PYTHON) ppa/power_metrics.py --stream $1 $(@D) $(call asic_period,$*) $(POWER_STIMULUS) > $@.tmp 2> /dev/null && \
  mv $@.tmp $@

POWER_STREAM_DEPS := $(ASIC_CELLS) ppa/power_tb_stream.sv $(DPI_DEPS) ppa/asic/power.tcl \
                     ppa/asic/power_stream.tcl ppa/asic/stream_groups.tcl ppa/asic/stream_regs.tcl \
                     ppa/power_metrics.py

$(BUILD)/%/$(DEC_DIR)/$(POWER_DIR)/metrics.json: $(BUILD)/%/$(DEC_DIR)/$(ASIC_DIR)/metrics.json $(POWER_STREAM_DEPS)
	@$(call power_stream,dec,$(DEC_PARAMS),$(if $(POWER_SUFFIX),1,$(POWER_DEC_DELAYS)))

$(BUILD)/%/$(ENC_DIR)/$(POWER_DIR)/metrics.json: $(BUILD)/%/$(ENC_DIR)/$(ASIC_DIR)/metrics.json $(POWER_STREAM_DEPS)
	@$(call power_stream,enc,$(ENC_PARAMS),0)

power-dec-stream-%: $(BUILD)/%/$(DEC_DIR)/$(POWER_DIR)/metrics.json
	@$(PYTHON) ppa/power_metrics.py --stream dec $(<D) $(call asic_period,$*) $(POWER_STIMULUS) > /dev/null

power-enc-stream-%: $(BUILD)/%/$(ENC_DIR)/$(POWER_DIR)/metrics.json
	@$(PYTHON) ppa/power_metrics.py --stream enc $(<D) $(call asic_period,$*) $(POWER_STIMULUS) > /dev/null

power-stream-%: power-dec-stream-% power-enc-stream-% ;

# ---- Both units of a config
lint-stream-%: lint-dec-stream-% lint-enc-stream-% ;
sim-stream-%: sim-dec-stream-% sim-enc-stream-% sim-stream-rt-% ;
ppa-stream-%: ppa-dec-stream-% ppa-enc-stream-% ;

# ==== Every config =================================================================================================
pkg: $(addprefix pkg-,$(CONFIGS))
lint: $(addprefix lint-,$(CONFIGS))
refuse: $(addprefix refuse-,$(REFUSED))
inverse: $(addprefix inverse-,$(CONFIGS))
stream-ref: $(addprefix stream-ref-,$(CONFIGS))
stream-refuse: $(addprefix stream-refuse-,$(STREAM_REFUSED))
sim: sim-$(CFG)

# ---- CI
ci:
	$(PYTHON) ci/run.py --tier smoke

ci-hooks:
	ln -sf ../../ci/hooks/post-commit .git/hooks/post-commit
	@echo "installed .git/hooks/post-commit -> ci/hooks/post-commit"

matrix:
	@$(PYTHON) ci/matrix.py show

# ---- Rate sweep (SPEC.md §5): the build and its unit variants against the exact-math coder at the config's P, SW
$(BUILD)/%/sweep: ref/sweep.cpp $(REF) $(BUILD)/%/defs
	@mkdir -p $(@D)
	$(CXX) -O2 -std=c++17 -fopenmp -Wall -Wextra $(call defs,$*) $< -o $@

sweep-%: $(BUILD)/%/sweep
	mkdir -p ref/results && OMP_NUM_THREADS=16 $< ref/results/sweep_$*.csv

sweep: sweep-p16s16

clean:
	rm -rf $(BUILD)
