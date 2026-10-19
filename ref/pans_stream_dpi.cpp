// SPDX-License-Identifier: Apache-2.0
// pans_stream_dpi.cpp: the stream model of the hardware build as DPI-C functions, for the stream unit testbenches.
// Mirrors ref/pans_stream_ref.h. Specification: STREAM.md §1. ref/test_stream.cpp drives it as a testbench does.
// Link it with ref/pans1_dpi.cpp, which provides pans_ref_config_id.
// SV int maps to int, longint to long long, an output to a pointer. A job is a handle, an int >= 0.
// A bad handle, index or call order aborts with a message on stderr. The return codes below are the exceptions.
//
// Jobs
//   int  pans_stream_job_new(int seed, int n_lanes, longint n, int kind, int delay)
//          A random job, built. kind is 0 CONST, 1 EMA, 2 LIST or 3 INT (STREAM.md §1.7). Returns the handle or -1.
//   int  pans_stream_job_begin(int n_lanes, longint n, int kind, int delay)
//          A directed job with every symbol and b at 0. Set its symbols, then build it.
//   void pans_stream_job_set(int job, longint i, int symbol, longint b)
//          Symbol i and its b. CONST takes the b of symbol 0. EMA and INT ignore b.
//   int  pans_stream_job_build(int job)
//          Encodes and decodes a begun job. Returns 0, or -1 for a symbol or b out of range.
//   int  pans_stream_job_bound(int seed, int n_lanes, output longint index0, output longint index1)
//          A built LIST job whose lane 0 and lane 1 escapes read the raw push's bound B and B - 1 (STREAM.md §1.9).
//          index0 and index1 are their global indices, -1 where absent. Returns the handle or -1.
//   void pans_stream_job_corrupt(int job, int how, longint pos, int value)
//          how 0 XORs byte pos with value, 1 keeps the first pos bytes, 2 appends the byte value.
//          It recomputes the decoder's expectations and resets the scale source. The encoder's stay the clean job's.
//   void pans_stream_job_free(int job)
//
// The encoder unit
//   void    pans_stream_job_push(int job, longint k, output int lane, output int symbol, output longint b)
//                                                                 sym_in beat k, global index n - 1 - k
//   void    pans_stream_job_enc_step(int job, longint k, output int lane, output longint t, output int symbol,
//                                    output int start, output int freq, output int escape, output int words,
//                                    output longint state, output longint b, output int scale_exp,
//                                    output int scale_man)
//          Push k, as EncStep in pans_stream_ref.h.
//   longint pans_stream_job_n_words(int job)
//   longint pans_stream_job_word(int job, longint k)              word k in emission order
//   longint pans_stream_job_state(int job, int lane)              the lane's final state
//   longint pans_stream_job_n_escapes(int job)
//
// The decoder unit
//   longint pans_stream_job_n(int job)                            symbol count, job_in n_sym
//   longint pans_stream_job_n_bytes(int job)                      array length, job_in n_bytes
//   int     pans_stream_job_byte(int job, longint k)              byte k of the array
//   void    pans_stream_job_step(int job, longint i, output int lane, output longint t, output int slot,
//                                output int start, output int freq, output int escape, output int raw,
//                                output int symbol, output longint b, output int scale_exp, output int scale_man,
//                                output int words, output longint state, output int errors)
//          Decode step i, as DecStep in pans_stream_ref.h.
//   void    pans_stream_job_done(int job, output int ok, output longint n_escapes, output longint words_consumed,
//                                output int err_words, output int err_header)
//
// The consumer, the testbench's scale source fed by the RTL's sym_out
//   int  pans_stream_scale(int job, int lane, longint t, output longint b)
//          Returns 1 with the b of (lane, t), 0 if it is not computable yet, -1 on a protocol error.
//   int  pans_stream_observe(int job, int lane, longint t, int symbol)
//          Feeds one sym_out symbol. Returns 0, or -1 if t is not the lane's next position.
//   void pans_stream_src_reset(int job)        a fresh source, to decode the same job again
//
// The build's stream widths, to compare with pans_pkg
//   void pans_stream_widths(output int word_width, output int state_width, output int state_bytes)
#include <cstdio>
#include <memory>
#include <vector>

#include "pans1_config.h"
#include "pans_stream_ref.h"

using namespace pans1hw;

namespace {

const Config& config() {
    static const Config c = build_config();
    return c;
}

constexpr int KIND_MAX = int(SourceKind::INT);

enum Corruption : int { CORRUPT_XOR = 0, CORRUPT_TRUNCATE = 1, CORRUPT_APPEND = 2 };

struct DpiJob {
    StreamJob job;
    bool built = false;
    StreamOut enc;
    std::vector<uint8_t> bytes;  // the decoder's array (the encoder's, or corrupted)
    DecodeOut dec;
    // The testbench's scale source and its per-lane protocol counters.
    std::unique_ptr<ScaleSource> tb_src;
    std::vector<i64> next_scale, observed;
};

std::vector<std::unique_ptr<DpiJob>>& jobs() {
    static std::vector<std::unique_ptr<DpiJob>> v;
    return v;
}

[[noreturn]] void dpi_fail(const char* fn, const char* what) {
    std::fprintf(stderr, "pans_stream_dpi: %s: %s\n", fn, what);
    std::abort();
}

DpiJob& job_at(int h, const char* fn, bool need_built = true) {
    if (h < 0 || h >= (int)jobs().size() || !jobs()[h]) dpi_fail(fn, "no such job handle");
    DpiJob& j = *jobs()[h];
    if (need_built && !j.built) dpi_fail(fn, "the job is not built (pans_stream_job_build)");
    return j;
}

int add_job(std::unique_ptr<DpiJob> j) {
    std::vector<std::unique_ptr<DpiJob>>& v = jobs();
    for (size_t h = 0; h < v.size(); ++h)
        if (!v[h]) {
            v[h] = std::move(j);
            return (int)h;
        }
    v.push_back(std::move(j));
    return (int)v.size() - 1;
}

bool args_ok(const char* fn, int n_lanes, long long n, int kind, int delay) {
    const bool ok = n_lanes >= 1 && n >= 0 && kind >= 0 && kind <= KIND_MAX && delay >= 0;
    if (!ok)
        std::fprintf(stderr, "pans_stream_dpi: %s: bad arguments n_lanes %d, n %lld, kind %d, delay %d\n", fn,
                     n_lanes, n, kind, delay);
    return ok;
}

void reset_tb_source(DpiJob& j) {
    j.tb_src = job_source(config(), j.job);
    j.tb_src->begin(j.job.n_lanes);
    j.next_scale.assign(j.job.n_lanes, 0);
    j.observed.assign(j.job.n_lanes, 0);
}

void decode_expected(DpiJob& j) {
    j.dec = job_decode(config(), j.job, j.bytes);
    reset_tb_source(j);
}

int build(DpiJob& j, const char* fn) {
    const Config& c = config();
    if (j.job.kind == SourceKind::CONST && !j.job.scales.empty()) j.job.const_scale = j.job.scales[0];
    job_replay(c, j.job);
    for (size_t i = 0; i < j.job.symbols.size(); ++i)
        if (!symbol_valid(c, j.job.symbols[i]) || !b_valid(c, j.job.scales[i])) {
            std::fprintf(stderr, "pans_stream_dpi: %s: symbol %zu: symbol %lld or b %lld out of range\n", fn, i,
                         (long long)j.job.symbols[i], (long long)j.job.scales[i]);
            return -1;
        }
    j.enc = encode_stream(c, j.job.n_lanes, j.job.symbols, j.job.scales);
    j.bytes = j.enc.bytes;
    j.built = true;
    decode_expected(j);
    return 0;
}

}  // namespace

extern "C" {

int pans_stream_job_new(int seed, int n_lanes, long long n, int kind, int delay) {
    if (!args_ok("pans_stream_job_new", n_lanes, n, kind, delay)) return -1;
    auto j = std::make_unique<DpiJob>();
    j->job = JobGen(config()).make((u64)(unsigned)seed, n_lanes, n, SourceKind(kind), delay);
    if (build(*j, "pans_stream_job_new") != 0) return -1;
    return add_job(std::move(j));
}

int pans_stream_job_begin(int n_lanes, long long n, int kind, int delay) {
    if (!args_ok("pans_stream_job_begin", n_lanes, n, kind, delay)) return -1;
    auto j = std::make_unique<DpiJob>();
    j->job.n_lanes = n_lanes;
    j->job.kind = SourceKind(kind);
    j->job.delay = delay;
    j->job.symbols.assign(n, 0);
    j->job.scales.assign(n, 0);
    return add_job(std::move(j));
}

void pans_stream_job_set(int job, long long i, int symbol, long long b) {
    DpiJob& j = job_at(job, "pans_stream_job_set", false);
    if (i < 0 || i >= (long long)j.job.symbols.size()) dpi_fail("pans_stream_job_set", "index beyond the job");
    j.job.symbols[i] = symbol;
    j.job.scales[i] = (ScaleB)b;
    j.built = false;
}

int pans_stream_job_build(int job) { return build(job_at(job, "pans_stream_job_build", false), "pans_stream_job_build"); }

int pans_stream_job_bound(int seed, int n_lanes, long long* index0, long long* index1) {
    *index0 = -1;
    *index1 = -1;
    if (!args_ok("pans_stream_job_bound", n_lanes, 0, int(SourceKind::LIST), 0)) return -1;
    auto j = std::make_unique<DpiJob>();
    JobGen::BoundJob bj = JobGen(config()).bound_job((u64)(unsigned)seed, n_lanes);
    j->job = std::move(bj.job);
    if (build(*j, "pans_stream_job_bound") != 0) return -1;
    *index0 = bj.esc_index[0];
    *index1 = bj.esc_index[1];
    return add_job(std::move(j));
}

void pans_stream_job_corrupt(int job, int how, long long pos, int value) {
    DpiJob& j = job_at(job, "pans_stream_job_corrupt");
    const long long size = (long long)j.bytes.size();
    if (how == CORRUPT_XOR && pos >= 0 && pos < size) {
        j.bytes[pos] ^= uint8_t(value);
    } else if (how == CORRUPT_TRUNCATE && pos >= 0 && pos <= size) {
        j.bytes.resize(pos);
    } else if (how == CORRUPT_APPEND) {
        j.bytes.push_back(uint8_t(value));
    } else {
        dpi_fail("pans_stream_job_corrupt", "unknown corruption or position beyond the array");
    }
    decode_expected(j);
}

void pans_stream_job_free(int job) {
    job_at(job, "pans_stream_job_free", false);
    jobs()[job].reset();
}

// ---- The encoder unit

void pans_stream_job_push(int job, long long k, int* lane, int* symbol, long long* b) {
    const DpiJob& j = job_at(job, "pans_stream_job_push");
    if (k < 0 || k >= (long long)j.enc.trace.size()) dpi_fail("pans_stream_job_push", "beat beyond the job");
    const EncStep& s = j.enc.trace[k];
    *lane = s.lane;
    *symbol = (int)s.symbol;
    *b = (long long)s.b;
}

void pans_stream_job_enc_step(int job, long long k, int* lane, long long* t, int* symbol, int* start, int* freq,
                              int* escape, int* words, long long* state, long long* b, int* scale_exp,
                              int* scale_man) {
    const DpiJob& j = job_at(job, "pans_stream_job_enc_step");
    if (k < 0 || k >= (long long)j.enc.trace.size()) dpi_fail("pans_stream_job_enc_step", "push beyond the job");
    const EncStep& s = j.enc.trace[k];
    *lane = s.lane;
    *t = s.t;
    *symbol = (int)s.symbol;
    *start = (int)s.start;
    *freq = (int)s.freq;
    *escape = s.escape;
    *words = s.words;
    *state = (long long)s.state;
    *b = (long long)s.b;
    *scale_exp = (int)s.scale.exp;
    *scale_man = (int)s.scale.man;
}

long long pans_stream_job_n_words(int job) { return (long long)job_at(job, "pans_stream_job_n_words").enc.words.size(); }

long long pans_stream_job_word(int job, long long k) {
    const DpiJob& j = job_at(job, "pans_stream_job_word");
    if (k < 0 || k >= (long long)j.enc.words.size()) dpi_fail("pans_stream_job_word", "word beyond the stream");
    return (long long)j.enc.words[k];
}

long long pans_stream_job_state(int job, int lane) {
    const DpiJob& j = job_at(job, "pans_stream_job_state");
    if (lane < 0 || lane >= j.job.n_lanes) dpi_fail("pans_stream_job_state", "lane out of range");
    return (long long)j.enc.states[lane];
}

long long pans_stream_job_n_escapes(int job) { return job_at(job, "pans_stream_job_n_escapes").enc.n_escapes; }

// ---- The decoder unit

long long pans_stream_job_n(int job) { return (long long)job_at(job, "pans_stream_job_n").job.symbols.size(); }

long long pans_stream_job_n_bytes(int job) { return (long long)job_at(job, "pans_stream_job_n_bytes").bytes.size(); }

int pans_stream_job_byte(int job, long long k) {
    const DpiJob& j = job_at(job, "pans_stream_job_byte");
    if (k < 0 || k >= (long long)j.bytes.size()) dpi_fail("pans_stream_job_byte", "byte beyond the array");
    return j.bytes[k];
}

void pans_stream_job_step(int job, long long i, int* lane, long long* t, int* slot, int* start, int* freq,
                          int* escape, int* raw, int* symbol, long long* b, int* scale_exp, int* scale_man,
                          int* words, long long* state, int* errors) {
    const DpiJob& j = job_at(job, "pans_stream_job_step");
    if (i < 0 || i >= (long long)j.dec.trace.size()) dpi_fail("pans_stream_job_step", "step beyond the job");
    const DecStep& s = j.dec.trace[i];
    *lane = s.lane;
    *t = s.t;
    *slot = (int)s.slot;
    *start = (int)s.start;
    *freq = (int)s.freq;
    *escape = s.escape;
    *raw = (int)s.raw;
    *symbol = (int)s.symbol;
    *b = (long long)s.b;
    *scale_exp = (int)s.scale.exp;
    *scale_man = (int)s.scale.man;
    *words = s.words;
    *state = (long long)s.state;
    *errors = (int)s.errors;
}

void pans_stream_job_done(int job, int* ok, long long* n_escapes, long long* words_consumed, int* err_words,
                          int* err_header) {
    const DecodeOut& d = job_at(job, "pans_stream_job_done").dec;
    *ok = d.ok;
    *n_escapes = d.n_escapes;
    *words_consumed = d.words_consumed;
    *err_words = d.err_words;
    *err_header = d.err_header;
}

// ---- The consumer

int pans_stream_scale(int job, int lane, long long t, long long* b) {
    DpiJob& j = job_at(job, "pans_stream_scale");
    const i64 n = (i64)j.job.symbols.size();
    if (lane < 0 || lane >= j.job.n_lanes || t != j.next_scale[lane] || t >= lane_symbols(j.job.n_lanes, n, lane))
        return -1;
    if (j.tb_src->history_needed(t) > j.observed[lane]) return 0;
    *b = (long long)j.tb_src->scale(lane, t);
    ++j.next_scale[lane];
    return 1;
}

int pans_stream_observe(int job, int lane, long long t, int symbol) {
    DpiJob& j = job_at(job, "pans_stream_observe");
    const i64 n = (i64)j.job.symbols.size();
    if (lane < 0 || lane >= j.job.n_lanes || t != j.observed[lane] || t >= lane_symbols(j.job.n_lanes, n, lane))
        return -1;
    j.tb_src->observe(lane, t, symbol);
    ++j.observed[lane];
    return 0;
}

void pans_stream_src_reset(int job) { reset_tb_source(job_at(job, "pans_stream_src_reset")); }

void pans_stream_widths(int* word_width, int* state_width, int* state_bytes) {
    *word_width = config().word_width();
    *state_width = config().state_width();
    *state_bytes = config().state_bytes();
}

}  // extern "C"
