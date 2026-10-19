// SPDX-License-Identifier: Apache-2.0
// test_stream: the self-test of the stream model and of its DPI, on the C++ reference of the build.
// Specification: STREAM.md §1.9. The numbered sections below are its parts.
// Every coverage corner must be reached. Exit status 0 iff every check passes.
//
//   make stream-ref-<cfg>   builds it into build/<cfg>/ for a config of ci/matrix.toml and runs it, with the
//                           config's package and stream_tool (every config: make stream-ref); by hand:
//   g++ -O2 -std=c++17 -Wall -Wextra -Werror -Iref <config -D flags> ref/test_stream.cpp ref/pans_stream_dpi.cpp
//       -o test_stream
//   test_stream [--pkg <pans_pkg.sv>] [--tool <stream_tool>] [seed]   (section 6's package check and 7 need them)
#include <sys/stat.h>
#include <sys/wait.h>

#include <algorithm>
#include <cmath>
#include <cstdarg>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <deque>
#include <functional>
#include <memory>
#include <regex>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "pans1_config.h"
#include "pans_stream_ref.h"

using namespace pans1hw;

// The DPI functions (ref/pans_stream_dpi.cpp), as a testbench imports them.
extern "C" {
int pans_stream_job_new(int seed, int n_lanes, long long n, int kind, int delay);
int pans_stream_job_begin(int n_lanes, long long n, int kind, int delay);
void pans_stream_job_set(int job, long long i, int symbol, long long b);
int pans_stream_job_build(int job);
int pans_stream_job_bound(int seed, int n_lanes, long long* index0, long long* index1);
void pans_stream_job_corrupt(int job, int how, long long pos, int value);
void pans_stream_job_free(int job);
void pans_stream_job_push(int job, long long k, int* lane, int* symbol, long long* b);
void pans_stream_job_enc_step(int job, long long k, int* lane, long long* t, int* symbol, int* start, int* freq,
                              int* escape, int* words, long long* state, long long* b, int* scale_exp,
                              int* scale_man);
long long pans_stream_job_n_words(int job);
long long pans_stream_job_word(int job, long long k);
long long pans_stream_job_state(int job, int lane);
long long pans_stream_job_n_escapes(int job);
long long pans_stream_job_n(int job);
long long pans_stream_job_n_bytes(int job);
int pans_stream_job_byte(int job, long long k);
void pans_stream_job_step(int job, long long i, int* lane, long long* t, int* slot, int* start, int* freq, int* escape,
                          int* raw, int* symbol, long long* b, int* scale_exp, int* scale_man, int* words,
                          long long* state, int* errors);
void pans_stream_job_done(int job, int* ok, long long* n_escapes, long long* words_consumed, int* err_words,
                          int* err_header);
int pans_stream_scale(int job, int lane, long long t, long long* b);
int pans_stream_observe(int job, int lane, long long t, int symbol);
void pans_stream_src_reset(int job);
void pans_stream_widths(int* word_width, int* state_width, int* state_bytes);
}

namespace {

constexpr int FAIL_PRINT_MAX = 12;   // failures printed (all are counted)
constexpr i64 N_MAX_RANDOM = 100000;  // the largest job
const int LANE_COUNTS[] = {1, 2, 8, 16};

// ---- Checks and coverage

long g_checks = 0, g_errors = 0;

__attribute__((format(printf, 2, 3))) bool check(bool ok, const char* fmt, ...) {
    ++g_checks;
    if (!ok && ++g_errors <= FAIL_PRINT_MAX) {
        char buf[512];
        va_list ap;
        va_start(ap, fmt);
        std::vsnprintf(buf, sizeof buf, fmt, ap);
        va_end(ap);
        std::printf("FAIL: %s\n", buf);
    }
    return ok;
}

struct Coverage {
    long symbols = 0, escapes = 0, f_one = 0, f_max = 0, b_zero = 0, b_top = 0, empty_lanes = 0;
    std::vector<long> index_hits;  // steps per scale table index (the build's)
    long enc_words[3] = {0, 0, 0}, dec_words[3] = {0, 0, 0};  // steps by the words they move
    long esc_words[3] = {0, 0, 0};                             // escaped symbols by the words they move
    long corrupt = 0, collisions = 0, err_words = 0, err_header = 0, end_only = 0;
    long raw_below = 0, raw_at = 0;  // raw pushes from x = B - 1 and x = B (single states and streams)
    long bound_hi = 0, bound_lo = 0;  // stream escapes reading exactly B and B - 1
    long raw_cross = 0;               // escapes whose raw value crosses a word boundary (round trips)
    long est_scales = 0, est_floor = 0, est_up = 0, est_down = 0;  // the integer estimator's
    long zero_fill = 0;  // steps whose table index is zero-filled, 0 < b < 2^B_K
    long outside = 0;  // inputs outside a symmetric alphabet, each escaped
} g_cov;

u64 mix_seed(u64 base, u64 k) { return StreamRng(base * 0x9e3779b97f4a7c15ull + k).next(); }

int log2_escape(const Config& c) {  // ceil(log2 E)
    int e = 0;
    while ((i64(1) << e) < c.escape_slots) ++e;
    return e;
}

bool same_scale(Scale a, Scale b) { return a.exp == b.exp && a.man == b.man; }

bool same_step(const DecStep& a, const DecStep& b) {
    return a.lane == b.lane && a.t == b.t && a.b == b.b && same_scale(a.scale, b.scale) && a.slot == b.slot &&
           a.start == b.start &&
           a.freq == b.freq && a.escape == b.escape && a.raw == b.raw && a.symbol == b.symbol && a.words == b.words &&
           a.state == b.state && a.errors == b.errors;
}

u64 get_le(const std::vector<uint8_t>& b, i64 at, int n_bytes) {
    u64 v = 0;
    for (int k = 0; k < n_bytes; ++k) v |= u64(b[at + k]) << (BYTE_BITS * k);
    return v;
}

const char* kind_name(SourceKind k) {
    switch (k) {
    case SourceKind::CONST: return "const";
    case SourceKind::EMA: return "ema";
    case SourceKind::INT: return "int";
    case SourceKind::LIST: break;
    }
    return "list";
}

// The first step of a decode's trace whose reads reach array word w, n if none does.
i64 step_reading_word(const DecodeOut& d, i64 w) {
    i64 pos = 0;
    for (i64 i = 0; i < (i64)d.trace.size(); ++i) {
        pos += d.trace[i].words;
        if (pos > w) return i;
    }
    return (i64)d.trace.size();
}

// ---- 1, 2: round trips
// Round trips at N in {1, 2, 8, 16} with every scale source, then directed jobs.

// m as b, written out plainly as floor(m 2^(B_FRAC - frac)).
i64 plain_b_of_m(const Config& c, i64 m, int frac) {
    return (i64)std::floor(double(m) * std::exp2(c.b_frac - frac));
}

// The EMA source's scale inputs, written out plainly from its definition (STREAM.md §1.7).
bool ema_as_defined(const Config& c, const StreamJob& j) {
    using E = EmaScaleSource;
    const int N = j.n_lanes;
    const i64 n = (i64)j.symbols.size();
    const auto m_of = [](double sigma) { return (i64)std::llround(sigma / std::sqrt(2.0) * std::exp2(E::FRAC)); };
    const i64 m_floor = m_of(E::SIGMA_FLOOR);
    bool ok = true;
    for (int l = 0; l < N; ++l) {
        i64 m = m_of(E::SIGMA0), folded = 0;
        for (i64 t = 0, i = l; i < n; ++t, i += N) {
            for (; folded < t - j.delay; ++folded) {
                const i64 d = std::abs(j.symbols[folded * N + l]) * (i64(1) << E::FRAC) - m;
                m += (i64)std::floor(double(d) / double(i64(1) << E::SHIFT));
            }
            ok = ok && j.scales[i] == plain_b_of_m(c, std::max(m, m_floor), E::FRAC);
        }
    }
    return ok;
}

// The integer estimator's update in real arithmetic, exact in a double below 2^53 (STREAM.md §1.7).
namespace plain_est {

i64 update(i64 m, i64 x, int frac, int shift) {
    return m + (i64)std::floor((double(std::abs(x)) * std::exp2(frac) - double(m)) / std::exp2(shift));
}

}  // namespace plain_est

// The coder's scale table in real arithmetic, every value exact in a double (SPEC.md §1).
namespace plain_table {

// The index j = lead 2^K + below, lead the largest l with 2^l <= b, below = floor(b 2^K / 2^lead) - 2^K.
i64 index(i64 b, int k) {
    if (b == 0) return 0;
    int lead = 0;
    while (std::exp2(lead + 1) <= double(b)) ++lead;
    const i64 below = (i64)std::floor(double(b) * std::exp2(k) / std::exp2(lead)) - (i64)std::exp2(k);
    return i64(lead) * (i64)std::exp2(k) + below;
}

// Bucket j's midpoint sqrt(lo hi) / 2^B_FRAC, 0 for bucket 0.
double mid(i64 j, int frac, int k) {
    if (j == 0) return 0.0;
    const i64 lead = j / (i64)std::exp2(k), below = j % (i64)std::exp2(k);
    const double w = std::exp2(double(lead - k)), lo = (std::exp2(k) + double(below)) * w, hi = lo + w;
    return std::sqrt(lo * hi) / std::exp2(frac);
}

// Entry j, scale_from_b of the bucket's midpoint with a special code replaced by the nearest finite one.
Scale table_scale(const Config& c, i64 j) {
    const Scale s = scale_from_b(c, mid(j, c.b_frac, c.b_k));
    if (s.exp == 0) return Scale{1, 0};
    if (s.exp == c.scale_exp_inf()) return Scale{c.scale_exp_inf() - 1, (1u << c.scale_man_width) - 1};
    return s;
}

// The slope k of a normal code times 2^(BIAS + 3), (8 + man) 2^exp.
double slope_units(const Config& c, Scale s) {
    return std::ldexp(double((1u << c.scale_man_width) + s.man), int(s.exp));
}

}  // namespace plain_table

// The estimator a job of kind INT runs.
EstParams job_est(const Config& c, const StreamJob& j) {
    return j.est_custom ? j.est : IntScaleSource::example(c, j.delay);
}

// The INT scale inputs, and a fresh IntScaleSource's m, against the plain definition at every step.
bool int_as_defined(const Config& c, const StreamJob& j) {
    const EstParams p = job_est(c, j);
    const int N = j.n_lanes;
    const i64 n = (i64)j.symbols.size(), bound = i64(1) << IntScaleSource::m_width(c, p.frac);
    IntScaleSource src(c, p);
    src.begin(N);
    std::vector<i64> m(N, p.m0), folded(N, 0);
    bool ok = true;
    for (i64 i = 0; i < n; ++i) {
        const LanePos q = lane_pos(N, i);
        i64& ml = m[q.lane];
        for (; folded[q.lane] < q.t - p.delay; ++folded[q.lane]) {
            const i64 before = ml;
            ml = plain_est::update(ml, j.symbols[folded[q.lane] * N + q.lane], p.frac, p.shift);
            ok = ok && ml >= 0 && ml < bound;
            g_cov.est_up += ml > before;
            g_cov.est_down += ml < before;
        }
        const i64 want = plain_b_of_m(c, std::max(ml, p.m_floor), p.frac);
        const ScaleB got = src.scale(q.lane, q.t);
        ok = ok && src.m(q.lane) == ml && got == want && j.scales[i] == want && b_valid(c, want);
        ++g_cov.est_scales;
        g_cov.est_floor += ml < p.m_floor;
        src.observe(q.lane, q.t, j.symbols[i]);
    }
    return ok;
}

// Where an escape's raw value goes when its escape push emits nothing (STREAM.md §1.4).
// The lane's next emitted word must hold its low bits. E not a power of two is not checked.
void raw_layout(const Config& c, const StreamOut& e, const std::vector<i64>& emitted_before, int N, i64 i, u64 x0,
                const std::string& at) {
    const int P = c.precision, W = c.word_width(), S = c.symbol_width;
    const i64 E = c.escape_slots, n = (i64)e.trace.size();
    if ((E & (E - 1)) != 0 || i - N < 0) return;
    int ebits = 0;
    while ((i64(1) << ebits) < E) ++ebits;
    const EncStep& es = e.trace[n - 1 - i];
    const i64 v = raw_of(c, es.symbol);
    u64 x = x0;
    std::vector<u64> w;
    stream_push_raw(c, x, w, v);
    const size_t raw_words = w.size();
    stream_push(c, x, w, c.coded_slots(), E);
    check(x == es.state, "%s: escape %lld: its pushes from the lane's state give %llx, the trace %llx", at.c_str(),
          (long long)i, (unsigned long long)x, (unsigned long long)es.state);
    if (w.size() != raw_words) return;  // the escape push emitted: v went out whole in its word
    const EncStep& nx = e.trace[n - 1 - (i - N)];
    const bool takes_low = nx.escape ? x >= raw_push_bound(c) : nx.words == 1;
    if (!takes_low) return;
    const u64 word = e.words[(size_t)emitted_before[i - N]];
    bool ok = word == (x & word_mask(c)) && (word & (u64)(E - 1)) == (u64)(v & (E - 1));
    if (W > P) ok = ok && ((word >> P) & ((u64(1) << (W - P)) - 1)) == ((u64)(v >> ebits) & ((u64(1) << (W - P)) - 1));
    check(ok, "%s: escape %lld (raw %lld): the next push's word %llx is not the state's low word with the raw bits",
          at.c_str(), (long long)i, (long long)v, (unsigned long long)word);
    g_cov.raw_cross += W > P && W < P + S - ebits;
}

// Everything a clean round trip must satisfy. Returns the decode.
DecodeOut round_trip(const Config& c, const StreamJob& j, const char* tag) {
    const int N = j.n_lanes;
    const i64 n = (i64)j.symbols.size();
    const StreamOut e = encode_stream(c, N, j.symbols, j.scales);
    const DecodeOut d = job_decode(c, j, e.bytes);
    const u64 L = state_L(c), top = L << c.word_width();
    const i64 nw = (i64)e.words.size(), head = header_bytes(c, N);
    const int wb = word_bytes(c), sb = c.state_bytes();
    const std::string at = std::string(tag) + " " + kind_name(j.kind) + " d" + std::to_string(j.delay) + " N " +
                           std::to_string(N) + " n " + std::to_string(n);

    check(d.ok && !d.err_words && !d.err_header, "%s: ok %d, errors words %d header %d", at.c_str(), d.ok,
          d.err_words, d.err_header);
    check(j.kind != SourceKind::EMA || ema_as_defined(c, j), "%s: the EMA's b differ from their definition",
          at.c_str());
    check(j.kind != SourceKind::INT || int_as_defined(c, j), "%s: the INT's b differ from their definition",
          at.c_str());
    check(d.symbols == j.symbols, "%s: symbols differ", at.c_str());
    check(d.n_escapes == e.n_escapes && d.words_consumed == nw && d.words_total == nw,
          "%s: escapes %lld / %lld, words consumed %lld, in the array %lld, emitted %lld", at.c_str(),
          (long long)d.n_escapes, (long long)e.n_escapes, (long long)d.words_consumed, (long long)d.words_total,
          (long long)nw);

    // The array: N states, lane 0 first, then the words last-emitted first, little-endian.
    if (!check((i64)e.bytes.size() == head + nw * wb, "%s: %zu bytes, expected %lld", at.c_str(), e.bytes.size(),
               (long long)(head + nw * wb)))
        return d;
    bool layout = true;
    for (int l = 0; l < N; ++l) layout = layout && get_le(e.bytes, i64(l) * sb, sb) == e.states[l];
    for (i64 k = 0; k < nw; ++k) layout = layout && get_le(e.bytes, head + k * wb, wb) == e.words[nw - 1 - k];
    check(layout, "%s: the array is not the states then the words reversed", at.c_str());

    // The orders, step by step.
    if (!check((i64)e.trace.size() == n && (i64)d.trace.size() == n, "%s: trace lengths", at.c_str())) return d;
    std::vector<i64> emitted_before(n);  // words emitted before push i
    for (i64 k = 0, q = 0; k < n; q += e.trace[k].words, ++k) emitted_before[e.trace[k].i] = q;
    std::vector<u64> dec_state(e.states);  // each lane's decoder state before its next step: the header first
    i64 pos = 0;                          // the array's next word
    for (i64 i = 0; i < n; ++i) {
        const EncStep& es = e.trace[n - 1 - i];
        const DecStep& ds = d.trace[i];
        const LanePos p = lane_pos(N, i);
        bool ok = es.i == i && es.lane == p.lane && es.t == p.t && ds.lane == p.lane && ds.t == p.t;
        ok = ok && ds.b == es.b && ds.b == j.scales[i] && same_scale(ds.scale, es.scale) &&
             same_scale(ds.scale, scale_of_b(c, j.scales[i]));
        ok = ok && ds.escape == es.escape && ds.start == es.start && ds.freq == es.freq && ds.errors == 0;
        ok = ok && ds.symbol == j.symbols[i] && (ds.escape || (ds.slot >= ds.start && ds.slot < ds.start + ds.freq));
        ok = ok && (j.symbols[i] >= c.symbol_min() || (es.escape && ds.raw == raw_of(c, j.symbols[i])));
        ok = ok && ds.words == es.words;
        for (int k = 0; ok && k < ds.words; ++k)
            ok = get_le(e.bytes, head + (pos + k) * wb, wb) == e.words[emitted_before[i] + es.words - 1 - k];
        pos += ds.words;
        const u64 enc_before = i + N < n ? e.trace[n - 1 - (i + N)].state : L;
        ok = ok && dec_state[p.lane] == es.state && ds.state == enc_before;
        ok = ok && ds.state >= L && ds.state < top && es.state >= L && es.state < top;
        dec_state[p.lane] = ds.state;
        check(ok, "%s: step %lld (lane %d, t %lld) breaks the orders", at.c_str(), (long long)i, p.lane,
              (long long)p.t);
        ++g_cov.symbols;
        g_cov.escapes += es.escape;
        g_cov.outside += j.symbols[i] < c.symbol_min();
        g_cov.f_one += !es.escape && es.freq == 1;
        g_cov.f_max += !es.escape && es.freq == c.coded_slots();
        const i64 idx = scale_index_of_b(c, es.b);
        g_cov.b_zero += es.b == 0;
        g_cov.b_top += idx == c.scale_table_size() - 1;
        g_cov.zero_fill += es.b > 0 && es.b < (i64(1) << c.b_k);
        ++g_cov.index_hits[(size_t)idx];
        ++g_cov.enc_words[es.words];
        ++g_cov.dec_words[ds.words];
        g_cov.esc_words[es.words] += es.escape;
        if (es.escape) raw_layout(c, e, emitted_before, N, i, enc_before, at);
    }
    check(pos == nw, "%s: the steps read %lld words of %lld", at.c_str(), (long long)pos, (long long)nw);

    // Every lane's words are those of the lane encoded alone, in the same order.
    std::vector<std::vector<u64>> lane_words(N);
    for (i64 k = 0, q = 0; k < n; q += e.trace[k].words, ++k)
        for (int w = 0; w < e.trace[k].words; ++w) lane_words[e.trace[k].lane].push_back(e.words[q + w]);
    for (int l = 0; l < N; ++l) {
        std::vector<i64> sym;
        std::vector<ScaleB> sc;
        for (i64 i = l; i < n; i += N) {
            sym.push_back(j.symbols[i]);
            sc.push_back(j.scales[i]);
        }
        const StreamOut alone = encode_stream(c, 1, sym, sc);
        check(alone.words == lane_words[l] && alone.states[0] == e.states[l],
              "%s: lane %d encoded alone differs from its share of the stream", at.c_str(), l);
        if (sym.empty()) {
            ++g_cov.empty_lanes;
            check(e.states[l] == L && d.states[l] == L, "%s: empty lane %d not at L", at.c_str(), l);
        }
    }
    return d;
}

void round_trips(const Config& c, u64 seed) {
    JobGen gen(c);
    const long before = g_errors;
    long jobs = 0, symbols = 0;
    u64 k = 0;
    struct Src {
        SourceKind kind;
        int delay;
    };
    const Src srcs[] = {{SourceKind::LIST, 0}, {SourceKind::CONST, 0}, {SourceKind::EMA, 0}, {SourceKind::EMA, 1},
                        {SourceKind::EMA, 2},  {SourceKind::INT, 0},   {SourceKind::INT, 1}, {SourceKind::INT, 2}};
    for (const int N : LANE_COUNTS) {
        StreamRng r(mix_seed(seed, 1000 + N));
        const double lo = std::log2(double(N + 2)), hi = std::log2(double(N_MAX_RANDOM));
        for (const Src& s : srcs) {
            std::vector<i64> sizes = {0, 1, N - 1, N, N + 1, 3 * N + N / 2 + 1,
                                      (i64)std::exp2(lo + r.unit() * (hi - lo))};
            if (s.kind != SourceKind::LIST && s.kind != SourceKind::CONST && s.delay == 1)
                sizes.push_back(N_MAX_RANDOM);
            for (const i64 n : sizes) {
                const StreamJob j = gen.make(mix_seed(seed, ++k), N, n, s.kind, s.delay);
                round_trip(c, j, "round trip");
                ++jobs;
                symbols += n;
            }
        }
    }
    std::printf("round trips: %ld jobs, %ld symbols (N in {1, 2, 8, 16}; list, const, ema and int d = 0, 1, 2): %ld "
                "errors\n",
                jobs, symbols, g_errors - before);
}

// A LIST job whose lanes in `storm` escape every symbol at the narrowest code. The other lanes are random.
StreamJob storm_job(const Config& c, JobGen& gen, u64 seed, int N, i64 n, const std::vector<bool>& storm) {
    StreamRng r(seed);
    StreamJob j = gen.make(seed, N, n, SourceKind::LIST, 0);
    for (i64 i = 0; i < n; ++i) {
        if (!storm[lane_pos(N, i).lane]) continue;
        j.scales[i] = r.below(2);
        const i64 pick[] = {c.symbol_min(), c.symbol_max(), -1, 1, c.symbol_input_min()};  // the last: symmetric only
        i64 x = r.chance(0.25) ? pick[r.below(c.symbol_input_min() < c.symbol_min() ? 5 : 4)] : gen.uniform_symbol(r);
        j.symbols[i] = x == 0 ? 1 : x;
    }
    return j;
}

void directed(const Config& c, u64 seed) {
    JobGen gen(c);
    const long before = g_errors;
    long jobs = 0;
    u64 k = 0;
    // Escape storms: every lane, then one lane; every escaped step must be one.
    for (const int N : LANE_COUNTS) {
        for (int one = -1; one < N; one += (N > 1 ? N / 2 + 1 : 2)) {  // -1: every lane; else lane `one` alone
            std::vector<bool> storm(N, one < 0);
            if (one >= 0) storm[one] = true;
            const i64 n = 40 * N + 3;
            const StreamJob j = storm_job(c, gen, mix_seed(seed, 2000 + ++k), N, n, storm);
            const DecodeOut d = round_trip(c, j, one < 0 ? "storm all lanes" : "storm one lane");
            bool all = (i64)d.escapes.size() == n;
            for (i64 i = 0; all && i < n; ++i) all = !storm[lane_pos(N, i).lane] || d.escapes[i];
            check(all, "storm N %d lane %d: a storm symbol was not escaped", N, one);
            ++jobs;
        }
    }
    // Jobs of f = 1 and f = M' only, symbol 0 at the narrowest code or a symbol with f = 1.
    for (const int N : LANE_COUNTS) {
        StreamRng r(mix_seed(seed, 3000 + N));
        StreamJob j = gen.make(mix_seed(seed, 3100 + N), N, 50 * N + 1, SourceKind::LIST, 0);
        long f_one = 0, f_max = 0;
        for (i64 i = 0; i < (i64)j.symbols.size(); ++i) {
            const ScaleB b = gen.random_b(r, 0.0);
            const std::vector<i64>& ones = gen.unit_freq_symbols(scale_of_b(c, b));
            if (r.chance(0.5) || ones.empty()) {
                j.scales[i] = r.below(2);
                j.symbols[i] = 0;
                ++f_max;
            } else {
                j.scales[i] = b;
                j.symbols[i] = ones[r.below((i64)ones.size())] * (r.chance(0.5) ? 1 : -1);
                ++f_one;
            }
        }
        const StreamOut e = encode_stream(c, N, j.symbols, j.scales);
        long seen_one = 0, seen_max = 0;
        for (const EncStep& s : e.trace) {
            seen_one += !s.escape && s.freq == 1;
            seen_max += !s.escape && s.freq == c.coded_slots();
        }
        check(seen_one == f_one && seen_max == f_max, "extremes N %d: f = 1 %ld of %ld, f = M' %ld of %ld", N,
              seen_one, f_one, seen_max, f_max);
        round_trip(c, j, "extremes");
        ++jobs;
    }
    // A symmetric alphabet's input -2^(S-1) at every reachable table index, escaped and decoded back as itself.
    long outside = 0;
    if (c.symbol_input_min() < c.symbol_min()) {
        std::vector<ScaleB> bs = {c.b_max()};
        for (i64 q = 0; q < c.scale_table_size(); ++q)
            if (b_of_index(c, q) >= 0) bs.push_back(b_of_index(c, q));
        const i64 n_bs = (i64)bs.size();
        for (const int N : LANE_COUNTS) {
            StreamJob j = gen.make(mix_seed(seed, 3500 + N), N, 2 * n_bs + N, SourceKind::LIST, 0);
            for (i64 i = 0; i + 1 < (i64)j.symbols.size(); i += 2) {
                j.symbols[i] = c.symbol_input_min();
                j.scales[i] = bs[(size_t)((i / 2) % n_bs)];
                ++outside;
            }
            const DecodeOut d = round_trip(c, j, "outside the alphabet");
            long escaped = 0;
            for (i64 i = 0; i < (i64)d.trace.size(); ++i)
                escaped += j.symbols[i] == c.symbol_input_min() && d.trace[i].escape &&
                           d.trace[i].raw == (i64(1) << (c.symbol_width - 1)) && d.symbols[i] == c.symbol_input_min();
            long want = 0;
            for (const i64 x : j.symbols) want += x == c.symbol_input_min();
            check(escaped == want, "outside the alphabet N %d: %ld of %ld inputs %lld escaped with raw 2^(S-1)", N,
                  escaped, want, (long long)c.symbol_input_min());
            ++jobs;
        }
    }
    const std::string outside_what = outside ? ", the input " + std::to_string(c.symbol_input_min()) +
                                                   " outside the alphabet at every table index, " +
                                                   std::to_string(outside) + " times"
                                             : "";
    std::printf("directed: %ld jobs (escape storms on every lane and on one lane, f = 1 and f = M' only%s): %ld "
                "errors\n",
                jobs, outside_what.c_str(), g_errors - before);
}

// ---- 2b: the escape's raw push (contract version 5)

// One raw push and its pop from state x with raw value v (STREAM.md §1.4).
bool raw_push_case(const Config& c, u64 x, i64 v) {
    const int W = c.word_width(), S = c.symbol_width;
    const u64 L = state_L(c), B = raw_push_bound(c);
    u64 y = x;
    std::vector<u64> w;
    stream_push_raw(c, y, w, v);
    const bool emit = x >= B;
    bool ok = y == (((emit ? x >> W : x) << S) | (u64)v) && y >= L && y < (L << W) && w.size() == (emit ? 1u : 0u) &&
              (!emit || w[0] == (x & word_mask(c)));
    std::vector<uint8_t> bytes;  // one lane: a header (unused) and the word, if any
    put_le(bytes, L, c.state_bytes());
    for (const u64 q : w) put_le(bytes, q, word_bytes(c));
    StreamReader r(c, 1, bytes);
    r.header_state(0);
    u64 z = y;
    i64 got = -1;
    const int words = stream_pop_raw(c, z, got, r);
    ok = ok && got == v && z == x && words == (int)w.size() && r.over_reads() == 0;
    if (S == c.precision) {
        u64 y1 = x;
        std::vector<u64> w1;
        stream_push(c, y1, w1, v, 1);
        StreamReader r1(c, 1, bytes);
        r1.header_state(0);
        u64 z1 = y;
        const int words1 = stream_pop(c, z1, v, v, 1, r1);
        ok = ok && y1 == y && w1 == w && z1 == z && words1 == words;
    }
    g_cov.raw_below += x == B - 1;
    g_cov.raw_at += x == B;
    return ok;
}

void raw_push(const Config& c, u64 seed) {
    const long before = g_errors;
    const int W = c.word_width(), S = c.symbol_width;
    const u64 L = state_L(c), top = L << W, B = raw_push_bound(c);
    const i64 V = i64(1) << S;
    StreamRng r(mix_seed(seed, 7000));
    // States on both sides of B, at the range's ends and random, or every state where there are at most 2^16.
    std::vector<u64> xs;
    if (top - L <= (u64(1) << 16)) {
        for (u64 x = L; x < top; ++x) xs.push_back(x);
    } else {
        for (const u64 x : {L, L + 1, B - 2, B - 1, B, B + 1, top - 2, top - 1})
            if (x >= L && x < top) xs.push_back(x);
        for (int k = 0; k < (1 << 16); ++k) xs.push_back(L + (u64)r.below((i64)(top - L)));
    }
    long cases = 0, bad = 0, emitted = 0;
    for (const u64 x : xs) {
        const bool corner = x + 2 >= B && x <= B + 1;
        std::vector<i64> vs = {0, 1, V / 2 - 1, V / 2, V - 2, V - 1, r.below(V)};
        if (corner && V <= 1024)
            for (i64 v = 0; v < V; ++v) vs.push_back(v);  // every raw value on both sides of the bound
        for (const i64 v : vs) {
            ++cases;
            emitted += x >= B;
            bad += !raw_push_case(c, x, v);
        }
    }
    check(bad == 0, "raw push: %ld of %ld single-state cases wrong", bad, cases);
    // Whole streams: the bound job's escapes read exactly B (lane 0) and B - 1 (lane 1, where B - 1 >= L).
    JobGen gen(c);
    long jobs = 0;
    for (const int N : {1, 2, 8}) {
        const JobGen::BoundJob bj = gen.bound_job(mix_seed(seed, 7100 + N), N);
        const StreamJob& j = bj.job;
        const i64 n = (i64)j.symbols.size();
        round_trip(c, j, "raw bound");
        const StreamOut e = encode_stream(c, N, j.symbols, j.scales);
        for (int k = 0; k < 2 && k < N; ++k) {
            const i64 i = bj.esc_index[k];
            const bool want = k == 0 || B - 1 >= L;
            if (!check((i >= 0) == want, "raw bound N %d lane %d: escape index %lld", N, k, (long long)i) || i < 0)
                continue;
            const u64 x0 = i + N < n ? e.trace[n - 1 - (i + N)].state : L;
            const EncStep& es = e.trace[n - 1 - i];
            check(es.escape && es.lane == k && x0 == bj.esc_state[k] && x0 == (k == 0 ? B : B - 1),
                  "raw bound N %d lane %d: the escape (%d) read %llx, the target %llx", N, k, es.escape,
                  (unsigned long long)x0, (unsigned long long)bj.esc_state[k]);
            (k == 0 ? g_cov.bound_hi : g_cov.bound_lo) += 1;
        }
        ++jobs;
    }
    std::printf("raw push: B = 2^(P + W - S) = %llu; %ld single-state cases (%zu states%s, %ld emitting), %ld bound "
                "jobs (N in {1, 2, 8}: escapes reading B and B - 1, each lane's path from L found by the model): %ld "
                "errors\n",
                (unsigned long long)B, cases, xs.size(), top - L <= (u64(1) << 16) ? ": every state" : "", emitted, jobs,
                g_errors - before);
}

// ---- 3: corrupt arrays

// Re-encodes a decode's symbols with its scales and escapes. The result equals `bytes` iff ok = 1 was a collision.
bool reencodes_to(const Config& c, int N, const DecodeOut& d, const std::vector<uint8_t>& bytes) {
    const i64 n = (i64)d.trace.size();
    std::vector<u64> x(N, state_L(c)), w;
    for (i64 i = n - 1; i >= 0; --i) {
        const DecStep& s = d.trace[i];
        if (s.escape) {
            stream_push_raw(c, x[s.lane], w, s.raw);
            stream_push(c, x[s.lane], w, c.coded_slots(), c.escape_slots);
        } else {
            stream_push(c, x[s.lane], w, s.start, s.freq);
        }
    }
    std::vector<uint8_t> b;
    for (const u64 v : x) put_le(b, v, c.state_bytes());
    for (size_t k = w.size(); k-- > 0;) put_le(b, w[k], word_bytes(c));
    return b == bytes;
}

// One corrupted array against its clean decode (STREAM.md §1.6). `must` holds the flags below.
// NONE_ALLOWED wants no flag and ok = 0. HEADER_ANY accepts err_header either way.
constexpr unsigned MUST_OK = 1u << 1, MUST_WORDS = 1u << 2, MUST_HEADER = 1u << 3;
constexpr unsigned NONE_ALLOWED = 1u << 4, HEADER_ANY = 1u << 5;

void check_corrupt(const Config& c, const StreamJob& j, const DecodeOut& clean, const std::vector<uint8_t>& bad,
                   i64 first_bad, unsigned must, const char* what) {
    const i64 n = (i64)j.symbols.size();
    const DecodeOut d = job_decode(c, j, bad);
    ++g_cov.corrupt;
    g_cov.err_words += d.err_words;
    g_cov.err_header += d.err_header;
    const bool any = d.err_words || d.err_header;
    g_cov.end_only += !d.ok && !any;
    check((i64)d.trace.size() == n && (i64)d.symbols.size() == n, "%s: N %d n %lld: %zu steps", what, j.n_lanes,
          (long long)n, d.trace.size());
    bool fits = true, inside = true;  // the state is a STATE_WIDTH-bit register on any input
    for (const DecStep& s : d.trace) {
        fits = fits && s.state <= state_mask(c);
        inside = inside && (s.escape || (s.slot >= s.start && s.slot < s.start + s.freq && s.freq >= 1));
    }
    for (const u64 x : d.states) fits = fits && x <= state_mask(c);
    check(fits, "%s: N %d n %lld: a state beyond STATE_WIDTH bits", what, j.n_lanes, (long long)n);
    bool prefix = true;
    for (i64 i = 0; prefix && i < std::min<i64>(first_bad, (i64)d.trace.size()); ++i)
        prefix = same_step(d.trace[i], clean.trace[i]);
    check(prefix, "%s: N %d n %lld: the trace differs before step %lld", what, j.n_lanes, (long long)n,
          (long long)first_bad);
    if (d.ok) {
        const bool collision = reencodes_to(c, j.n_lanes, d, bad);
        g_cov.collisions += collision;
        check(collision, "%s: N %d n %lld: ok = 1 on a corrupt array that is not a collision", what, j.n_lanes,
              (long long)n);
    }
    check(inside, "%s: N %d n %lld: a coded slot outside the interval decoded for it", what, j.n_lanes,
          (long long)n);
    check(!(must & MUST_OK) || d.ok, "%s: ok = 0, a collision expected", what);
    check(!(must & MUST_WORDS) || d.err_words, "%s: no err_words", what);
    check((must & HEADER_ANY) || ((must & MUST_HEADER) ? d.err_header : !d.err_header), "%s: err_header %d", what,
          d.err_header);
    check(!(must & NONE_ALLOWED) || (!any && !d.ok), "%s: flags %d%d ok %d (expected none, ok 0)", what,
          d.err_words, d.err_header, d.ok);
    if (must & (MUST_WORDS | MUST_HEADER)) check(!d.ok, "%s: ok = 1", what);
}

// A one-lane, one-symbol array whose header state is `x` followed by `words`: the directed error cases.
std::vector<uint8_t> tiny_array(const Config& c, u64 x, const std::vector<u64>& words) {
    std::vector<uint8_t> b;
    put_le(b, x, c.state_bytes());
    for (const u64 w : words) put_le(b, w, word_bytes(c));
    return b;
}

void corrupt_arrays(const Config& c, u64 seed) {
    JobGen gen(c);
    const long before = g_errors;
    const i64 head_lane = c.state_bytes();
    const int wb = word_bytes(c), spare = c.state_bytes() * BYTE_BITS - c.state_width();
    const u64 L = state_L(c);
    long arrays = 0;
    u64 k = 0;
    const SourceKind kinds[] = {SourceKind::LIST, SourceKind::EMA, SourceKind::CONST, SourceKind::INT};
    for (const int N : {1, 8}) {
        for (const SourceKind kind : kinds) {
            for (int rep = 0; rep < 6; ++rep) {
                StreamRng r(mix_seed(seed, 4000 + ++k));
                const i64 n = 1 + r.below(rep < 3 ? 40 * N : 3000);
                const StreamJob j = gen.make(mix_seed(seed, 4500 + k), N, n, kind, int(rep % 3));
                const StreamOut e = encode_stream(c, N, j.symbols, j.scales);
                const DecodeOut clean = job_decode(c, j, e.bytes);
                const i64 head = header_bytes(c, N), nw = (i64)e.words.size(), size = (i64)e.bytes.size();
                if (nw > 0) {
                    // A flipped word.
                    std::vector<uint8_t> b = e.bytes;
                    const i64 w = r.below(nw);
                    const u64 flip = 1 + (u64)r.below((i64)word_mask(c));
                    for (int q = 0; q < wb; ++q) b[head + w * wb + q] ^= uint8_t((flip >> (BYTE_BITS * q)) & BYTE_MASK);
                    check_corrupt(c, j, clean, b, step_reading_word(clean, w), 0, "flipped word");
                    // Truncated inside the words. The steps up to the first missing word are the clean ones.
                    const i64 cut = 1 + r.below(nw * wb);
                    const std::vector<uint8_t> t(e.bytes.begin(), e.bytes.end() - cut);
                    const i64 missing = (size - cut - head) / wb;
                    check_corrupt(c, j, clean, t, step_reading_word(clean, missing), MUST_WORDS, "truncated words");
                    const DecodeOut dt = job_decode(c, j, t);
                    check(dt.words_consumed == missing && dt.words_total == missing,
                          "truncated words: consumed %lld, in the array %lld, expected %lld",
                          (long long)dt.words_consumed, (long long)dt.words_total, (long long)missing);
                    arrays += 2;
                }
                // Truncated inside the header.
                {
                    const std::vector<uint8_t> t(e.bytes.begin(), e.bytes.begin() + r.below(head));
                    check_corrupt(c, j, clean, t, 0, MUST_WORDS | HEADER_ANY, "truncated header");
                }
                const int lane = int(r.below(N));
                const i64 first_on_lane = lane < n ? lane : n;
                // A header state below L.
                {
                    std::vector<uint8_t> b = e.bytes;
                    const u64 low = (u64)r.below((i64)L);
                    for (int q = 0; q < c.state_bytes(); ++q)
                        b[lane * head_lane + q] = uint8_t((low >> (BYTE_BITS * q)) & BYTE_MASK);
                    check_corrupt(c, j, clean, b, first_on_lane, MUST_HEADER, "header state < L");
                }
                // A header bit above STATE_WIDTH. It is flagged, and the decode is otherwise the clean one.
                if (spare > 0) {
                    std::vector<uint8_t> b = e.bytes;
                    const int bit = c.state_width() + int(r.below(spare));
                    b[lane * head_lane + bit / BYTE_BITS] ^= uint8_t(1u << (bit % BYTE_BITS));
                    check_corrupt(c, j, clean, b, n, MUST_HEADER, "header bit above STATE_WIDTH");
                    ++arrays;
                }
                // A header state flipped in a slot bit, staying in [L, 2^STATE_WIDTH).
                {
                    std::vector<uint8_t> b = e.bytes;
                    const int bit = int(r.below(c.precision));
                    b[lane * head_lane + bit / BYTE_BITS] ^= uint8_t(1u << (bit % BYTE_BITS));
                    check_corrupt(c, j, clean, b, first_on_lane, 0, "header state flipped in range");
                }
                // Bytes appended: a partial word, a whole word. Decoded as the clean array, then left over.
                {
                    std::vector<uint8_t> b = e.bytes;
                    b.push_back(uint8_t(r.below(BYTE_MASK + 1)));
                    check_corrupt(c, j, clean, b, n, NONE_ALLOWED, "byte appended");
                    for (int q = 1; q < wb; ++q) b.push_back(uint8_t(r.below(BYTE_MASK + 1)));
                    check_corrupt(c, j, clean, b, n, NONE_ALLOWED, "word appended");
                }
                arrays += 5;
            }
        }
    }
    // An escape's raw value flipped. Only that symbol changes, and ok = 1 is a proven collision (STREAM.md §1.6).
    long raw_flips = 0;
    for (const int N : {1, 8}) {
        const StreamJob j = storm_job(c, gen, mix_seed(seed, 4900 + N), N, 30 * N + 5, std::vector<bool>(N, true));
        const StreamOut e = encode_stream(c, N, j.symbols, j.scales);
        const DecodeOut clean = job_decode(c, j, e.bytes);
        i64 pos = 0, at = -1;
        for (i64 i = 0; at < 0 && i < (i64)clean.trace.size(); pos += clean.trace[i].words, ++i)
            if (clean.trace[i].escape && clean.trace[i].words == 2) at = i;
        if (at < 0) continue;
        pos = 0;
        for (i64 i = 0; i < at; ++i) pos += clean.trace[i].words;
        std::vector<uint8_t> b = e.bytes;
        const int bit = c.symbol_width - 1;
        b[header_bytes(c, N) + pos * word_bytes(c) + bit / BYTE_BITS] ^= uint8_t(1u << (bit % BYTE_BITS));
        check_corrupt(c, j, clean, b, at, MUST_OK, "raw value flipped");
        const DecodeOut d = job_decode(c, j, b);
        bool one = d.symbols.size() == j.symbols.size();
        for (i64 i = 0; one && i < (i64)j.symbols.size(); ++i)
            one = i == at ? d.symbols[i] == symbol_of_raw(c, raw_of(c, j.symbols[i]) ^ (i64(1) << bit))
                          : d.symbols[i] == j.symbols[i];
        check(one, "raw value flipped N %d: not exactly the symbol of step %lld changed", N, (long long)at);
        ++raw_flips;
        ++arrays;
    }
    const bool raw_flip_possible = c.word_width() + log2_escape(c) < c.precision + c.symbol_width;
    check(raw_flips > 0 || !raw_flip_possible, "raw value flipped: no escape reading two words in the storm jobs");
    // The slots just beyond the alphabet, C(MIN) - 1 and C(MAX + 1), decode inside MIN's and MAX's intervals.
    // The header x = L + slot with no words makes the refill read past the array.
    bool found = false;
    for (i64 q = 0; !found && q < c.scale_table_size(); ++q) {
        const ScaleB b_q = b_of_index(c, q);
        if (b_q < 0) continue;
        const Scale s = scale_of_b(c, b_q);
        const Slope k = slope_of(c, s);
        const i64 below = cdf(c, k, c.symbol_min()) - 1, above = cdf(c, k, c.symbol_max() + 1);
        if (below < 0 || above >= c.coded_slots()) continue;
        found = true;
        for (const i64 slot : {below, above}) {
            const DecOut dd = decode(c, s, slot);
            const bool inside = slot == below ? dd.symbol == c.symbol_min() && dd.start == 0 && slot < dd.freq
                                              : dd.symbol == c.symbol_max() && dd.start <= slot &&
                                                    dd.start + dd.freq == c.coded_slots();
            check(inside, "slot beyond the alphabet: slot %lld decodes as %lld [%lld, +%lld)", (long long)slot,
                  (long long)dd.symbol, (long long)dd.start, (long long)dd.freq);
            StreamJob j;
            j.kind = SourceKind::CONST;
            j.const_scale = b_q;
            j.symbols.assign(1, 0);
            check_corrupt(c, j, DecodeOut{}, tiny_array(c, L + (u64)slot, {}), 0, MUST_WORDS,
                          slot == below ? "slot C(MIN) - 1" : "slot C(MAX + 1)");
            ++arrays;
        }
    }
    check(found, "no table entry's code has C(MIN) > 0: no slot beyond the alphabet to test");
    std::printf("corrupt arrays: %ld (flipped word, truncated words / header, header < L, header bit above "
                "STATE_WIDTH, header flipped in range, byte / word appended, raw value flipped %ld%s, slots C(MIN) - 1 "
                "and C(MAX + 1)): flags words %ld, header %ld, end check only %ld, collisions %ld: %ld errors\n",
                g_cov.corrupt, raw_flips, raw_flip_possible ? "" : " (n/a: no escape reads two words)",
                g_cov.err_words, g_cov.err_header, g_cov.end_only, g_cov.collisions, g_errors - before);
}

// ---- 4: the pans1.cuh transcription (P = W = S = 16)
// At S = P the raw push and pop are pans1.cuh's push(raw, 1) and dec_pop(v, v, 1).

// pANS 1.0's pans1.cuh transcribed literally, device qualifiers dropped. Line numbers are that file's.
// The cores' (c0, f, escape) replace its C, cand and resolve.
namespace pans1_transcription {

constexpr int M_TOT = 65536;           // line 36
constexpr unsigned L_LOW = 1u << 16;   // line 38

// Lines 67-74.
inline void push(unsigned& x, uint16_t* w, int& k, int c0, int f) {
    if (f < M_TOT && x >= ((unsigned)f << 16)) {
        w[k++] = (uint16_t)(x & 0xFFFFu);
        x >>= 16;
    }
    x = (x / (unsigned)f) * (unsigned)M_TOT + (unsigned)c0 + x % (unsigned)f;
}

// Lines 76-88. The direct-code condition is the core's f >= 1.
template <int E>
inline bool encode_one(unsigned& x, uint16_t* w, int& k, int sym, const Config& cfg, Scale p) {
    const EncOut core = encode(cfg, p, sym);
    const int c0 = (int)core.start, f = (int)core.freq;
    if (!core.escape) {
        push(x, w, k, c0, f);
        return false;
    }
    push(x, w, k, (int)(uint16_t)(int16_t)sym, 1);  // raw 16 bits (ANS decodes in reverse: pushed first)
    push(x, w, k, M_TOT - E, E);                    // the escape
    return true;
}

// Lines 145-150.
struct Dec {
    unsigned x;         // state
    int p;              // words left (w[p - 1] is the next to read)
    unsigned nw;        // prefetched w[p - 1] (0 past the stream's start)
    const uint16_t* w;  // the stream's words
};

// Lines 152-158 (dec_init, OPT dropped).
inline Dec dec_init(const uint16_t* w, int wcount, unsigned fstate) {
    Dec d{fstate, wcount, 0u, w};
    d.nw = wcount > 0 ? w[wcount - 1] : 0u;
    return d;
}

// Lines 160-167.
inline void dec_pop(Dec& d, int s, int c, int f) {
    d.x = (d.x >> 16) * (unsigned)f + (unsigned)(s - c);
    if (d.x < L_LOW) {
        d.x = (d.x << 16) | d.nw;
        --d.p;
        d.nw = d.p > 0 ? d.w[d.p - 1] : 0u;
    }
}

// Line 169.
inline bool dec_ok(const Dec& d) { return d.x == L_LOW && d.p == 0; }

// Lines 171-203, OPT_REF: resolve<E> is the core's decode.
template <int E>
inline int decode_next(Dec& d, const Config& cfg, Scale pr, int& esc) {
    const int s = (int)(d.x & 0xFFFFu);
    if (s >= M_TOT - E) {
        dec_pop(d, s, M_TOT - E, E);
        const int v = (int)(d.x & 0xFFFFu);
        dec_pop(d, v, v, 1);
        ++esc;
        return (int)(int16_t)(uint16_t)v;
    }
    const DecOut core = decode(cfg, pr, s);
    const int sym = (int)core.symbol, c = (int)core.start, f = (int)core.freq;
    dec_pop(d, s, c, f);
    return sym;
}

}  // namespace pans1_transcription

struct Pans1Decode {
    std::vector<i64> symbols;
    unsigned x;
    int p;
    bool ok;
    int esc;
};

// pans1.cuh's decoder over words w[0 .. k) in emission order, fed by the job's scale source.
Pans1Decode pans1_decode(const Config& c, const StreamJob& j, const std::vector<uint16_t>& w, unsigned fstate) {
    namespace t = pans1_transcription;
    Pans1Decode g{{}, 0, 0, false, 0};
    t::Dec d = t::dec_init(w.data(), (int)w.size(), fstate);
    const std::unique_ptr<ScaleSource> src = job_source(c, j);
    src->begin(1);
    for (i64 i = 0; i < (i64)j.symbols.size(); ++i) {
        const int sym = t::decode_next<PANS_ESCAPE_SLOTS>(d, c, scale_of_b(c, src->scale(0, i)), g.esc);
        g.symbols.push_back(sym);
        src->observe(0, i, sym);
    }
    g.x = d.x;
    g.p = d.p;
    g.ok = t::dec_ok(d);
    return g;
}

void pans1_cuh(const Config& c, u64 seed) {
    namespace t = pans1_transcription;
    if (c.precision != 16 || c.symbol_width != 16 || c.word_width() != 16) {
        std::printf("pans1.cuh transcription: not applicable (needs P = W = S = 16; this build P = %d, W = %d, "
                    "S = %d)\n", c.precision, c.word_width(), c.symbol_width);
        return;
    }
    JobGen gen(c);
    const long before = g_errors;
    long jobs = 0, symbols = 0, words = 0, escapes = 0, corrupt = 0;
    u64 k = 0;
    struct Case {
        SourceKind kind;
        i64 n;
    };
    const Case cases[] = {{SourceKind::LIST, 0},     {SourceKind::LIST, 1},    {SourceKind::LIST, 2},
                          {SourceKind::LIST, 1000},  {SourceKind::LIST, N_MAX_RANDOM}, {SourceKind::CONST, 5000},
                          {SourceKind::EMA, 20000},  {SourceKind::EMA, N_MAX_RANDOM},  {SourceKind::INT, 20000}};
    for (const Case& cs : cases) {
        for (int storm = 0; storm < 2; ++storm) {
            const StreamJob j = storm ? storm_job(c, gen, mix_seed(seed, 5000 + ++k), 1, cs.n / 4 + 1, {true})
                                      : gen.make(mix_seed(seed, 5000 + ++k), 1, cs.n, cs.kind, 1);
            const i64 n = (i64)j.symbols.size();
            // The transcription's encoder, in this stream's push order (i = n - 1 ... 0).
            unsigned x = t::L_LOW;
            std::vector<uint16_t> w(2 * n + 1);
            int kw = 0, esc = 0;
            for (i64 i = n - 1; i >= 0; --i)
                esc += t::encode_one<PANS_ESCAPE_SLOTS>(x, w.data(), kw, (int)j.symbols[i], c,
                                                        scale_of_b(c, j.scales[i]));
            w.resize(kw);
            const StreamOut e = encode_stream(c, 1, j.symbols, j.scales);
            bool same = (i64)e.words.size() == kw && e.states[0] == x && e.n_escapes == esc;
            for (int q = 0; same && q < kw; ++q) same = e.words[q] == w[q];
            check(same, "pans1.cuh %s n %lld: encoder words %zu / %d, state %llx / %x, escapes %lld / %d",
                  kind_name(j.kind), (long long)n, e.words.size(), kw, (unsigned long long)e.states[0], x,
                  (long long)e.n_escapes, esc);
            // The array is the state then w[k - 1] ... w[0], as pans1.cuh's decoder reads them.
            std::vector<uint8_t> b;
            put_le(b, x, 4);
            for (int q = kw; q-- > 0;) put_le(b, w[q], 2);
            check(b == e.bytes, "pans1.cuh n %lld: the array is not the state then the words from the last",
                  (long long)n);
            // The transcription's decoder against the model's.
            const Pans1Decode g = pans1_decode(c, j, w, x);
            const DecodeOut d = job_decode(c, j, e.bytes);
            check(g.symbols == j.symbols && d.symbols == j.symbols && g.ok && d.ok && g.x == d.states[0] &&
                      g.esc == d.n_escapes,
                  "pans1.cuh n %lld: decoder symbols %d / %d, ok %d / %d", (long long)n, g.symbols == j.symbols,
                  d.symbols == j.symbols, g.ok, d.ok);
            ++jobs;
            symbols += n;
            words += kw;
            escapes += esc;
            if (kw == 0) continue;
            // Corrupt arrays give the same symbols and state, and the same end check without the model's flags.
            StreamRng r(mix_seed(seed, 5500 + k));
            for (int how = 0; how < 2; ++how) {
                std::vector<uint16_t> wc = w;
                std::vector<uint8_t> bc = e.bytes;
                if (how == 0) {
                    const int q = (int)r.below(kw);
                    wc[q] ^= uint16_t(1 + r.below(0xFFFF));
                    bc[4 + 2 * (kw - 1 - q)] = uint8_t(wc[q] & 0xFF);
                    bc[4 + 2 * (kw - 1 - q) + 1] = uint8_t(wc[q] >> 8);
                } else {
                    const int drop = 1 + (int)r.below(kw);
                    wc.erase(wc.begin(), wc.begin() + drop);
                    bc.resize(bc.size() - 2 * drop);
                }
                const Pans1Decode gc = pans1_decode(c, j, wc, x);
                const DecodeOut dc = job_decode(c, j, bc);
                const bool model_end = dc.states[0] == t::L_LOW && dc.words_consumed == dc.words_total && !dc.err_words;
                check(gc.symbols == dc.symbols && gc.x == dc.states[0] && gc.ok == model_end && gc.esc == dc.n_escapes,
                      "pans1.cuh corrupt (%s) n %lld: symbols %d, state %x / %llx, end %d / %d",
                      how ? "truncated" : "flipped word", (long long)n, gc.symbols == dc.symbols, gc.x,
                      (unsigned long long)dc.states[0], gc.ok, model_end);
                ++corrupt;
            }
        }
    }
    std::printf("pans1.cuh transcription (P = W = S = 16, N = 1): %ld jobs, %ld symbols, %ld words, %ld escapes, "
                "%ld corrupt arrays: words, state, symbols, escapes and end check identical: %ld errors\n",
                jobs, symbols, words, escapes, corrupt, g_errors - before);
}

// ---- 5: the DPI as a testbench drives it

// A testbench's decoder loop, every b from the DPI's source as early as it allows. Its steps must be the DPI's.
void tb_loop(const Config& c, int h, int lanes, const char* what) {
    const i64 n = pans_stream_job_n(h);
    std::vector<uint8_t> bytes(pans_stream_job_n_bytes(h));
    for (i64 q = 0; q < (i64)bytes.size(); ++q) bytes[q] = uint8_t(pans_stream_job_byte(h, q));
    pans_stream_src_reset(h);
    std::vector<std::deque<ScaleB>> ready(lanes);  // scale inputs the testbench holds per lane
    std::vector<i64> asked(lanes, 0);
    const auto prefetch = [&] {
        for (int l = 0; l < lanes; ++l)
            for (;;) {
                if (asked[l] >= lane_symbols(lanes, n, l)) break;
                long long b = -1;
                const int got = pans_stream_scale(h, l, asked[l], &b);
                if (got != 1) {
                    check(got == 0, "%s: pans_stream_scale(%d, %lld) = %d", what, l, (long long)asked[l], got);
                    break;
                }
                ready[l].push_back((ScaleB)b);
                ++asked[l];
            }
    };
    StreamReader r(c, lanes, bytes);
    std::vector<u64> x(lanes);
    bool err_header = false;
    for (int l = 0; l < lanes; ++l) {
        const u64 v = r.header_state(l);
        err_header = err_header || v < state_L(c) || v > state_mask(c);
        x[l] = v & state_mask(c);
    }
    bool err_words = r.over_reads() != 0, steps_ok = true;
    i64 escapes = 0;
    prefetch();
    for (i64 i = 0; i < n; ++i) {
        const LanePos p = lane_pos(lanes, i);
        if (!check(!ready[p.lane].empty(), "%s: no scale for step %lld although every earlier symbol is observed",
                   what, (long long)i))
            return;
        const ScaleB b = ready[p.lane].front();
        ready[p.lane].pop_front();
        const DecStep st = stream_decode_next(c, x[p.lane], b, r);
        int l = 0, sc_e = 0, sc_m = 0, sl = 0, stt = 0, fr = 0, es = 0, rw = 0, sy = 0, wd = 0, er = 0;
        long long t = 0, xs = 0, sb = -1;
        pans_stream_job_step(h, i, &l, &t, &sl, &stt, &fr, &es, &rw, &sy, &sb, &sc_e, &sc_m, &wd, &xs, &er);
        steps_ok = steps_ok && l == p.lane && t == p.t && sl == st.slot && stt == st.start && fr == st.freq &&
                   es == st.escape && rw == st.raw && sy == st.symbol && sb == b && sc_e == (int)st.scale.exp &&
                   sc_m == (int)st.scale.man && same_scale(st.scale, scale_of_b(c, b)) && wd == st.words &&
                   (u64)xs == st.state && (unsigned)er == st.errors;
        err_words = err_words || (st.errors & ERR_WORDS);
        escapes += st.escape;
        check(pans_stream_observe(h, p.lane, p.t, (int)st.symbol) == 0, "%s: observe rejected", what);
        prefetch();
    }
    check(steps_ok, "%s: a step differs from pans_stream_job_step", what);
    bool at_L = true;
    for (const u64 v : x) at_L = at_L && v == state_L(c);
    const bool ok = !err_words && !err_header && at_L && r.header() + r.words_read() * word_bytes(c) == r.size();
    int d_ok = 0, d_words = 0, d_head = 0;
    long long d_esc = 0, d_consumed = 0;
    pans_stream_job_done(h, &d_ok, &d_esc, &d_consumed, &d_words, &d_head);
    check(d_ok == ok && d_esc == escapes && d_consumed == r.words_read() && d_words == err_words &&
              d_head == err_header,
          "%s: done fields differ from the testbench loop's", what);
}

void dpi(const Config& c, u64 seed) {
    const long before = g_errors;
    long jobs = 0;
    int ww = 0, sw = 0, sb = 0;
    pans_stream_widths(&ww, &sw, &sb);
    check(ww == c.word_width() && sw == c.state_width() && sb == c.state_bytes(), "pans_stream_widths");
    JobGen gen(c);
    for (const int N : {1, 3, 8, 16}) {
        for (int kind = 0; kind <= int(SourceKind::INT); ++kind) {
            const bool loop = kind == int(SourceKind::EMA) || kind == int(SourceKind::INT);  // the consumer's loop
            for (int delay = 0; delay <= (loop ? 2 : 0); ++delay) {
                const int s = int(mix_seed(seed, 6000 + 64 * N + 4 * kind + delay) & 0x7fffffff);
                const i64 n = 7 * N + (s % 997);
                const int h = pans_stream_job_new(s, N, n, kind, delay);
                if (!check(h >= 0, "pans_stream_job_new(%d, %d, %lld, %d, %d) = %d", s, N, (long long)n, kind, delay,
                           h))
                    continue;
                // The job's expectations are the model's for the same job.
                const StreamJob j = gen.make((u64)(unsigned)s, N, n, SourceKind(kind), delay);
                const StreamOut e = encode_stream(c, N, j.symbols, j.scales);
                const DecodeOut d = job_decode(c, j, e.bytes);
                bool same = pans_stream_job_n(h) == n && pans_stream_job_n_bytes(h) == (long long)e.bytes.size() &&
                            pans_stream_job_n_words(h) == (long long)e.words.size() &&
                            pans_stream_job_n_escapes(h) == e.n_escapes;
                for (i64 q = 0; same && q < (i64)e.bytes.size(); ++q) same = pans_stream_job_byte(h, q) == e.bytes[q];
                for (i64 q = 0; same && q < (i64)e.words.size(); ++q) same = (u64)pans_stream_job_word(h, q) == e.words[q];
                for (int l = 0; same && l < N; ++l) same = (u64)pans_stream_job_state(h, l) == e.states[l];
                // Every push beat and step is the model's EncStep, and the steps chain through each lane's states.
                i64 step_words = 0;
                for (i64 q = 0; same && q < n; ++q) {
                    int lane = 0, sym = 0, s_lane = 0, s_sym = 0, s_start = 0, s_freq = 0, s_esc = 0, s_words = 0,
                        s_e = 0, s_m = 0;
                    long long s_t = 0, s_state = 0, b = -1, s_b = -1;
                    pans_stream_job_push(h, q, &lane, &sym, &b);
                    pans_stream_job_enc_step(h, q, &s_lane, &s_t, &s_sym, &s_start, &s_freq, &s_esc, &s_words,
                                             &s_state, &s_b, &s_e, &s_m);
                    const EncStep& es = e.trace[q];
                    same = lane == es.lane && sym == es.symbol && b == es.b && s_b == es.b &&
                           same_scale(Scale{(unsigned)s_e, (unsigned)s_m}, es.scale) &&
                           same_scale(es.scale, scale_of_b(c, es.b));
                    same = same && s_lane == es.lane && s_t == es.t && s_sym == es.symbol && s_start == es.start &&
                           s_freq == es.freq && s_esc == es.escape && s_words == es.words && (u64)s_state == es.state;
                    same = same && s_lane == lane_pos(N, n - 1 - q).lane && s_t == lane_pos(N, n - 1 - q).t &&
                           (u64)s_state >= state_L(c) && (u64)s_state <= state_mask(c);
                    step_words += s_words;
                }
                same = same && step_words == (i64)e.words.size();
                // Each lane's last push, global index l (push n - 1 - l), leaves the lane's final state.
                for (int l = 0; same && l < N && l < n; ++l) {
                    int a = 0, b = 0, f = 0, x = 0, w = 0, s = 0, se = 0, sm = 0;
                    long long t = 0, st = 0, sb = 0;
                    pans_stream_job_enc_step(h, n - 1 - l, &a, &t, &b, &f, &x, &w, &s, &st, &sb, &se, &sm);
                    same = (u64)st == e.states[l] && (u64)st == (u64)pans_stream_job_state(h, l);
                }
                check(same && d.ok, "DPI job N %d kind %d delay %d: expectations differ from the model", N, kind, delay);
                // The protocol's return codes, -1 out of order and 0 before the history is observed.
                if (lane_symbols(N, n, 0) >= 2) {
                    long long b0 = 0;
                    pans_stream_src_reset(h);
                    check(pans_stream_scale(h, 0, 1, &b0) == -1, "DPI: a scale out of order was not refused");
                    check(pans_stream_observe(h, 0, 1, 0) == -1, "DPI: an observation out of order was not refused");
                    if (loop && delay == 0) {
                        check(pans_stream_scale(h, 0, 0, &b0) == 1, "DPI: the first scale was refused");
                        check(pans_stream_scale(h, 0, 1, &b0) == 0, "DPI: a scale before its history");
                    }
                }
                tb_loop(c, h, N, "DPI clean");
                // A flipped byte, then a truncation. The testbench's loop must follow the recomputed expectations.
                StreamRng r((u64)s);
                const long long size = pans_stream_job_n_bytes(h);
                pans_stream_job_corrupt(h, 0, (long long)r.below(size), 1 + (int)r.below(BYTE_MASK));
                tb_loop(c, h, N, "DPI flipped byte");
                pans_stream_job_corrupt(h, 1, (long long)r.below(size), 0);
                tb_loop(c, h, N, "DPI truncated");
                pans_stream_job_corrupt(h, 2, 0, (int)r.below(BYTE_MASK + 1));
                tb_loop(c, h, N, "DPI byte appended");
                pans_stream_job_free(h);
                ++jobs;
            }
        }
    }
    // A directed job through begin, set and build, an escape storm on lane 1 of 3.
    {
        const int N = 3;
        const i64 n = 31;
        const int h = pans_stream_job_begin(N, n, int(SourceKind::LIST), 0);
        StreamJob j;
        j.n_lanes = N;
        j.kind = SourceKind::LIST;
        for (i64 i = 0; i < n; ++i) {
            const bool storm = lane_pos(N, i).lane == 1;
            const i64 sym = storm ? (i % 2 ? c.symbol_min() : c.symbol_max()) : 0;
            j.symbols.push_back(sym);
            j.scales.push_back(0);
            pans_stream_job_set(h, i, (int)sym, 0);
        }
        check(pans_stream_job_build(h) == 0, "DPI directed job: build failed");
        const StreamOut e = encode_stream(c, N, j.symbols, j.scales);
        bool same = pans_stream_job_n_bytes(h) == (long long)e.bytes.size() &&
                    pans_stream_job_n_escapes(h) == lane_symbols(N, n, 1) && e.n_escapes == lane_symbols(N, n, 1);
        for (i64 q = 0; same && q < (i64)e.bytes.size(); ++q) same = pans_stream_job_byte(h, q) == e.bytes[q];
        check(same, "DPI directed job: bytes or escapes differ from the model");
        tb_loop(c, h, N, "DPI directed");
        pans_stream_job_free(h);
        ++jobs;
    }
    // The bound job, its escapes' indices, and the loop.
    for (const int N : {1, 8}) {
        const int s = int(mix_seed(seed, 6900 + N) & 0x7fffffff);
        long long i0 = -2, i1 = -2;
        const int h = pans_stream_job_bound(s, N, &i0, &i1);
        if (!check(h >= 0, "pans_stream_job_bound(%d, %d) = %d", s, N, h)) continue;
        const JobGen::BoundJob bj = JobGen(c).bound_job((u64)(unsigned)s, N);
        const StreamOut e = encode_stream(c, N, bj.job.symbols, bj.job.scales);
        bool same = i0 == bj.esc_index[0] && i1 == bj.esc_index[1] && i0 >= 0 &&
                    pans_stream_job_n(h) == (long long)bj.job.symbols.size() &&
                    pans_stream_job_n_bytes(h) == (long long)e.bytes.size();
        for (i64 q = 0; same && q < (i64)e.bytes.size(); ++q) same = pans_stream_job_byte(h, q) == e.bytes[q];
        check(same, "DPI bound job N %d: indexes %lld %lld or bytes differ from the model", N, i0, i1);
        tb_loop(c, h, N, "DPI bound");
        pans_stream_job_free(h);
        ++jobs;
    }
    std::printf("DPI: %ld jobs (N in {1, 3, 8, 16}, every source kind (INT: kind 3) and EMA / INT delay, flipped / "
                "truncated / appended, a directed job, the raw push's bound job): expectations (push beats and steps "
                "included) equal the model's, a testbench's loop follows them: %ld errors\n",
                jobs, g_errors - before);
}

// ---- 6: the package, the scale table, the integer scale estimator

std::string read_text(const std::string& path) {
    std::string s;
    std::FILE* f = std::fopen(path.c_str(), "rb");
    if (!f) return s;
    char buf[1 << 14];
    size_t got;
    while ((got = std::fread(buf, 1, sizeof buf, f)) > 0) s.append(buf, got);
    std::fclose(f);
    return s;
}

// The text without its // comments.
std::string strip_comments(const std::string& s) {
    std::string out;
    for (size_t at = 0; at < s.size();) {
        const size_t c = s.find("//", at), nl = s.find('\n', at);
        const size_t end = nl == std::string::npos ? s.size() : nl;
        out.append(s, at, (c != std::string::npos && c < end ? c : end) - at);
        out += '\n';
        at = end + 1;
    }
    return out;
}

// "localparam <type> NAME = <value>;", the value decimal, optionally 64'sd; false if absent.
bool pkg_param(const std::string& pkg, const char* name, i64* v) {
    std::smatch mt;
    const std::regex re(std::string("localparam\\s+\\w+\\s+") + name + "\\s*=\\s*(?:64'sd)?(-?[0-9]+)\\s*;");
    if (!std::regex_search(pkg, mt, re)) return false;
    *v = std::stoll(mt[1].str());
    return true;
}

// The generated package against the model, SCALE_TABLE entry by entry. No EST_* may be left.
void check_package(const Config& c, const std::string& path) {
    const long before = g_errors;
    const std::string pkg = strip_comments(read_text(path));
    if (!check(!pkg.empty(), "package %s: cannot read it", path.c_str())) return;
    std::smatch mt;
    const bool id = std::regex_search(pkg, mt, std::regex("CONFIG_ID\\s*=\\s*64'h([0-9a-f]+)\\s*;")) &&
                    std::stoull(mt[1].str(), nullptr, 16) == config_id(c);
    check(id, "package %s: CONFIG_ID is not this build's %016llx", path.c_str(), (unsigned long long)config_id(c));
    const struct {
        const char* name;
        i64 want;
    } params[] = {{"CONTRACT_VERSION", CONTRACT_VERSION},
                  {"SYMMETRIC_ALPHABET", c.symmetric_alphabet ? 1 : 0},
                  {"SYMBOL_MIN", c.symbol_min()},
                  {"SYMBOL_MAX", c.symbol_max()},
                  {"B_FRAC", c.b_frac},
                  {"B_WIDTH", c.b_width()},
                  {"B_K", c.b_k},
                  {"B_LEAD_BITS", c.b_lead_bits()},
                  {"SCALE_INDEX_BITS", c.scale_index_bits()},
                  {"SCALE_TABLE_SIZE", c.scale_table_size()}};
    for (const auto& q : params) {
        i64 v = 0;
        const bool found = pkg_param(pkg, q.name, &v);
        check(found && v == q.want, "package: %s = %lld%s, the build's %lld", q.name, (long long)v,
              found ? "" : " (absent)", (long long)q.want);
    }
    check(pkg.find("EST_") == std::string::npos, "package: an EST_ name is left (the estimator left the build)");
    const std::vector<Scale>& t = scale_table(c);
    std::vector<Scale> got;
    bool widths = true;
    if (std::regex_search(pkg, mt,
                          std::regex("scale_t\\s+SCALE_TABLE\\s*\\[[^\\]]*\\]\\s*=\\s*'\\{([^;]*)\\}\\s*;"))) {
        const std::string body = mt[1].str();
        const std::regex entry("\\{\\s*([0-9]+)'d([0-9]+)\\s*,\\s*([0-9]+)'d([0-9]+)\\s*\\}");
        for (auto it = std::sregex_iterator(body.begin(), body.end(), entry); it != std::sregex_iterator(); ++it) {
            widths = widths && std::stoi((*it)[1].str()) == c.scale_exp_width &&
                     std::stoi((*it)[3].str()) == c.scale_man_width;
            got.push_back(Scale{(unsigned)std::stoul((*it)[2].str()), (unsigned)std::stoul((*it)[4].str())});
        }
    }
    bool same = got.size() == t.size() && widths;
    for (size_t j = 0; same && j < t.size(); ++j) same = same_scale(got[j], t[j]);
    check(same, "package: SCALE_TABLE (%zu entries) is not scale_table (%zu entries)%s", got.size(), t.size(),
          widths ? "" : ", or its entries' widths are not the scale's");
    std::printf("scale table: package %s: CONFIG_ID, CONTRACT_VERSION, the alphabet, B_* and SCALE_TABLE (%zu entries) "
                "against the model's, no EST_*: %ld errors\n",
                path.c_str(), got.size(), g_errors - before);
}

// ---- 6a: the coder's scale table (pans1_ref.h, contract version 4)

void scale_table_checks(const Config& c, u64 seed, const std::string& pkg) {
    const long before = g_errors;
    StreamRng r(mix_seed(seed, 7000));
    long pieces = 0, wide = 0, narrow = 0;  // entries whose midpoint's code was special (b beyond / below the format)
    for (const int frac : {0, 3, 8, 12}) {
        for (const int k : {0, 1, 2, 3, 4, 5, 8}) {
            Config cc = c;
            cc.b_frac = frac;
            cc.b_k = k;
            const i64 bound = cc.b_max() + 1;
            if (!check(cc.b_width() >= 1 && cc.b_width() <= 48, "table B_FRAC %d: B_WIDTH %d", frac, cc.b_width()))
                continue;
            // The index of every b below 2^16 (and of b_max), and of random b above, against the plain one.
            const i64 dense = std::min<i64>(bound, i64(1) << 16);
            for (i64 q = 0; q < dense + 200; ++q) {
                const i64 b = q < dense ? q : q == dense ? cc.b_max() : r.below(bound);
                const i64 j = scale_index_of_b(cc, b);
                check(j == plain_table::index(b, k) && j >= 0 && j < cc.scale_table_size(),
                      "table index: b %lld B_K %d gives %lld, plain %lld", (long long)b, k, (long long)j,
                      (long long)plain_table::index(b, k));
                ++pieces;
            }
            // Every entry's midpoint and code against the plain ones. No entry is a special code.
            const std::vector<Scale> t = build_scale_table(cc);
            bool ok = (i64)t.size() == cc.scale_table_size();
            for (i64 j = 0; ok && j < (i64)t.size(); ++j)
                ok = scale_b_mid(cc, j) == plain_table::mid(j, frac, k) &&
                     same_scale(t[j], plain_table::table_scale(cc, j));
            check(ok, "table B_FRAC %d B_K %d: a midpoint or an entry differs from the plain bucket's", frac, k);
            pieces += (long)t.size();
            for (i64 j = 0; j < (i64)t.size(); ++j) {
                check(t[j].exp != 0 && t[j].exp != cc.scale_exp_inf(),
                      "table B_FRAC %d B_K %d: entry %lld is the special code {%u, %u}", frac, k, (long long)j,
                      t[j].exp, t[j].man);
                const Scale raw = scale_from_b(cc, plain_table::mid(j, frac, k));
                wide += raw.exp == 0;
                narrow += raw.exp == cc.scale_exp_inf();
                const ScaleB b = b_of_index(cc, j);
                const bool reached = b >= 0;
                check(!reached || (scale_index_of_b(cc, b) == j && (b == 0 || scale_index_of_b(cc, b - 1) != j)),
                      "b_of_index B_FRAC %d B_K %d: entry %lld gives b %lld", frac, k, (long long)j, (long long)b);
                // no integer b has the index iff b_of_index says so
                const int lead = int(j >> k);
                const bool has_integer =
                    j == 0 || lead >= k || ((j & ((i64(1) << k) - 1)) & ((i64(1) << (k - lead)) - 1)) == 0;
                check(reached == has_integer, "b_of_index B_FRAC %d B_K %d: entry %lld reached %d", frac, k,
                      (long long)j, reached);
            }
            // As b grows the code's slope k never increases. b = 0 gets the narrowest finite code.
            const std::vector<Scale>& tc = scale_table(cc);
            bool mono = true, lookup = true;
            double prev = 0;
            for (i64 q = 0; q <= dense; ++q) {
                const i64 b = q < dense ? q : cc.b_max();
                const Scale sb = scale_of_b(cc, b);
                lookup = lookup && same_scale(sb, t[(size_t)plain_table::index(b, k)]) &&
                         same_scale(sb, tc[(size_t)plain_table::index(b, k)]);
                const double sl = plain_table::slope_units(cc, sb);
                mono = mono && (q == 0 || sl <= prev);
                prev = sl;
            }
            check(lookup, "table B_FRAC %d B_K %d: scale_of_b is not the entry of the plain index", frac, k);
            check(mono, "table B_FRAC %d B_K %d: the slope k grows with b somewhere", frac, k);
            check(same_scale(scale_of_b(cc, 0), Scale{cc.scale_exp_inf() - 1, (1u << cc.scale_man_width) - 1}),
                  "table B_FRAC %d B_K %d: b = 0 is not the narrowest finite code", frac, k);
        }
    }
    // finite_scale on every scale code.
    for (unsigned code = 0; code < (1u << (c.scale_exp_width + c.scale_man_width)); ++code) {
        const Scale s{code >> c.scale_man_width, code & ((1u << c.scale_man_width) - 1)};
        const Scale want = s.exp == 0                  ? Scale{1, 0}
                           : s.exp == c.scale_exp_inf() ? Scale{c.scale_exp_inf() - 1, (1u << c.scale_man_width) - 1}
                                                        : s;
        check(same_scale(finite_scale(c, s), want), "finite_scale {%u, %u}", s.exp, s.man);
        ++pieces;
    }
    // scale_from_sigma is scale_from_b of the discrete variance match (sigmas over the whole range, 0 and inf).
    {
        bool ok = same_scale(scale_from_sigma(c, 0.0), scale_from_b(c, 0.0)) &&
                  same_scale(scale_from_sigma(c, HUGE_VAL), scale_from_b(c, HUGE_VAL));
        for (int q = 0; q < 4000; ++q) {
            const double sigma = std::exp2(-6.0 + 0.01 * q);
            ok = ok && same_scale(scale_from_sigma(c, sigma), scale_from_b(c, laplace_b(sigma)));
        }
        check(ok, "scale_from_sigma differs from scale_from_b(laplace_b(sigma))");
        pieces += 4002;
    }
    // The build's table and the entries that took the nearest finite code.
    {
        const std::vector<Scale>& t = scale_table(c);
        std::string moved;
        long reachable = 0;
        for (i64 j = 0; j < (i64)t.size(); ++j) {
            check(t[j].exp != 0 && t[j].exp != c.scale_exp_inf(), "the build's SCALE_TABLE[%lld] is a special code",
                  (long long)j);
            reachable += b_of_index(c, j) >= 0;
            const Scale raw = scale_from_b(c, scale_b_mid(c, j));
            if ((raw.exp == 0 || raw.exp == c.scale_exp_inf()) && b_of_index(c, j) >= 0)
                moved += " " + std::to_string(j) + " {" + std::to_string(raw.exp) + "," + std::to_string(raw.man) +
                         "}->{" + std::to_string(t[j].exp) + "," + std::to_string(t[j].man) + "}";
        }
        std::printf("scale table: the build's, B_FRAC %d, B_WIDTH %d, B_K %d: %zu entries (%ld reached by an integer b), "
                    "no special code; reachable entries whose midpoint maps to a special code, given the nearest "
                    "finite one:%s\n",
                    c.b_frac, c.b_width(), c.b_k, t.size(), reachable, moved.empty() ? " none" : moved.c_str());
    }
    if (!pkg.empty()) check_package(c, pkg);
    else std::printf("scale table: package not checked (no --pkg)\n");
    std::printf("scale table: %ld definition pieces (index, bucket midpoint, entry, b_of_index, monotone k, b = 0 "
                "the narrowest code, at B_FRAC 0, 3, 8, 12 x B_K 0-5, 8; finite_scale on every code; scale_from_sigma = "
                "scale_from_b(laplace_b)) against the plain ones, no special code in a table (%ld midpoints beyond the "
                "widest normal scale, %ld below the narrowest, each given the nearest finite code): %ld errors\n",
                pieces, wide, narrow, g_errors - before);
}

// ---- 6b: the integer scale estimator (the consumer's, STREAM.md §1.7)

// A start and a floor that fit m at this FRAC on every build.
EstParams sweep_est(const Config& c, int frac, int shift, int delay) {
    const double start = std::min(32.0, std::exp2(c.symbol_width - 2));
    return EstParams{frac, shift, IntScaleSource::m_of_sigma(start, frac),
                     std::max<i64>(1, IntScaleSource::m_of_sigma(0.25, frac)), delay};
}

void int_estimator(const Config& c, u64 seed) {
    const long before = g_errors;
    // The update and m as b against the plain definition.
    StreamRng r(mix_seed(seed, 7050));
    long pieces = 0;
    for (const int frac : {0, 3, 8, 12}) {
        const int mw = IntScaleSource::m_width(c, frac);
        const i64 bound = i64(1) << mw;
        const i64 corner_m[] = {0, 1, bound - 2, bound - 1};
        for (const int shift : {1, 2, 5, 8}) {
            if (shift > mw) continue;
            for (int q = 0; q < 2000; ++q) {
                const i64 m = q < 4 ? corner_m[q] : r.below(bound);
                const i64 x = q % 3 == 0 ? (r.chance(0.5) ? c.symbol_input_min() : c.symbol_max())
                                         : c.symbol_input_min() + r.below(c.symbol_max() - c.symbol_input_min() + 1);
                const i64 u = IntScaleSource::update(m, x, frac, shift);
                check(u == plain_est::update(m, x, frac, shift) && u >= 0 && u < bound,
                      "estimator update: m %lld x %lld FRAC %d SHIFT %d gives %lld, plain %lld, bound 2^%d",
                      (long long)m, (long long)x, frac, shift, (long long)u,
                      (long long)plain_est::update(m, x, frac, shift), mw);
                ++pieces;
            }
        }
        for (int q = 0; q < 2000; ++q) {
            const i64 m = q < 4 ? corner_m[q] : r.below(bound);
            const ScaleB b = b_of_m(c, m, frac);
            check(b == plain_b_of_m(c, m, frac) && b_valid(c, b), "m as b: m %lld FRAC %d gives %lld, plain %lld",
                  (long long)m, frac, (long long)b, (long long)plain_b_of_m(c, m, frac));
            ++pieces;
        }
    }
    // The parameter checks: the example set is an estimator; each bound refuses one step beyond it.
    {
        const EstParams b = IntScaleSource::example(c);
        const int mw = IntScaleSource::m_width(c, b.frac);
        check(IntScaleSource::problems(c, b).empty(), "the example estimator parameters are refused");
        const auto refused = [&c, &b](const char* what, auto change) {
            EstParams p = b;
            change(p);
            check(!IntScaleSource::problems(c, p).empty(), "estimator parameters not refused: %s", what);
        };
        refused("SHIFT 0", [](EstParams& p) { p.shift = 0; });
        refused("SHIFT beyond m's width", [mw](EstParams& p) { p.shift = mw + 1; });
        refused("m0 = 2^(S-1+FRAC)", [mw](EstParams& p) { p.m0 = i64(1) << mw; });
        refused("m0 < 0", [](EstParams& p) { p.m0 = -1; });
        refused("m_floor < 0", [](EstParams& p) { p.m_floor = -1; });
        refused("m_floor = 2^(S-1+FRAC)", [mw](EstParams& p) { p.m_floor = i64(1) << mw; });
        refused("delay < 0", [](EstParams& p) { p.delay = -1; });
        refused("FRAC < 0", [](EstParams& p) { p.frac = -1; });
        refused("m wider than M_WIDTH_MAX", [](EstParams& p) { p.frac = IntScaleSource::M_WIDTH_MAX; });
    }

    // Round trips over a sweep of the parameters, every set's b recomputed from the definition (round_trip).
    JobGen gen(c);
    long sets = 0, symbols = 0;
    u64 k = 0;
    for (const int frac : {0, 3, 8, 12})
        for (const int shift : {1, 3, 5, 8}) {
            const int N = LANE_COUNTS[sets % 4], delay = int(sets % 3);
            StreamJob j = gen.make(mix_seed(seed, 7100 + ++k), N, 300 + r.below(1500), SourceKind::INT, delay);
            j.est_custom = true;
            j.est = sweep_est(c, frac, std::min(shift, IntScaleSource::m_width(c, frac)), delay);
            job_replay(c, j);
            round_trip(c, j, "estimator sweep");
            ++sets;
            symbols += (i64)j.symbols.size();
        }
    // Each parameter moved alone from the example set changes the scale inputs of one job.
    long moved = 0;
    {
        const EstParams b = IntScaleSource::example(c);
        StreamJob base = gen.make(mix_seed(seed, 7900), 2, 6000, SourceKind::INT, b.delay);
        base.est_custom = true;
        base.est = b;
        job_replay(c, base);
        round_trip(c, base, "estimator, the example set");
        const int mw = IntScaleSource::m_width(c, b.frac);
        const auto move = [&](const char* what, auto change) {
            StreamJob j = base;
            change(j.est);
            if (!check(IntScaleSource::problems(c, j.est).empty(), "estimator move %s: not an estimator", what)) return;
            job_replay(c, j);
            round_trip(c, j, "estimator, one parameter moved");
            bool differ = false;
            for (size_t i = 0; !differ && i < j.scales.size(); ++i) differ = j.scales[i] != base.scales[i];
            check(differ, "estimator: moving %s alone leaves every scale input as it was", what);
            ++moved;
        };
        move("FRAC", [](EstParams& p) {  // FRAC - 2, m0 and m_floor in m's new units
            p.frac -= 2;
            p.m0 /= 4;
            p.m_floor = std::max<i64>(1, p.m_floor / 4);
        });
        move("SHIFT", [&b](EstParams& p) { p.shift = b.shift > 1 ? b.shift - 1 : b.shift + 1; });
        move("m0", [&b](EstParams& p) { p.m0 = std::max<i64>(1, b.m0 / 16); });
        // The floor 16 times higher and at least 2^(FRAC + 3), so that it changes the scale inputs.
        move("m_floor", [&b, mw](EstParams& p) {
            p.m_floor = std::min((i64(1) << mw) - 1, std::max(b.m_floor * 16, i64(1) << (b.frac + 3)));
        });
        move("delay", [&b](EstParams& p) { p.delay = b.delay + 1; });
    }
    std::printf("integer estimator: %ld definition pieces (update at FRAC 0, 3, 8, 12 x SHIFT 1, 2, 5, 8; m as b) against "
                "the plain ones; parameter bounds refused; %ld swept sets (FRAC x SHIFT, delays 0-2, N in {1, 2, 8, "
                "16}), %ld symbols; %ld single moves (FRAC, SHIFT, m0, m_floor, delay) each change the scale inputs: "
                "%ld errors\n",
                pieces, sets, symbols, moved, g_errors - before);
}

// ---- 7: ref/stream_tool.cpp

// A symbol file of `width`-bit two's-complement values, little-endian.
bool write_symbols(const std::string& path, const std::vector<i64>& s, int width) {
    std::FILE* f = std::fopen(path.c_str(), "wb");
    if (!f) return false;
    for (const i64 x : s)
        for (int k = 0; k < width / 8; ++k) std::fputc(int((u64(x) >> (8 * k)) & 0xff), f);
    return std::fclose(f) == 0;
}

// A b file: one integer per line.
bool write_b_list(const std::string& path, const std::vector<ScaleB>& bs) {
    std::FILE* f = std::fopen(path.c_str(), "w");
    if (!f) return false;
    for (const ScaleB b : bs) std::fprintf(f, "%lld\n", (long long)b);
    return std::fclose(f) == 0;
}

std::vector<std::string> split_csv(const std::string& line) {  // the tool's rows here have no quoted fields
    std::vector<std::string> out;
    for (size_t at = 0;;) {
        const size_t comma = line.find(',', at);
        out.push_back(line.substr(at, comma == std::string::npos ? std::string::npos : comma - at));
        if (comma == std::string::npos) return out;
        at = comma + 1;
    }
}

// The tool with `args`, its stderr into `log`: the exit status, and stdout's lines.
int run_tool(const std::string& tool, const std::string& args, const std::string& log,
             std::vector<std::string>& lines) {
    lines.clear();
    std::FILE* p = popen((tool + " " + args + " 2> " + log).c_str(), "r");
    if (!p) return -1;
    std::string line;
    for (int ch; (ch = std::fgetc(p)) != EOF;) {
        if (ch != '\n') {
            line += char(ch);
            continue;
        }
        lines.push_back(line);
        line.clear();
    }
    if (!line.empty()) lines.push_back(line);
    const int st = pclose(p);
    return WIFEXITED(st) ? WEXITSTATUS(st) : -1;
}

// A fresh scale source as the tool builds one: the host's replay and the decoder's consumer.
using SourceMaker = std::function<std::unique_ptr<ScaleSource>()>;

// A row against the model: the stream's counts exactly, the bit columns to their printed decimals.
void check_row(const Config& c, int N, const SourceMaker& make, const std::string& file, const std::vector<i64>& s,
               const std::vector<std::string>& row, const char* what) {
    constexpr double BITS_ULP = 5e-4, BPS_ULP = 5e-7;  // half the last printed decimal (%.3f, %.6f)
    constexpr size_t COLUMNS = 13;
    if (!check(row.size() == COLUMNS && row[0] == file, "stream_tool %s %s: a row of %zu fields for %s", what,
               file.c_str(), row.size(), row.empty() ? "" : row[0].c_str()))
        return;
    const std::unique_ptr<ScaleSource> host = make(), consumer = make();
    const StreamOut e = encode_stream(c, N, s, replay_scales(N, s, *host));
    const DecodeOut d = decode_stream(c, N, e.bytes, (i64)s.size(), *consumer);
    double coded = 0;
    for (const EncStep& st : e.trace)
        if (!st.escape) coded += c.precision - std::log2((double)st.freq);
    const double esc = double(e.n_escapes) * (c.precision - std::log2((double)c.escape_slots) + c.symbol_width);
    const double n = double(s.size()), actual = 8.0 * double(e.bytes.size());
    const auto near = [](const std::string& f, double want, double ulp) {
        return std::fabs(std::stod(f) - want) <= ulp * (1 + 1e-9);
    };
    bool ok = d.ok && std::stoll(row[1]) == (i64)s.size() && std::stoll(row[2]) == N &&
              std::stoll(row[3]) == (i64)e.bytes.size() && std::stoll(row[4]) == header_bytes(c, N) &&
              std::stoll(row[5]) == (i64)e.words.size() && std::stoll(row[6]) == e.n_escapes;
    ok = ok && near(row[7], coded, BITS_ULP) && near(row[8], esc, BITS_ULP) && near(row[9], coded + esc, BITS_ULP) &&
         std::stoll(row[10]) == (i64)actual;
    ok = ok && (s.empty() ? row[11] == "nan" && row[12] == "nan"
                          : near(row[11], actual / n, BPS_ULP) && near(row[12], (coded + esc) / n, BPS_ULP));
    check(ok, "stream_tool %s %s: the row differs from encode_stream (bytes %lld, words %lld, escapes %lld)", what,
          file.c_str(), (long long)e.bytes.size(), (long long)e.words.size(), (long long)e.n_escapes);
}

void stream_tool(const Config& c, u64 seed, const std::string& tool) {
    if (tool.empty()) {
        std::printf("stream_tool: not checked (no --tool)\n");
        return;
    }
    const long before = g_errors;
    const std::string dir = tool.substr(0, tool.find_last_of('/') + 1) + "stream_tool_check";
    ::mkdir(dir.c_str(), 0755);
    const std::string log = dir + "/stderr.log";
    const int width = c.symbol_width <= 8 ? 8 : 16;  // the tool's default
    JobGen gen(c);
    StreamRng r(mix_seed(seed, 8000));
    // Streams with drifting sigma, empty, extreme and random symbols.
    std::vector<std::pair<std::string, std::vector<i64>>> files = {
        {dir + "/drift.bin", gen.make(mix_seed(seed, 8001), 8, 6000, SourceKind::INT, 1).symbols},
        {dir + "/empty.bin", {}},
        {dir + "/extremes.bin", {}},
        {dir + "/mix.bin", gen.make(mix_seed(seed, 8002), 8, 3000, SourceKind::LIST, 0).symbols}};
    for (int i = 0; i < 37; ++i)
        files[2].second.push_back(i % 3 == 0 ? c.symbol_input_min() : i % 3 == 1 ? c.symbol_max() : r.below(3) - 1);
    std::string names;
    for (const auto& f : files) {
        check(write_symbols(f.first, f.second, width), "stream_tool: cannot write %s", f.first.c_str());
        names += " " + f.first;
    }
    std::vector<std::string> out;
    long rows = 0;
    // The default source, the example adaptive estimator, at the default lane count.
    const EstParams example = IntScaleSource::example(c);
    const SourceMaker adaptive = [&c, &example] { return std::make_unique<IntScaleSource>(c, example); };
    int st = run_tool(tool, names, log, out);
    if (check(st == 0 && out.size() == files.size() + 1, "stream_tool (the example estimator): exit %d, %zu lines; "
              "see %s", st, out.size(), log.c_str())) {
        check(out[0] == "file,n,lanes,bytes,header_bytes,words,n_escapes,ideal_coded_bits,escape_bits,"
                        "ideal_total_bits,actual_bits,actual_bps,ideal_bps",
              "stream_tool: header row %s", out[0].c_str());
        for (size_t f = 0; f < files.size(); ++f, ++rows)
            check_row(c, 8, adaptive, files[f].first, files[f].second, split_csv(out[f + 1]), "example estimator");
    }
    // Another estimator and lane count, the start and the floor given as sigmas.
    const double start = std::min(8.0, std::exp2(c.symbol_width - 2));
    const EstParams other{6, 3, IntScaleSource::m_of_sigma(start, 6), IntScaleSource::m_of_sigma(0.5, 6), 2};
    const SourceMaker other_src = [&c, &other] { return std::make_unique<IntScaleSource>(c, other); };
    char args[200];
    std::snprintf(args, sizeof args, "--lanes 3 --adaptive --frac 6 --shift 3 --start-sigma %g --floor-sigma 0.5 "
                  "--delay 2", start);
    st = run_tool(tool, std::string(args) + " " + files[0].first + " " + files[3].first, log, out);
    if (check(st == 0 && out.size() == 3, "stream_tool (%s): exit %d, %zu lines; see %s", args, st, out.size(),
              log.c_str())) {
        check_row(c, 3, other_src, files[0].first, files[0].second, split_csv(out[1]), "other estimator");
        check_row(c, 3, other_src, files[3].first, files[3].second, split_csv(out[2]), "other estimator");
        rows += 2;
    }
    // A constant b: the table's first and last indices and one in between.
    for (const ScaleB b : {ScaleB(0), ScaleB(1) << c.b_frac, c.b_max()}) {
        const SourceMaker cst = [b] { return std::make_unique<ConstScaleSource>(b); };
        st = run_tool(tool, "--lanes 5 --const " + std::to_string(b) + " " + files[0].first + " " + files[2].first,
                      log, out);
        if (check(st == 0 && out.size() == 3, "stream_tool (--const %lld): exit %d, %zu lines; see %s", (long long)b,
                  st, out.size(), log.c_str())) {
            check_row(c, 5, cst, files[0].first, files[0].second, split_csv(out[1]), "constant b");
            check_row(c, 5, cst, files[2].first, files[2].second, split_csv(out[2]), "constant b");
            rows += 2;
        }
    }
    // A b list per symbol file, random log-uniform b, two files in one call.
    std::vector<std::vector<ScaleB>> lists(2);
    const size_t listed[2] = {0, 3};
    for (int k = 0; k < 2; ++k) {
        for (size_t i = 0; i < files[listed[k]].second.size(); ++i) {
            const int nbits = int(r.below(c.b_width() + 1));  // b's leading one at bit nbits - 1, or b = 0
            const ScaleB b = nbits == 0 ? 0 : (ScaleB(1) << (nbits - 1)) + r.below(ScaleB(1) << (nbits - 1));
            lists[k].push_back(i % 97 == 0 ? 0 : i % 97 == 1 ? c.b_max() : b);
        }
        check(write_b_list(files[listed[k]].first + ".b", lists[k]), "stream_tool: cannot write the b list");
    }
    st = run_tool(tool, "--b-list " + files[0].first + " " + files[0].first + ".b " + files[3].first + " " +
                            files[3].first + ".b", log, out);
    if (check(st == 0 && out.size() == 3, "stream_tool (--b-list): exit %d, %zu lines; see %s", st, out.size(),
              log.c_str())) {
        for (int k = 0; k < 2; ++k) {
            const std::vector<ScaleB>& bs = lists[k];
            const SourceMaker lst = [&bs] { return std::make_unique<ListScaleSource>(bs); };
            check_row(c, 8, lst, files[listed[k]].first, files[listed[k]].second, split_csv(out[k + 1]), "b list");
            ++rows;
        }
    }
    // The other symbol width gives the same rows from the same symbols.
    {
        const int w2 = width == 8 ? 16 : 8;
        bool fits = true;
        for (const i64 x : files[0].second) fits = fits && x >= INT8_MIN && x <= INT8_MAX;
        if (w2 == 16 || fits) {
            const std::string f2 = dir + "/drift_w" + std::to_string(w2) + ".bin";
            check(write_symbols(f2, files[0].second, w2), "stream_tool: cannot write %s", f2.c_str());
            st = run_tool(tool, "--width " + std::to_string(w2) + " " + f2, log, out);
            if (check(st == 0 && out.size() == 2, "stream_tool (--width %d): exit %d, %zu lines", w2, st, out.size())) {
                check_row(c, 8, adaptive, f2, files[0].second, split_csv(out[1]), "the other width");
                ++rows;
            }
        }
    }
    // Failures: a missing file, a symbol outside the inputs, an odd int16 file, bad b lists, bad options.
    st = run_tool(tool, dir + "/missing.bin " + files[2].first, log, out);
    check(st == 1 && out.size() == 2, "stream_tool: a missing file: exit %d (expected 1), %zu lines (expected 2)", st,
          out.size());
    if (c.symbol_width < 16) {
        const std::string bad = dir + "/outside.bin";
        check(write_symbols(bad, {0, 1, c.symbol_max() + 1}, 16), "stream_tool: cannot write %s", bad.c_str());
        st = run_tool(tool, "--width 16 " + bad, log, out);
        check(st == 1 && out.size() == 1, "stream_tool: a symbol outside the inputs: exit %d (expected 1), %zu lines",
              st, out.size());
    }
    {
        const std::string odd = dir + "/odd.bin";
        check(write_symbols(odd, {0, 1, 2}, 8), "stream_tool: cannot write %s", odd.c_str());
        st = run_tool(tool, "--width 16 " + odd, log, out);
        check(st == 1 && out.size() == 1, "stream_tool: an odd byte count at width 16: exit %d (expected 1)", st);
    }
    {
        std::vector<ScaleB> short_list(lists[0].begin(), lists[0].end() - 1), wide = lists[0];
        wide[wide.size() / 2] = c.b_max() + 1;
        check(write_b_list(dir + "/short.b", short_list) && write_b_list(dir + "/wide.b", wide),
              "stream_tool: cannot write the bad b lists");
        st = run_tool(tool, "--b-list " + files[0].first + " " + dir + "/short.b", log, out);
        check(st == 1 && out.size() == 1, "stream_tool: a b list one short: exit %d (expected 1)", st);
        st = run_tool(tool, "--b-list " + files[0].first + " " + dir + "/wide.b", log, out);
        check(st == 1 && out.size() == 1, "stream_tool: a b beyond B_WIDTH bits: exit %d (expected 1)", st);
    }
    const std::string one = " " + files[2].first;
    const std::pair<const char*, std::string> usage[] = {
        {"--const two", "--const two" + one},
        {"--const 2^B_WIDTH", "--const " + std::to_string(c.b_max() + 1) + one},
        {"--const with --shift", "--const 3 --shift 2" + one},
        {"two sources", "--const 3 --adaptive" + one},
        {"--b-list with an odd operand count", "--b-list" + one},
        {"--frac without m0 and m_floor", "--frac 6" + one},
        {"--width 12", "--width 12" + one},
    };
    for (const auto& u : usage) {
        st = run_tool(tool, u.second, log, out);
        check(st == 2, "stream_tool: %s accepted (exit %d, expected 2)", u.first, st);
    }
    std::printf("stream_tool: %ld rows (the example estimator at N = 8 on drift / empty / extremes / mix; FRAC 6, "
                "SHIFT 3, delay 2, sigmas for m0 and the floor at N = 3; constant b at 0, 1 and 2^B_WIDTH - 1; b lists "
                "of random b; int%d files) equal to encode_stream's; a missing file, %san odd int16 file, b lists too "
                "short and too wide, %zu bad option sets fail: %ld errors\n",
                rows, width == 8 ? 16 : 8, c.symbol_width < 16 ? "a symbol outside the inputs, " : "",
                sizeof usage / sizeof usage[0], g_errors - before);
}

}  // namespace

int main(int argc, char** argv) {
    const Config c = build_config();
    u64 seed = 1;
    std::string pkg, tool;
    for (int a = 1; a < argc; ++a) {
        const std::string opt = argv[a];
        if ((opt == "--pkg" || opt == "--tool") && a + 1 < argc) (opt == "--pkg" ? pkg : tool) = argv[++a];
        else seed = std::strtoull(argv[a], nullptr, 0);
    }
    std::printf("P=%d SW=%d E=%d: words W=%d, state %d bits in %d bytes; scale input b B_FRAC %d B_WIDTH %d, table "
                "B_K %d (%lld entries); CONFIG_ID %016llx (contract version %d); seed %llu\n",
                c.precision, c.symbol_width, c.escape_slots, c.word_width(), c.state_width(), c.state_bytes(),
                c.b_frac, c.b_width(), c.b_k, (long long)c.scale_table_size(), (unsigned long long)config_id(c),
                CONTRACT_VERSION, (unsigned long long)seed);
    g_cov.index_hits.assign((size_t)c.scale_table_size(), 0);
    round_trips(c, seed);
    directed(c, seed);
    raw_push(c, seed);
    corrupt_arrays(c, seed);
    pans1_cuh(c, seed);
    dpi(c, seed);
    scale_table_checks(c, seed, pkg);
    int_estimator(c, seed);
    stream_tool(c, seed, tool);

    // Coverage. A run that never reaches one of these corners fails.
    const bool two_words_possible = c.word_width() + log2_escape(c) < c.precision + c.symbol_width;
    const bool cross_possible = (c.escape_slots & (c.escape_slots - 1)) == 0 && c.word_width() > c.precision &&
                                c.word_width() < c.precision + c.symbol_width - log2_escape(c);
    long idx_reachable = 0, idx_hit = 0;
    for (i64 j = 0; j < c.scale_table_size(); ++j) {
        if (b_of_index(c, j) < 0) continue;
        ++idx_reachable;
        idx_hit += g_cov.index_hits[(size_t)j] > 0;
    }
    std::printf("coverage: %ld symbols, %ld escapes (moving 0/1/2 words %ld/%ld/%ld), %ld with f = 1, %ld with "
                "f = M', %ld at b = 0, %ld at the table's last index, %ld with b zero-filled (0 < b < 2^B_K), table "
                "indexes reached %ld of the %ld an integer b has, %ld empty lanes; encoder steps emitting 0/1/2 words "
                "%ld/%ld/%ld, decoder steps reading 0/1/2 words %ld/%ld/%ld%s\n",
                g_cov.symbols, g_cov.escapes, g_cov.esc_words[0], g_cov.esc_words[1], g_cov.esc_words[2], g_cov.f_one,
                g_cov.f_max, g_cov.b_zero, g_cov.b_top, g_cov.zero_fill, idx_hit, idx_reachable, g_cov.empty_lanes,
                g_cov.enc_words[0], g_cov.enc_words[1], g_cov.enc_words[2], g_cov.dec_words[0], g_cov.dec_words[1],
                g_cov.dec_words[2], two_words_possible ? "" : " (2 words impossible: W + log2 E >= P + S)");
    check(g_cov.escapes > 0 && g_cov.f_one > 0 && g_cov.f_max > 0 && g_cov.b_zero > 0 && g_cov.b_top > 0 &&
              g_cov.zero_fill > 0 && idx_hit == idx_reachable && g_cov.empty_lanes > 0,
          "coverage: escapes, f = 1, f = M', b = 0, the table's last index, a zero-filled index, every table index an "
          "integer b has, and empty lanes must all occur");
    check(g_cov.enc_words[0] > 0 && g_cov.enc_words[1] > 0 && (g_cov.enc_words[2] > 0 || !two_words_possible) &&
              (g_cov.dec_words[2] > 0 || !two_words_possible),
          "coverage: steps moving 0, 1 and (if W + log2 E < P + S) 2 words must all occur");
    check(g_cov.err_words > 0 && g_cov.err_header > 0 && g_cov.end_only > 0,
          "coverage: every error flag (words, header) and an end-check-only failure must occur");
    const bool below_possible = raw_push_bound(c) > state_L(c);  // B - 1 is a state (S < W)
    std::printf("coverage: raw push from x = B - 1 %ld, x = B %ld (single states); stream escapes reading B %ld, B - 1 "
                "%ld%s; escapes whose raw value crosses a word boundary %ld%s\n",
                g_cov.raw_below, g_cov.raw_at, g_cov.bound_hi, g_cov.bound_lo,
                below_possible ? "" : " (B - 1 < L: S = W)", g_cov.raw_cross,
                cross_possible ? "" : " (impossible: not P < W < P + S - log2 E)");
    check(g_cov.raw_at > 0 && g_cov.bound_hi > 0 && ((g_cov.raw_below > 0 && g_cov.bound_lo > 0) || !below_possible),
          "coverage: the raw push at x = B and (if B - 1 >= L) x = B - 1, single states and stream escapes");
    check(g_cov.raw_cross > 0 || !cross_possible, "coverage: a raw value crossing a word boundary must occur");
    if (c.symbol_input_min() < c.symbol_min()) {
        std::printf("coverage: %ld inputs %lld outside the symmetric alphabet, every one escaped\n", g_cov.outside,
                    (long long)c.symbol_input_min());
        check(g_cov.outside > 0, "coverage: a symmetric alphabet's input outside it must occur");
    }
    std::printf("integer estimator coverage: %ld scale inputs checked against the definition, %ld below the floor; m "
                "moved up %ld and down %ld times\n",
                g_cov.est_scales, g_cov.est_floor, g_cov.est_up, g_cov.est_down);
    check(g_cov.est_floor > 0 && g_cov.est_up > 0 && g_cov.est_down > 0,
          "coverage: the estimator's floor and m moving both ways must all occur");
    std::printf("%ld checks, %ld errors\n", g_checks, g_errors);
    return g_errors ? 1 : 0;
}
