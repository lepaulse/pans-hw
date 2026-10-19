// SPDX-License-Identifier: Apache-2.0
// pans_stream_ref.h: the bit-exact model of the stream, N interleaved ANS lanes over the cores' model.
// It is the normative definition. The stream units implement it, and their function files name each counterpart.
// Specification: STREAM.md §1.
// Corrupt input decodes deterministically, so the RTL compares bit for bit on any input.
// The last section, Jobs, is seeded stimulus shared by ref/test_stream.cpp and ref/pans_stream_dpi.cpp.
#pragma once
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <deque>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "pans1_ref.h"

namespace pans1hw {

using u64 = uint64_t;
using ScaleB = i64;  // the scale input b, below 2^B_WIDTH

constexpr u64 BYTE_MASK = (u64(1) << BYTE_BITS) - 1;
constexpr int STATE_WIDTH_MAX = 63;  // the model's state is a u64, the DPI's a longint (signed 64 bits)

// ---- Preconditions ------------------------------------------------------------------------------
// Misuse of the model is the caller's bug. It is reported and aborts.

[[noreturn]] inline void stream_fail(const char* what) {
    std::fflush(stdout);  // the caller's report so far, before the reason
    std::fprintf(stderr, "pans_stream_ref: %s\n", what);
    std::abort();
}

inline void stream_require(bool ok, const char* what) {
    if (!ok) stream_fail(what);
}

inline bool scale_valid(const Config& c, Scale s) {
    return s.exp <= c.scale_exp_inf() && s.man < (1u << c.scale_man_width);
}

inline bool b_valid(const Config& c, ScaleB b) { return b >= 0 && b <= c.b_max(); }

// A symbol the stream takes, any S-bit value (STREAM.md §1.4).
inline bool symbol_valid(const Config& c, i64 x) { return x >= c.symbol_input_min() && x <= c.symbol_max(); }

inline void stream_require_build(const Config& c, int n_lanes) {
    stream_require(c.state_width() <= STATE_WIDTH_MAX, "STATE_WIDTH beyond the model's 63 bits");
    stream_require(c.word_width() >= c.precision, "WORD_WIDTH < PRECISION");  // by construction; the one-word rule
    stream_require(n_lanes >= 1, "a stream needs at least one lane");
}

// ---- Geometry -----------------------------------------------------------------------------------

inline u64 word_mask(const Config& c) { return (u64(1) << c.word_width()) - 1; }
inline u64 state_mask(const Config& c) { return (u64(1) << c.state_width()) - 1; }
inline int word_bytes(const Config& c) { return c.word_width() / BYTE_BITS; }
inline i64 header_bytes(const Config& c, int n_lanes) { return i64(n_lanes) * c.state_bytes(); }
inline u64 state_L(const Config& c) { return (u64)c.slots(); }  // L = M (pANS 1.0's L_LOW = 2^16)

struct LanePos {
    int lane;  // i mod N
    i64 t;     // i div N, the position within the lane
};

inline LanePos lane_pos(int n_lanes, i64 i) { return {int(i % n_lanes), i / n_lanes}; }

// The number of symbols of `lane` among n.
inline i64 lane_symbols(int n_lanes, i64 n, int lane) { return n / n_lanes + (lane < n % n_lanes ? 1 : 0); }

// An escaped symbol's raw value sym mod 2^S, and back by sign extension (RTL: raw_symbol).
inline i64 raw_of(const Config& c, i64 sym) { return sym & ((i64(1) << c.symbol_width) - 1); }

inline i64 symbol_of_raw(const Config& c, i64 v) {
    const i64 m = i64(1) << c.symbol_width, low = v & (m - 1);
    return low >= m / 2 ? low - m : low;
}

inline void put_le(std::vector<uint8_t>& b, u64 v, int n_bytes) {
    for (int k = 0; k < n_bytes; ++k) b.push_back(uint8_t((v >> (BYTE_BITS * k)) & BYTE_MASK));
}

// ---- Encoder ------------------------------------------------------------------------------------

// One ANS push, pANS 1.0's push generalized to P and W (STREAM.md §1.4).
inline void stream_push(const Config& c, u64& x, std::vector<u64>& w, i64 c0, i64 f) {
    const int W = c.word_width();
    const u64 M = (u64)c.slots();
    if ((u64)f < M && x >= ((u64)f << W)) {
        w.push_back(x & word_mask(c));
        x >>= W;
    }
    x = (x / (u64)f) * M + (u64)c0 + x % (u64)f;
}

// The raw push's renormalization bound 2^(P + W - S) (stream_push_raw).
inline u64 raw_push_bound(const Config& c) { return u64(1) << (c.precision + c.word_width() - c.symbol_width); }

// An escape's raw push of v as S bits of the state (STREAM.md §1.4, contract version 5).
inline void stream_push_raw(const Config& c, u64& x, std::vector<u64>& w, i64 raw) {
    stream_require(raw >= 0 && raw < (i64(1) << c.symbol_width), "stream_push_raw: a raw value beyond S bits");
    if (x >= raw_push_bound(c)) {
        w.push_back(x & word_mask(c));
        x >>= c.word_width();
    }
    x = (x << c.symbol_width) | (u64)raw;
}

// One symbol, pANS 1.0's encode_one: a push, or the raw push and then the escape (M', E).
inline EncOut stream_encode_one(const Config& c, u64& x, std::vector<u64>& w, i64 sym, Scale s) {
    const EncOut e = encode(c, s, sym);
    if (!e.escape) {
        stream_push(c, x, w, e.start, e.freq);
    } else {
        stream_push_raw(c, x, w, raw_of(c, sym));
        stream_push(c, x, w, c.coded_slots(), c.escape_slots);
    }
    return e;
}

struct EncStep {      // one symbol through the encoder
    i64 i;            // global index
    int lane;         // i mod N
    i64 t;            // i div N
    i64 symbol;
    ScaleB b;         // the scale input
    Scale scale;      // its code, the core's scale
    i64 start, freq;  // the core's interval; (M', E) for an escape
    bool escape;
    int words;        // words emitted, up to 2 for an escape
    u64 state;        // the lane's state after the symbol
};

struct StreamOut {
    std::vector<uint8_t> bytes;  // the byte array, exactly as the decoder reads it
    std::vector<u64> states;     // the final state of each lane (the array's header)
    std::vector<u64> words;      // the words in emission order
    i64 n_escapes = 0;
    std::vector<EncStep> trace;  // push order: trace[k] is global index i = n - 1 - k
};

// The encoder: symbols and their scale inputs in global order, pushed in reverse into the byte array (STREAM.md §1.5).
inline StreamOut encode_stream(const Config& c, int n_lanes, const std::vector<i64>& symbols,
                               const std::vector<ScaleB>& scales) {
    stream_require_build(c, n_lanes);
    stream_require(symbols.size() == scales.size(), "encode_stream: one scale input per symbol");
    const i64 n = (i64)symbols.size();
    StreamOut o;
    o.states.assign(n_lanes, state_L(c));
    o.trace.reserve(n);
    for (i64 i = n - 1; i >= 0; --i) {
        stream_require(symbol_valid(c, symbols[i]), "encode_stream: a symbol outside the S-bit range");
        stream_require(b_valid(c, scales[i]), "encode_stream: a scale input b beyond B_WIDTH bits");
        const LanePos p = lane_pos(n_lanes, i);
        u64& x = o.states[p.lane];
        const size_t before = o.words.size();
        const Scale code = scale_of_b(c, scales[i]);
        const EncOut e = stream_encode_one(c, x, o.words, symbols[i], code);
        o.n_escapes += e.escape ? 1 : 0;
        o.trace.push_back({i, p.lane, p.t, symbols[i], scales[i], code, e.start, e.freq, e.escape,
                           int(o.words.size() - before), x});
    }
    o.bytes.reserve(header_bytes(c, n_lanes) + i64(o.words.size()) * word_bytes(c));
    for (const u64 x : o.states) put_le(o.bytes, x, c.state_bytes());
    for (size_t k = o.words.size(); k-- > 0;) put_le(o.bytes, o.words[k], word_bytes(c));
    return o;
}

// ---- Decoder ------------------------------------------------------------------------------------

// The decoder's view of the array. A read beyond the array gives 0 and counts as an over-read.
class StreamReader {
  public:
    StreamReader(const Config& c, int n_lanes, const std::vector<uint8_t>& bytes)
        : bytes_(bytes), state_bytes_(c.state_bytes()), word_bytes_(word_bytes(c)),
          header_(header_bytes(c, n_lanes)) {
        const i64 size = (i64)bytes.size();
        words_total_ = size > header_ ? (size - header_) / word_bytes_ : 0;
    }

    u64 header_state(int lane) { return read_le(i64(lane) * state_bytes_, state_bytes_); }

    u64 next_word() {
        if (words_read_ >= words_total_) {
            ++over_reads_;
            return 0;
        }
        return read_le(header_ + (words_read_++) * word_bytes_, word_bytes_);
    }

    i64 words_read() const { return words_read_; }
    i64 words_total() const { return words_total_; }
    i64 over_reads() const { return over_reads_; }
    i64 header() const { return header_; }
    i64 size() const { return (i64)bytes_.size(); }

  private:
    u64 read_le(i64 at, int n_bytes) {
        u64 v = 0;
        bool beyond = false;
        for (int k = 0; k < n_bytes; ++k) {
            const bool in = at + k < (i64)bytes_.size();
            beyond = beyond || !in;
            v |= u64(in ? bytes_[at + k] : 0) << (BYTE_BITS * k);
        }
        over_reads_ += beyond ? 1 : 0;
        return v;
    }

    const std::vector<uint8_t>& bytes_;
    int state_bytes_, word_bytes_;
    i64 header_, words_total_ = 0, words_read_ = 0, over_reads_ = 0;
};

// One ANS pop, pANS 1.0's dec_pop generalized to P and W (STREAM.md §1.4). Returns the words read.
inline int stream_pop(const Config& c, u64& x, i64 s, i64 c0, i64 f, StreamReader& r) {
    x = ((x >> c.precision) * (u64)f + (u64)(s - c0)) & state_mask(c);
    if (x < (u64)c.slots()) {
        x = (x << c.word_width()) | r.next_word();
        return 1;
    }
    return 0;
}

// The escape's raw pop, stream_push_raw undone (STREAM.md §1.4). Returns the words read.
inline int stream_pop_raw(const Config& c, u64& x, i64& raw, StreamReader& r) {
    raw = (i64)(x & ((u64(1) << c.symbol_width) - 1));
    x >>= c.symbol_width;
    if (x < (u64)c.slots()) {
        x = (x << c.word_width()) | r.next_word();
        return 1;
    }
    return 0;
}

// A decode step's error bits. The only one is a read beyond the array (STREAM.md §1.6).
constexpr unsigned ERR_WORDS = 1u << 0;     // a read beyond the array

struct DecStep {      // one symbol through the decoder
    int lane;         // i mod N
    i64 t;            // i div N
    ScaleB b;         // the scale input, from the scale source
    Scale scale;      // its code, the core's scale
    i64 slot;         // x mod M before the step
    i64 start, freq;  // the core's interval for the slot; (M', E) for an escape
    bool escape;
    i64 raw;          // an escape's raw value v, else 0
    i64 symbol;       // the decoded symbol
    int words;        // words read, up to 2 for an escape
    u64 state;        // the lane's state after the step
    unsigned errors;  // the ERR_* bits this step raised
};

// One symbol, pANS 1.0's decode_next with the cores' exact inverse (STREAM.md §1.4).
inline DecStep stream_decode_next(const Config& c, u64& x, ScaleB b, StreamReader& r) {
    const i64 over_before = r.over_reads();
    DecStep st{};
    st.b = b;
    st.scale = scale_of_b(c, b);
    st.slot = (i64)(x & (u64)(c.slots() - 1));
    const DecOut d = decode(c, st.scale, st.slot);
    st.start = d.start;
    st.freq = d.freq;
    st.escape = d.escape;
    if (d.escape) {
        st.words = stream_pop(c, x, st.slot, c.coded_slots(), c.escape_slots, r);
        st.words += stream_pop_raw(c, x, st.raw, r);
        st.symbol = symbol_of_raw(c, st.raw);
    } else {
        st.words = stream_pop(c, x, st.slot, d.start, d.freq, r);
        st.symbol = d.symbol;
    }
    st.errors |= r.over_reads() != over_before ? ERR_WORDS : 0u;
    st.state = x;
    return st;
}

// The consumer's scale model (STREAM.md §1.7). It gives the scale input b of each (lane, t) and sees every symbol.
// begin(N) is called once per job.
// scale(lane, t) is called once per (lane, t), t ascending, after the lane's symbols below history_needed(t).
// observe(lane, t, symbol) is called once per symbol, t ascending.
class ScaleSource {
  public:
    virtual ~ScaleSource() = default;
    virtual void begin(int n_lanes) = 0;
    virtual ScaleB scale(int lane, i64 t) = 0;
    virtual void observe(int lane, i64 t, i64 symbol) = 0;
    virtual i64 history_needed(i64 t) const = 0;  // <= t
};

struct DecodeOut {
    std::vector<i64> symbols;      // global order
    std::vector<uint8_t> escapes;  // 1 per escaped symbol
    // The end check (STREAM.md §1.6).
    bool ok = false;
    bool err_words = false;     // a read beyond the array
    bool err_header = false;    // a header state outside [L, 2^STATE_WIDTH)
    i64 words_consumed = 0;     // words read from the array (reads beyond it not counted)
    i64 words_total = 0;        // whole words in the array after the header
    i64 n_escapes = 0;
    std::vector<u64> states;     // per lane, after the last symbol
    std::vector<DecStep> trace;  // decode order: trace[i] is global index i
};

// The decoder: n symbols from the array, the scale of each from `src` (STREAM.md §1.6).
inline DecodeOut decode_stream(const Config& c, int n_lanes, const std::vector<uint8_t>& bytes, i64 n,
                               ScaleSource& src) {
    stream_require_build(c, n_lanes);
    stream_require(n >= 0, "decode_stream: a negative symbol count");
    DecodeOut o;
    StreamReader r(c, n_lanes, bytes);
    o.states.resize(n_lanes);
    for (int j = 0; j < n_lanes; ++j) {
        const u64 v = r.header_state(j);
        o.err_header = o.err_header || v < state_L(c) || v > state_mask(c);
        o.states[j] = v & state_mask(c);
    }
    o.err_words = r.over_reads() != 0;
    o.symbols.reserve(n);
    o.escapes.reserve(n);
    o.trace.reserve(n);
    src.begin(n_lanes);
    for (i64 i = 0; i < n; ++i) {
        const LanePos p = lane_pos(n_lanes, i);
        const ScaleB b = src.scale(p.lane, p.t);
        stream_require(b_valid(c, b), "decode_stream: the scale source gave a b beyond B_WIDTH bits");
        DecStep st = stream_decode_next(c, o.states[p.lane], b, r);
        st.lane = p.lane;
        st.t = p.t;
        o.err_words = o.err_words || (st.errors & ERR_WORDS);
        o.n_escapes += st.escape ? 1 : 0;
        o.symbols.push_back(st.symbol);
        o.escapes.push_back(st.escape ? 1 : 0);
        o.trace.push_back(st);
        src.observe(p.lane, p.t, st.symbol);
    }
    o.words_consumed = r.words_read();
    o.words_total = r.words_total();
    bool at_L = true;
    for (const u64 x : o.states) at_L = at_L && x == state_L(c);
    o.ok = !o.err_words && !o.err_header && at_L &&
           r.header() + o.words_consumed * word_bytes(c) == r.size();
    return o;
}

// ---- Scale sources ------------------------------------------------------------------------------

// One scale input for every symbol.
class ConstScaleSource final : public ScaleSource {
  public:
    explicit ConstScaleSource(ScaleB b) : b_(b) {}
    void begin(int) override {}
    ScaleB scale(int, i64) override { return b_; }
    void observe(int, i64, i64) override {}
    i64 history_needed(i64) const override { return 0; }

  private:
    ScaleB b_;
};

// A list of b in global order, independent of the symbols.
class ListScaleSource final : public ScaleSource {
  public:
    explicit ListScaleSource(std::vector<ScaleB> scales) : scales_(std::move(scales)) {}
    void begin(int n_lanes) override { n_lanes_ = n_lanes; }
    ScaleB scale(int lane, i64 t) override {
        const i64 i = t * n_lanes_ + lane;
        stream_require(i >= 0 && i < (i64)scales_.size(), "ListScaleSource: (lane, t) beyond the list");
        return scales_[i];
    }
    void observe(int, i64, i64) override {}
    i64 history_needed(i64) const override { return 0; }

  private:
    std::vector<ScaleB> scales_;
    int n_lanes_ = 1;
};

// m with `frac` fraction bits as the scale input b with B_FRAC fraction bits.
inline ScaleB b_of_m(const Config& c, i64 m, int frac) {
    return frac <= c.b_frac ? m << (c.b_frac - frac) : m >> (frac - c.b_frac);
}

// An integer moving average of |symbol| per lane, the units' tests' first scale source (STREAM.md §1.7).
class EmaScaleSource final : public ScaleSource {
  public:
    static constexpr int FRAC = 8;   // fraction bits of m
    static constexpr int SHIFT = 3;  // the newest symbol's weight 2^-SHIFT
    static constexpr double SIGMA0 = 4.0, SIGMA_FLOOR = 0.25;

    static i64 m_of_sigma(double sigma) { return (i64)std::llround(std::ldexp(sigma / std::sqrt(2.0), FRAC)); }

    EmaScaleSource(const Config& c, int delay, double sigma0 = SIGMA0, double sigma_floor = SIGMA_FLOOR)
        : c_(c), delay_(delay), m_floor_(m_of_sigma(sigma_floor)), m0_(m_of_sigma(sigma0)) {
        stream_require(delay >= 0, "EmaScaleSource: a negative delay");
    }

    void begin(int n_lanes) override { lanes_.assign(n_lanes, Lane{m0_, 0, {}}); }

    ScaleB scale(int lane, i64 t) override {
        stream_require(lane >= 0 && lane < (int)lanes_.size(), "EmaScaleSource: lane out of range");
        Lane& l = lanes_[lane];
        const i64 need = history_needed(t);
        stream_require(need <= l.folded + (i64)l.pending.size(),
                       "EmaScaleSource: a scale whose history is not observed yet");
        stream_require(need >= l.folded, "EmaScaleSource: scales must be asked for in position order");
        for (; l.folded < need; ++l.folded) {
            const i64 d = (std::abs(l.pending.front()) << FRAC) - l.m;
            l.m += d >= 0 ? d >> SHIFT : -((-d + (i64(1) << SHIFT) - 1) >> SHIFT);  // floor(d / 2^SHIFT)
            l.pending.pop_front();
        }
        return b_of_m(c_, std::max(l.m, m_floor_), FRAC);
    }

    void observe(int lane, i64 t, i64 symbol) override {
        stream_require(lane >= 0 && lane < (int)lanes_.size(), "EmaScaleSource: lane out of range");
        Lane& l = lanes_[lane];
        stream_require(t == l.folded + (i64)l.pending.size(), "EmaScaleSource: symbols must come in position order");
        l.pending.push_back(symbol);
    }

    i64 history_needed(i64 t) const override { return std::max<i64>(0, t - delay_); }

  private:
    struct Lane {
        i64 m;                    // the average, FRAC fraction bits
        i64 folded;               // symbols folded into m: positions [0, folded)
        std::deque<i64> pending;  // observed, not folded yet: positions [folded, folded + size)
    };

    Config c_;
    int delay_;
    i64 m_floor_, m0_;
    std::vector<Lane> lanes_;
};

// The example integer scale estimator's parameters (STREAM.md §1.7). None of them enters CONFIG_ID.
struct EstParams {
    int frac = 8;        // FRAC, fraction bits of m
    int shift = 4;       // SHIFT, the newest symbol's weight 2^-SHIFT
    i64 m0 = 5793;       // m's start, sigma 32
    i64 m_floor = 1;     // the output's floor, the least nonzero b
    int delay = 1;       // d, the consumer's lag in symbols
};

// The example integer scale estimator, an adaptive consumer (STREAM.md §1.7). Every operation is on integers.
class IntScaleSource final : public ScaleSource {
  public:
    static constexpr int M_WIDTH_MAX = 48;  // m, |symbol| << FRAC and their signed difference in an i64

    IntScaleSource(const Config& c, const EstParams& p = EstParams{}) : c_(c), p_(p) {
        const std::vector<std::string> why = problems(c, p);
        if (!why.empty()) stream_fail(("IntScaleSource: " + why.front()).c_str());
    }

    IntScaleSource(const Config& c, int frac, int shift, i64 m0, i64 m_floor, int delay)
        : IntScaleSource(c, EstParams{frac, shift, m0, m_floor, delay}) {}

    // m's width at this FRAC.
    static int m_width(const Config& c, int frac) { return c.symbol_width - 1 + frac; }

    // The example set for this build, m0 at sigma 4 where sigma 32 is beyond the alphabet.
    static EstParams example(const Config& c) {
        EstParams p;
        if (p.m0 >= (i64(1) << m_width(c, p.frac))) p.m0 = m_of_sigma(4.0, p.frac);
        return p;
    }
    static EstParams example(const Config& c, int delay) {
        EstParams p = example(c);
        p.delay = delay;
        return p;
    }

    // Every reason a parameter set is not an estimator for this build.
    static std::vector<std::string> problems(const Config& c, const EstParams& p) {
        std::vector<std::string> out;
        const int mw = m_width(c, p.frac);
        const auto num = [](i64 v) { return std::to_string(v); };
        if (p.frac < 0 || mw < 2 || mw > M_WIDTH_MAX) {
            out.push_back("FRAC = " + num(p.frac) + ": FRAC >= 0 and m's width SYMBOL_WIDTH - 1 + FRAC in [2, " +
                          num(M_WIDTH_MAX) + "]");
            return out;  // the other bounds are in m's width
        }
        const i64 bound = i64(1) << mw;
        if (p.shift < 1 || p.shift > mw)
            out.push_back("SHIFT = " + num(p.shift) + " must be in [1, " + num(mw) +
                          "] (SHIFT >= 1 keeps m below 2^(S-1+FRAC))");
        if (p.m0 < 0 || p.m0 >= bound)
            out.push_back("M0 = " + num(p.m0) + " must be in [0, 2^" + num(mw) + ") (m's width)");
        if (p.m_floor < 0 || p.m_floor >= bound)
            out.push_back("MFLOOR = " + num(p.m_floor) + " must be in [0, 2^" + num(mw) + ")");
        if (p.delay < 0) out.push_back("DELAY = " + num(p.delay) + " must be >= 0");
        return out;
    }

    // ---- The definition (shared by the source and the tests)

    // floor(d / 2^s), the arithmetic right shift of a two's-complement d.
    static i64 floor_shift(i64 d, int s) { return d >= 0 ? d >> s : -((-d + (i64(1) << s) - 1) >> s); }

    // One symbol folded into m.
    static i64 update(i64 m, i64 symbol, int frac, int shift) {
        return m + floor_shift((std::abs(symbol) << frac) - m, shift);
    }

    // m for a sigma, round(sigma / sqrt2 2^FRAC), a Laplace's mean |symbol|.
    static i64 m_of_sigma(double sigma, int frac) {
        return (i64)std::llround(std::ldexp(sigma / std::sqrt(2.0), frac));
    }

    // ---- The source

    void begin(int n_lanes) override { lanes_.assign(n_lanes, Lane{p_.m0, 0, {}}); }

    ScaleB scale(int lane, i64 t) override {
        Lane& l = lane_at(lane);
        const i64 need = history_needed(t);
        stream_require(need <= l.folded + (i64)l.pending.size(),
                       "IntScaleSource: a scale whose history is not observed yet");
        stream_require(need >= l.folded, "IntScaleSource: scales must be asked for in position order");
        for (; l.folded < need; ++l.folded) {
            l.m = update(l.m, l.pending.front(), p_.frac, p_.shift);
            l.pending.pop_front();
        }
        return b_of_m(c_, std::max(l.m, p_.m_floor), p_.frac);
    }

    void observe(int lane, i64 t, i64 symbol) override {
        Lane& l = lane_at(lane);
        stream_require(t == l.folded + (i64)l.pending.size(), "IntScaleSource: symbols must come in position order");
        stream_require(symbol_valid(c_, symbol), "IntScaleSource: a symbol outside the S-bit range");
        l.pending.push_back(symbol);
    }

    i64 history_needed(i64 t) const override { return std::max<i64>(0, t - p_.delay); }

    const EstParams& params() const { return p_; }
    i64 m(int lane) const { return lanes_.at(lane).m; }  // after the symbols folded so far (tests)

  private:
    struct Lane {
        i64 m;                    // the average, FRAC fraction bits
        i64 folded;               // symbols folded into m: positions [0, folded)
        std::deque<i64> pending;  // observed, not folded yet: positions [folded, folded + size)
    };

    Lane& lane_at(int lane) {
        stream_require(lane >= 0 && lane < (int)lanes_.size(), "IntScaleSource: lane out of range");
        return lanes_[lane];
    }

    Config c_;
    EstParams p_;
    std::vector<Lane> lanes_;
};

// The encoder's scale inputs, the consumer's model replayed over the symbols in decode order.
inline std::vector<ScaleB> replay_scales(int n_lanes, const std::vector<i64>& symbols, ScaleSource& src) {
    std::vector<ScaleB> out;
    out.reserve(symbols.size());
    src.begin(n_lanes);
    for (i64 i = 0; i < (i64)symbols.size(); ++i) {
        const LanePos p = lane_pos(n_lanes, i);
        out.push_back(src.scale(p.lane, p.t));
        src.observe(p.lane, p.t, symbols[i]);
    }
    return out;
}

// ---- Jobs: seeded stimulus for ref/test_stream.cpp and the DPI ------------------------------------

// splitmix64, the same sequence per seed on every platform.
class StreamRng {
  public:
    explicit StreamRng(u64 seed) : s_(seed) {}
    u64 next() {
        u64 z = (s_ += 0x9e3779b97f4a7c15ull);
        z = (z ^ (z >> 30)) * 0xbf58476d1ce4e5b9ull;
        z = (z ^ (z >> 27)) * 0x94d049bb133111ebull;
        return z ^ (z >> 31);
    }
    double unit() { return (double(next() >> 11) + 0.5) * 0x1.0p-53; }  // (0, 1)
    i64 below(i64 n) { return (i64)(next() % (u64)n); }                    // [0, n), n >= 1
    bool chance(double p) { return unit() < p; }

  private:
    u64 s_;
};

enum class SourceKind : int { CONST = 0, EMA = 1, LIST = 2, INT = 3 };

struct StreamJob {
    int n_lanes = 1;
    SourceKind kind = SourceKind::LIST;
    int delay = 0;               // the consumer's delay in symbols (EMA, INT)
    ScaleB const_scale = 0;      // CONST's b
    std::vector<i64> symbols;    // global order
    std::vector<ScaleB> scales;  // the scale inputs b in global order
    bool est_custom = false;     // INT uses `est`, not the example estimator
    EstParams est{};
};

// A fresh scale source of the job's kind.
inline std::unique_ptr<ScaleSource> job_source(const Config& c, const StreamJob& j) {
    switch (j.kind) {
    case SourceKind::CONST: return std::make_unique<ConstScaleSource>(j.const_scale);
    case SourceKind::EMA: return std::make_unique<EmaScaleSource>(c, j.delay);
    case SourceKind::INT:
        return std::make_unique<IntScaleSource>(c, j.est_custom ? j.est : IntScaleSource::example(c, j.delay));
    case SourceKind::LIST: break;
    }
    return std::make_unique<ListScaleSource>(j.scales);
}

// The encoder's scale inputs, replayed from the job's source. LIST has them already.
inline void job_replay(const Config& c, StreamJob& j) {
    if (j.kind == SourceKind::LIST) return;
    const std::unique_ptr<ScaleSource> src = job_source(c, j);
    j.scales = replay_scales(j.n_lanes, j.symbols, *src);
}

inline DecodeOut job_decode(const Config& c, const StreamJob& j, const std::vector<uint8_t>& bytes) {
    const std::unique_ptr<ScaleSource> src = job_source(c, j);
    return decode_stream(c, j.n_lanes, bytes, (i64)j.symbols.size(), *src);
}

// The Laplace scale b of a scale code, 0 or infinity for the special scales.
inline double laplace_b_of(const Config& c, Scale s) {
    if (s.exp == c.scale_exp_inf()) return 0.0;
    if (s.exp == 0) return HUGE_VAL;
    const Slope k = slope_of(c, s);
    return 1.0 / (std::ldexp((double)k.mant, k.exp2) * std::log(2.0));
}

// The smallest scale input b with table index j, or -1 if no integer b has it.
inline ScaleB b_of_index(const Config& c, i64 j) {
    if (j == 0) return 0;
    const int lead = int(j >> c.b_k);
    const i64 mant = (i64(1) << c.b_k) + (j & ((i64(1) << c.b_k) - 1));  // 2^K + below
    if (lead >= c.b_k) return mant << (lead - c.b_k);
    return (mant & ((i64(1) << (c.b_k - lead)) - 1)) != 0 ? -1 : mant >> (c.b_k - lead);
}

// Random jobs whose symbols and scale inputs reach the coder's corners.
class JobGen {
  public:
    explicit JobGen(const Config& c) : c_(c) {}

    // A discretized-Laplace symbol of scale b, clipped to the alphabet.
    i64 laplace_symbol(StreamRng& r, double b) {
        if (std::isinf(b)) return uniform_symbol(r);
        const double y = -b * std::log(r.unit());
        const double x = std::floor((r.chance(0.5) ? y : -y) + 0.5);
        return (i64)std::min<double>(std::max<double>(x, (double)c_.symbol_min()), (double)c_.symbol_max());
    }

    // A symbol uniform over the S-bit inputs.
    i64 uniform_symbol(StreamRng& r) {
        return c_.symbol_input_min() + r.below(c_.symbol_max() - c_.symbol_input_min() + 1);
    }

    // The alphabet's ends, 0, +-1, and -2^(S-1) where it lies outside the alphabet.
    i64 extreme_symbol(StreamRng& r) {
        const i64 pick[] = {c_.symbol_min(), c_.symbol_max(), 0, 1, -1, c_.symbol_input_min()};
        const i64 n = c_.symbol_input_min() < c_.symbol_min() ? 6 : 5;
        return pick[r.below(n)];
    }

    // A random scale input b, each end of its range with probability p_ends, else log-uniform.
    ScaleB random_b(StreamRng& r, double p_ends) {
        if (r.chance(2 * p_ends)) return r.chance(0.5) ? 0 : c_.b_max();
        const int lead = int(r.below(c_.b_width()));
        return (i64(1) << lead) | r.below(i64(1) << lead);
    }

    // The symbols x >= 0 for which both x and -x have f = 1 at scale s.
    const std::vector<i64>& unit_freq_symbols(Scale s) {
        const unsigned code = (s.exp << c_.scale_man_width) | s.man;
        const auto hit = unit_freq_.find(code);
        if (hit != unit_freq_.end()) return hit->second;
        std::vector<i64>& v = unit_freq_[code];
        const auto one = [&](i64 x) {
            const EncOut e = encode(c_, s, x);
            return !e.escape && e.freq == 1;
        };
        for (i64 x = 0; x <= c_.symbol_max(); ++x)
            if (one(x) && (x < c_.symbol_max() || one(-x))) v.push_back(x);
        return v;
    }

    // LIST's symbol at scale s, a mix of Laplace, uniform, extreme and f = 1 symbols.
    i64 list_symbol(StreamRng& r, Scale s) {
        const double u = r.unit();
        if (u < 0.5) return laplace_symbol(r, laplace_b_of(c_, s));
        if (u < 0.65) return uniform_symbol(r);
        if (u < 0.75) return extreme_symbol(r);
        const std::vector<i64>& ones = unit_freq_symbols(s);  // x >= 0, and -x has f = 1 too
        return ones.empty() ? laplace_symbol(r, laplace_b_of(c_, s))
                            : ones[r.below((i64)ones.size())] * (r.chance(0.5) ? 1 : -1);
    }

    // A random job of `kind`, its scale inputs from job_replay.
    StreamJob make(u64 seed, int n_lanes, i64 n, SourceKind kind, int delay) {
        StreamRng r(seed);
        StreamJob j;
        j.n_lanes = n_lanes;
        j.kind = kind;
        j.delay = delay;
        j.symbols.resize(n);
        const double log2_sigma_max = c_.symbol_width - 2;  // sigma up to MAX / 2
        const double sigma_min = 0.25;
        if (kind == SourceKind::LIST) {
            j.scales.resize(n);
            for (i64 i = 0; i < n; ++i) {
                const ScaleB b = random_b(r, 0.06);
                j.scales[i] = b;
                j.symbols[i] = list_symbol(r, scale_of_b(c_, b));
            }
        } else if (kind == SourceKind::CONST) {
            const double sigma = std::exp2(std::log2(sigma_min) + r.unit() * (log2_sigma_max - std::log2(sigma_min)));
            j.const_scale = r.chance(0.1) ? (r.chance(0.5) ? 0 : c_.b_max()) : b_fixed_of(c_, laplace_b(sigma));
            const double b = laplace_b_of(c_, scale_of_b(c_, j.const_scale));
            for (i64 i = 0; i < n; ++i) j.symbols[i] = r.chance(0.03) ? uniform_symbol(r) : laplace_symbol(r, b);
        } else {
            std::vector<double> log2_sigma(n_lanes);
            for (double& v : log2_sigma) v = std::log2(sigma_min) + r.unit() * (log2_sigma_max - std::log2(sigma_min));
            const i64 drift_every = 64;  // symbols of a lane between sigma steps of up to one octave
            for (i64 i = 0; i < n; ++i) {
                const LanePos p = lane_pos(n_lanes, i);
                double& ls = log2_sigma[p.lane];
                if (p.t % drift_every == drift_every - 1)
                    ls = std::min(std::max(ls + 2 * r.unit() - 1, std::log2(sigma_min)), log2_sigma_max);
                j.symbols[i] = r.chance(0.02) ? uniform_symbol(r) : laplace_symbol(r, std::exp2(ls) / std::sqrt(2.0));
            }
        }
        job_replay(c_, j);
        return j;
    }

    // ---- The raw push's bound (contract version 5)
    // path_to walks at random from L until one push reaches the target. Returns false when the budget runs out.
    struct LanePath {
        std::vector<i64> symbols;    // push order: the first pushed first, from x = L
        std::vector<ScaleB> scales;
    };

    bool path_to(u64 target, u64 seed, LanePath& out) {
        const int P = c_.precision, W = c_.word_width(), S = c_.symbol_width;
        const u64 L = state_L(c_), M = (u64)c_.slots(), top = L << W;
        if (target < L || target >= top) return false;
        if (target == L) {  // every lane starts there (B = L where S = W)
            out = LanePath{};
            return true;
        }
        struct Pred {
            bool escape;  // an escape or a coded symbol
            i64 symbol;
            ScaleB b;
        };
        std::unordered_map<u64, Pred> at, block;  // single states; blocks of 2^W states keyed by x >> W
        const auto add = [&](std::unordered_map<u64, Pred>& m, u64 k, Pred p) { m.emplace(k, p); };
        const u64 slot = target & (M - 1), q = target >> P;
        for (i64 idx = 0; idx < c_.scale_table_size(); ++idx) {
            const ScaleB b = b_of_index(c_, idx);
            if (b < 0) continue;
            const DecOut d = decode(c_, scale_of_b(c_, b), (i64)slot);
            if (d.escape) continue;
            const u64 x1 = q * (u64)d.freq + (slot - (u64)d.start);
            if (x1 >= L) add(at, x1, {false, d.symbol, b});
            else add(block, x1, {false, d.symbol, b});
        }
        if (slot >= (u64)c_.coded_slots()) {  // the escape pop, then the raw pop, the words free
            const u64 x1 = q * (u64)c_.escape_slots + (slot - (u64)c_.coded_slots());
            const u64 lo = x1 >= L ? x1 : x1 << W, n = x1 >= L ? 1 : u64(1) << W;
            for (u64 y = lo; y < lo + n; ++y) {
                const u64 x0 = y >> S;
                const Pred p{true, 0, 0};
                if (x0 >= L) add(at, x0, p);
                else add(block, x0, p);
            }
        }
        // The push from x that ends at `target`, if x is a predecessor.
        const auto last_push = [&](u64 x, i64& sym, ScaleB& b) {
            const auto try_one = [&](const Pred& p) {
                if (!p.escape) {
                    u64 y = x;
                    std::vector<u64> w;
                    stream_encode_one(c_, y, w, p.symbol, scale_of_b(c_, p.b));
                    sym = p.symbol;
                    b = p.b;
                    return y == target;
                }
                // an escape at b = 0, the first raw value that lands on the target
                for (i64 v = 0; v < (i64(1) << S); ++v) {
                    const i64 s = symbol_of_raw(c_, v);
                    if (!encode(c_, scale_of_b(c_, 0), s).escape) continue;
                    u64 y = x;
                    std::vector<u64> w;
                    stream_encode_one(c_, y, w, s, scale_of_b(c_, 0));
                    if (y == target) {
                        sym = s;
                        b = 0;
                        return true;
                    }
                }
                return false;
            };
            const auto a = at.find(x);
            if (a != at.end() && try_one(a->second)) return true;
            const auto k = block.find(x >> W);
            return k != block.end() && try_one(k->second);
        };
        StreamRng r(seed);
        constexpr int WALK = 48;           // pushes per walk before it restarts from L
        constexpr long BUDGET = 1L << 22;  // pushes in all
        for (long spent = 0; spent < BUDGET;) {
            LanePath p;
            u64 x = L;
            for (int k = 0; k < WALK && spent < BUDGET; ++k, ++spent) {
                i64 sym = 0;
                ScaleB b = 0;
                if (last_push(x, sym, b)) {
                    p.symbols.push_back(sym);
                    p.scales.push_back(b);
                    out = p;
                    return true;
                }
                b = random_b(r, 0.06);
                sym = list_symbol(r, scale_of_b(c_, b));
                std::vector<u64> w;
                stream_encode_one(c_, x, w, sym, scale_of_b(c_, b));
                p.symbols.push_back(sym);
                p.scales.push_back(b);
            }
        }
        return false;
    }

    // A LIST job whose lane 0 escape reads the state B and whose lane 1 escape reads B - 1, where they exist.
    struct BoundJob {
        StreamJob job;
        i64 esc_index[2] = {-1, -1};  // global index of lane k's escape, -1 if none
        u64 esc_state[2] = {0, 0};  // the state lane k's raw push must read
    };

    BoundJob bound_job(u64 seed, int n_lanes) {
        BoundJob bj;
        StreamRng r(seed);
        const u64 B = raw_push_bound(c_), L = state_L(c_);
        std::vector<LanePath> path(n_lanes);
        std::vector<int> directed(n_lanes, -1);
        for (int k = 0; k < 2 && k < n_lanes; ++k) {
            const u64 target = k == 0 ? B : B - 1;
            if (target < L) continue;
            if (path_to(target, r.next(), path[k])) directed[k] = k;
        }
        i64 len = 1;
        for (int k = 0; k < n_lanes; ++k)
            if (directed[k] >= 0) len = std::max<i64>(len, (i64)path[k].symbols.size() + 3);
        StreamJob& j = bj.job;
        j.n_lanes = n_lanes;
        j.kind = SourceKind::LIST;
        j.symbols.resize(len * n_lanes);
        j.scales.resize(len * n_lanes);
        for (int l = 0; l < n_lanes; ++l) {
            for (i64 t = 0; t < len; ++t) {  // random LIST symbols, overwritten on a directed lane
                const ScaleB b = random_b(r, 0.06);
                j.scales[t * n_lanes + l] = b;
                j.symbols[t * n_lanes + l] = list_symbol(r, scale_of_b(c_, b));
            }
            if (directed[l] < 0) continue;
            const LanePath& p = path[l];
            const i64 n_path = (i64)p.symbols.size(), t_esc = len - 1 - n_path;
            for (i64 k = 0; k < n_path; ++k) {  // pushed first: the highest positions, in push order downward
                j.symbols[(len - 1 - k) * n_lanes + l] = p.symbols[k];
                j.scales[(len - 1 - k) * n_lanes + l] = p.scales[k];
            }
            i64 s = 0;
            while (s == 0) s = uniform_symbol(r);
            j.symbols[t_esc * n_lanes + l] = s;
            j.scales[t_esc * n_lanes + l] = 0;
            bj.esc_index[l] = t_esc * n_lanes + l;
            bj.esc_state[l] = l == 0 ? B : B - 1;
        }
        return bj;
    }

  private:
    Config c_;
    std::unordered_map<unsigned, std::vector<i64>> unit_freq_;
};

}  // namespace pans1hw
