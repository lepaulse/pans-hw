// SPDX-License-Identifier: Apache-2.0
// stream_tool: files of symbols through the stream model with a chosen scale source, and where their bits go.
// Mirrors encode_stream and decode_stream of ref/pans_stream_ref.h. Specification: STREAM.md §1.10.
// Each file holds one stream's symbols in decode order, int8 or int16 little-endian.
// A file that fails the round trip gets no row. The exit status is then 1, or 2 for bad options.
// The model keeps both traces, about 150 bytes of memory per symbol.
//
// Scale sources:
//   --const B        every symbol's b is B
//   --b-list         b per symbol from a second file, given after each symbol file, one integer per symbol
//   --adaptive       the example estimator IntScaleSource, the default, set by the options below
//
// Output: CSV on stdout, one row per file. The build, the source and its parameters go to stderr.
//   file              the symbol file's path
//   n                 symbols
//   lanes             N
//   bytes             the array's length, job_in n_bytes
//   header_bytes      N STATE_BYTES, the lanes' final states
//   words             the W-bit words after the header
//   n_escapes         symbols sent raw
//   ideal_coded_bits  the sum of log2(M / f) over the coded symbols
//   escape_bits       n_escapes (log2(M / E) + S)
//   ideal_total_bits  ideal_coded_bits + escape_bits
//   actual_bits       8 bytes
//   actual_bps        actual_bits / n, nan for n = 0
//   ideal_bps         ideal_total_bits / n, nan for n = 0
//
//   make stream-tool-<cfg>    builds build/<cfg>/stream_tool for a config of ci/matrix.toml; by hand:
//   g++ -O2 -std=c++17 -Wall -Wextra -Werror -Iref <config -D flags> ref/stream_tool.cpp -o stream_tool
//   stream_tool [--lanes N] [--width 8|16] [--const B | --b-list | --adaptive [--frac F] [--shift S]
//               [--m0 M | --start-sigma X] [--mfloor M | --floor-sigma X] [--delay D]] file...
// --start-sigma and --floor-sigma give m0 and m_floor as sigmas at the FRAC in use.
#include <cerrno>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <string>
#include <vector>

#include "pans1_config.h"
#include "pans_stream_ref.h"

using namespace pans1hw;

namespace {

constexpr int DEFAULT_LANES = 8;
constexpr int EXIT_FILE_FAILED = 1;
constexpr int EXIT_USAGE = 2;

const char* const USAGE =
    "usage: stream_tool [--lanes N] [--width 8|16] [--const B | --b-list | --adaptive [--frac F] [--shift S]\n"
    "                   [--m0 M | --start-sigma X] [--mfloor M | --floor-sigma X] [--delay D]] file...\n"
    "  files of symbols (int8, or int16 little-endian with --width 16) in decode order, one per stream; one CSV\n"
    "  row per file on stdout (ref/stream_tool.cpp). The scale input b of every symbol: --const B (b = B / 2^B_FRAC),\n"
    "  --b-list (operands in pairs: a symbol file, then a text file of one integer b per symbol), or the example\n"
    "  adaptive estimator (the default, its parameters as options)\n";

[[noreturn]] void usage_fail(const std::string& why) {
    std::fprintf(stderr, "stream_tool: %s\n%s", why.c_str(), USAGE);
    std::exit(EXIT_USAGE);
}

i64 parse_int(const char* opt, const char* s) {
    char* end = nullptr;
    errno = 0;
    const long long v = std::strtoll(s, &end, 10);
    if (errno != 0 || end == s || *end != '\0') usage_fail(std::string(opt) + " needs an integer, not '" + s + "'");
    return (i64)v;
}

double parse_double(const char* opt, const char* s) {
    char* end = nullptr;
    errno = 0;
    const double v = std::strtod(s, &end);
    if (errno != 0 || end == s || *end != '\0' || !std::isfinite(v))
        usage_fail(std::string(opt) + " needs a number, not '" + s + "'");
    return v;
}

// A CSV field: quoted (quotes doubled) iff it holds a comma, a quote or a line break.
std::string csv_field(const std::string& s) {
    if (s.find_first_of(",\"\r\n") == std::string::npos) return s;
    std::string q = "\"";
    for (const char ch : s) q += ch == '"' ? std::string("\"\"") : std::string(1, ch);
    return q + "\"";
}

bool read_file(const std::string& path, std::vector<uint8_t>& out) {
    std::FILE* f = std::fopen(path.c_str(), "rb");
    if (!f) return false;
    uint8_t buf[1 << 16];
    size_t got;
    while ((got = std::fread(buf, 1, sizeof buf, f)) > 0) out.insert(out.end(), buf, buf + got);
    const bool ok = !std::ferror(f);
    std::fclose(f);
    return ok;
}

enum class Mode { ADAPTIVE, CONST, LIST };

struct Source {
    Mode mode = Mode::ADAPTIVE;
    ScaleB b = 0;  // CONST
    EstParams p;   // ADAPTIVE
};

// A fresh source for the host's replay or the decoder. `list` is LIST's b per symbol.
std::unique_ptr<ScaleSource> make_source(const Config& c, const Source& s, const std::vector<ScaleB>& list) {
    switch (s.mode) {
    case Mode::CONST: return std::make_unique<ConstScaleSource>(s.b);
    case Mode::LIST: return std::make_unique<ListScaleSource>(list);
    case Mode::ADAPTIVE: break;
    }
    return std::make_unique<IntScaleSource>(c, s.p);
}

// The symbols of a file of `width`-bit two's-complement values (little-endian), each an S-bit input of the build.
bool read_symbols(const Config& c, int width, const std::string& path, std::vector<i64>& symbols) {
    std::vector<uint8_t> raw;
    if (!read_file(path, raw)) {
        std::fprintf(stderr, "stream_tool: %s: cannot read (%s)\n", path.c_str(), std::strerror(errno));
        return false;
    }
    const size_t bytes = size_t(width / 8);
    if (raw.size() % bytes != 0) {
        std::fprintf(stderr, "stream_tool: %s: %zu bytes, not a whole number of int%d symbols\n", path.c_str(),
                     raw.size(), width);
        return false;
    }
    symbols.resize(raw.size() / bytes);
    for (size_t i = 0; i < symbols.size(); ++i) {
        u64 u = 0;
        for (size_t k = 0; k < bytes; ++k) u |= u64(raw[i * bytes + k]) << (8 * k);
        const i64 half = i64(1) << (width - 1);
        symbols[i] = i64(u) >= half ? i64(u) - 2 * half : i64(u);  // two's complement
        if (!symbol_valid(c, symbols[i])) {
            std::fprintf(stderr, "stream_tool: %s: symbol %zu = %lld outside the build's [%lld, %lld]\n", path.c_str(),
                         i, (long long)symbols[i], (long long)c.symbol_input_min(), (long long)c.symbol_max());
            return false;
        }
    }
    return true;
}

// A b list: whitespace-separated integers, one per symbol, each in [0, 2^B_WIDTH).
bool read_b_list(const Config& c, const std::string& path, i64 n, std::vector<ScaleB>& list) {
    std::vector<uint8_t> raw;
    if (!read_file(path, raw)) {
        std::fprintf(stderr, "stream_tool: %s: cannot read (%s)\n", path.c_str(), std::strerror(errno));
        return false;
    }
    const std::string text(raw.begin(), raw.end());
    const char* at = text.c_str();
    list.clear();
    for (;;) {
        while (*at == ' ' || *at == '\t' || *at == '\n' || *at == '\r') ++at;
        if (*at == '\0') break;
        char* end = nullptr;
        errno = 0;
        const long long v = std::strtoll(at, &end, 10);
        if (errno != 0 || end == at || (*end != '\0' && *end != ' ' && *end != '\t' && *end != '\n' && *end != '\r')) {
            std::fprintf(stderr, "stream_tool: %s: entry %zu is not an integer\n", path.c_str(), list.size());
            return false;
        }
        if (!b_valid(c, (ScaleB)v)) {
            std::fprintf(stderr, "stream_tool: %s: entry %zu, b = %lld, outside [0, 2^%d)\n", path.c_str(),
                         list.size(), v, c.b_width());
            return false;
        }
        list.push_back((ScaleB)v);
        at = end;
    }
    if ((i64)list.size() != n) {
        std::fprintf(stderr, "stream_tool: %s: %zu scale inputs for %lld symbols\n", path.c_str(), list.size(),
                     (long long)n);
        return false;
    }
    return true;
}

struct Row {
    i64 n, bytes, header_bytes, words, n_escapes;
    double ideal_coded_bits, escape_bits;
};

// One stream's round trip and bits, false if it fails. b_path is LIST's b file.
bool run_file(const Config& c, int n_lanes, int width, const Source& src, const std::string& path,
              const std::string& b_path, Row& row) {
    std::vector<i64> symbols;
    if (!read_symbols(c, width, path, symbols)) return false;
    const i64 n = (i64)symbols.size();
    std::vector<ScaleB> list;
    if (src.mode == Mode::LIST && !read_b_list(c, b_path, n, list)) return false;
    const std::unique_ptr<ScaleSource> host = make_source(c, src, list), consumer = make_source(c, src, list);
    const StreamOut e = encode_stream(c, n_lanes, symbols, replay_scales(n_lanes, symbols, *host));
    const DecodeOut d = decode_stream(c, n_lanes, e.bytes, n, *consumer);
    if (!d.ok || d.symbols != symbols) {
        std::fprintf(stderr, "stream_tool: %s: the round trip failed (ok %d, symbols %s)\n", path.c_str(), d.ok,
                     d.symbols == symbols ? "equal" : "differ");
        return false;
    }
    row = Row{n, (i64)e.bytes.size(), header_bytes(c, n_lanes), (i64)e.words.size(), e.n_escapes, 0.0, 0.0};
    for (const EncStep& s : e.trace)
        if (!s.escape) row.ideal_coded_bits += c.precision - std::log2((double)s.freq);
    row.escape_bits = double(e.n_escapes) * (c.precision - std::log2((double)c.escape_slots) + c.symbol_width);
    return true;
}

}  // namespace

int main(int argc, char** argv) {
    const Config c = build_config();
    const EstParams example = IntScaleSource::example(c);
    Source src;
    src.p = example;
    int n_lanes = DEFAULT_LANES, width = c.symbol_width <= 8 ? 8 : 16;
    bool mode_set = false, est_set = false, frac_set = false, m0_set = false, floor_set = false;
    double start_sigma = NAN, floor_sigma = NAN;
    std::vector<std::string> operands;
    const auto set_mode = [&](Mode m) {
        if (mode_set) usage_fail("give one scale source: --const, --b-list or --adaptive");
        src.mode = m;
        mode_set = true;
    };
    for (int a = 1; a < argc; ++a) {
        const std::string opt = argv[a];
        if (opt == "-h" || opt == "--help") {
            std::fputs(USAGE, stdout);
            return 0;
        }
        if (opt.rfind("--", 0) != 0) {
            operands.push_back(opt);
            continue;
        }
        if (opt == "--b-list" || opt == "--adaptive") {
            set_mode(opt == "--b-list" ? Mode::LIST : Mode::ADAPTIVE);
            continue;
        }
        if (a + 1 >= argc) usage_fail(opt + " needs a value");
        const char* o = opt.c_str();
        const char* v = argv[++a];
        if (opt == "--lanes") {
            n_lanes = (int)parse_int(o, v);
        } else if (opt == "--width") {
            width = (int)parse_int(o, v);
        } else if (opt == "--const") {
            set_mode(Mode::CONST);
            src.b = parse_int(o, v);
        } else if (opt == "--frac") {
            src.p.frac = (int)parse_int(o, v);
            frac_set = est_set = true;
        } else if (opt == "--shift") {
            src.p.shift = (int)parse_int(o, v);
            est_set = true;
        } else if (opt == "--m0" || opt == "--start-sigma") {
            if (m0_set) usage_fail("give --m0 or --start-sigma, once");
            if (opt == "--m0") src.p.m0 = parse_int(o, v);
            else start_sigma = parse_double(o, v);
            m0_set = est_set = true;
        } else if (opt == "--mfloor" || opt == "--floor-sigma") {
            if (floor_set) usage_fail("give --mfloor or --floor-sigma, once");
            if (opt == "--mfloor") src.p.m_floor = parse_int(o, v);
            else floor_sigma = parse_double(o, v);
            floor_set = est_set = true;
        } else if (opt == "--delay") {
            src.p.delay = (int)parse_int(o, v);
            est_set = true;
        } else {
            usage_fail("unknown option " + opt);
        }
    }
    if (operands.empty()) usage_fail("no input files");
    if (n_lanes < 1) usage_fail("--lanes must be >= 1");
    if (width != 8 && width != 16) usage_fail("--width must be 8 or 16");
    if (est_set && src.mode != Mode::ADAPTIVE)
        usage_fail("--frac, --shift, --m0, --start-sigma, --mfloor, --floor-sigma and --delay set the adaptive source");
    if (src.mode == Mode::CONST && !b_valid(c, src.b))
        usage_fail("--const " + std::to_string(src.b) + " is outside [0, 2^" + std::to_string(c.b_width()) + ")");
    if (src.mode == Mode::LIST && operands.size() % 2 != 0)
        usage_fail("--b-list takes the operands in pairs: a symbol file, then its b file");
    if (src.mode == Mode::ADAPTIVE) {
        if (frac_set && src.p.frac != example.frac && !(m0_set && floor_set))
            usage_fail("--frac changes m's units: give --m0 or --start-sigma, and --mfloor or --floor-sigma");
        if (!std::isnan(start_sigma)) src.p.m0 = IntScaleSource::m_of_sigma(start_sigma, src.p.frac);
        if (!std::isnan(floor_sigma)) src.p.m_floor = IntScaleSource::m_of_sigma(floor_sigma, src.p.frac);
        const std::vector<std::string> why = IntScaleSource::problems(c, src.p);
        if (!why.empty()) usage_fail("not an estimator for this build: " + why.front());
    }

    std::string what;
    char buf[400];
    if (src.mode == Mode::CONST) {
        std::snprintf(buf, sizeof buf, "constant b = %lld (%.6f symbol steps)", (long long)src.b,
                      std::ldexp((double)src.b, -c.b_frac));
        what = buf;
    } else if (src.mode == Mode::LIST) {
        what = "a list of b per symbol from each symbol file's b file";
    } else {
        const EstParams& p = src.p;
        const auto sigma = [&p](i64 m) { return std::sqrt(2.0) * std::ldexp((double)m, -p.frac); };
        const bool is_example = p.frac == example.frac && p.shift == example.shift && p.m0 == example.m0 &&
                                p.m_floor == example.m_floor && p.delay == example.delay;
        std::snprintf(buf, sizeof buf,
                      "the adaptive estimator IntScaleSource FRAC %d SHIFT %d M0 %lld (sigma %.4f%s) MFLOOR %lld "
                      "(sigma %.4f%s) DELAY %d%s",
                      p.frac, p.shift, (long long)p.m0, sigma(p.m0), std::isnan(start_sigma) ? "" : ", from --start-sigma",
                      (long long)p.m_floor, sigma(p.m_floor), std::isnan(floor_sigma) ? "" : ", from --floor-sigma",
                      p.delay, is_example ? " (the example set)" : "");
        what = buf;
    }
    std::fprintf(stderr,
                 "stream_tool: P=%d SW=%d E=%d, N=%d lanes, int%d symbol files; scale source: %s; scale input b: "
                 "B_FRAC %d, B_WIDTH %d, the coder's table B_K %d\n",
                 c.precision, c.symbol_width, c.escape_slots, n_lanes, width, what.c_str(), c.b_frac, c.b_width(),
                 c.b_k);

    std::printf("file,n,lanes,bytes,header_bytes,words,n_escapes,ideal_coded_bits,escape_bits,ideal_total_bits,"
                "actual_bits,actual_bps,ideal_bps\n");
    const size_t step = src.mode == Mode::LIST ? 2 : 1;
    int failed = 0;
    for (size_t k = 0; k < operands.size(); k += step) {
        const std::string& f = operands[k];
        Row r{};
        if (!run_file(c, n_lanes, width, src, f, step == 2 ? operands[k + 1] : std::string(), r)) {
            ++failed;
            continue;
        }
        const double total = r.ideal_coded_bits + r.escape_bits, actual = 8.0 * double(r.bytes);
        std::printf("%s,%lld,%d,%lld,%lld,%lld,%lld,%.3f,%.3f,%.3f,%.0f,", csv_field(f).c_str(), (long long)r.n,
                    n_lanes, (long long)r.bytes, (long long)r.header_bytes, (long long)r.words,
                    (long long)r.n_escapes, r.ideal_coded_bits, r.escape_bits, total, actual);
        if (r.n > 0) std::printf("%.6f,%.6f\n", actual / double(r.n), total / double(r.n));
        else std::printf("nan,nan\n");
        std::fflush(stdout);
    }
    if (failed) std::fprintf(stderr, "stream_tool: %d of %zu files failed\n", failed, operands.size() / step);
    return failed ? EXIT_FILE_FAILED : 0;
}
