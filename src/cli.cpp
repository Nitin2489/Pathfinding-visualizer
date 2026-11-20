#include "cli.h"

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstdlib>
#include <set>

namespace {

bool parseInt(const std::string& s, long long minValue, long long maxValue, long long& out) {
    if (s.empty()) return false;
    const char* begin = s.c_str();
    char* end = nullptr;
    errno = 0;
    const long long v = std::strtoll(begin, &end, 10);
    if (errno != 0 || end == begin || *end != '\0') return false;
    if (v < minValue || v > maxValue) return false;
    out = v;
    return true;
}

bool parseUnsigned64(const std::string& s, std::uint64_t& out) {
    if (s.empty() || !std::all_of(s.begin(), s.end(), [](char c) { return c >= '0' && c <= '9'; })) return false;
    errno = 0;
    char* end = nullptr;
    const unsigned long long v = std::strtoull(s.c_str(), &end, 10);
    if (errno != 0 || *end != '\0') return false;
    out = static_cast<std::uint64_t>(v);
    return true;
}

// Percentage in [0, 100]; returns the fraction in [0, 1].
bool parsePercent(const std::string& s, double& fraction) {
    if (s.empty()) return false;
    char* end = nullptr;
    errno = 0;
    const double v = std::strtod(s.c_str(), &end);
    if (errno != 0 || end == s.c_str() || *end != '\0' || !std::isfinite(v) || v < 0.0 || v > 100.0) return false;
    fraction = v / 100.0;
    return true;
}

bool parseSize(const std::string& s, int& rows, int& cols) {
    const auto x = s.find_first_of("xX");
    if (x == std::string::npos) return false;
    long long r = 0;
    long long c = 0;
    if (!parseInt(s.substr(0, x), 3, kMaxDimension, r) || !parseInt(s.substr(x + 1), 3, kMaxDimension, c)) return false;
    if (r * c > kMaxCells) return false;
    rows = static_cast<int>(r);
    cols = static_cast<int>(c);
    return true;
}

}  // namespace

bool parseCommandLine(const std::vector<std::string>& args, CliOptions& out, std::string& error) {
    out = CliOptions{};
    Scenario scenario;
    scenario.placement = Placement::Reachable;  // CLI default: every instance solvable
    std::set<std::string> seen;

    for (std::size_t i = 0; i < args.size(); ++i) {
        const std::string& arg = args[i];
        if (arg == "--help" || arg == "-h") {
            out.mode = Mode::Help;
            return true;
        }
        if (arg.rfind("--", 0) != 0) {
            error = "unexpected argument '" + arg + "'";
            return false;
        }
        if (!seen.insert(arg).second) {
            error = "option " + arg + " given more than once";
            return false;
        }
        // Flags without a value.
        if (arg == "--benchmark") { out.mode = Mode::Benchmark; continue; }
        if (arg == "--force") { out.force = true; continue; }
        if (arg == "--quiet") { out.quiet = true; continue; }

        static const std::set<std::string> kValued = {"--algorithm", "--size", "--density", "--terrain",
                                                      "--scenario", "--placement", "--seed", "--grids",
                                                      "--runs", "--warmup", "--suite", "--csv", "--summary-csv",
                                                      "--grid-csv", "--paired-csv"};
        if (!kValued.count(arg)) {
            error = "unknown option '" + arg + "'";
            return false;
        }
        if (i + 1 >= args.size() || args[i + 1].rfind("--", 0) == 0) {
            error = "option " + arg + " requires a value";
            return false;
        }
        const std::string& value = args[++i];
        long long n = 0;

        if (arg == "--algorithm") {
            out.bench.algorithms.clear();
            if (value == "all") {
                out.bench.algorithms = allAlgorithms();
                continue;
            }
            std::size_t pos = 0;
            while (pos <= value.size()) {
                const std::size_t comma = value.find(',', pos);
                const std::string name = value.substr(pos, comma == std::string::npos ? std::string::npos : comma - pos);
                Algorithm a;
                if (!parseAlgorithm(name, a)) {
                    error = "unknown algorithm '" + name + "' (expected dijkstra, astar, bfs, dfs or all)";
                    return false;
                }
                if (std::find(out.bench.algorithms.begin(), out.bench.algorithms.end(), a) == out.bench.algorithms.end()) {
                    out.bench.algorithms.push_back(a);
                }
                if (comma == std::string::npos) break;
                pos = comma + 1;
            }
        } else if (arg == "--size") {
            if (!parseSize(value, scenario.rows, scenario.cols)) {
                error = "invalid --size '" + value + "' (expected ROWSxCOLS, each 3.." + std::to_string(kMaxDimension) +
                        ", at most " + std::to_string(kMaxCells) + " cells)";
                return false;
            }
        } else if (arg == "--density") {
            if (!parsePercent(value, scenario.obstacleDensity)) {
                error = "invalid --density '" + value + "' (expected a percentage from 0 to 100)";
                return false;
            }
        } else if (arg == "--terrain") {
            if (!parsePercent(value, scenario.terrainDensity)) {
                error = "invalid --terrain '" + value + "' (expected a percentage from 0 to 100)";
                return false;
            }
        } else if (arg == "--scenario") {
            if (value == "random") scenario.type = ScenarioType::Random;
            else if (value == "maze") scenario.type = ScenarioType::Maze;
            else {
                error = "unknown --scenario '" + value + "' (expected random or maze)";
                return false;
            }
        } else if (arg == "--placement") {
            if (!parsePlacement(value, scenario.placement)) {
                error = "unknown --placement '" + value + "' (expected reachable, random or corners)";
                return false;
            }
        } else if (arg == "--seed") {
            if (!parseUnsigned64(value, out.bench.baseSeed)) {
                error = "invalid --seed '" + value + "' (expected an unsigned 64-bit integer)";
                return false;
            }
            out.seedGiven = true;
        } else if (arg == "--grids") {
            if (!parseInt(value, 1, kMaxGrids, n)) {
                error = "invalid --grids '" + value + "' (expected 1.." + std::to_string(kMaxGrids) + ")";
                return false;
            }
            out.bench.grids = static_cast<int>(n);
        } else if (arg == "--runs") {
            if (!parseInt(value, 1, kMaxRuns, n)) {
                error = "invalid --runs '" + value + "' (expected 1.." + std::to_string(kMaxRuns) + ")";
                return false;
            }
            out.bench.runs = static_cast<int>(n);
        } else if (arg == "--warmup") {
            if (!parseInt(value, 0, kMaxWarmup, n)) {
                error = "invalid --warmup '" + value + "' (expected 0.." + std::to_string(kMaxWarmup) + ")";
                return false;
            }
            out.bench.warmup = static_cast<int>(n);
        } else if (arg == "--suite") {
            out.suite = value;
        } else if (arg == "--csv") {
            out.csvPath = value;
        } else if (arg == "--summary-csv") {
            out.summaryPath = value;
        } else if (arg == "--grid-csv") {
            out.gridCsvPath = value;
        } else if (arg == "--paired-csv") {
            out.pairedPath = value;
        }
    }

    // Cross-option validation.
    if (out.mode != Mode::Benchmark) {
        for (const std::string& opt : seen) {
            if (opt != "--seed") {
                error = "option " + opt + " requires --benchmark";
                return false;
            }
        }
        return true;
    }
    if (!out.suite.empty()) {
        for (const char* opt : {"--size", "--density", "--terrain", "--scenario", "--placement"}) {
            if (seen.count(opt)) {
                error = std::string(opt) + " cannot be combined with --suite (suites define their own scenarios)";
                return false;
            }
        }
        if (!buildSuite(out.suite, out.bench.scenarios)) {
            std::string names;
            for (const std::string& s : suiteNames()) names += (names.empty() ? "" : ", ") + s;
            error = "unknown --suite '" + out.suite + "' (expected one of: " + names + ")";
            return false;
        }
    } else {
        if (scenario.type == ScenarioType::Maze && seen.count("--density")) {
            error = "--density does not apply to --scenario maze";
            return false;
        }
        if (scenario.type == ScenarioType::Maze) scenario.obstacleDensity = 0.0;
        out.bench.scenarios = {scenario};
    }
    const long long totalRuns = static_cast<long long>(out.bench.scenarios.size()) * out.bench.grids *
                                out.bench.runs * static_cast<long long>(out.bench.algorithms.size());
    if (totalRuns > kMaxTotalRuns) {
        error = "configuration requests " + std::to_string(totalRuns) + " timed runs (scenarios x grids x runs x algorithms); the limit is " +
                std::to_string(kMaxTotalRuns);
        return false;
    }
    // Destinations, metadata sidecars and staging files must all be
    // distinct; checked here, before any file is opened.
    if (!checkOutputPathCollisions(out.outputRequests(), error)) return false;
    const std::string configError = validateConfig(out.bench);
    if (!configError.empty()) {
        error = configError;
        return false;
    }
    return true;
}

std::vector<OutputRequest> CliOptions::outputRequests() const {
    std::vector<OutputRequest> requests;
    const std::pair<const char*, const std::string*> outputs[] = {
        {"--csv", &csvPath}, {"--grid-csv", &gridCsvPath}, {"--summary-csv", &summaryPath}, {"--paired-csv", &pairedPath}};
    for (const auto& [option, path] : outputs) {
        if (!path->empty()) requests.push_back({option, *path});
    }
    return requests;
}

std::vector<std::string> CliOptions::outputPaths() const {
    std::vector<std::string> paths;
    for (const std::string* p : {&csvPath, &gridCsvPath, &summaryPath, &pairedPath}) {
        if (!p->empty()) paths.push_back(*p);
    }
    return paths;
}

void printUsage(std::ostream& out, const char* program) {
    out << "Usage:\n"
           "  " << program << "                      interactive visualizer (default)\n"
           "  " << program << " --seed N             interactive visualizer with a fixed random seed\n"
           "  " << program << " --benchmark [options]\n"
           "  " << program << " --help\n"
           "\n"
           "Benchmark options:\n"
           "  --algorithm LIST    all (default) or comma list of dijkstra,astar,bfs,dfs\n"
           "  --scenario TYPE     random (default) or maze\n"
           "  --size RxC          grid rows x columns incl. border walls (default 100x100)\n"
           "  --density PCT       random-wall probability in percent, 0-100 (default 30)\n"
           "  --terrain PCT       percent of free cells given weighted terrain (default 0)\n"
           "  --placement P       reachable (default): random start/end redrawn until connected\n"
           "                      random: random start/end, may be unreachable\n"
           "                      corners: (1,1) to (rows-2,cols-2), may be unreachable\n"
           "  --suite NAME        preset scenarios: baseline, density, density-corners, size,\n"
           "                      size-corners, maze, terrain, all\n"
           "  --seed N            base seed (default 42); grid i uses seed N+i\n"
           "  --grids N           independent seeded grids per scenario (default 10)\n"
           "  --runs N            timed runs per algorithm per grid (default 12)\n"
           "  --warmup N          untimed warm-up runs per algorithm per grid (default 1)\n"
           "  --csv PATH          per-run CSV (streamed while the benchmark runs)\n"
           "  --grid-csv PATH     per-grid statistics over repeated runs\n"
           "  --summary-csv PATH  per-scenario statistics across grids\n"
           "  --paired-csv PATH   paired per-grid time ratios between algorithms\n"
           "                      Every CSV gets a PATH.meta.json sidecar describing the run.\n"
           "  --force             overwrite existing output files\n"
           "  --quiet             no progress or metadata on the terminal (files are unaffected)\n"
           "\n"
           "Example:\n"
           "  " << program << " --benchmark --size 100x100 --density 30 --seed 42 --grids 10 --runs 12 --csv results.csv\n"
           "\n"
           "Exit status: 0 success, 1 usage or I/O error, 2 a correctness check failed,\n"
           "             3 some grids could not be generated (e.g. no reachable endpoints).\n";
}
