#include "metadata.h"

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <ctime>
#include <fstream>
#include <locale>
#include <sstream>
#include <thread>

#if __has_include("git_info.h")
#include "git_info.h"  // generated at build time by cmake/GitInfo.cmake
#endif
#ifndef PATHFINDING_GIT_COMMIT
#define PATHFINDING_GIT_COMMIT "unknown"
#endif
#ifndef PATHFINDING_GIT_DIRTY
#define PATHFINDING_GIT_DIRTY "unknown"
#endif
#ifndef PATHFINDING_VERSION
#define PATHFINDING_VERSION "unknown"
#endif
#ifndef PATHFINDING_BUILD_TYPE
#define PATHFINDING_BUILD_TYPE "unknown"
#endif

#if defined(__unix__) || defined(__APPLE__)
#include <sys/utsname.h>
#endif
#if defined(__APPLE__)
#include <sys/sysctl.h>
#endif

namespace {

using Clock = std::chrono::steady_clock;

std::string num(double v) {
    std::ostringstream s;
    s.imbue(std::locale::classic());
    s.precision(6);
    s << v;
    return s.str();
}

std::string compilerName() {
    std::ostringstream s;
#if defined(__clang__)
    s << "Clang " << __clang_major__ << '.' << __clang_minor__ << '.' << __clang_patchlevel__;
#elif defined(__GNUC__)
    s << "GCC " << __GNUC__ << '.' << __GNUC_MINOR__ << '.' << __GNUC_PATCHLEVEL__;
#elif defined(_MSC_VER)
    s << "MSVC " << _MSC_FULL_VER;
#else
    s << "unknown";
#endif
    return s.str();
}

std::string cxxStandard() {
#if defined(_MSVC_LANG)
    return std::to_string(_MSVC_LANG);
#else
    return std::to_string(__cplusplus);
#endif
}

std::string cpuModel() {
#if defined(__linux__)
    std::ifstream in("/proc/cpuinfo");
    for (std::string line; std::getline(in, line);) {
        if (line.rfind("model name", 0) == 0) {
            const auto colon = line.find(':');
            if (colon != std::string::npos && colon + 2 <= line.size()) return line.substr(colon + 2);
        }
    }
#elif defined(__APPLE__)
    char buf[256];
    size_t len = sizeof buf;
    if (sysctlbyname("machdep.cpu.brand_string", buf, &len, nullptr, 0) == 0) return std::string(buf);
#endif
    return "unknown";
}

std::string utcTimestamp() {
    const std::time_t now = std::time(nullptr);
    std::tm tm{};
#if defined(_WIN32)
    if (gmtime_s(&tm, &now) != 0) return "unknown";
#else
    if (gmtime_r(&now, &tm) == nullptr) return "unknown";
#endif
    char buf[32];
    if (std::strftime(buf, sizeof buf, "%Y-%m-%dT%H:%M:%SZ", &tm) == 0) return "unknown";
    return buf;
}

}  // namespace

TimerInfo calibrateTimer() {
    TimerInfo t;
    t.nominalTickNs = 1e9 * static_cast<double>(Clock::period::num) / static_cast<double>(Clock::period::den);

    std::vector<double> overhead;
    overhead.reserve(20001);
    for (int i = 0; i < 20001; ++i) {
        const auto a = Clock::now();
        const auto b = Clock::now();
        overhead.push_back(std::chrono::duration<double, std::nano>(b - a).count());
    }
    std::nth_element(overhead.begin(), overhead.begin() + 10000, overhead.end());
    t.overheadNs = overhead[10000];

    double smallest = 0;
    for (int i = 0; i < 2000; ++i) {
        const auto a = Clock::now();
        auto b = Clock::now();
        for (int spin = 0; b == a && spin < 1000000; ++spin) b = Clock::now();
        const double d = std::chrono::duration<double, std::nano>(b - a).count();
        if (d > 0 && (smallest == 0 || d < smallest)) smallest = d;
    }
    t.granularityNs = smallest > 0 ? smallest : t.nominalTickNs;
    t.floorMs = 100.0 * std::max(t.granularityNs, t.overheadNs) / 1e6;
    return t;
}

EnvironmentInfo collectEnvironment() {
    EnvironmentInfo e;
    e.programVersion = PATHFINDING_VERSION;
    e.gitCommit = PATHFINDING_GIT_COMMIT;
    e.gitDirty = PATHFINDING_GIT_DIRTY;
    e.compiler = compilerName();
    e.cxxStandard = cxxStandard();
    e.buildType = std::string(PATHFINDING_BUILD_TYPE).empty() ? "unknown" : PATHFINDING_BUILD_TYPE;
#if defined(NDEBUG)
    e.buildType += " (NDEBUG)";
#else
    e.buildType += " (assertions enabled)";
#endif
    e.os = "unknown";
    e.osRelease = "unknown";
    e.machine = "unknown";
#if defined(_WIN32)
    e.os = "Windows";
#endif
#if defined(__unix__) || defined(__APPLE__)
    struct utsname u {};
    if (uname(&u) == 0) {
        e.os = u.sysname;
        e.osRelease = u.release;  // e.g. contains "microsoft-standard-WSL2" under WSL2
        e.machine = u.machine;
    }
#endif
    e.cpuModel = cpuModel();
    e.logicalCpus = std::thread::hardware_concurrency();
    e.timestampUtc = utcTimestamp();
    return e;
}

std::string jsonString(const std::string& text) {
    std::string out = "\"";
    for (char ch : text) {
        const auto u = static_cast<unsigned char>(ch);
        switch (ch) {
            case '"': out += "\\\""; break;
            case '\\': out += "\\\\"; break;
            case '\n': out += "\\n"; break;
            case '\r': out += "\\r"; break;
            case '\t': out += "\\t"; break;
            default:
                if (u < 0x20) {
                    char buf[8];
                    std::snprintf(buf, sizeof buf, "\\u%04x", u);
                    out += buf;
                } else {
                    out += ch;
                }
        }
    }
    return out + "\"";
}

std::string metadataJson(const EnvironmentInfo& env, const TimerInfo& timer, const BenchmarkConfig& config,
                         const std::string& suite, const std::vector<std::string>& outputs, const RunOutcome& outcome) {
    auto list = [](const std::vector<std::string>& items) {
        std::string s = "[";
        for (std::size_t i = 0; i < items.size(); ++i) s += (i ? ", " : "") + jsonString(items[i]);
        return s + "]";
    };
    std::ostringstream j;
    j.imbue(std::locale::classic());
    j << "{\n";
    j << "  \"benchmark_format_version\": " << kBenchmarkFormatVersion << ",\n";
    j << "  \"status\": " << jsonString(outcome.status) << ",\n";
    j << "  \"program\": {\"version\": " << jsonString(env.programVersion) << ", \"git_commit\": "
      << jsonString(env.gitCommit) << ", \"git_dirty_at_build\": " << jsonString(env.gitDirty) << "},\n";
    j << "  \"build\": {\"compiler\": " << jsonString(env.compiler) << ", \"cxx_standard\": "
      << jsonString(env.cxxStandard) << ", \"build_type\": " << jsonString(env.buildType) << "},\n";
    j << "  \"environment\": {\"os\": " << jsonString(env.os) << ", \"os_release\": " << jsonString(env.osRelease)
      << ", \"machine\": " << jsonString(env.machine) << ", \"cpu_model\": " << jsonString(env.cpuModel)
      << ", \"logical_cpus\": "
      << (env.logicalCpus > 0 ? std::to_string(env.logicalCpus) : std::string("\"unknown\""))
      << ", \"timestamp_utc\": " << jsonString(env.timestampUtc) << "},\n";
    j << "  \"timer\": {\"clock\": \"std::chrono::steady_clock\", \"nominal_tick_ns\": " << num(timer.nominalTickNs)
      << ", \"measured_overhead_ns\": " << num(timer.overheadNs) << ", \"measured_granularity_ns\": "
      << num(timer.granularityNs) << ", \"near_floor_threshold_ms\": " << num(timer.floorMs) << "},\n";
    j << "  \"timing_methodology\": {\n"
      << "    \"search_time_ms\": \"core search: from after per-cell arrays are allocated and initialised to the end "
         "of path reconstruction (frontier growth included)\",\n"
      << "    \"end_to_end_time_ms\": \"one complete search call: allocation, initialisation, search, path "
         "reconstruction and freeing of the per-cell arrays\",\n"
      << "    \"measurement\": \"both values come from the same call; one fresh call per timed run\",\n"
      << "    \"excluded\": \"grid generation, reference validation, rendering, progress output, CSV writing\",\n"
      << "    \"run_order\": \"interleaved; algorithm order rotated by run index\"\n  },\n";
    std::vector<std::string> algorithms;
    for (Algorithm a : config.algorithms) algorithms.emplace_back(algorithmName(a));
    j << "  \"configuration\": {\"suite\": " << (suite.empty() ? std::string("null") : jsonString(suite))
      << ", \"algorithms\": " << list(algorithms) << ", \"base_seed\": " << jsonString(std::to_string(config.baseSeed))
      << ", \"grids_per_scenario\": " << config.grids << ", \"runs_per_grid\": " << config.runs
      << ", \"warmup_runs\": " << config.warmup << ", \"seed_rule\": \"grid i uses seed base_seed + i\",\n";
    j << "    \"rng_streams\": \"obstacles: seed; endpoints: splitmix64-derived stream 1; terrain: stream 2\",\n";
    j << "    \"max_placement_attempts\": " << kMaxPlacementAttempts << ",\n    \"scenarios\": [\n";
    for (std::size_t i = 0; i < config.scenarios.size(); ++i) {
        const Scenario& s = config.scenarios[i];
        j << "      {\"scenario_id\": " << jsonString(scenarioId(s)) << ", \"type\": "
          << jsonString(scenarioTypeName(s.type)) << ", \"rows\": " << s.rows << ", \"columns\": " << s.cols
          << ", \"obstacle_density\": " << formatRoundTrip(s.obstacleDensity)
          << ", \"terrain_density\": " << formatRoundTrip(s.terrainDensity) << ", \"placement\": "
          << jsonString(placementName(s.placement)) << "}" << (i + 1 < config.scenarios.size() ? "," : "") << '\n';
    }
    j << "    ]\n  },\n";
    j << "  \"validation\": {\"oracle\": \"independent reachability BFS and Dial bucket-queue minimum cost "
         "(validation.cpp)\", \"invalid_runs\": "
      << outcome.invalidRuns << ", \"generation_failures\": " << outcome.generationFailures
      << ", \"generation_errors\": " << list(outcome.generationErrors) << "},\n";
    j << "  \"warnings\": " << list(outcome.warnings) << ",\n";
    j << "  \"outputs\": " << list(outputs) << "\n";
    j << "}\n";
    return j.str();
}
