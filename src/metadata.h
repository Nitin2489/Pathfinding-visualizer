// Benchmark metadata: timer calibration, build/environment facts and the
// JSON sidecar written next to every CSV. Fields that cannot be determined
// are reported as "unknown" rather than guessed.
#pragma once

#include <string>
#include <vector>

#include "benchmark.h"

// Version of the CSV/metadata format (bumped on incompatible schema changes).
constexpr int kBenchmarkFormatVersion = 2;

struct TimerInfo {
    double nominalTickNs = 0;  // steady_clock::period, in ns
    double overheadNs = 0;     // median duration of an empty timed region
    double granularityNs = 0;  // smallest non-zero difference between consecutive reads
    // Measurements below this are flagged as approximate:
    // 100 x max(granularity, overhead).
    double floorMs = 0;
};

// Measures the steady clock (takes a few milliseconds).
TimerInfo calibrateTimer();

struct EnvironmentInfo {
    std::string programVersion;
    std::string gitCommit;  // at build time; "unknown" without git
    std::string gitDirty;   // "true"/"false"/"unknown": uncommitted or untracked files at build time
    std::string compiler;
    std::string cxxStandard;
    std::string buildType;
    std::string os;
    std::string osRelease;
    std::string machine;
    std::string cpuModel;
    unsigned logicalCpus = 0;  // 0 = unknown
    std::string timestampUtc;  // ISO 8601
};

EnvironmentInfo collectEnvironment();

struct RunOutcome {
    std::string status;  // "complete", "check_failures", "generation_failures"
    int invalidRuns = 0;
    int generationFailures = 0;
    std::vector<std::string> generationErrors;
    std::vector<std::string> warnings;
};

// JSON document describing one benchmark invocation.
std::string metadataJson(const EnvironmentInfo& env, const TimerInfo& timer, const BenchmarkConfig& config,
                         const std::string& suite, const std::vector<std::string>& outputs, const RunOutcome& outcome);

// JSON string literal (with quotes) for arbitrary text.
std::string jsonString(const std::string& text);
