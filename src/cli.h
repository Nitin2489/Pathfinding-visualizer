// Command-line parsing. Pure: no I/O besides printUsage(), so it is unit
// testable.
#pragma once

#include <cstdint>
#include <ostream>
#include <string>
#include <vector>

#include "benchmark.h"
#include "output.h"

enum class Mode { Interactive, Benchmark, Help };

// Upper bounds accepted from the command line.
constexpr int kMaxDimension = 10000;
constexpr long long kMaxCells = 10000000;  // bounds per-search memory (see docs/benchmark-methodology.md §13)
static_assert(kMaxCells <= Grid::kMaxCells, "CLI cell limit must not exceed what Grid accepts");
constexpr int kMaxRuns = 100000;
constexpr int kMaxGrids = 100000;
constexpr int kMaxWarmup = 1000;
// Every timed run is kept in memory as a RunRecord until the CSV is written.
constexpr long long kMaxTotalRuns = 2000000;

struct CliOptions {
    Mode mode = Mode::Interactive;
    BenchmarkConfig bench;
    std::string suite;        // empty unless --suite
    std::string csvPath;      // per-run CSV, empty = none
    std::string gridCsvPath;  // per-grid (level-1) CSV, empty = none
    std::string summaryPath;  // across-grid (level-2) CSV, empty = none
    std::string pairedPath;   // paired-comparison CSV, empty = none
    bool force = false;       // allow overwriting existing output files

    // Non-empty output paths, in a fixed order.
    std::vector<std::string> outputPaths() const;
    // The same outputs with the option that requested each one.
    std::vector<OutputRequest> outputRequests() const;
    bool quiet = false;       // no progress/metadata output
    bool seedGiven = false;   // interactive: use --seed instead of a random seed
};

// Parses argv[1..]. On failure returns false and sets `error`.
bool parseCommandLine(const std::vector<std::string>& args, CliOptions& out, std::string& error);

void printUsage(std::ostream& out, const char* program);
