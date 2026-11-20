// Entry point: interactive visualizer by default, headless benchmark with
// --benchmark. See --help.
#include <algorithm>
#include <iostream>
#include <random>
#include <stdexcept>
#include <string>
#include <vector>

#include "benchmark.h"
#include "cli.h"
#include "metadata.h"
#include "output.h"
#include "report.h"
#include "visualizer.h"

namespace {

constexpr int kExitOk = 0;
constexpr int kExitUsageOrIo = 1;
constexpr int kExitCheckFailed = 2;
constexpr int kExitGenerationFailed = 3;

void printConsoleMetadata(const EnvironmentInfo& env, const TimerInfo& timer, const BenchmarkConfig& c) {
    std::cout << "Pathfinding benchmark (format v" << kBenchmarkFormatVersion << ")\n"
              << "  program:     " << env.programVersion << ", git " << env.gitCommit.substr(0, 12)
              << (env.gitDirty == "true" ? " (modified)" : "") << '\n'
              << "  build:       " << buildDescription() << '\n'
              << "  platform:    " << env.os << ' ' << env.osRelease << ' ' << env.machine << ", " << env.cpuModel << '\n'
              << "  timer:       steady_clock, overhead " << timer.overheadNs << " ns, granularity "
              << timer.granularityNs << " ns; values below " << timer.floorMs * 1000.0 << " us are flagged\n"
              << "  algorithms: ";
    for (Algorithm a : c.algorithms) std::cout << ' ' << algorithmName(a);
    std::cout << "\n  seeds:       " << c.baseSeed << " .. " << c.baseSeed + static_cast<std::uint64_t>(c.grids - 1)
              << " (" << c.grids << " grid(s) per scenario)\n"
              << "  runs:        " << c.runs << " timed + " << c.warmup << " warm-up per algorithm per grid\n"
              << "  scenarios:   " << c.scenarios.size() << '\n';
    for (const Scenario& s : c.scenarios) std::cout << "    - " << scenarioId(s) << '\n';
}

int runBenchmarkMode(const CliOptions& opt) {
    BenchmarkConfig config = opt.bench;
    const std::vector<std::string> paths = opt.outputPaths();
    std::string error;
    if (!checkOutputsWritable(paths, opt.force, error)) {
        std::cerr << "error: " << error << '\n';
        return kExitUsageOrIo;
    }

    // Stage every output now so a bad path fails before any work is done.
    // On any early return below, the StagedFile destructors remove the
    // staging files opened so far; final files are never touched.
    StagedFile runs, grid, summary, paired;
    const std::pair<const std::string*, StagedFile*> staged[] = {
        {&opt.csvPath, &runs}, {&opt.gridCsvPath, &grid}, {&opt.summaryPath, &summary}, {&opt.pairedPath, &paired}};
    for (const auto& [path, file] : staged) {
        if (!path->empty() && !file->open(*path, error)) {
            std::cerr << "error: " << error << '\n';
            return kExitUsageOrIo;
        }
    }

    const std::vector<std::string> warnings = configWarnings(config);
    for (const std::string& w : warnings) std::cerr << "warning: " << w << '\n';

    const TimerInfo timer = calibrateTimer();
    config.timerFloorMs = timer.floorMs;
    const EnvironmentInfo env = collectEnvironment();
    if (!opt.quiet) printConsoleMetadata(env, timer, config);

    BenchmarkCallbacks callbacks;
    callbacks.progress = opt.quiet ? nullptr : &std::cerr;
    if (runs.isOpen()) {
        writeRunsCsvHeader(runs.stream());
        callbacks.onRun = [&runs](const RunRecord& r) { writeRunsCsvRow(runs.stream(), r); };
        callbacks.onGridDone = [&runs] {
            runs.stream().flush();
            if (!runs.stream()) throw std::runtime_error("write error on '" + runs.partialPath() + "'");
        };
    }

    BenchmarkResult result;
    try {
        result = runBenchmark(config, callbacks);
    } catch (const std::exception& e) {
        std::cerr << "error: benchmark aborted: " << e.what() << '\n';
        if (runs.isOpen()) {
            runs.retain();  // keep the completed per-run rows for recovery
            std::cerr << "  completed runs were kept in '" << runs.partialPath() << "'\n";
        }
        std::cerr << "  final output files were not modified\n";
        return kExitUsageOrIo;
    }

    const std::vector<Aggregate> aggregates = aggregate(result.grids, result.scenarios);
    const std::vector<PairedComparison> pairs = pairedComparisons(result.grids);
    printSummaryTable(std::cout, aggregates, pairs, config.timerFloorMs);

    if (grid.isOpen()) writeGridCsv(grid.stream(), result.grids);
    if (summary.isOpen()) writeSummaryCsv(summary.stream(), aggregates);
    if (paired.isOpen()) writePairedCsv(paired.stream(), pairs);

    RunOutcome outcome;
    outcome.invalidRuns = result.invalidRuns;
    outcome.generationFailures = result.generationFailures;
    for (const ScenarioStatus& s : result.scenarios) {
        for (const std::string& e : s.generationErrors) outcome.generationErrors.push_back(s.scenarioId + " " + e);
    }
    outcome.warnings = warnings;
    outcome.status = !result.failures.empty() ? "check_failures"
                     : result.generationFailures > 0 ? "generation_failures"
                                                     : "complete";
    const std::string json = metadataJson(env, timer, config, opt.suite, paths, outcome);

    // Commit data files first, then their metadata sidecars.
    bool ioOk = true;
    for (const auto& [path, file] : staged) {
        if (path->empty()) continue;
        StagedFile meta;
        if (!file->commit(error) || !meta.open(metadataPathFor(*path), error)) {
            std::cerr << "error: " << error << '\n';
            ioOk = false;
            continue;
        }
        meta.stream() << json;
        if (!meta.commit(error)) {
            std::cerr << "error: " << error << '\n';
            ioOk = false;
            continue;
        }
        if (!opt.quiet) std::cout << "Wrote " << *path << " (+ .meta.json)\n";
    }

    if (!outcome.generationErrors.empty()) {
        std::cerr << '\n' << outcome.generationErrors.size() << " grid(s) could not be generated:\n";
        for (std::size_t i = 0; i < std::min<std::size_t>(outcome.generationErrors.size(), 20); ++i) {
            std::cerr << "  " << outcome.generationErrors[i] << '\n';
        }
    }
    if (!result.failures.empty()) {
        std::cerr << '\n' << result.failures.size() << " correctness check failure(s) (" << result.invalidRuns
                  << " invalid run(s), excluded from statistics):\n";
        for (std::size_t i = 0; i < std::min<std::size_t>(result.failures.size(), 20); ++i) {
            std::cerr << "  " << result.failures[i] << '\n';
        }
    }
    if (!ioOk) return kExitUsageOrIo;
    if (!result.failures.empty()) return kExitCheckFailed;
    if (result.generationFailures > 0) return kExitGenerationFailed;
    if (!opt.quiet) std::cout << "All correctness checks passed.\n";
    return kExitOk;
}

}  // namespace

int main(int argc, char* argv[]) {
    const std::vector<std::string> args(argv + 1, argv + argc);
    CliOptions opt;
    std::string error;
    if (!parseCommandLine(args, opt, error)) {
        std::cerr << "error: " << error << "\nTry '" << argv[0] << " --help'.\n";
        return kExitUsageOrIo;
    }
    switch (opt.mode) {
        case Mode::Help:
            printUsage(std::cout, argv[0]);
            return kExitOk;
        case Mode::Benchmark:
            return runBenchmarkMode(opt);
        case Mode::Interactive:
        default: {
            std::uint64_t seed = opt.bench.baseSeed;
            if (!opt.seedGiven) {
                std::random_device rd;
                seed = (static_cast<std::uint64_t>(rd()) << 32) ^ rd();
            }
            return runInteractive(seed);
        }
    }
}
