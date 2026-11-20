// Benchmark engine: reproducible scenario generation, repeated headless
// execution of the shared search implementations, independent per-run
// validation (validation.h), and hierarchical aggregation. No rendering,
// no file I/O: per-run records are handed to a callback as they are made.
#pragma once

#include <cstdint>
#include <functional>
#include <optional>
#include <ostream>
#include <string>
#include <vector>

#include "grid.h"
#include "search.h"
#include "validation.h"

enum class ScenarioType {
    Random,  // border walls + independent random walls at obstacleDensity
    Maze,    // border walls + recursive-division maze (obstacleDensity unused)
};

// How start and end are chosen. Walls are generated first and never depend
// on the placement mode (see generateScenario()).
enum class Placement {
    Corners,    // start (1,1), end (rows-2, cols-2). Unrestricted: may be unreachable.
    Random,     // uniform over distinct passable interior cells. Unrestricted.
    Reachable,  // like Random, but redrawn until start and end are connected.
};

struct Scenario {
    ScenarioType type = ScenarioType::Random;
    int rows = 100;
    int cols = 100;
    double obstacleDensity = 0.30;  // wall probability in [0, 1], Random only
    double terrainDensity = 0.0;    // fraction of free cells given terrain cost
    Placement placement = Placement::Corners;
};

// Shortest decimal text that parses back to exactly `v` (C locale).
std::string formatRoundTrip(double v);

// Exact, injective identifier used both for display and for grouping, e.g.
// "random_100x100_obs0.3_ter0_reachable". Densities are written as
// round-trip fractions, so distinct parameters never share an id.
std::string scenarioId(const Scenario& s);
const char* scenarioTypeName(ScenarioType t);
const char* placementName(Placement p);
bool parsePlacement(const std::string& text, Placement& out);

// Upper bound on endpoint draws for Placement::Reachable.
constexpr int kMaxPlacementAttempts = 10000;

// Seed of an independent RNG stream derived from a grid seed. Stream 0 is
// the seed itself (obstacles / maze); 1 is endpoints; 2 is terrain.
std::uint64_t deriveStreamSeed(std::uint64_t seed, int stream);

struct ScenarioInstance {
    Grid grid;                  // meaningful only when ok
    bool ok = false;
    int placementAttempts = 0;  // endpoint draws used (0 for Corners)
    std::string error;          // why generation failed
};

// Deterministically builds the grid for (scenario, seed):
//  1. walls/maze from stream 0, with (1,1) and (rows-2, cols-2) never walled;
//  2. endpoints from stream 1 (Random/Reachable), chosen among passable
//     interior cells, so no wall is ever removed or moved;
//  3. terrain from stream 2 on free cells other than start/end.
// The obstacle layout is therefore identical for every placement mode and
// terrain density. Reachable placement uses rejection sampling: each
// attempt draws an ordered pair of distinct passable cells and accepts it if
// both are in the same 4-connected component; this samples uniformly from
// connected pairs. It fails (ok = false) when fewer than two passable cells
// exist or no connected pair is drawn within kMaxPlacementAttempts.
ScenarioInstance generateScenario(const Scenario& scenario, std::uint64_t seed);
// generateScenario() that throws std::runtime_error on failure.
Grid buildScenarioGrid(const Scenario& scenario, std::uint64_t seed);

struct BenchmarkConfig {
    std::vector<Scenario> scenarios{Scenario{}};
    std::vector<Algorithm> algorithms = allAlgorithms();
    std::uint64_t baseSeed = 42;
    int grids = 10;  // independent grids per scenario, seeds baseSeed .. baseSeed+grids-1
    int runs = 12;   // timed repetitions of every algorithm on every grid
    int warmup = 1;  // untimed repetitions per algorithm per grid, discarded
    // Per-grid median core times below this are flagged as near the timer
    // floor (0 disables). The CLI sets it from a timer calibration.
    double timerFloorMs = 0.0;
};

// Empty if the configuration is usable; otherwise the reason. Checks counts,
// algorithms, scenario parameters and that baseSeed + grids - 1 does not
// overflow 64 bits.
std::string validateConfig(const BenchmarkConfig& config);
// Non-fatal methodology warnings (single grid, unbalanced run order, seeds
// that spreadsheet tools cannot represent exactly).
std::vector<std::string> configWarnings(const BenchmarkConfig& config);

// Named preset suites (see docs/benchmark-methodology.md §2): "baseline", "density",
// "density-corners", "size", "size-corners", "maze", "terrain", "all".
bool buildSuite(const std::string& name, std::vector<Scenario>& out);
const std::vector<std::string>& suiteNames();

// One timed execution of one algorithm on one grid.
struct RunRecord {
    std::string scenarioId;
    Scenario scenario;
    Algorithm algorithm = Algorithm::Dijkstra;
    std::uint64_t seed = 0;
    int gridIndex = 0;
    int runIndex = 0;
    int orderPosition = 0;  // 0-based position of this algorithm within its run
    std::uint64_t gridHash = 0;
    int placementAttempts = 0;
    Cell start;
    Cell end;
    ReferenceSolution reference;
    SearchMetrics metrics;
    bool valid = true;    // every check passed; invalid runs are excluded from statistics
    std::string failure;  // "; "-joined failed checks, empty when valid
};

struct Stats {
    double min = 0, max = 0, mean = 0, median = 0, stddev = 0;  // stddev: sample (n-1), 0 if n < 2
};
// Returns all zeros for an empty input.
Stats computeStats(std::vector<double> values);
// Linear-interpolation quantile (R type 7 / NumPy default) of a SORTED,
// non-empty vector; p in [0, 1].
double quantileSorted(const std::vector<double>& sorted, double p);

// Level 1: one algorithm on one grid, over its repeated runs.
struct GridSummary {
    std::string scenarioId;
    Scenario scenario;
    Algorithm algorithm = Algorithm::Dijkstra;
    int gridIndex = 0;
    std::uint64_t seed = 0;
    std::uint64_t gridHash = 0;
    int placementAttempts = 0;
    Cell start;
    Cell end;
    ReferenceSolution reference;
    int runs = 0;       // timed runs attempted
    int validRuns = 0;  // runs that passed every check
    // Non-time metrics of the valid runs (identical across valid runs by
    // construction: a deviating run is marked invalid). Unset if validRuns == 0.
    std::optional<SearchMetrics> metrics;
    std::optional<Stats> searchMs;     // core time over valid runs
    std::optional<Stats> endToEndMs;   // end-to-end time over valid runs
    std::optional<double> searchCv;    // stddev / mean of core time, if n >= 2 and mean > 0
    // pathCost / reference minimum cost; set only when a path was found and
    // the reference minimum is > 0 (0/0 when start == end is left unset).
    std::optional<double> costRatio;
    bool nearTimerFloor = false;  // median core time below config.timerFloorMs
};

// Generation outcome per scenario.
struct ScenarioStatus {
    std::string scenarioId;
    Scenario scenario;
    int gridsRequested = 0;
    int gridsGenerated = 0;
    std::vector<std::string> generationErrors;
};

struct BenchmarkResult {
    std::vector<GridSummary> grids;  // scenario-major, then grid, then algorithm
    std::vector<ScenarioStatus> scenarios;
    std::vector<std::string> failures;  // failed checks and caught exceptions
    int invalidRuns = 0;
    int generationFailures = 0;
};

struct BenchmarkCallbacks {
    std::function<void(const RunRecord&)> onRun;   // every timed run, as it completes
    std::function<void()> onGridDone;              // after all runs of one grid
    std::ostream* progress = nullptr;              // one line per grid
};

// Runs every scenario x grid x run x algorithm. Each grid is generated once
// and shared (as one const object) by every algorithm and run. Runs are
// interleaved: run r executes all algorithms, the order rotated by r, so each
// algorithm takes each position equally often when runs is a multiple of
// the number of algorithms (configWarnings() reports otherwise).
//
// Every run is validated by validateRun() against computeReference(), both
// outside the timed region; a run whose non-time metrics differ from the
// first valid run on that grid is also invalid. Exceptions thrown by a
// search are caught and recorded as invalid runs. Throws
// std::invalid_argument if validateConfig() fails. Exceptions thrown by the
// callbacks propagate.
BenchmarkResult runBenchmark(const BenchmarkConfig& config, const BenchmarkCallbacks& callbacks = {});

// Distribution of per-grid values across grids (n = number of grids).
struct Distribution {
    int n = 0;
    double median = 0, mean = 0, q1 = 0, q3 = 0;
    double iqr() const { return q3 - q1; }
};
// Empty vector -> nullopt.
std::optional<Distribution> distributionOf(std::vector<double> values);

// Level 2: one algorithm in one scenario, across grids. Time distributions
// are over per-grid MEDIANS; a grid counts once no matter how many runs.
struct Aggregate {
    std::string scenarioId;
    Scenario scenario;
    Algorithm algorithm = Algorithm::Dijkstra;
    int gridsRequested = 0;
    int gridsGenerated = 0;
    int gridsMeasured = 0;  // grids with at least one valid run
    int solvableGrids = 0;  // measured grids where the reference finds a path
    int gridsSolved = 0;    // measured grids where this algorithm found a path
    int runsPerGrid = 0;    // timed runs per grid (max over grids)
    int invalidRuns = 0;
    int gridsNearTimerFloor = 0;
    std::optional<double> solvabilityRate;  // solvableGrids / gridsMeasured
    std::optional<double> successRate;      // gridsSolved / gridsMeasured
    std::optional<Distribution> searchMs, endToEndMs;              // all measured grids
    std::optional<Distribution> searchMsSolvable, endToEndMsSolvable;  // solvable grids only
    std::optional<double> medianSearchCv;  // median over grids of within-grid CV
    // Means over measured grids of the (deterministic) per-grid metrics.
    std::optional<double> meanNodesExpanded, meanNodesDiscovered, meanPeakFrontier, meanExplorationRatio;
    std::optional<double> meanPathLength;  // over grids where a path was found
    std::optional<double> meanCostRatio;   // over grids with a defined cost ratio
    int costRatioGrids = 0;
    // Median over grids of nodesExpanded / median core ms (grids with
    // nodesExpanded > 0 and core time > 0, not near the timer floor).
    std::optional<double> medianExpansionsPerSearchMs;
};

// `scenarios` supplies grids requested/generated; if empty those fields are
// set to the number of distinct grids seen.
std::vector<Aggregate> aggregate(const std::vector<GridSummary>& grids,
                                 const std::vector<ScenarioStatus>& scenarios = {});

enum class TimingKind { Search, EndToEnd };
const char* timingKindName(TimingKind k);

// Paired comparison of algorithms a and b on the same grids: per grid,
// ratio = median time(a) / median time(b). Grids where either side has no
// valid runs or a non-positive median are excluded (and counted), never
// substituted.
struct PairedComparison {
    std::string scenarioId;
    Scenario scenario;
    Algorithm a = Algorithm::Dijkstra;
    Algorithm b = Algorithm::AStar;
    TimingKind timing = TimingKind::Search;
    int gridsCompared = 0;
    int gridsExcluded = 0;
    std::optional<Distribution> ratio;     // of per-grid ratios
    std::optional<double> geomeanRatio;    // exp(mean(log ratio))
    std::optional<double> aFasterFraction; // share of compared grids with ratio < 1
};

// Every unordered algorithm pair (in first-seen order) per scenario, for
// both timing kinds.
std::vector<PairedComparison> pairedComparisons(const std::vector<GridSummary>& grids);
