#include "benchmark.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <limits>
#include <map>
#include <set>
#include <sstream>
#include <stdexcept>
#include <utility>

namespace {

std::uint64_t splitmix64(std::uint64_t x) {
    x += 0x9E3779B97F4A7C15ULL;
    x = (x ^ (x >> 30)) * 0xBF58476D1CE4E5B9ULL;
    x = (x ^ (x >> 27)) * 0x94D049BB133111EBULL;
    return x ^ (x >> 31);
}

bool sameNonTimeMetrics(const SearchMetrics& a, const SearchMetrics& b) {
    return a.pathFound == b.pathFound && a.nodesDiscovered == b.nodesDiscovered &&
           a.nodesExpanded == b.nodesExpanded && a.pathLength == b.pathLength && a.pathCost == b.pathCost &&
           a.peakFrontierSize == b.peakFrontierSize;
}

std::string join(const std::vector<std::string>& parts, const char* sep) {
    std::string out;
    for (const std::string& p : parts) out += (out.empty() ? "" : sep) + p;
    return out;
}

std::optional<double> meanOf(double sum, int n) {
    if (n <= 0) return std::nullopt;
    return sum / n;
}

}  // namespace

// ------------------------------------------------------------- scenarios

std::string formatRoundTrip(double v) {
    char buf[64];
    for (int precision = 1; precision <= 17; ++precision) {
        std::snprintf(buf, sizeof buf, "%.*g", precision, v);
        if (std::strtod(buf, nullptr) == v) break;
    }
    return buf;
}

const char* scenarioTypeName(ScenarioType t) { return t == ScenarioType::Maze ? "maze" : "random"; }

const char* placementName(Placement p) {
    switch (p) {
        case Placement::Corners: return "corners";
        case Placement::Random: return "random";
        case Placement::Reachable: return "reachable";
    }
    return "unknown";
}

bool parsePlacement(const std::string& text, Placement& out) {
    if (text == "corners") out = Placement::Corners;
    else if (text == "random") out = Placement::Random;
    else if (text == "reachable") out = Placement::Reachable;
    else return false;
    return true;
}

std::string scenarioId(const Scenario& s) {
    std::ostringstream id;
    id << scenarioTypeName(s.type) << '_' << s.rows << 'x' << s.cols;
    if (s.type == ScenarioType::Random) id << "_obs" << formatRoundTrip(s.obstacleDensity);
    id << "_ter" << formatRoundTrip(s.terrainDensity) << '_' << placementName(s.placement);
    return id.str();
}

std::uint64_t deriveStreamSeed(std::uint64_t seed, int stream) {
    if (stream == 0) return seed;
    return splitmix64(seed + static_cast<std::uint64_t>(stream) * 0xD1B54A32D192ED03ULL);
}

ScenarioInstance generateScenario(const Scenario& s, std::uint64_t seed) {
    ScenarioInstance inst;
    try {
        Grid grid(s.rows, s.cols);  // throws on invalid dimensions

        // 1. Obstacles (stream 0). The default corner endpoints are protected
        //    here in every placement mode, so the wall layout never depends
        //    on the placement mode.
        Rng obstacles(deriveStreamSeed(seed, 0));
        if (s.type == ScenarioType::Random) grid.generateRandomWalls(s.obstacleDensity, obstacles);
        else grid.generateMaze(obstacles);

        // 2. Endpoints (stream 1), among existing passable interior cells.
        if (s.placement != Placement::Corners) {
            std::vector<int> cells;
            for (int r = 1; r < grid.rows() - 1; ++r) {
                for (int c = 1; c < grid.cols() - 1; ++c) {
                    if (grid.isPassable(r, c)) cells.push_back(grid.index(r, c));
                }
            }
            Rng endpoints(deriveStreamSeed(seed, 1));
            const auto n = static_cast<std::uint64_t>(cells.size());
            auto drawPair = [&](int& a, int& b) {
                const std::uint64_t i = endpoints.uniformInt(n);
                std::uint64_t j = endpoints.uniformInt(n - 1);
                if (j >= i) ++j;
                a = cells[i];
                b = cells[j];
            };
            int startIdx = -1;
            int endIdx = -1;
            if (s.placement == Placement::Random) {
                if (n == 0) {
                    inst.error = "no passable interior cell for random placement";
                    return inst;
                }
                if (n == 1) {
                    startIdx = endIdx = cells[0];  // only one cell: start == end
                } else {
                    drawPair(startIdx, endIdx);
                }
                inst.placementAttempts = 1;
            } else {
                if (n < 2) {
                    inst.error = "fewer than two passable interior cells; no reachable pair exists";
                    return inst;
                }
                const std::vector<int> label = labelComponents(grid);
                for (int attempt = 1; attempt <= kMaxPlacementAttempts; ++attempt) {
                    int a = 0;
                    int b = 0;
                    drawPair(a, b);
                    inst.placementAttempts = attempt;
                    if (label[static_cast<std::size_t>(a)] == label[static_cast<std::size_t>(b)]) {
                        startIdx = a;
                        endIdx = b;
                        break;
                    }
                }
                if (startIdx < 0) {
                    inst.error = "no connected start/end pair found in " + std::to_string(kMaxPlacementAttempts) +
                                 " attempts";
                    return inst;
                }
            }
            grid.setStart(grid.cellOf(startIdx));
            grid.setEnd(grid.cellOf(endIdx));
        }

        // 3. Terrain (stream 2) on free cells; never on walls, start or end.
        if (s.terrainDensity > 0.0) {
            Rng terrain(deriveStreamSeed(seed, 2));
            grid.generateRandomTerrain(s.terrainDensity, terrain);
        }
        inst.grid = std::move(grid);
        inst.ok = true;
    } catch (const std::exception& e) {
        inst.error = e.what();
    }
    return inst;
}

Grid buildScenarioGrid(const Scenario& scenario, std::uint64_t seed) {
    ScenarioInstance inst = generateScenario(scenario, seed);
    if (!inst.ok) throw std::runtime_error("cannot generate " + scenarioId(scenario) + " seed " + std::to_string(seed) +
                                           ": " + inst.error);
    return std::move(inst.grid);
}

// ------------------------------------------------------------- configuration

std::string validateConfig(const BenchmarkConfig& c) {
    if (c.algorithms.empty()) return "no algorithms selected";
    for (std::size_t i = 0; i < c.algorithms.size(); ++i) {
        for (std::size_t j = i + 1; j < c.algorithms.size(); ++j) {
            if (c.algorithms[i] == c.algorithms[j]) return "algorithm listed twice";
        }
    }
    if (c.scenarios.empty()) return "no scenarios";
    if (c.grids < 1) return "grids must be at least 1";
    if (c.runs < 1) return "runs must be at least 1";
    if (c.warmup < 0) return "warmup must not be negative";
    if (!(c.timerFloorMs >= 0.0) || !std::isfinite(c.timerFloorMs)) return "timer floor must be finite and >= 0";
    if (static_cast<std::uint64_t>(c.grids - 1) > std::numeric_limits<std::uint64_t>::max() - c.baseSeed) {
        return "seed range overflows: base seed + grids - 1 exceeds 2^64 - 1";
    }
    for (const Scenario& s : c.scenarios) {
        if (s.rows < 3 || s.cols < 3) return "scenario grid must be at least 3x3";
        if (static_cast<long long>(s.rows) * s.cols > Grid::kMaxCells) return "scenario grid has too many cells";
        for (double d : {s.obstacleDensity, s.terrainDensity}) {
            if (!std::isfinite(d) || d < 0.0 || d > 1.0) return "scenario densities must be within [0, 1]";
        }
    }
    return "";
}

std::vector<std::string> configWarnings(const BenchmarkConfig& c) {
    std::vector<std::string> w;
    if (c.grids == 1) {
        w.push_back("only one grid per scenario: results describe a single instance, not the scenario "
                    "(use --grids 10 or more)");
    }
    const auto n = static_cast<int>(c.algorithms.size());
    if (n > 1 && c.runs % n != 0) {
        w.push_back("runs (" + std::to_string(c.runs) + ") is not a multiple of the number of algorithms (" +
                    std::to_string(n) + "): run-order positions are not balanced across algorithms");
    }
    // Largest seed used; only meaningful when validateConfig() accepts the range.
    const std::uint64_t maxExact = (std::uint64_t{1} << 53) - 1;
    const bool rangeValid = c.grids >= 1 && static_cast<std::uint64_t>(c.grids - 1) <=
                                                std::numeric_limits<std::uint64_t>::max() - c.baseSeed;
    if (rangeValid && c.baseSeed + static_cast<std::uint64_t>(c.grids - 1) > maxExact) {
        w.push_back("seeds above 2^53 - 1 cannot be represented exactly by spreadsheet tools or JavaScript; "
                    "read the seed column as text");
    }
    return w;
}

// ------------------------------------------------------------- suites

const std::vector<std::string>& suiteNames() {
    static const std::vector<std::string> names = {"baseline", "density", "density-corners", "size",
                                                   "size-corners", "maze", "terrain", "all"};
    return names;
}

bool buildSuite(const std::string& name, std::vector<Scenario>& out) {
    auto random = [](int rows, int cols, double density, Placement placement) {
        Scenario s;
        s.type = ScenarioType::Random;
        s.rows = rows;
        s.cols = cols;
        s.obstacleDensity = density;
        s.placement = placement;
        return s;
    };
    const double densities[] = {0.10, 0.20, 0.30, 0.40, 0.50};
    const int sizes[][2] = {{20, 50}, {50, 100}, {100, 100}, {200, 200}};

    std::vector<Scenario> baseline = {random(100, 100, 0.30, Placement::Reachable)};
    std::vector<Scenario> density, densityCorners, size, sizeCorners, maze;
    for (double d : densities) {
        density.push_back(random(100, 100, d, Placement::Reachable));
        densityCorners.push_back(random(100, 100, d, Placement::Corners));
    }
    for (const auto& rc : sizes) {
        size.push_back(random(rc[0], rc[1], 0.30, Placement::Reachable));
        sizeCorners.push_back(random(rc[0], rc[1], 0.30, Placement::Corners));
    }
    for (int n : {51, 101, 201}) {
        Scenario s;
        s.type = ScenarioType::Maze;
        s.rows = n;
        s.cols = n;
        s.obstacleDensity = 0.0;
        s.placement = Placement::Corners;  // perfect mazes: always connected
        maze.push_back(s);
    }
    Scenario weighted = random(100, 100, 0.20, Placement::Reachable);
    weighted.terrainDensity = 0.30;
    std::vector<Scenario> terrain = {weighted};

    out.clear();
    std::set<std::string> seen;
    auto append = [&](const std::vector<Scenario>& v) {
        for (const Scenario& s : v) {
            if (seen.insert(scenarioId(s)).second) out.push_back(s);
        }
    };
    if (name == "baseline") append(baseline);
    else if (name == "density") append(density);
    else if (name == "density-corners") append(densityCorners);
    else if (name == "size") append(size);
    else if (name == "size-corners") append(sizeCorners);
    else if (name == "maze") append(maze);
    else if (name == "terrain") append(terrain);
    else if (name == "all") {
        for (const auto* v : {&baseline, &density, &densityCorners, &size, &sizeCorners, &maze, &terrain}) append(*v);
    } else {
        return false;
    }
    return true;
}

// ------------------------------------------------------------- statistics

Stats computeStats(std::vector<double> v) {
    Stats s;
    if (v.empty()) return s;
    std::sort(v.begin(), v.end());
    const std::size_t n = v.size();
    s.min = v.front();
    s.max = v.back();
    s.median = (n % 2 == 1) ? v[n / 2] : (v[n / 2 - 1] + v[n / 2]) / 2.0;
    double sum = 0;
    for (double x : v) sum += x;
    s.mean = sum / static_cast<double>(n);
    if (n >= 2) {
        double sq = 0;
        for (double x : v) sq += (x - s.mean) * (x - s.mean);
        s.stddev = std::sqrt(sq / static_cast<double>(n - 1));
    }
    return s;
}

double quantileSorted(const std::vector<double>& v, double p) {
    const double h = (static_cast<double>(v.size()) - 1.0) * p;
    const auto lo = static_cast<std::size_t>(std::floor(h));
    const std::size_t hi = std::min(lo + 1, v.size() - 1);
    return v[lo] + (h - static_cast<double>(lo)) * (v[hi] - v[lo]);
}

std::optional<Distribution> distributionOf(std::vector<double> v) {
    if (v.empty()) return std::nullopt;
    std::sort(v.begin(), v.end());
    Distribution d;
    d.n = static_cast<int>(v.size());
    d.median = quantileSorted(v, 0.5);
    d.q1 = quantileSorted(v, 0.25);
    d.q3 = quantileSorted(v, 0.75);
    double sum = 0;
    for (double x : v) sum += x;
    d.mean = sum / static_cast<double>(v.size());
    return d;
}

// ------------------------------------------------------------- runner

BenchmarkResult runBenchmark(const BenchmarkConfig& config, const BenchmarkCallbacks& cb) {
    const std::string configError = validateConfig(config);
    if (!configError.empty()) throw std::invalid_argument(configError);

    BenchmarkResult result;
    const std::size_t numAlgorithms = config.algorithms.size();

    struct Accumulator {
        std::vector<double> search, endToEnd;
        std::optional<SearchMetrics> reference;  // non-time metrics of the first valid run
        int runs = 0;
        int valid = 0;
    };

    for (const Scenario& scenario : config.scenarios) {
        const std::string id = scenarioId(scenario);
        ScenarioStatus status;
        status.scenarioId = id;
        status.scenario = scenario;
        status.gridsRequested = config.grids;

        for (int gi = 0; gi < config.grids; ++gi) {
            const std::uint64_t seed = config.baseSeed + static_cast<std::uint64_t>(gi);  // no overflow: validated
            ScenarioInstance inst = generateScenario(scenario, seed);
            if (!inst.ok) {
                status.generationErrors.push_back("grid " + std::to_string(gi) + " seed " + std::to_string(seed) +
                                                  ": " + inst.error);
                ++result.generationFailures;
                if (cb.progress) *cb.progress << "[" << id << "] grid " << (gi + 1) << "/" << config.grids
                                              << " seed=" << seed << " GENERATION FAILED: " << inst.error << '\n';
                continue;
            }
            ++status.gridsGenerated;
            const Grid& grid = inst.grid;  // the one grid every algorithm and run uses
            const std::uint64_t hash = grid.fingerprint();
            const ReferenceSolution reference = computeReference(grid);  // independent, untimed

            if (cb.progress) {
                *cb.progress << "[" << id << "] grid " << (gi + 1) << "/" << config.grids << " seed=" << seed
                             << " reachable=" << (reference.reachable ? "yes" : "no")
                             << " placement_attempts=" << inst.placementAttempts << '\n';
            }

            for (int w = 0; w < config.warmup; ++w) {
                for (Algorithm a : config.algorithms) {
                    try {
                        (void)runSearch(a, grid);
                    } catch (const std::exception& e) {
                        result.failures.push_back(id + " seed=" + std::to_string(seed) + " " + algorithmName(a) +
                                                  " warm-up: exception: " + e.what());
                    }
                }
            }

            std::vector<Accumulator> acc(numAlgorithms);
            for (int run = 0; run < config.runs; ++run) {
                for (std::size_t k = 0; k < numAlgorithms; ++k) {
                    const std::size_t ai = (k + static_cast<std::size_t>(run)) % numAlgorithms;
                    const Algorithm algorithm = config.algorithms[ai];

                    RunRecord rec;
                    rec.scenarioId = id;
                    rec.scenario = scenario;
                    rec.algorithm = algorithm;
                    rec.seed = seed;
                    rec.gridIndex = gi;
                    rec.runIndex = run;
                    rec.orderPosition = static_cast<int>(k);
                    rec.gridHash = hash;
                    rec.placementAttempts = inst.placementAttempts;
                    rec.start = grid.getStartPos();
                    rec.end = grid.getEndPos();
                    rec.reference = reference;

                    std::vector<std::string> problems;
                    try {
                        const SearchResult sr = runSearch(algorithm, grid);  // the only timed region
                        rec.metrics = sr.metrics;
                        problems = validateRun(grid, reference, algorithm, sr);  // untimed
                    } catch (const std::exception& e) {
                        problems.push_back(std::string("exception: ") + e.what());
                    } catch (...) {
                        problems.push_back("unknown exception");
                    }
                    Accumulator& a = acc[ai];
                    if (problems.empty() && a.reference && !sameNonTimeMetrics(*a.reference, rec.metrics)) {
                        problems.push_back("non-deterministic metrics (differ from the first valid run)");
                    }
                    ++a.runs;
                    rec.valid = problems.empty();
                    if (rec.valid) {
                        ++a.valid;
                        if (!a.reference) a.reference = rec.metrics;
                        a.search.push_back(rec.metrics.searchTimeMs);
                        a.endToEnd.push_back(rec.metrics.executionTimeMs);
                    } else {
                        ++result.invalidRuns;
                        rec.failure = join(problems, "; ");
                        result.failures.push_back(id + " seed=" + std::to_string(seed) + " " + algorithmName(algorithm) +
                                                  " run=" + std::to_string(run) + ": " + rec.failure);
                    }
                    if (cb.onRun) cb.onRun(rec);
                }
            }

            if (grid.fingerprint() != hash) {
                result.failures.push_back(id + " seed=" + std::to_string(seed) + ": grid changed during the runs");
            }

            for (std::size_t ai = 0; ai < numAlgorithms; ++ai) {
                const Accumulator& a = acc[ai];
                GridSummary g;
                g.scenarioId = id;
                g.scenario = scenario;
                g.algorithm = config.algorithms[ai];
                g.gridIndex = gi;
                g.seed = seed;
                g.gridHash = hash;
                g.placementAttempts = inst.placementAttempts;
                g.start = grid.getStartPos();
                g.end = grid.getEndPos();
                g.reference = reference;
                g.runs = a.runs;
                g.validRuns = a.valid;
                if (a.valid > 0) {
                    g.metrics = a.reference;
                    g.searchMs = computeStats(a.search);
                    g.endToEndMs = computeStats(a.endToEnd);
                    if (a.valid >= 2 && g.searchMs->mean > 0) g.searchCv = g.searchMs->stddev / g.searchMs->mean;
                    if (g.metrics->pathFound && reference.reachable && reference.minimumCost > 0) {
                        g.costRatio = static_cast<double>(g.metrics->pathCost) / static_cast<double>(reference.minimumCost);
                    }
                    g.nearTimerFloor = config.timerFloorMs > 0 && g.searchMs->median < config.timerFloorMs;
                }
                result.grids.push_back(std::move(g));
            }
            if (cb.onGridDone) cb.onGridDone();
        }
        result.scenarios.push_back(std::move(status));
    }
    return result;
}

// ------------------------------------------------------------- aggregation

std::vector<Aggregate> aggregate(const std::vector<GridSummary>& grids, const std::vector<ScenarioStatus>& scenarios) {
    std::vector<std::pair<std::string, Algorithm>> order;
    std::map<std::pair<std::string, int>, std::vector<const GridSummary*>> groups;
    for (const GridSummary& g : grids) {
        auto& v = groups[{g.scenarioId, static_cast<int>(g.algorithm)}];
        if (v.empty()) order.emplace_back(g.scenarioId, g.algorithm);
        v.push_back(&g);
    }
    std::map<std::string, const ScenarioStatus*> statusById;
    for (const ScenarioStatus& s : scenarios) statusById[s.scenarioId] = &s;

    std::vector<Aggregate> out;
    out.reserve(order.size());
    for (const auto& [sid, algorithm] : order) {
        const auto& rows = groups[{sid, static_cast<int>(algorithm)}];
        Aggregate a;
        a.scenarioId = sid;
        a.scenario = rows.front()->scenario;
        a.algorithm = algorithm;
        if (auto it = statusById.find(sid); it != statusById.end()) {
            a.gridsRequested = it->second->gridsRequested;
            a.gridsGenerated = it->second->gridsGenerated;
        } else {
            std::set<int> distinct;
            for (const GridSummary* g : rows) distinct.insert(g->gridIndex);
            a.gridsRequested = a.gridsGenerated = static_cast<int>(distinct.size());
        }

        std::vector<double> search, e2e, searchSolvable, e2eSolvable, cvs, ratios, throughput;
        double sumExp = 0, sumDisc = 0, sumPeak = 0, sumExplore = 0, sumLen = 0;
        int explored = 0;
        for (const GridSummary* g : rows) {
            a.runsPerGrid = std::max(a.runsPerGrid, g->runs);
            a.invalidRuns += g->runs - g->validRuns;
            if (g->validRuns == 0 || !g->metrics) continue;
            const SearchMetrics& m = *g->metrics;
            ++a.gridsMeasured;
            if (g->reference.reachable) ++a.solvableGrids;
            if (m.pathFound) {
                ++a.gridsSolved;
                sumLen += m.pathLength;
            }
            search.push_back(g->searchMs->median);
            e2e.push_back(g->endToEndMs->median);
            if (g->reference.reachable) {
                searchSolvable.push_back(g->searchMs->median);
                e2eSolvable.push_back(g->endToEndMs->median);
            }
            if (g->searchCv) cvs.push_back(*g->searchCv);
            if (g->costRatio) ratios.push_back(*g->costRatio);
            if (g->nearTimerFloor) ++a.gridsNearTimerFloor;
            else if (m.nodesExpanded > 0 && g->searchMs->median > 0) throughput.push_back(m.nodesExpanded / g->searchMs->median);
            sumExp += m.nodesExpanded;
            sumDisc += m.nodesDiscovered;
            sumPeak += m.peakFrontierSize;
            if (g->reference.passableCells > 0) {
                sumExplore += static_cast<double>(m.nodesExpanded) / g->reference.passableCells;
                ++explored;
            }
        }
        if (a.gridsMeasured > 0) {
            a.solvabilityRate = static_cast<double>(a.solvableGrids) / a.gridsMeasured;
            a.successRate = static_cast<double>(a.gridsSolved) / a.gridsMeasured;
        }
        a.searchMs = distributionOf(search);
        a.endToEndMs = distributionOf(e2e);
        a.searchMsSolvable = distributionOf(searchSolvable);
        a.endToEndMsSolvable = distributionOf(e2eSolvable);
        if (auto d = distributionOf(cvs)) a.medianSearchCv = d->median;
        if (auto d = distributionOf(throughput)) a.medianExpansionsPerSearchMs = d->median;
        a.meanNodesExpanded = meanOf(sumExp, a.gridsMeasured);
        a.meanNodesDiscovered = meanOf(sumDisc, a.gridsMeasured);
        a.meanPeakFrontier = meanOf(sumPeak, a.gridsMeasured);
        a.meanExplorationRatio = meanOf(sumExplore, explored);
        a.meanPathLength = meanOf(sumLen, a.gridsSolved);
        a.costRatioGrids = static_cast<int>(ratios.size());
        if (auto d = distributionOf(ratios)) a.meanCostRatio = d->mean;
        out.push_back(std::move(a));
    }
    return out;
}

const char* timingKindName(TimingKind k) { return k == TimingKind::Search ? "search" : "end_to_end"; }

std::vector<PairedComparison> pairedComparisons(const std::vector<GridSummary>& grids) {
    // scenario -> (algorithms in first-seen order, grid indices, lookup)
    struct ScenarioData {
        Scenario scenario;
        std::vector<Algorithm> algorithms;
        std::set<int> gridIndices;
        std::map<std::pair<int, int>, const GridSummary*> byGridAlgorithm;
    };
    std::vector<std::string> order;
    std::map<std::string, ScenarioData> data;
    for (const GridSummary& g : grids) {
        auto [it, inserted] = data.try_emplace(g.scenarioId);
        if (inserted) {
            order.push_back(g.scenarioId);
            it->second.scenario = g.scenario;
        }
        ScenarioData& d = it->second;
        if (std::find(d.algorithms.begin(), d.algorithms.end(), g.algorithm) == d.algorithms.end()) {
            d.algorithms.push_back(g.algorithm);
        }
        d.gridIndices.insert(g.gridIndex);
        d.byGridAlgorithm[{g.gridIndex, static_cast<int>(g.algorithm)}] = &g;
    }

    std::vector<PairedComparison> out;
    for (const std::string& sid : order) {
        const ScenarioData& d = data[sid];
        for (std::size_t i = 0; i < d.algorithms.size(); ++i) {
            for (std::size_t j = i + 1; j < d.algorithms.size(); ++j) {
                for (TimingKind kind : {TimingKind::Search, TimingKind::EndToEnd}) {
                    PairedComparison pc;
                    pc.scenarioId = sid;
                    pc.scenario = d.scenario;
                    pc.a = d.algorithms[i];
                    pc.b = d.algorithms[j];
                    pc.timing = kind;
                    std::vector<double> ratios;
                    double sumLog = 0;
                    int aFaster = 0;
                    for (int gi : d.gridIndices) {
                        auto ia = d.byGridAlgorithm.find({gi, static_cast<int>(pc.a)});
                        auto ib = d.byGridAlgorithm.find({gi, static_cast<int>(pc.b)});
                        const GridSummary* ga = ia == d.byGridAlgorithm.end() ? nullptr : ia->second;
                        const GridSummary* gb = ib == d.byGridAlgorithm.end() ? nullptr : ib->second;
                        auto median = [kind](const GridSummary* g) -> std::optional<double> {
                            const auto& st = kind == TimingKind::Search ? g->searchMs : g->endToEndMs;
                            if (!st) return std::nullopt;
                            return st->median;
                        };
                        std::optional<double> ta = ga ? median(ga) : std::nullopt;
                        std::optional<double> tb = gb ? median(gb) : std::nullopt;
                        const bool sameGrid = ga && gb && ga->gridHash == gb->gridHash;
                        if (!sameGrid || !ta || !tb || *ta <= 0 || *tb <= 0) {
                            ++pc.gridsExcluded;
                            continue;
                        }
                        const double r = *ta / *tb;
                        ratios.push_back(r);
                        sumLog += std::log(r);
                        if (r < 1.0) ++aFaster;
                    }
                    pc.gridsCompared = static_cast<int>(ratios.size());
                    pc.ratio = distributionOf(ratios);
                    if (pc.gridsCompared > 0) {
                        pc.geomeanRatio = std::exp(sumLog / pc.gridsCompared);
                        pc.aFasterFraction = static_cast<double>(aFaster) / pc.gridsCompared;
                    }
                    out.push_back(std::move(pc));
                }
            }
        }
    }
    return out;
}
