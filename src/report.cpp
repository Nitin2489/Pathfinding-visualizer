#include "report.h"

#include <cstdio>
#include <iomanip>
#include <locale>
#include <sstream>

#ifndef PATHFINDING_BUILD_TYPE
#define PATHFINDING_BUILD_TYPE "unknown"
#endif

namespace {

std::string fixed(double v, int precision) {
    std::ostringstream s;
    s.imbue(std::locale::classic());
    s << std::fixed << std::setprecision(precision) << v;
    return s.str();
}

std::string opt(const std::optional<double>& v, int precision) { return v ? fixed(*v, precision) : std::string(); }

// Integer metric that is "not applicable" when negative (search uses -1).
std::string nonNegative(long long v) { return v < 0 ? std::string() : std::to_string(v); }

const char* boolField(bool b) { return b ? "1" : "0"; }

// Common leading columns describing the scenario.
void scenarioColumns(std::ostream& out, const std::string& id, const Scenario& s) {
    out << csvEscape(id) << ',' << scenarioTypeName(s.type) << ',' << s.rows << ',' << s.cols << ','
        << formatRoundTrip(s.obstacleDensity) << ',' << formatRoundTrip(s.terrainDensity) << ','
        << placementName(s.placement);
}
const char* kScenarioHeader = "scenario_id,scenario_type,rows,columns,obstacle_density,terrain_density,placement";

void distributionColumns(std::ostream& out, const std::optional<Distribution>& d) {
    if (d) {
        out << ',' << fixed(d->median, 6) << ',' << fixed(d->mean, 6) << ',' << fixed(d->q1, 6) << ','
            << fixed(d->q3, 6) << ',' << fixed(d->iqr(), 6);
    } else {
        out << ",,,,,";
    }
}

std::string distributionHeader(const std::string& name) {
    return "," + name + "_median," + name + "_mean," + name + "_q1," + name + "_q3," + name + "_iqr";
}

}  // namespace

std::string csvEscape(const std::string& field) {
    if (field.find_first_of(",\"\r\n") == std::string::npos) return field;
    std::string quoted = "\"";
    for (char ch : field) {
        if (ch == '"') quoted += '"';
        quoted += ch;
    }
    quoted += '"';
    return quoted;
}

std::string formatHash(std::uint64_t hash) {
    char buf[24];
    std::snprintf(buf, sizeof buf, "0x%016llx", static_cast<unsigned long long>(hash));
    return buf;
}

void writeRunsCsvHeader(std::ostream& out) {
    out << kScenarioHeader
        << ",algorithm,grid_index,seed,grid_hash,placement_attempts,start_row,start_col,end_row,end_col,"
           "passable_cells,reachable,run_index,order_position,path_found,nodes_discovered,nodes_expanded,"
           "path_length,path_cost,reference_min_cost,reference_min_moves,peak_frontier_entries,search_time_ms,"
           "end_to_end_time_ms,checks_passed,failure\n";
}

void writeRunsCsvRow(std::ostream& out, const RunRecord& r) {
    const SearchMetrics& m = r.metrics;
    scenarioColumns(out, r.scenarioId, r.scenario);
    out << ',' << algorithmName(r.algorithm) << ',' << r.gridIndex << ',' << r.seed << ',' << formatHash(r.gridHash)
        << ',' << r.placementAttempts << ',' << r.start.row << ',' << r.start.col << ',' << r.end.row << ','
        << r.end.col << ',' << r.reference.passableCells << ',' << boolField(r.reference.reachable) << ','
        << r.runIndex << ',' << r.orderPosition << ',' << boolField(m.pathFound) << ',' << m.nodesDiscovered << ','
        << m.nodesExpanded << ',' << nonNegative(m.pathFound ? m.pathLength : -1) << ','
        << nonNegative(m.pathFound ? m.pathCost : -1) << ',' << nonNegative(r.reference.minimumCost) << ','
        << nonNegative(r.reference.shortestMoves) << ',' << m.peakFrontierSize << ','
        << fixed(m.searchTimeMs, 6) << ',' << fixed(m.executionTimeMs, 6) << ',' << boolField(r.valid) << ','
        << csvEscape(r.failure) << '\n';
}

void writeRunsCsv(std::ostream& out, const std::vector<RunRecord>& records) {
    writeRunsCsvHeader(out);
    for (const RunRecord& r : records) writeRunsCsvRow(out, r);
}

void writeGridCsv(std::ostream& out, const std::vector<GridSummary>& grids) {
    out << kScenarioHeader
        << ",algorithm,grid_index,seed,grid_hash,placement_attempts,start_row,start_col,end_row,end_col,"
           "passable_cells,reachable,reference_min_cost,reference_min_moves,runs,valid_runs,path_found,"
           "nodes_discovered,nodes_expanded,path_length,path_cost,cost_ratio,peak_frontier_entries,"
           "search_ms_median,search_ms_mean,search_ms_stddev,search_ms_min,search_ms_max,search_cv,"
           "end_to_end_ms_median,end_to_end_ms_mean,end_to_end_ms_stddev,end_to_end_ms_min,end_to_end_ms_max,"
           "near_timer_floor\n";
    for (const GridSummary& g : grids) {
        scenarioColumns(out, g.scenarioId, g.scenario);
        out << ',' << algorithmName(g.algorithm) << ',' << g.gridIndex << ',' << g.seed << ','
            << formatHash(g.gridHash) << ',' << g.placementAttempts << ',' << g.start.row << ',' << g.start.col
            << ',' << g.end.row << ',' << g.end.col << ',' << g.reference.passableCells << ','
            << boolField(g.reference.reachable) << ',' << nonNegative(g.reference.minimumCost) << ','
            << nonNegative(g.reference.shortestMoves) << ',' << g.runs << ',' << g.validRuns << ',';
        if (g.metrics) {
            const SearchMetrics& m = *g.metrics;
            out << boolField(m.pathFound) << ',' << m.nodesDiscovered << ',' << m.nodesExpanded << ','
                << nonNegative(m.pathFound ? m.pathLength : -1) << ',' << nonNegative(m.pathFound ? m.pathCost : -1)
                << ',' << opt(g.costRatio, 6) << ',' << m.peakFrontierSize;
        } else {
            out << ",,,,,,";
        }
        for (const auto* st : {&g.searchMs, &g.endToEndMs}) {
            if (*st) {
                out << ',' << fixed((*st)->median, 6) << ',' << fixed((*st)->mean, 6) << ','
                    << fixed((*st)->stddev, 6) << ',' << fixed((*st)->min, 6) << ',' << fixed((*st)->max, 6);
            } else {
                out << ",,,,,";
            }
            if (st == &g.searchMs) out << ',' << opt(g.searchCv, 4);
        }
        out << ',' << boolField(g.nearTimerFloor) << '\n';
    }
}

void writeSummaryCsv(std::ostream& out, const std::vector<Aggregate>& aggregates) {
    out << kScenarioHeader
        << ",algorithm,grids_requested,grids_generated,grids_measured,solvable_grids,grids_solved,runs_per_grid,"
           "invalid_runs,solvability_rate,success_rate"
        << distributionHeader("search_ms") << distributionHeader("end_to_end_ms")
        << distributionHeader("search_ms_solvable") << distributionHeader("end_to_end_ms_solvable")
        << ",median_search_cv,grids_near_timer_floor,mean_nodes_expanded,mean_nodes_discovered,"
           "mean_peak_frontier_entries,mean_exploration_ratio,mean_path_length,mean_cost_ratio,cost_ratio_grids,"
           "median_expansions_per_search_ms\n";
    for (const Aggregate& a : aggregates) {
        scenarioColumns(out, a.scenarioId, a.scenario);
        out << ',' << algorithmName(a.algorithm) << ',' << a.gridsRequested << ',' << a.gridsGenerated << ','
            << a.gridsMeasured << ',' << a.solvableGrids << ',' << a.gridsSolved << ',' << a.runsPerGrid << ','
            << a.invalidRuns << ',' << opt(a.solvabilityRate, 4) << ',' << opt(a.successRate, 4);
        distributionColumns(out, a.searchMs);
        distributionColumns(out, a.endToEndMs);
        distributionColumns(out, a.searchMsSolvable);
        distributionColumns(out, a.endToEndMsSolvable);
        out << ',' << opt(a.medianSearchCv, 4) << ',' << a.gridsNearTimerFloor << ',' << opt(a.meanNodesExpanded, 2)
            << ',' << opt(a.meanNodesDiscovered, 2) << ',' << opt(a.meanPeakFrontier, 2) << ','
            << opt(a.meanExplorationRatio, 6) << ',' << opt(a.meanPathLength, 2) << ',' << opt(a.meanCostRatio, 6)
            << ',' << a.costRatioGrids << ',' << opt(a.medianExpansionsPerSearchMs, 1) << '\n';
    }
}

void writePairedCsv(std::ostream& out, const std::vector<PairedComparison>& pairs) {
    out << kScenarioHeader
        << ",algorithm_a,algorithm_b,timing,grids_compared,grids_excluded,ratio_a_over_b_median,"
           "ratio_a_over_b_mean,ratio_a_over_b_q1,ratio_a_over_b_q3,ratio_a_over_b_iqr,ratio_a_over_b_geomean,"
           "a_faster_fraction\n";
    for (const PairedComparison& p : pairs) {
        scenarioColumns(out, p.scenarioId, p.scenario);
        out << ',' << algorithmName(p.a) << ',' << algorithmName(p.b) << ',' << timingKindName(p.timing) << ','
            << p.gridsCompared << ',' << p.gridsExcluded;
        distributionColumns(out, p.ratio);
        out << ',' << opt(p.geomeanRatio, 6) << ',' << opt(p.aFasterFraction, 4) << '\n';
    }
}

void printSummaryTable(std::ostream& out, const std::vector<Aggregate>& aggregates,
                       const std::vector<PairedComparison>& pairs, double timerFloorMs) {
    auto cell = [](const std::optional<double>& v, int precision) { return v ? fixed(*v, precision) : std::string("-"); };
    bool anyNearFloor = false;
    std::string current;
    for (std::size_t i = 0; i < aggregates.size(); ++i) {
        const Aggregate& a = aggregates[i];
        if (a.scenarioId != current) {
            current = a.scenarioId;
            out << "\nScenario " << a.scenarioId << "\n  grids: " << a.gridsGenerated << "/" << a.gridsRequested
                << " generated, " << a.solvableGrids << "/" << a.gridsMeasured << " solvable; " << a.runsPerGrid
                << " timed runs per algorithm per grid\n";
            out << "  Times are per-grid medians summarised across grids (ms). Counts are means per grid.\n";
            out << "  " << std::left << std::setw(10) << "algorithm" << std::right << std::setw(9) << "solved"
                << std::setw(14) << "core median" << std::setw(12) << "core IQR" << std::setw(13) << "e2e median"
                << std::setw(10) << "noise CV" << std::setw(12) << "expanded" << std::setw(10) << "path len"
                << std::setw(11) << "cost ratio" << std::setw(8) << "checks" << '\n';
        }
        const bool nearFloor = a.gridsNearTimerFloor > 0;
        anyNearFloor = anyNearFloor || nearFloor;
        out << "  " << std::left << std::setw(10) << algorithmName(a.algorithm) << std::right << std::setw(8)
            << (a.successRate ? fixed(*a.successRate * 100.0, 0) + "%" : std::string("-")) << ' ' << std::setw(13)
            << (a.searchMs ? fixed(a.searchMs->median, 4) + (nearFloor ? "*" : "") : std::string("-"))
            << std::setw(12) << (a.searchMs ? fixed(a.searchMs->iqr(), 4) : std::string("-")) << std::setw(13)
            << (a.endToEndMs ? fixed(a.endToEndMs->median, 4) : std::string("-")) << std::setw(10)
            << cell(a.medianSearchCv, 3) << std::setw(12) << cell(a.meanNodesExpanded, 1) << std::setw(10)
            << cell(a.meanPathLength, 1) << std::setw(11) << cell(a.meanCostRatio, 3) << std::setw(8)
            << (a.invalidRuns == 0 ? "ok" : "FAIL") << '\n';

        const bool lastOfScenario = i + 1 == aggregates.size() || aggregates[i + 1].scenarioId != current;
        if (!lastOfScenario) continue;
        bool header = false;
        for (const PairedComparison& p : pairs) {
            if (p.scenarioId != current || p.timing != TimingKind::Search) continue;
            if (!header) {
                out << "  Paired core-time ratio a/b on the same grids (< 1: a faster):\n";
                header = true;
            }
            out << "    " << std::left << std::setw(18)
                << (std::string(algorithmName(p.a)) + "/" + algorithmName(p.b)) << std::right;
            if (p.ratio) {
                out << "median " << fixed(p.ratio->median, 3) << "  IQR " << fixed(p.ratio->q1, 3) << "-"
                    << fixed(p.ratio->q3, 3) << "  geomean " << fixed(*p.geomeanRatio, 3) << "  " << algorithmName(p.a)
                    << " faster on " << static_cast<int>(*p.aFasterFraction * p.gridsCompared + 0.5) << "/"
                    << p.gridsCompared << " grids";
            } else {
                out << "no comparable grids";
            }
            if (p.gridsExcluded > 0) out << " (" << p.gridsExcluded << " excluded)";
            out << '\n';
        }
    }
    if (anyNearFloor && timerFloorMs > 0) {
        out << "\n* some grids have a median core time below " << fixed(timerFloorMs * 1000.0, 2)
            << " us (100x the measured timer granularity); treat those values as approximate.\n";
    }
}

std::string buildDescription() {
    std::ostringstream s;
    s << PATHFINDING_BUILD_TYPE;
#ifdef NDEBUG
    s << " (NDEBUG)";
#else
    s << " (assertions on; timings are not representative of an optimized build)";
#endif
#if defined(__clang__)
    s << ", Clang " << __clang_major__ << '.' << __clang_minor__ << '.' << __clang_patchlevel__;
#elif defined(__GNUC__)
    s << ", GCC " << __GNUC__ << '.' << __GNUC_MINOR__ << '.' << __GNUC_PATCHLEVEL__;
#elif defined(_MSC_VER)
    s << ", MSVC " << _MSC_VER;
#endif
    return s.str();
}
