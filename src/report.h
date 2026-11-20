// Reporting layer: CSV export and human-readable summaries of benchmark
// results. Formatting only; no search or benchmark logic.
//
// CSV conventions (all four files): RFC 4180 quoting, C-locale numbers,
// one header line, no comment lines. Values that do not apply (e.g. path
// length when no path exists, a ratio that is undefined) are EMPTY fields,
// never sentinel numbers. Hashes are "0x" + 16 hex digits. Densities are
// round-trip decimal fractions.
#pragma once

#include <ostream>
#include <string>
#include <vector>

#include "benchmark.h"

// Quotes a CSV field per RFC 4180 when it contains a comma, quote, CR or LF.
std::string csvEscape(const std::string& field);
// "0x" followed by exactly 16 lowercase hex digits.
std::string formatHash(std::uint64_t hash);

// Per-run CSV, written incrementally: header once, then one row per run.
void writeRunsCsvHeader(std::ostream& out);
void writeRunsCsvRow(std::ostream& out, const RunRecord& record);
// Convenience: header + all rows.
void writeRunsCsv(std::ostream& out, const std::vector<RunRecord>& records);

// One row per (scenario, grid, algorithm): level-1 statistics.
void writeGridCsv(std::ostream& out, const std::vector<GridSummary>& grids);
// One row per (scenario, algorithm): level-2 statistics across grids.
void writeSummaryCsv(std::ostream& out, const std::vector<Aggregate>& aggregates);
// One row per (scenario, algorithm pair, timing kind).
void writePairedCsv(std::ostream& out, const std::vector<PairedComparison>& pairs);

// Terminal summary: per scenario, the level-2 table and the paired core-time
// ratios. `timerFloorMs` > 0 adds a note for grids near the timer floor.
void printSummaryTable(std::ostream& out, const std::vector<Aggregate>& aggregates,
                       const std::vector<PairedComparison>& pairs, double timerFloorMs);

// Build description, e.g. "Release (NDEBUG), GCC 15.2.0".
std::string buildDescription();
