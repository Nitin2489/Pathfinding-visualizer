// Search algorithm layer: Dijkstra, A*, BFS and DFS over a Grid.
//
// These functions perform no I/O, rendering, sleeping or file access. They
// are shared by the interactive visualizer (which replays the optional
// expansion trace *after* the search has finished) and the headless
// benchmark engine.
//
// Movement model: 4-connected (up, right, down, left). Moving into a cell
// costs getTerrainCost() of that cell (1 for EMPTY). Walls are impassable.
#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "grid.h"

enum class Algorithm { Dijkstra, AStar, BFS, DFS };

// Metric definitions (identical for all four algorithms):
//
// pathFound        A path from start to end was found.
// nodesDiscovered  Unique cells ever inserted into the frontier, start
//                  included. Re-insertions/duplicate entries count once.
// nodesExpanded    Unique cells whose neighbors were examined. Stale
//                  priority-queue/stack entries are skipped and never
//                  counted. The goal is *not* counted: the search stops as
//                  soon as the goal is removed from the frontier.
// pathLength       Number of edges (moves) on the returned path; 0 when
//                  start == end; -1 when no path was found.
// pathCost         Sum of the costs of entering each cell after the start
//                  (equals pathLength on terrain-free grids); -1 if no path.
// peakFrontierSize Maximum number of entries held in the frontier container
//                  (queue, stack or heap) at any moment, *including* stale
//                  duplicate entries, i.e. it reflects real memory use.
// executionTimeMs  END-TO-END time (std::chrono::steady_clock) of one call:
//                  allocating and initialising the O(cells) per-cell arrays
//                  (distance/parent/visited flags), the search, path
//                  reconstruction, and freeing those arrays.
// searchTimeMs     CORE time: the same call measured from the moment the
//                  per-cell arrays are allocated and initialised until the
//                  path is reconstructed. Frontier containers (heap, queue,
//                  stack) start empty and grow during the search, so their
//                  allocations are part of the core time. searchTimeMs <=
//                  executionTimeMs; both come from the same execution.
//
// Neither includes rendering, sleeping, grid generation, validation or
// reporting, which all happen outside these functions. When an expansion
// trace is requested, appending to it is included in both (one push_back
// per expansion); the benchmark never requests a trace.
struct SearchMetrics {
    bool pathFound = false;
    int nodesDiscovered = 0;
    int nodesExpanded = 0;
    int pathLength = -1;
    long long pathCost = -1;
    int peakFrontierSize = 0;
    double executionTimeMs = 0.0;  // end-to-end
    double searchTimeMs = 0.0;     // core search only
};

struct SearchResult {
    SearchMetrics metrics;
    std::vector<Cell> path;  // start ... end, empty if no path
};

// Optional per-search output used by the visualizer: cells in the order in
// which they were expanded (each cell at most once).
using ExpansionTrace = std::vector<Cell>;

// Uniform-cost search. Optimal for any non-negative terrain costs.
SearchResult dijkstraSearch(const Grid& grid, ExpansionTrace* trace = nullptr);
// A* with the Manhattan heuristic (admissible and consistent because every
// move costs >= 1). Optimal for the same cost model as Dijkstra.
SearchResult astarSearch(const Grid& grid, ExpansionTrace* trace = nullptr);
// Breadth-first search. Ignores terrain cost: optimal in number of moves,
// not in pathCost when terrain is present.
SearchResult bfsSearch(const Grid& grid, ExpansionTrace* trace = nullptr);
// Depth-first search (cell marked when popped, neighbors explored in
// up/right/down/left order). Finds *a* path if one exists; not optimal.
//
// The visit order and parent links equal those of recursive DFS. The price
// is that a cell can be pushed more than once (once per expanded neighbor)
// before it is popped; later copies are stale and skipped, and they *are*
// counted in peakFrontierSize. Marking cells on insertion instead would
// remove the duplicates but is no longer DFS: a cell keeps the parent that
// first saw it, so e.g. on rows "S." / "E." it returns S->E (1 move) where
// DFS returns S->right->down->E (3 moves). An explicit iterator stack would
// keep DFS order without duplicates, but holds the whole current path and
// measured larger peaks on the benchmark grids, so it was not adopted.
SearchResult dfsSearch(const Grid& grid, ExpansionTrace* trace = nullptr);

SearchResult runSearch(Algorithm algorithm, const Grid& grid, ExpansionTrace* trace = nullptr);

// "dijkstra", "astar", "bfs", "dfs".
const char* algorithmName(Algorithm algorithm);
// Human readable, e.g. "A* Search".
const char* algorithmDisplayName(Algorithm algorithm);
// Accepts the names above (case-insensitive) plus "a*" and "a-star".
bool parseAlgorithm(const std::string& text, Algorithm& out);
const std::vector<Algorithm>& allAlgorithms();
// True for algorithms that guarantee a minimum-cost path under the
// terrain cost model (Dijkstra, A*).
bool isCostOptimal(Algorithm algorithm);

// Checks that `path` starts at start, ends at end, consists of 4-adjacent
// in-bounds passable cells and visits no cell twice. On failure returns
// false and, if `error` is non-null, describes the problem.
bool validatePath(const Grid& grid, const std::vector<Cell>& path, std::string* error = nullptr);
// Sum of entry costs along the path (0 for a single-cell path). Returns -1
// if the path is empty or contains an out-of-bounds or wall cell. Does not
// check adjacency; use validatePath() for that.
long long computePathCost(const Grid& grid, const std::vector<Cell>& path);
