// Independent correctness oracle for the benchmark.
//
// Nothing here calls into search.h/search.cpp. The reference solvers are
// written differently on purpose (row/column BFS over a std::deque, and a
// Dial bucket queue for weighted costs) and use their own terrain-cost
// table, so a defect in the production search code cannot hide by
// agreeing with itself. Only the Grid's raw contents and endpoints are
// shared.
//
// The visualizer never runs this code; it is benchmark/test-only.
#pragma once

#include <string>
#include <vector>

#include "grid.h"
#include "search.h"

// Reference facts about one grid, computed independently of the
// production search.
struct ReferenceSolution {
    bool reachable = false;       // end reachable from start
    int shortestMoves = -1;       // fewest moves start -> end, -1 if unreachable
    long long minimumCost = -1;   // least total entry cost, -1 if unreachable
    int startComponentSize = 0;   // passable cells reachable from start (incl. start)
    int passableCells = 0;        // all non-wall cells
};

// Entry cost of a cell type, from a table kept separate from
// getTerrainCost(). Returns -1 for walls and unknown contents.
int referenceEntryCost(char cell);

ReferenceSolution computeReference(const Grid& grid);

// Connected-component label per cell (row-major; -1 for walls), using
// 4-connectivity. Used by reachable endpoint placement.
std::vector<int> labelComponents(const Grid& grid);

// Checks one search result against the reference. Returns an empty vector
// when everything holds; otherwise one message per violated check:
//  * path: starts at start, ends at end, in bounds, no walls, 4-adjacent
//    steps, no repeated cell;
//  * pathLength / pathCost equal the values recomputed from the path;
//  * pathFound equals independent reachability;
//  * Dijkstra / A*: pathCost == reference minimum cost (weighted optimality);
//  * BFS: pathLength == reference fewest moves (move optimality only);
//  * every algorithm: pathLength >= fewest moves, pathCost >= minimum cost;
//  * metric invariants: 1 <= nodesDiscovered <= startComponentSize,
//    nodesExpanded <= nodesDiscovered, every path cell except the goal was
//    expanded, peak frontier within [1, 1 + 4 * nodesExpanded], BFS frontier
//    never exceeds nodesDiscovered (BFS holds no duplicates), and when no
//    path exists the whole start component was discovered and expanded;
//  * times are finite, non-negative and searchTimeMs <= executionTimeMs.
std::vector<std::string> validateRun(const Grid& grid, const ReferenceSolution& reference, Algorithm algorithm,
                                     const SearchResult& result);
