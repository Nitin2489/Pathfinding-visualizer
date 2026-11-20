// Visualization layer: ANSI console rendering, animation and the
// interactive menu. Searching is delegated to search.h; animation replays
// the recorded expansion trace after the search has completed, so
// rendering and sleeping are never part of the measured search time.
#pragma once

#include <cstdint>
#include <ostream>
#include <vector>

#include "grid.h"
#include "search.h"

// Per-cell overlay drawn on top of the grid contents.
constexpr char VISITED = '.';
constexpr char PATH = '*';

// Renders the grid (with an optional overlay of VISITED/PATH marks, one
// char per cell, 0 = none) to `out`, preceded by a clear-screen sequence.
void renderGrid(std::ostream& out, const Grid& grid, const std::vector<char>* overlay = nullptr);

// Interactive entry points (signatures kept from v1). Each runs the shared
// search core, animates the exploration and path, prints the metrics and
// returns whether a path was found.
bool dijkstra(Grid& grid, int animationSpeed = 50);
bool astar(Grid& grid, int animationSpeed = 50);
bool bfs(Grid& grid, int animationSpeed = 50);
bool dfs(Grid& grid, int animationSpeed = 50);

// Runs the interactive menu loop on stdin/stdout. Returns the exit code.
int runInteractive(std::uint64_t seed);
