# Changes from v1

Version 1 was a single `pathfinding.cpp` file. This page records the defects that version 2 fixed and the methodology added in v2.1. The original v1 screenshots are kept in [`images/v1/`](images/v1/).

- **Dijkstra and A\*:** the priority-queue comparator read the live `distance`/`f` arrays, which change after insertion and corrupt the heap order. There were no stale-entry checks, so cells were re-expanded and "nodes explored" was inflated. Fixed with priority snapshots and stale-entry skipping.
- **Terrain:** `SAND` and `START` were both `'S'`, terrain was never used in search costs, and visited marks overwrote cell contents. Terrain now has real costs used by Dijkstra and A\*; overlays are kept separate from the grid.
- **Maze generator:** it ignored wall and passage parity, so passages could be walled off and mazes could be disconnected. It also drew over existing walls. The maze now always clears the grid first and every open cell is connected (this is tested).
- **Randomness:** `rand()` with `time()` became seeded, platform-independent generation.
- **Timing:** it was integer milliseconds and included rendering and `sleep_for`. It now covers the search only: core and end-to-end times at sub-microsecond resolution ([methodology](benchmark-methodology.md#7-timing)).
- **Code removed:** the unused `smoothPath()`, which collapsed any path to two points, and the per-call vector allocation in `getNeighbors`.
- **CI:** it previously ran a non-existent `./PathfindingVisualizer` binary and no tests. It now builds and tests on three platforms.
- **Benchmark methodology (v2.1):**
  - reachable placement and independent RNG streams;
  - defaults of 10 grids × 12 runs;
  - two-level statistics and paired ratios;
  - an independent validation oracle;
  - separate core and end-to-end timing, plus timer calibration;
  - exact scenario IDs and empty-field CSV missing values;
  - staged output files with metadata sidecars.
