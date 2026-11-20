# Experimental plan

Written **before** the full experiment runs. Only the pilot below had been executed. Deviations made during execution are recorded at the end of [`results.md`](results.md).

## Framework facts this plan relies on

Verified in the source at commit `3d42fc8`.

| Fact | Where |
|---|---|
| Each interior cell is a wall independently with probability *d*. Cells (1,1) and (rows−2, cols−2) are never walls. | `Grid::generateRandomWalls` |
| Each free cell independently gets terrain with probability *p*, then sand (3), water (5) or mountain (10) with equal probability. Expected entry cost of a free cell is therefore 1 + 5*p*. | `Grid::generateRandomTerrain`, `getTerrainCost` |
| Walls use the seed, endpoints use stream 1, terrain uses stream 2. The same seed gives identical walls in every placement mode and terrain level, and terrain never moves the endpoints. | `generateScenario` |
| `reachable` uses rejection sampling of connected endpoint pairs (at most 10,000 draws). `random` and `corners` are unrestricted. | `generateScenario` |
| The A\* heuristic is unscaled Manhattan distance; it is admissible because the minimum step cost is 1. | `bestFirst` |
| Tie-breaking: Dijkstra uses (g, insertion order); A\* uses (f, h, insertion order). | `HeapGreater` |
| Core time starts after the per-cell arrays are allocated and initialised. End-to-end time covers the whole call, including freeing. | `search.cpp` |
| For each grid: one generation, one independent reference, `--warmup` untimed runs, then `--runs` interleaved timed runs with the algorithm order rotated by run index. | `runBenchmark` |
| Statistics are per-grid medians first, then distributions across grids. | `aggregate` |
| Invalid runs fail the process (exit 2). Generation failures give exit 3. | `main.cpp` |

## Pilot (exploratory, discarded)

Configuration: commit `3d42fc8`, Release, GCC 15.2, 20% walls unless noted.

| Workload | Wall time | Peak RSS |
|---|---|---|
| 100×100, 30% walls, 10 grids × 12 runs | 0.10 s | 5 MB |
| 400×400, 4 grids × 4 runs | 0.27 s | 9 MB |
| 800×800, 2 grids × 4 runs | 0.85 s | 19 MB |
| 1600×1600, 1 grid × 4 runs | 2.5 s | 46 MB |

- The median within-grid coefficient of variation (CV) of core time on 100×100 was 0.11–0.29, both pinned (`taskset -c 2`) and unpinned. The pilot cannot tell whether pinning helps.
- **Decision:** all experiments run pinned to logical CPU 2 for consistency. Experiment G measures pinned against unpinned explicitly.
- **Resource budget:** the full plan below is under 100k timed runs and a few minutes of compute, so no sample sizes need to be reduced.

## Common settings

| Setting | Value |
|---|---|
| Binary | Release build of commit `3d42fc8`, built from a clean worktree (`git_dirty_at_build=false`) |
| Pinning | `taskset -c 2`, except where noted |
| Seeds | base seed **1000** for every experiment, so grid *i* uses seed 1000 + *i*. Experiment G adds base seed **5000** as an independent replicate. |
| Warm-up | 1 untimed run per algorithm per grid |
| Runs per grid | 12 (a multiple of 4 and of 2, so run order is balanced), except where noted |
| Primary timing metric | per-grid **median core time** (`search_ms_median`). End-to-end time is reported alongside. |
| Unit of analysis | the **grid**; repeated runs are only noise samples |
| Recorded per run | the CSVs and `.meta.json`, plus the command, its exit code and its wall time in `raw/commands.log` |

## Experiments

### A. Baseline algorithm comparison (exploratory)

- **Workload:** 100×100, 30% walls, `reachable`, no terrain; 50 grids × 12 runs; all four algorithms.
- **Measures:** core and end-to-end time, nodes expanded and discovered, peak frontier entries, path length, path cost, success rate.
- **Controlled:** the same grids and endpoints for all algorithms (paired).

### B. Obstacle density (exploratory)

- **B1:** 100×100, `reachable`, densities {0, 0.10, 0.20, 0.30, 0.35, 0.40, 0.45, 0.50}; 50 grids × 12 runs; all algorithms.
- **B2:** the same densities with `corners` (mixed reachability); 50 grids × 4 runs. Used for success rate, and for search effort including unsolvable instances.
- **Independent variable:** wall density. For B1, the endpoint population also changes with density (see F); this confound is reported, not removed.

### C. Weighted terrain (exploratory)

- **Workload:** 100×100, 20% walls, `reachable`, terrain *p* ∈ {0, 0.10, 0.30, 0.50, 1.00}; 50 grids × 12 runs; all algorithms.
- **Design:** walls and endpoints are identical across the five terrain levels for each seed, so terrain is a **within-grid** factor.
- **Valid comparisons:**
  - Dijkstra and A\* both minimise cost, so their costs must be equal.
  - BFS minimises moves, not cost. Its cost ratio is reported as the price of ignoring weights, not as a ranking against cost-optimal algorithms.
  - DFS has no optimality objective.

### D. Grid scaling (exploratory)

- **D1 (reachable):** 20% walls, sizes 50², 100², 200² (50/50/30 grids × 12 runs), 400² (20 × 8), 800² (10 × 4), 1600² (5 × 4); all algorithms.
- **D2 (corners):** the same sizes, 20 grids × 4 runs (5 grids at 1600²), with solvable grids analysed separately. Corner-to-corner problems grow with the grid; reachable pairs have a random distance.
- **Analysis:** core time against grid cells and against nodes expanded (log-log), and time per expansion.
- **Fewer runs for large grids** is a declared resource choice: per-grid medians at large sizes are far above the timer floor.

### E. A\* against Dijkstra (CONFIRMATORY)

- **Workloads:** 200×200, 20% walls, `reachable`, terrain *p* ∈ {0, 0.30}; **100 grids × 12 runs**; algorithms `dijkstra,astar` only.
- **Pre-registered hypotheses**, for each terrain level separately:
  - **H1 (time):** the per-grid ratio *r* = median core time(A\*) ÷ median core time(Dijkstra) has median below 1.
  - **H2 (effort):** A\* expands fewer nodes than Dijkstra on most grids.
  - **H3 (correctness):** A\* and Dijkstra return equal path costs on every grid, equal to the independent reference minimum (validation must pass).
- **Test:**
  - two-sided Wilcoxon signed-rank test on log *r*, α = 0.05;
  - a 95% percentile-bootstrap CI for the median of *r* (10,000 resamples of grids, RNG seed 1).
  - **Effect size:** the median ratio and its bootstrap CI.
- **Decision rule:** H1 is supported only if the CI lies entirely below 1 **and** p < 0.05. Otherwise it is not supported for that workload.
- **Secondary:** expansion ratio, peak frontier ratio, end-to-end ratio. These are descriptive, with no tests.

### F. Endpoint placement (exploratory)

- **Workload:** 100×100, densities {0.10, 0.20, 0.30, 0.40} × placement {corners, random, reachable}; 100 grids × 4 runs; all algorithms.
- **Controlled:** walls are identical across placements for each seed. This is checked in the analysis via equal `passable_cells` per seed.
- **Analysis:** solvability rate, path length and nodes expanded for each placement. Placements are treated as **different populations**, and no pooled "overall" figure is computed.

### G. Reproducibility (verification)

- **G1:** repeat A's exact command twice more (A-rep1, A-rep2).
  - Check that every non-timing column of the per-run CSVs is byte-identical, and that grid hashes match.
  - Quantify between-invocation variation of the across-grid median core time.
- **G2:** repeat A's workload unpinned (no `taskset`) and compare timing variability with A.
- **G3:** repeat A with base seed 5000, a different sample of 50 grids from the same scenario. This separates workload-sampling variation from measurement noise in the aggregate statistics.

## What will not be concluded

- **No general claims.** No statement such as "algorithm X is faster" without its workload; results cover these grid models on this machine only.
- **No cross-objective comparisons.** Costs are not compared across algorithms with different objectives (BFS, DFS against Dijkstra or A\*) except as a "price of ignoring weights" or as suboptimality.
- **No significance claims** for exploratory experiments A–D and F. They get descriptive statistics with spread only.
- **No timing claims from flagged grids.** Grids flagged `near_timer_floor` are reported but not used for timing conclusions.
