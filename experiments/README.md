# Benchmarking study

A reproducible experimental study of Dijkstra, A\*, BFS and DFS using this repository's own benchmark framework at commit `3d42fc8`. It covers 82,400 timed searches on 2,970 generated grids, every run independently validated.

The study was **independently audited** after completion ([`audit/README.md`](audit/README.md)). The audit regenerated all grids bit-exactly, confirmed correctness and recomputed the statistics. `results.md` has been corrected accordingly (§16); raw data and original analysis outputs are unchanged.

**Read first:** [`results.md`](results.md), the full report: executive summary, methods, results, statistics, limitations and threats to validity.

| File | Contents |
|---|---|
| [`plan.md`](plan.md) | experimental plan, written before execution, including the hypotheses for Experiment E. Unchanged since execution; not committed beforehand, so the pre-registration is not tamper-evident. |
| [`results.md`](results.md) | the report |
| [`environment.md`](environment.md) | build, platform and timer details, all taken from recorded metadata |
| [`scripts/run_experiments.sh`](scripts/run_experiments.sh) | runs every experiment through the real `pathfinding --benchmark` CLI |
| [`scripts/analyze.py`](scripts/analyze.py) | turns `raw/` into `processed/` and `plots/` (deterministic) |
| `raw/<EXP>/` | per configuration: `*.runs.csv.gz` (one row per timed run), `*.grid.csv`, `*.summary.csv`, `*.paired.csv`, and one `.meta.json` per CSV |
| [`raw/commands.log`](raw/commands.log) | every executed command, with exit code and wall time |
| [`processed/stats.md`](processed/stats.md) | every statistic quoted in the report |
| `processed/*.csv` | derived tables |
| `plots/*.png` | the seven charts used in the report |
| [`audit/`](audit/README.md) | independent audit: summary, grid-regeneration oracle and correctness check, audit-derived statistics |

## Experiments

| | Question | Status |
|---|---|---|
| A | baseline comparison of all four algorithms | done |
| B | wall density, reachable and corner endpoints | done |
| C | weighted terrain, within-grid across 5 levels | done |
| D | grid scaling, 50² to 1600² | done |
| E | A\* vs Dijkstra, **confirmatory** | done: H1–H3 supported; a second execution agreed (its data not archived) |
| F | endpoint placement on identical walls | done |
| G | reproducibility and timing variation | done; a second full execution matched in all non-timing columns (data not archived) |

## Reproduce

```bash
git worktree add --detach ../pf-3d42fc8 3d42fc8
cmake -S ../pf-3d42fc8 -B ../pf-3d42fc8/build -DCMAKE_BUILD_TYPE=Release && cmake --build ../pf-3d42fc8/build -j
BIN=../pf-3d42fc8/build/pathfinding OUT=experiments/raw-repro experiments/scripts/run_experiments.sh   # ~70 s
python3 -m pip install pandas numpy scipy matplotlib
python3 experiments/scripts/analyze.py experiments/raw-repro experiments/repro-out
```

- **Timing:** non-timing results reproduce exactly. Absolute times depend on the machine.
- **Pinning:** the orchestrator pins to logical CPU 2 (`CPU=…` overrides it).
- **Requirements:** `run_experiments.sh` needs bash, `taskset` (util-linux), `bc` and `gzip`.
- **Overwrite protection:** it refuses to overwrite existing raw data unless `FORCE=1`.

What can be reproduced from this repository:
- **The original study's statistics:** `scripts/analyze.py` on `raw/`.
- **The audit's checks:** `python3 experiments/audit/verify_grids.py` and `audit/audit_stats.py`.
- **A fresh replication:** the commands above. Expect identical non-timing data and different absolute times.

The second full execution mentioned in `results.md` cannot be re-checked here, because its raw data were not archived.

## Size

The raw data is 5.6 MB, with per-run CSVs gzip-compressed; processed tables and plots add about 1.5 MB, and the audit scripts about 25 KB. No binaries or build files are stored here. Reproduction output directories are git-ignored.
