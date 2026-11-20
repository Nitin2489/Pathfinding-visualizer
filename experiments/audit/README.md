# Independent audit of the benchmark study

After the study, an independent audit re-derived its results from `../raw/`, without relying on `../results.md` or `../scripts/analyze.py`. The audit changed nothing in the study's data or outputs; the corrections it prompted are listed at the end and applied in [`../results.md`](../results.md) (§16).

## Scope

| Area | What was checked | Result |
|---|---|---|
| Inventory | All 52 invocations from `raw/commands.log` against `plan.md`: sizes, densities, terrain, placement, seeds, grids, runs, warm-up, algorithms; row counts; metadata (commit, clean tree, status) | All match the plan; 82,400 runs, 2,970 grids, 11,480 warm-ups; nothing missing or unplanned |
| Run integrity | `checks_passed`, failures, run-order balance, determinism of non-timing metrics across a grid's runs, core ≤ end-to-end time | All hold in all 82,400 rows |
| Per-grid statistics | Median, mean and stddev recomputed from the per-run CSVs against the published per-grid CSVs | Identical |
| Grid reconstruction | `regen.py`: separately written Python generator (mt19937_64 checked against the C++ standard's conformance value) | All 2,970 grids regenerated **bit-exactly**: FNV-1a fingerprints, endpoints, open-cell counts, reachable-placement attempts |
| Correctness | Own BFS and Dijkstra on each regenerated grid | Every reference minimum cost and move count correct; every Dijkstra and A\* cost optimal; every BFS move count minimal; no result below the optimum; every unreachable grid reported unsolved |
| Statistics | Independent recomputation of A–G: ratios, bootstrap CIs (own RNG), tests, slopes, placement and reproducibility figures | All headline numbers reproduce, CIs within resampling noise; three errors and several qualifications found (below) |

## Files

| File | Purpose |
|---|---|
| [`regen.py`](regen.py) | Independent generator and shortest-path oracle, as used in the audit |
| [`verify_grids.py`](verify_grids.py) | Regenerates every grid and checks the per-grid CSVs (exit 1 on any discrepancy) |
| [`audit_stats.py`](audit_stats.py) | Computes the audit-derived figures quoted in `results.md`: exact tests, timer-floor sensitivity, BFS excess by level, slopes by pooling, pinning comparison |

All three read `../raw/` only and write nothing. They need Python 3 with pandas, NumPy and SciPy.

```bash
python3 experiments/audit/verify_grids.py
python3 experiments/audit/audit_stats.py
```

**Not included.** The audit's exploratory scratch scripts (inventory checks and the first statistical recomputation) and its full working report are not archived; they contained machine-specific paths. Their results are summarised in the tables here and in `results.md` §16. `verify_grids.py` was adapted only to use relative paths and to stop writing an auxiliary file; its checks are unchanged. `audit_stats.py` was written for this archive and reproduces the audit's numbers. Bootstrap CIs can differ in the last digit from the audit's original run because the random draws differ.

## Corrections made after the audit

1. **Wilcoxon p.** 3.9 × 10⁻¹⁸ is the normal approximation, not "the smallest possible p for n = 100". The exact Wilcoxon and sign-test p is 2 × 0.5¹⁰⁰ ≈ 1.6 × 10⁻³⁰.
2. **Timer-floor exclusion.** Flagged grids were retained in the published medians, CIs and test, contrary to the plan's and report's wording.
   - Affected: A, one grid per algorithm; E, grids 28 and 50 at 0% terrain.
   - Without them, E = 0.0690 [0.056, 0.090]. No conclusion changes.
3. **BFS excess-cost range.** 28–78%, not 36–78%; the 100% terrain level is 27.7%.

## Qualifications added

- **Equal counts, not cells:** equal BFS/Dijkstra expansion counts do not prove identical cell sets, because cell identity is not recorded.
- **Withdrawn explanation:** the cost-variability explanation for BFS's excess cost was withdrawn; it is contradicted at 10% terrain.
- **Hypotheses:** the terrain mechanisms for A\* and Dijkstra are labelled as untested hypotheses.
- **DFS slope:** the DFS scaling slope depends on pooling (1.035 pooled, 0.987 corners only, 1.053 reachable only).
- **Pinning:** the pinning result covers one observed comparison only (Mann–Whitney p = 0.29–0.94).
- **Test choice:** the sign test, not the Wilcoxon test, directly matches a hypothesis about the median.

## Audit limitations

- **Paths not stored:** the CSVs do not record paths, so path *structure* (adjacency, walls, no revisits), and DFS path validity in particular, could not be re-checked. It rests on the framework's validator (`src/validation.cpp`), which code inspection shows does not call the search code.
- **Same specification:** `regen.py` reimplements the generator from the C++ semantics. It is independent code, not an independent specification, so a shared misunderstanding of the intended generator would not be caught; bit-exact fingerprints show only that it matches the C++ behaviour.
- **Second execution:** the study's second full execution was checked against data that existed outside the repository at audit time. Those data are not archived here, so that check cannot be repeated from the repository.
- **Pre-registration:** `plan.md`'s timestamp precedes the first run, but the plan was not committed beforehand, so its pre-registration is not tamper-evident.
- **Timing environment:** the audit did not rerun the benchmark, so it does not assess timing on other hardware.
