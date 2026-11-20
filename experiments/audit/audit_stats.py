#!/usr/bin/env python3
"""Audit-derived statistics for experiments/results.md (independent audit).

Recomputes, from experiments/raw only, the figures that the audit corrected or
qualified. Writes nothing; prints a report. Per-grid medians are computed here
from the per-run CSVs (not taken from the benchmark's per-grid files).

Usage: python3 experiments/audit/audit_stats.py      (needs pandas, numpy, scipy)
"""
import json
import os

import numpy as np
import pandas as pd
from scipy import stats

RAW = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "raw")
RNG = np.random.default_rng(20261121)


def per_grid(key):
    r = pd.read_csv(os.path.join(RAW, f"{key}.runs.csv.gz"), dtype={"grid_hash": str, "failure": str})
    floor = json.load(open(os.path.join(RAW, f"{key}.runs.csv.meta.json")))["timer"]["near_floor_threshold_ms"]
    g = r.groupby(["grid_index", "algorithm"]).agg(
        core=("search_time_ms", "median"), mean=("search_time_ms", "mean"), sd=("search_time_ms", "std"),
        exp=("nodes_expanded", "first"), cost=("path_cost", "first"), ref=("reference_min_cost", "first"),
        reach=("reachable", "first")).reset_index()
    g["flag"] = g.core < floor  # same rule as the benchmark's near_timer_floor flag
    return g, floor


def pv(g, col):
    return g.pivot(index="grid_index", columns="algorithm", values=col)


def boot_median(v, n=20000):
    v = np.asarray(v, float)
    s = np.median(v[RNG.integers(0, len(v), (n, len(v)))], axis=1)
    return np.percentile(s, [2.5, 97.5])


print("== Experiment E: tests and timer-floor sensitivity")
for t in (0, 30):
    g, floor = per_grid(f"E/astar_dijkstra_p{t}")
    core, flag = pv(g, "core"), pv(g, "flag")
    r = (core.astar / core.dijkstra).to_numpy()
    lr = np.log(r)
    w_asym = stats.wilcoxon(lr, method="asymptotic").pvalue
    w_exact = stats.wilcoxon(lr, method="exact").pvalue
    sign = stats.binomtest(int((r < 1).sum()), len(r), 0.5).pvalue
    keep = ~(flag.astar | flag.dijkstra).to_numpy()
    lo, hi = boot_median(r[keep])
    print(f"terrain {t}%: n={len(r)}, A* faster on {int((r < 1).sum())}; median ratio {np.median(r):.4f}")
    print(f"  Wilcoxon (log ratio) normal approximation p = {w_asym:.3g}; exact p = {w_exact:.3g}; "
          f"exact sign test p = {sign:.3g}; smallest possible two-sided p for n=100 = {2 * 0.5 ** 100:.3g}")
    flagged = sorted(int(i) for i in flag.index[~keep])
    print(f"  flagged grids (A* or Dijkstra median below {floor * 1000:.2f} us): {len(flagged)} -> grid_index {flagged}")
    print(f"  excluding them: n={keep.sum()}, median {np.median(r[keep]):.4f}, 95% CI [{lo:.3f}, {hi:.3f}] "
          f"(percentile bootstrap, 20,000 resamples)")

print("\n== Experiment A: flagged grids and paired ratios without them")
g, floor = per_grid("A/baseline")
core, flag = pv(g, "core"), pv(g, "flag")
for a in ("dijkstra", "astar", "bfs", "dfs"):
    print(f"  {a}: flagged grid_index {sorted(int(i) for i in flag.index[flag[a]])}")
for a in ("astar", "bfs", "dfs"):
    keep = ~(flag[a] | flag.dijkstra)
    r = (core[a] / core.dijkstra)[keep]
    lo, hi = boot_median(r)
    print(f"  {a}/dijkstra without flagged pairs: n={int(keep.sum())}, median {r.median():.3f} [{lo:.3f}, {hi:.3f}]")

print("\n== Experiment C: BFS cost ratio (path cost / optimal) by terrain level")
for t in (0, 10, 30, 50, 100):
    g, _ = per_grid(f"C/terrain_p{t}")
    b = g[g.algorithm == "bfs"]
    ratio = b.cost / b.ref
    p = t / 100
    m = (1 - p) + p * 18 / 3
    cv = ((1 - p) + p * 134 / 3 - m * m) ** 0.5 / m
    print(f"  p={p:<4} median excess {100 * (ratio.median() - 1):5.1f}%  max ratio {ratio.max():.3f}  "
          f"entry-cost CV {cv:.2f}")

print("\n== Experiment D: log-log slope of core time on nodes expanded (grids with >= 1000 expansions)")
pts = []
for mode in ("corners", "reachable"):
    for n in (50, 100, 200, 400, 800, 1600):
        g, _ = per_grid(f"D/{mode}_n{n}")
        s = g[g.reach == 1]
        pts += [(mode, a, e, c) for a, e, c in zip(s.algorithm, s.exp, s.core)]
p = pd.DataFrame(pts, columns=["mode", "alg", "exp", "core"])
for a in ("dijkstra", "astar", "bfs", "dfs"):
    out = []
    for label, sel in (("pooled", p.alg == a), ("corners", (p.alg == a) & (p["mode"] == "corners")),
                       ("reachable", (p.alg == a) & (p["mode"] == "reachable"))):
        s = p[sel & (p.exp >= 1000)]
        res = stats.linregress(np.log10(s.exp), np.log10(s.core))
        out.append(f"{label} {res.slope:.3f} (n={len(s)})")
    s = p[(p.alg == a) & (p.exp >= 1000)]
    ts = stats.theilslopes(np.log10(s.core), np.log10(s.exp))
    print(f"  {a:8s} OLS " + ", ".join(out) + f"; Theil-Sen pooled {ts.slope:.3f}")

print("\n== Experiment G: within-grid CV, pinned (A) vs the single unpinned invocation")
gp, _ = per_grid("A/baseline")
gu, _ = per_grid("G/unpinned")
for a in ("dijkstra", "astar", "bfs", "dfs"):
    cp = (gp.sd / gp["mean"])[gp.algorithm == a]
    cu = (gu.sd / gu["mean"])[gu.algorithm == a]
    print(f"  {a:8s} median CV pinned {cp.median():.3f} vs unpinned {cu.median():.3f}; "
          f"Mann-Whitney U p = {stats.mannwhitneyu(cp, cu).pvalue:.2f}")
