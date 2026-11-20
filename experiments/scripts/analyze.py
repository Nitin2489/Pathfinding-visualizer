#!/usr/bin/env python3
"""Analysis of the raw benchmark data in experiments/raw (see plan.md).

Reads only the CSVs written by `pathfinding --benchmark`. Writes:
  experiments/processed/*.csv   tables derived from the raw data
  experiments/processed/stats.md  every table/statistic quoted in results.md
  experiments/plots/*.png       charts

Unit of analysis: the grid. Per-grid medians over repeated runs are taken
from the benchmark's own per-grid CSV; repeated runs are never treated as
independent observations.

Usage: python3 experiments/scripts/analyze.py [RAW_DIR [OUTPUT_DIR]]   (needs pandas, numpy, scipy, matplotlib)
"""
import gzip
import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from scipy import stats

import sys

ROOT = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
# Optional: analyze.py [RAW_DIR [OUTPUT_DIR]]  (defaults: experiments/raw, experiments)
RAW = os.path.abspath(sys.argv[1]) if len(sys.argv) > 1 else os.path.join(ROOT, "raw")
_OUTDIR = os.path.abspath(sys.argv[2]) if len(sys.argv) > 2 else ROOT
PROC, PLOTS = os.path.join(_OUTDIR, "processed"), os.path.join(_OUTDIR, "plots")
os.makedirs(PROC, exist_ok=True)
os.makedirs(PLOTS, exist_ok=True)

ALGOS = ["dijkstra", "astar", "bfs", "dfs"]
LABEL = {"dijkstra": "Dijkstra", "astar": "A*", "bfs": "BFS", "dfs": "DFS"}
COLOR = {"dijkstra": "#1D4ED8", "astar": "#7C3AED", "bfs": "#0E7490", "dfs": "#B45309"}
MACHINE = "i5-1235U, WSL2, GCC 15.2 -O3 (Release), commit 3d42fc8, pinned to 1 vCPU"
RNG = np.random.default_rng(1)
OUT = []  # markdown fragments for stats.md


def md(text=""):
    OUT.append(text)


def table(df, floatfmt=4):
    cols = list(df.columns)
    lines = ["| " + " | ".join(str(c) for c in cols) + " |", "|" + "---|" * len(cols)]
    for _, row in df.iterrows():
        cells = []
        for c in cols:
            v = row[c]
            if isinstance(v, (float, np.floating)):
                cells.append("" if np.isnan(v) else f"{v:.{floatfmt}g}")
            else:
                cells.append(str(v))
        lines.append("| " + " | ".join(cells) + " |")
    md("\n".join(lines))
    md()


def grid_csv(exp, name):
    return pd.read_csv(os.path.join(RAW, exp, f"{name}.grid.csv"), dtype={"grid_hash": str})


def runs_csv(exp, name):
    with gzip.open(os.path.join(RAW, exp, f"{name}.runs.csv.gz"), "rt") as f:
        return pd.read_csv(f, dtype={"grid_hash": str, "failure": str})


def boot_ci(values, stat=np.median, n=10000, rng=None):
    """Percentile bootstrap CI over grids."""
    rng = rng or RNG
    v = np.asarray(values, dtype=float)
    idx = rng.integers(0, len(v), size=(n, len(v)))
    s = stat(v[idx], axis=1)
    return np.percentile(s, 2.5), np.percentile(s, 97.5)


def pct(x, q):
    return np.percentile(np.asarray(x, dtype=float), q)


def style(ax, title, subtitle, xlabel, ylabel):
    ax.set_title(title + "\n", fontsize=12, fontweight="bold", loc="left")
    ax.text(0, 1.02, subtitle, transform=ax.transAxes, fontsize=8.5, color="#52606D")
    ax.set_xlabel(xlabel)
    ax.set_ylabel(ylabel)
    ax.grid(True, which="major", alpha=0.3)


def save(fig, name):
    fig.tight_layout(rect=(0, 0.05, 1, 1))  # leave room for the conditions footer
    fig.savefig(os.path.join(PLOTS, name), dpi=150)
    plt.close(fig)


def check_valid(g, where):
    assert (g.valid_runs == g.runs).all(), f"invalid runs in {where}"


# --------------------------------------------------------------------------- A
def experiment_a():
    md("## A. Baseline (100×100, 30% walls, reachable, 50 grids × 12 runs)")
    g = grid_csv("A", "baseline")
    check_valid(g, "A")
    rows = []
    for a in ALGOS:
        s = g[g.algorithm == a]
        core = s.search_ms_median * 1000  # µs
        lo, hi = boot_ci(core)
        rows.append({
            "algorithm": LABEL[a], "grids": len(s), "solved": f"{(s.path_found == 1).sum()}/{len(s)}",
            "core µs median": core.median(), "95% CI": f"[{lo:.3g}, {hi:.3g}]",
            "core µs p10": pct(core, 10), "core µs p90": pct(core, 90),
            "e2e µs median": (s.end_to_end_ms_median * 1000).median(),
            "expanded median": s.nodes_expanded.median(), "discovered median": s.nodes_discovered.median(),
            "peak frontier median": s.peak_frontier_entries.median(),
            "path len median": s.path_length.median(), "cost ratio median": s.cost_ratio.median(),
            "cost ratio max": s.cost_ratio.max(), "within-grid CV median": s.search_cv.median(),
            "near floor": int(s.near_timer_floor.sum())})
    df = pd.DataFrame(rows)
    df.to_csv(os.path.join(PROC, "A_baseline_summary.csv"), index=False)
    md("Per-grid values summarised across 50 grids. 95% CI = percentile bootstrap of the median (10,000 resamples of grids).")
    md()
    table(df)

    # paired ratios vs Dijkstra
    piv = g.pivot(index="grid_index", columns="algorithm", values="search_ms_median")
    exp = g.pivot(index="grid_index", columns="algorithm", values="nodes_expanded")
    rows = []
    for a in ["astar", "bfs", "dfs"]:
        r = piv[a] / piv["dijkstra"]
        lo, hi = boot_ci(r)
        er = exp[a] / exp["dijkstra"]
        rows.append({"comparison": f"{LABEL[a]} / Dijkstra", "median time ratio": r.median(),
                     "95% CI": f"[{lo:.3g}, {hi:.3g}]", "grids faster": f"{(r < 1).sum()}/{len(r)}",
                     "median expansion ratio": er.median(), "min exp ratio": er.min(), "max exp ratio": er.max()})
    df = pd.DataFrame(rows)
    df.to_csv(os.path.join(PROC, "A_paired_vs_dijkstra.csv"), index=False)
    md("Paired per-grid ratios against Dijkstra (core time; same grid and endpoints).")
    md()
    table(df)

    fig, axes = plt.subplots(1, 3, figsize=(15, 4.8))
    for ax, col, scale, ylabel, title in (
            (axes[0], "search_ms_median", 1000, "per-grid median core time (µs, log)", "Core search time"),
            (axes[1], "nodes_expanded", 1, "nodes expanded per grid (log)", "Nodes expanded"),
            (axes[2], "peak_frontier_entries", 1, "peak frontier entries per grid (log)", "Peak frontier size")):
        data = [g[g.algorithm == a][col] * scale for a in ALGOS]
        bp = ax.boxplot(data, tick_labels=[LABEL[a] for a in ALGOS], patch_artist=True, showfliers=False, widths=0.55)
        for patch, a in zip(bp["boxes"], ALGOS):
            patch.set_facecolor(COLOR[a])
            patch.set_alpha(0.35)
        for i, (d, a) in enumerate(zip(data, ALGOS)):
            x = np.random.default_rng(i).normal(i + 1, 0.06, len(d))
            ax.scatter(x, d, s=10, color=COLOR[a], alpha=0.7, zorder=3)
        ax.set_yscale("log")
        style(ax, title, "100×100, 30% walls, reachable; 50 grids (dots), 12 runs each", "algorithm", ylabel)
    fig.suptitle("Experiment A — baseline comparison on identical grids  (" + MACHINE + ")", fontsize=9, y=0.035,
                 color="#52606D")
    save(fig, "A_baseline.png")


# --------------------------------------------------------------------------- B
def experiment_b():
    md("## B. Obstacle density (100×100)")
    dens = [0, 10, 20, 30, 35, 40, 45, 50]
    rows, corner_rows = [], []
    for d in dens:
        g = grid_csv("B", f"reachable_d{d}")
        check_valid(g, f"B reachable {d}")
        for a in ALGOS:
            s = g[g.algorithm == a]
            rows.append({"density": d / 100, "algorithm": a, "grids": len(s),
                         "core_us_median": s.search_ms_median.median() * 1000,
                         "core_us_q1": pct(s.search_ms_median * 1000, 25), "core_us_q3": pct(s.search_ms_median * 1000, 75),
                         "e2e_us_median": s.end_to_end_ms_median.median() * 1000,
                         "expanded_median": s.nodes_expanded.median(),
                         "expanded_q1": pct(s.nodes_expanded, 25), "expanded_q3": pct(s.nodes_expanded, 75),
                         "frontier_median": s.peak_frontier_entries.median(),
                         "path_len_median": s.path_length.median(), "cv_median": s.search_cv.median(),
                         "near_floor": int(s.near_timer_floor.sum()),
                         "attempts_median": s.placement_attempts.median(), "attempts_max": s.placement_attempts.max()})
        c = grid_csv("B", f"corners_d{d}")
        check_valid(c, f"B corners {d}")
        for a in ALGOS:
            s = c[c.algorithm == a]
            corner_rows.append({"density": d / 100, "algorithm": a, "grids": len(s),
                                "solvable": int(s.reachable.sum()), "solved": int(s.path_found.sum()),
                                "expanded_median_all": s.nodes_expanded.median(),
                                "expanded_median_unsolvable": s[s.reachable == 0].nodes_expanded.median(),
                                "core_us_median_all": s.search_ms_median.median() * 1000})
    b = pd.DataFrame(rows)
    bc = pd.DataFrame(corner_rows)
    b.to_csv(os.path.join(PROC, "B_density_reachable.csv"), index=False)
    bc.to_csv(os.path.join(PROC, "B_density_corners.csv"), index=False)

    md("B1, reachable placement (50 grids per density; medians across grids of per-grid values):")
    md()
    show = b.pivot_table(index="density", columns="algorithm", values="core_us_median")[ALGOS]
    show.columns = [f"{LABEL[a]} core µs" for a in ALGOS]
    show2 = b.pivot_table(index="density", columns="algorithm", values="expanded_median")[ALGOS]
    show2.columns = [f"{LABEL[a]} expanded" for a in ALGOS]
    extra = b[b.algorithm == "dijkstra"].set_index("density")[["path_len_median", "attempts_median", "attempts_max"]]
    extra.columns = ["path len (Dijkstra)", "placement attempts median", "attempts max"]
    table(pd.concat([show, show2, extra], axis=1).reset_index())
    cv = b.pivot_table(index="density", columns="algorithm", values="cv_median")[ALGOS]
    cv.columns = [f"{LABEL[a]} CV" for a in ALGOS]
    md("Median within-grid CV of core time (noise across the 12 runs of each grid):")
    md()
    table(cv.reset_index())
    floor = b.groupby("density").near_floor.sum()
    md("Grids flagged near the timer floor (all algorithms, out of 200 per density): " +
       ", ".join(f"{d:g}: {int(v)}" for d, v in floor.items()))
    md()
    md("B2, corner placement (50 grids per density; solvability from the independent reference):")
    md()
    t = bc[bc.algorithm == "dijkstra"][["density", "grids", "solvable"]].copy()
    t["solvable rate"] = t.solvable / t.grids
    for a in ALGOS:
        t[f"{LABEL[a]} solved"] = bc[bc.algorithm == a].solved.values
    t["Dijkstra expanded (unsolvable grids, median)"] = bc[bc.algorithm == "dijkstra"].expanded_median_unsolvable.values
    table(t)

    fig, axes = plt.subplots(1, 3, figsize=(15, 4.8))
    for a in ALGOS:
        s = b[b.algorithm == a]
        axes[0].plot(s.density * 100, s.core_us_median, "-o", color=COLOR[a], label=LABEL[a])
        axes[0].fill_between(s.density * 100, s.core_us_q1, s.core_us_q3, color=COLOR[a], alpha=0.12)
        axes[1].plot(s.density * 100, s.expanded_median, "-o", color=COLOR[a], label=LABEL[a])
        axes[1].fill_between(s.density * 100, s.expanded_q1, s.expanded_q3, color=COLOR[a], alpha=0.12)
    axes[0].set_yscale("log")
    axes[1].set_yscale("log")
    style(axes[0], "Core time vs wall density", "reachable endpoints; line = median, band = IQR over 50 grids",
          "wall density (%)", "per-grid median core time (µs, log)")
    style(axes[1], "Nodes expanded vs wall density", "median & IQR over 50 grids; Dijkstra and BFS coincide (unit costs)",
          "wall density (%)", "nodes expanded (log)")
    d0 = bc[bc.algorithm == "dijkstra"]
    axes[2].plot(d0.density * 100, d0.solvable / d0.grids * 100, "-o", color="#B91C1C", label="corners: solvable %")
    path = b[b.algorithm == "dijkstra"]
    ax2 = axes[2].twinx()
    ax2.plot(path.density * 100, path.path_len_median, "-s", color="#475569", label="reachable: median path length")
    ax2.set_ylabel("median shortest path length, reachable (moves)", color="#475569")
    axes[2].set_ylim(-3, 103)
    style(axes[2], "Solvability and problem size", "corners: share of 50 grids solvable; reachable: median path length",
          "wall density (%)", "corner-to-corner solvable (%)")
    h1, l1 = axes[2].get_legend_handles_labels()
    h2, l2 = ax2.get_legend_handles_labels()
    axes[2].legend(h1 + h2, l1 + l2, fontsize=8, loc="lower left")
    axes[0].legend(fontsize=8)
    axes[1].legend(fontsize=8)
    fig.suptitle("Experiment B — obstacle density, 100×100 grids  (" + MACHINE + ")", fontsize=9, y=0.035,
                 color="#52606D")
    save(fig, "B_density.png")


# --------------------------------------------------------------------------- C
def experiment_c():
    md("## C. Weighted terrain (100×100, 20% walls, reachable, 50 grids × 12 runs)")
    levels = [0, 10, 30, 50, 100]
    frames = {}
    for t in levels:
        g = grid_csv("C", f"terrain_p{t}")
        check_valid(g, f"C {t}")
        frames[t] = g
    # paired-design check: identical endpoints and open cells per grid across terrain levels
    base = frames[0][frames[0].algorithm == "dijkstra"].set_index("grid_index")
    same = all((f[f.algorithm == "dijkstra"].set_index("grid_index")[["start_row", "start_col", "end_row", "end_col",
                                                                      "passable_cells"]]
                == base[["start_row", "start_col", "end_row", "end_col", "passable_cells"]]).all().all()
               for f in frames.values())
    md(f"Design check: start, end and open-cell count identical for every grid across all five terrain levels: **{same}**.")
    md()
    rows = []
    for t, g in frames.items():
        piv = {c: g.pivot(index="grid_index", columns="algorithm", values=c)
               for c in ("nodes_expanded", "search_ms_median", "path_cost", "path_length", "cost_ratio",
                         "reference_min_cost", "peak_frontier_entries")}
        ratio_exp = piv["nodes_expanded"]["astar"] / piv["nodes_expanded"]["dijkstra"]
        ratio_t = piv["search_ms_median"]["astar"] / piv["search_ms_median"]["dijkstra"]
        lo, hi = boot_ci(ratio_t)
        cost_equal = bool((piv["path_cost"]["astar"] == piv["path_cost"]["dijkstra"]).all() and
                          (piv["path_cost"]["dijkstra"] == piv["reference_min_cost"]["dijkstra"]).all())
        rows.append({"terrain p": t / 100, "expected free-cell cost": 1 + 5 * t / 100,
                     "optimal cost median": piv["reference_min_cost"]["dijkstra"].median(),
                     "A*/Dijkstra expansions (median)": ratio_exp.median(),
                     "A*/Dijkstra core time (median)": ratio_t.median(), "time ratio 95% CI": f"[{lo:.3g}, {hi:.3g}]",
                     "A* = Dijkstra cost on all grids": cost_equal,
                     "Dijkstra expanded (median)": piv["nodes_expanded"]["dijkstra"].median(),
                     "Dijkstra peak frontier (median)": piv["peak_frontier_entries"]["dijkstra"].median(),
                     "A* peak frontier (median)": piv["peak_frontier_entries"]["astar"].median(),
                     "Dijkstra ns/expansion (median)": (piv["search_ms_median"]["dijkstra"] * 1e6 /
                                                        piv["nodes_expanded"]["dijkstra"]).median(),
                     "A* expanded (median)": piv["nodes_expanded"]["astar"].median(),
                     "BFS cost ratio median": piv["cost_ratio"]["bfs"].median(),
                     "BFS cost ratio max": piv["cost_ratio"]["bfs"].max(),
                     "DFS cost ratio median": piv["cost_ratio"]["dfs"].median(),
                     "Dijkstra path moves median": piv["path_length"]["dijkstra"].median(),
                     "BFS path moves median": piv["path_length"]["bfs"].median(),
                     "Dijkstra core µs median": piv["search_ms_median"]["dijkstra"].median() * 1000,
                     "A* core µs median": piv["search_ms_median"]["astar"].median() * 1000,
                     "BFS core µs median": piv["search_ms_median"]["bfs"].median() * 1000,
                     "DFS core µs median": piv["search_ms_median"]["dfs"].median() * 1000})
    c = pd.DataFrame(rows)
    c.to_csv(os.path.join(PROC, "C_terrain.csv"), index=False)
    table(c)

    fig, axes = plt.subplots(1, 3, figsize=(15, 4.8))
    x = c["terrain p"] * 100
    axes[0].plot(x, c["A*/Dijkstra expansions (median)"], "-o", color=COLOR["astar"], label="nodes expanded")
    axes[0].plot(x, c["A*/Dijkstra core time (median)"], "-s", color="#475569", label="core time")
    axes[0].axhline(1, color="k", lw=0.8, ls="--")
    axes[0].set_ylim(0, 1.1)
    style(axes[0], "A* relative to Dijkstra", "median over 50 grids of per-grid ratio A*/Dijkstra (same grid)",
          "terrain cells (% of free cells)", "ratio A* / Dijkstra")
    axes[0].legend(fontsize=8)
    for a, key in (("bfs", "BFS cost ratio median"), ("dfs", "DFS cost ratio median")):
        axes[1].plot(x, c[key], "-o", color=COLOR[a], label=f"{LABEL[a]} (median)")
    axes[1].plot(x, c["BFS cost ratio max"], ":", color=COLOR["bfs"], label="BFS (max)")
    axes[1].set_yscale("log")
    style(axes[1], "Price of ignoring terrain cost", "path cost ÷ optimal cost (Dijkstra/A* = 1.0 by validation)",
          "terrain cells (% of free cells)", "cost ratio (log)")
    axes[1].legend(fontsize=8)
    for a in ALGOS:
        axes[2].plot(x, c[f"{LABEL[a]} core µs median"], "-o", color=COLOR[a], label=LABEL[a])
    style(axes[2], "Core time vs terrain", "median over 50 grids of per-grid median core time",
          "terrain cells (% of free cells)", "core time (µs)")
    axes[2].legend(fontsize=8)
    fig.suptitle("Experiment C — weighted terrain, 100×100, 20% walls, reachable; walls & endpoints fixed per grid "
                 "across levels  (" + MACHINE + ")", fontsize=9, y=0.035, color="#52606D")
    save(fig, "C_terrain.png")


# --------------------------------------------------------------------------- D
def experiment_d():
    md("## D. Grid scaling (20% walls)")
    sizes = [50, 100, 200, 400, 800, 1600]
    rows, pts = [], []
    for mode in ("reachable", "corners"):
        for n in sizes:
            g = grid_csv("D", f"{mode}_n{n}")
            check_valid(g, f"D {mode} {n}")
            for a in ALGOS:
                s = g[g.algorithm == a]
                solv = s[s.reachable == 1]
                rows.append({"placement": mode, "side": n, "cells": n * n, "algorithm": a, "grids": len(s),
                             "solvable": len(solv), "runs/grid": int(s.runs.iloc[0]),
                             "core_ms_median": solv.search_ms_median.median(),
                             "core_ms_q1": pct(solv.search_ms_median, 25), "core_ms_q3": pct(solv.search_ms_median, 75),
                             "e2e_ms_median": solv.end_to_end_ms_median.median(),
                             "expanded_median": solv.nodes_expanded.median(),
                             "ns_per_expansion_median": (solv.search_ms_median * 1e6 / solv.nodes_expanded).median(),
                             "path_len_median": solv.path_length.median(),
                             "near_floor": int(s.near_timer_floor.sum())})
                for _, r in solv.iterrows():
                    pts.append({"placement": mode, "side": n, "algorithm": a, "expanded": r.nodes_expanded,
                                "core_ms": r.search_ms_median, "e2e_ms": r.end_to_end_ms_median})
    d = pd.DataFrame(rows)
    p = pd.DataFrame(pts)
    d.to_csv(os.path.join(PROC, "D_scaling.csv"), index=False)
    p.to_csv(os.path.join(PROC, "D_scaling_points.csv"), index=False)
    for mode in ("corners", "reachable"):
        md(f"{mode} placement, solvable grids only (medians across grids):")
        md()
        sub = d[d.placement == mode]
        t = sub.pivot_table(index="side", columns="algorithm", values="core_ms_median")[ALGOS]
        t.columns = [f"{LABEL[a]} core ms" for a in ALGOS]
        t2 = sub.pivot_table(index="side", columns="algorithm", values="ns_per_expansion_median")[ALGOS]
        t2.columns = [f"{LABEL[a]} ns/expansion" for a in ALGOS]
        meta = sub[sub.algorithm == "dijkstra"].set_index("side")[["grids", "solvable", "runs/grid", "path_len_median"]]
        table(pd.concat([meta, t, t2], axis=1).reset_index())
    # empirical exponent: log core time vs log nodes expanded (all solvable grids, both placements)
    md("Least-squares slope of log(core ms) on log(nodes expanded), all solvable grids of both placements, "
       "grids with ≥ 1,000 expansions (reduces timer-floor influence):")
    md()
    rows = []
    for a in ALGOS:
        s = p[(p.algorithm == a) & (p.expanded >= 1000)]
        res = stats.linregress(np.log10(s.expanded), np.log10(s.core_ms))
        rows.append({"algorithm": LABEL[a], "grids": len(s), "slope": res.slope,
                     "slope 95% CI": f"[{res.slope - 1.96 * res.stderr:.3f}, {res.slope + 1.96 * res.stderr:.3f}]",
                     "r²": res.rvalue ** 2})
    table(pd.DataFrame(rows))

    fig, axes = plt.subplots(1, 3, figsize=(15, 4.8))
    sub = d[d.placement == "corners"]
    for a in ALGOS:
        s = sub[sub.algorithm == a]
        axes[0].errorbar(s.cells, s.core_ms_median, yerr=[s.core_ms_median - s.core_ms_q1, s.core_ms_q3 - s.core_ms_median],
                         fmt="-o", color=COLOR[a], label=LABEL[a], capsize=3)
        q = p[(p.algorithm == a)]
        axes[1].scatter(q.expanded, q.core_ms, s=8, alpha=0.5, color=COLOR[a], label=LABEL[a])
        r = d[(d.placement == "corners") & (d.algorithm == a)]
        axes[2].plot(r.cells, r.ns_per_expansion_median, "-o", color=COLOR[a], label=LABEL[a])
    for ax in axes:
        ax.set_xscale("log")
        ax.set_yscale("log")
        ax.legend(fontsize=8)
    style(axes[0], "Core time vs grid size (corner to corner)",
          "solvable grids; median and IQR over 20 grids (5 at 1600²)", "grid cells (log)", "per-grid median core time (ms, log)")
    style(axes[1], "Core time vs work done", "every solvable grid, both placements; 50² … 1600²",
          "nodes expanded (log)", "per-grid median core time (ms, log)")
    style(axes[2], "Cost per expansion (corner to corner)", "median over solvable grids of core time ÷ nodes expanded",
          "grid cells (log)", "ns per expanded node (log)")
    fig.suptitle("Experiment D — scaling, 20% walls  (" + MACHINE + ")", fontsize=9, y=0.035, color="#52606D")
    save(fig, "D_scaling.png")


# --------------------------------------------------------------------------- E
def experiment_e():
    md("## E. A* vs Dijkstra — confirmatory (200×200, 20% walls, reachable, 100 grids × 12 runs)")
    rows = []
    fig, axes = plt.subplots(1, 3, figsize=(15, 4.8))
    for k, t in enumerate((0, 30)):
        g = grid_csv("E", f"astar_dijkstra_p{t}")
        check_valid(g, f"E {t}")
        piv = {c: g.pivot(index="grid_index", columns="algorithm", values=c)
               for c in ("search_ms_median", "end_to_end_ms_median", "nodes_expanded", "peak_frontier_entries",
                         "path_cost", "reference_min_cost", "near_timer_floor")}
        r = (piv["search_ms_median"]["astar"] / piv["search_ms_median"]["dijkstra"]).to_numpy()
        r_e2e = (piv["end_to_end_ms_median"]["astar"] / piv["end_to_end_ms_median"]["dijkstra"]).to_numpy()
        er = (piv["nodes_expanded"]["astar"] / piv["nodes_expanded"]["dijkstra"]).to_numpy()
        fr = (piv["peak_frontier_entries"]["astar"] / piv["peak_frontier_entries"]["dijkstra"]).to_numpy()
        w = stats.wilcoxon(np.log(r), alternative="two-sided")
        lo, hi = boot_ci(r, rng=np.random.default_rng(1))
        elo, ehi = boot_ci(er, rng=np.random.default_rng(2))
        cost_ok = bool((piv["path_cost"]["astar"] == piv["path_cost"]["dijkstra"]).all()
                       and (piv["path_cost"]["astar"] == piv["reference_min_cost"]["astar"]).all())
        h1 = hi < 1 and w.pvalue < 0.05
        rows.append({"terrain p": t / 100, "grids": len(r),
                     "median time ratio A*/Dijkstra": np.median(r), "95% CI (bootstrap)": f"[{lo:.3f}, {hi:.3f}]",
                     "geomean ratio": float(np.exp(np.mean(np.log(r)))),
                     "Wilcoxon p (two-sided, log r)": f"{w.pvalue:.2e}", "A* faster": f"{(r < 1).sum()}/{len(r)}",
                     "H1 supported": h1,
                     "median expansion ratio": np.median(er), "exp. ratio 95% CI": f"[{elo:.3f}, {ehi:.3f}]",
                     "A* fewer expansions": f"{(er < 1).sum()}/{len(er)}", "exp. ratio max": er.max(),
                     "median frontier ratio": np.median(fr), "median e2e ratio": np.median(r_e2e),
                     "H3 equal & optimal cost": cost_ok,
                     "near-floor grids": int(piv["near_timer_floor"].to_numpy().sum())})
        axes[k].scatter(piv["nodes_expanded"]["dijkstra"], piv["nodes_expanded"]["astar"], s=14,
                        color=COLOR["astar"], alpha=0.7)
        mx = piv["nodes_expanded"]["dijkstra"].max() * 1.05
        axes[k].plot([0, mx], [0, mx], "k--", lw=0.8, label="y = x (no reduction)")
        axes[k].set_xlim(0, mx)
        axes[k].set_ylim(0, mx)
        style(axes[k], f"Nodes expanded per grid, terrain {t}%", "each dot = one of 100 grids (same grid & endpoints)",
              "Dijkstra nodes expanded", "A* nodes expanded")
        axes[k].legend(fontsize=8)
        bins = np.linspace(-3.5, 0.5, 41)
        axes[2].hist(np.log2(r), bins=bins, alpha=0.55, label=f"terrain {t}%  (median {np.median(r):.3f})",
                     color=["#7C3AED", "#D97706"][k])
    axes[2].axvline(0, color="k", lw=0.8, ls="--")
    style(axes[2], "Paired core-time ratio", "log2(per-grid median A* / per-grid median Dijkstra); left of 0 = A* faster",
          "log2 time ratio", "grids")
    axes[2].legend(fontsize=8)
    fig.suptitle("Experiment E — A* vs Dijkstra, 200×200, 20% walls, reachable, 100 grids × 12 runs  (" + MACHINE + ")",
                 fontsize=9, y=0.035, color="#52606D")
    save(fig, "E_astar_vs_dijkstra.png")
    e = pd.DataFrame(rows)
    e.to_csv(os.path.join(PROC, "E_astar_vs_dijkstra.csv"), index=False)
    table(e)


# --------------------------------------------------------------------------- F
def experiment_f():
    md("## F. Endpoint placement (100×100, 100 grids × 4 runs per cell)")
    rows = []
    frames = {}
    for d in (10, 20, 30, 40):
        for pl in ("corners", "random", "reachable"):
            g = grid_csv("F", f"{pl}_d{d}")
            check_valid(g, f"F {pl} {d}")
            frames[(d, pl)] = g
            s = g[g.algorithm == "dijkstra"]
            a = g[g.algorithm == "astar"]
            solv = s[s.reachable == 1]
            rows.append({"density": d / 100, "placement": pl, "grids": len(s), "solvable": int(s.reachable.sum()),
                         "path len median (solvable)": solv.path_length.median(),
                         "path len p10": pct(solv.path_length, 10) if len(solv) else np.nan,
                         "path len p90": pct(solv.path_length, 90) if len(solv) else np.nan,
                         "Dijkstra expanded median (all)": s.nodes_expanded.median(),
                         "A* expanded median (all)": a.nodes_expanded.median(),
                         "A*/Dijkstra exp. ratio median (solvable)":
                             (a[a.reachable == 1].set_index("grid_index").nodes_expanded /
                              solv.set_index("grid_index").nodes_expanded).median(),
                         "attempts median": s.placement_attempts.median(), "attempts max": s.placement_attempts.max()})
        # walls identical across placements: same open-cell count per seed
        cnt = [frames[(d, pl)][frames[(d, pl)].algorithm == "dijkstra"].set_index("grid_index").passable_cells
               for pl in ("corners", "random", "reachable")]
        md(f"density {d}%: open-cell count identical per seed across placements: **{bool((cnt[0] == cnt[1]).all() and (cnt[1] == cnt[2]).all())}**")
    md()
    f = pd.DataFrame(rows)
    f.to_csv(os.path.join(PROC, "F_placement.csv"), index=False)
    table(f)

    fig, axes = plt.subplots(1, 3, figsize=(15, 4.8))
    pls = ["corners", "random", "reachable"]
    pcol = {"corners": "#B91C1C", "random": "#0E7490", "reachable": "#4338CA"}
    for pl in pls:
        s = f[f.placement == pl]
        axes[0].plot(s.density * 100, s.solvable / s.grids * 100, "-o", color=pcol[pl], label=pl)
    style(axes[0], "Solvable instances", "share of 100 grids with a path (independent reference)",
          "wall density (%)", "solvable (%)")
    axes[0].set_ylim(-3, 103)
    axes[0].legend(fontsize=8)
    pos = 0
    ticks, labels = [], []
    for d in (10, 20, 30, 40):
        for pl in pls:
            g = frames[(d, pl)]
            s = g[(g.algorithm == "dijkstra") & (g.reachable == 1)].path_length
            bp = axes[1].boxplot([s], positions=[pos], widths=0.7, patch_artist=True, showfliers=False)
            bp["boxes"][0].set_facecolor(pcol[pl])
            bp["boxes"][0].set_alpha(0.4)
            pos += 1
        ticks.append(pos - 2)
        labels.append(f"{d}%")
        pos += 1
    axes[1].set_xticks(ticks, labels)
    style(axes[1], "Shortest path length (solvable instances)", "boxes: corners | random | reachable; corners ≈ fixed 194 moves, n = 1 at 40%",
          "wall density (%)", "shortest path (moves)")
    for pl in pls:
        s = f[f.placement == pl]
        axes[2].plot(s.density * 100, s["Dijkstra expanded median (all)"], "-o", color=pcol[pl], label=pl)
    axes[2].set_yscale("log")
    style(axes[2], "Dijkstra search effort", "median nodes expanded over all 100 grids (solvable or not)",
          "wall density (%)", "nodes expanded (log)")
    axes[2].legend(fontsize=8)
    fig.suptitle("Experiment F — endpoint placement on identical walls, 100×100  (" + MACHINE + ")", fontsize=9,
                 y=0.035, color="#52606D")
    save(fig, "F_placement.png")


# --------------------------------------------------------------------------- G
def experiment_g():
    md("## G. Reproducibility (A's workload repeated)")
    timing = ["search_time_ms", "end_to_end_time_ms"]
    a = runs_csv("A", "baseline")
    r1 = runs_csv("G", "rep1")
    r2 = runs_csv("G", "rep2")
    un = runs_csv("G", "unpinned")
    nt = [c for c in a.columns if c not in timing]
    same = [a[nt].equals(x[nt]) for x in (r1, r2, un)]
    md(f"G1. Non-timing columns of the per-run CSV ({len(nt)} columns × {len(a)} rows) identical to A: "
       f"rep1 **{same[0]}**, rep2 **{same[1]}**, unpinned **{same[2]}**. "
       f"Grid hashes identical: **{all(a.grid_hash.equals(x.grid_hash) for x in (r1, r2, un))}**.")
    md()
    # between-invocation variation of the aggregate statistic
    rows = []
    gA, g1, g2, gU, gS = (grid_csv("A", "baseline"), grid_csv("G", "rep1"), grid_csv("G", "rep2"),
                          grid_csv("G", "unpinned"), grid_csv("G", "seed5000"))
    for alg in ALGOS:
        meds = [x[x.algorithm == alg].search_ms_median.median() * 1000 for x in (gA, g1, g2)]
        per_grid = pd.concat([x[x.algorithm == alg].set_index("grid_index").search_ms_median for x in (gA, g1, g2)], axis=1)
        rel = (per_grid.max(axis=1) - per_grid.min(axis=1)) / per_grid.median(axis=1)
        un_med = gU[gU.algorithm == alg].search_ms_median.median() * 1000
        s_med = gS[gS.algorithm == alg].search_ms_median.median() * 1000
        rows.append({"algorithm": LABEL[alg],
                     "A core µs": meds[0], "rep1": meds[1], "rep2": meds[2],
                     "spread of 3 pinned (max-min)/median": (max(meds) - min(meds)) / np.median(meds),
                     "per-grid spread, median": rel.median(), "per-grid spread, p90": pct(rel, 90),
                     "unpinned core µs": un_med,
                     "CV median pinned (A)": gA[gA.algorithm == alg].search_cv.median(),
                     "CV median unpinned": gU[gU.algorithm == alg].search_cv.median(),
                     "seed 5000 core µs": s_med,
                     "seed 5000 / A": s_med / meds[0],
                     "expanded median A": gA[gA.algorithm == alg].nodes_expanded.median(),
                     "expanded median seed 5000": gS[gS.algorithm == alg].nodes_expanded.median()})
    gdf = pd.DataFrame(rows)
    gdf.to_csv(os.path.join(PROC, "G_reproducibility.csv"), index=False)
    md("G1–G3. Across-grid median of per-grid median core time (µs), by invocation:")
    md()
    table(gdf)
    # paired ratio reproducibility (dijkstra/astar) across invocations
    pr = []
    for name, x in (("A", gA), ("rep1", g1), ("rep2", g2), ("unpinned", gU), ("seed5000", gS)):
        piv = x.pivot(index="grid_index", columns="algorithm", values="search_ms_median")
        r = piv["astar"] / piv["dijkstra"]
        lo, hi = boot_ci(r, rng=np.random.default_rng(3))
        pr.append({"invocation": name, "median A*/Dijkstra core ratio": r.median(), "95% CI": f"[{lo:.3f}, {hi:.3f}]"})
    md("Paired A*/Dijkstra ratio per invocation:")
    md()
    table(pd.DataFrame(pr))

    fig, axes = plt.subplots(1, 2, figsize=(12, 4.8))
    for alg in ALGOS:
        x = gA[gA.algorithm == alg].set_index("grid_index").search_ms_median * 1000
        for other, mk in ((g1, "o"), (g2, "^")):
            y = other[other.algorithm == alg].set_index("grid_index").search_ms_median * 1000
            axes[0].scatter(x, y, s=10, marker=mk, color=COLOR[alg], alpha=0.6,
                            label=LABEL[alg] if mk == "o" else None)
    lim = [1, 2000]
    axes[0].plot(lim, lim, "k--", lw=0.8)
    axes[0].set_xscale("log")
    axes[0].set_yscale("log")
    axes[0].legend(fontsize=8)
    style(axes[0], "Per-grid median core time: run A vs repeats", "same grids & seeds; ○ rep1, △ rep2; 50 grids × 4 algorithms",
          "invocation A (µs, log)", "repeat invocation (µs, log)")
    data = [gA.search_cv.dropna(), g1.search_cv.dropna(), g2.search_cv.dropna(), gU.search_cv.dropna()]
    axes[1].boxplot(data, tick_labels=["A (pinned)", "rep1 (pinned)", "rep2 (pinned)", "unpinned"], showfliers=False)
    style(axes[1], "Within-grid timing noise", "coefficient of variation of the 12 core-time runs per grid (all algorithms)",
          "invocation", "CV of core time")
    fig.suptitle("Experiment G — reproducibility, 100×100, 30% walls, reachable  (" +
                 MACHINE.replace(", pinned to 1 vCPU", "; A, rep1, rep2 pinned to 1 vCPU, 'unpinned' not") + ")", fontsize=9,
                 y=0.035, color="#52606D")
    save(fig, "G_reproducibility.png")


if __name__ == "__main__":
    md("# Statistics generated by experiments/scripts/analyze.py")
    md()
    md("All numbers below are computed from experiments/raw by this script. Units: µs/ms are core search time unless stated.")
    md()
    for fn in (experiment_a, experiment_b, experiment_c, experiment_d, experiment_e, experiment_f, experiment_g):
        fn()
        print("done", fn.__name__)
    with open(os.path.join(PROC, "stats.md"), "w", encoding="utf-8") as f:
        f.write("\n".join(OUT) + "\n")
    print("wrote processed/stats.md and plots/")
