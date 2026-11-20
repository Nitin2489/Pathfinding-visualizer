#!/usr/bin/env python3
"""Independent audit: regenerate every benchmark grid from its configuration and
seed with regen.py (no C++ code involved) and check the published per-grid CSVs.

For each grid: fingerprint, endpoints, open-cell count, placement attempts and
reachability must match; reference minimum cost / moves must equal this
script's own Dijkstra / BFS; Dijkstra and A* costs must be optimal, BFS move
counts minimal, and no algorithm may beat the optimum.

Reads experiments/raw only; writes nothing. Usage:
    python3 experiments/audit/verify_grids.py        (needs pandas; ~1 min with 10 processes)
"""
import os, sys
from multiprocessing import Pool
import pandas as pd
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from regen import generate, fnv, min_moves, min_cost, WALL

RAW = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "raw"))

def configs():
    out = []
    for line in open(os.path.join(RAW, "commands.log")):
        cmd = line.rstrip("\n").split("\t")[3].split()
        a = dict(zip(cmd, cmd[1:]))
        rows, cols = map(int, a["--size"].split("x"))
        grid_csv = a["--grid-csv"]
        rel = grid_csv[grid_csv.index("experiments/raw/") + len("experiments/raw/"):]
        out.append(dict(rows=rows, cols=cols, density=float(a.get("--density", "30")) / 100,
                        terrain=float(a.get("--terrain", "0")) / 100, placement=a.get("--placement", "reachable"),
                        seed=int(a["--seed"]), grids=int(a["--grids"]), runs=int(a["--runs"]),
                        warmup=int(a["--warmup"]), algos=a["--algorithm"], csv=rel))
    return out

def check_grid(job):
    cfg, gi, rowsdf = job
    seed = cfg["seed"] + gi
    res = generate(cfg["rows"], cfg["cols"], cfg["density"], cfg["terrain"], cfg["placement"], seed)
    issues = []
    if res is None:
        return (cfg["csv"], gi, ["regen: generation failed but CSV has the grid"])
    g, s, t, att = res
    R, C = cfg["rows"], cfg["cols"]
    h = "0x%016x" % fnv(g, R, C, s, t)
    mv = min_moves(g, R, C, s, t)
    mc = min_cost(g, R, C, s, t)
    passable = sum(1 for ch in g if ch != WALL)
    for r in rowsdf:
        alg = r["algorithm"]
        if r["grid_hash"] != h: issues.append(f"{alg}: hash {r['grid_hash']} != {h}")
        if (r["start_row"], r["start_col"], r["end_row"], r["end_col"]) != (s // C, s % C, t // C, t % C): issues.append("endpoints")
        if int(r["passable_cells"]) != passable: issues.append("passable")
        if int(r["placement_attempts"]) != att: issues.append(f"attempts {r['placement_attempts']} != {att}")
        if int(r["reachable"]) != (mv is not None): issues.append("reachable flag")
        if mv is None:
            if int(r["path_found"]) != 0: issues.append(f"{alg} found path on unreachable grid")
            if not pd.isna(r["reference_min_cost"]): issues.append("ref cost present on unreachable")
            continue
        if int(r["reference_min_cost"]) != mc: issues.append(f"ref cost {r['reference_min_cost']} != {mc}")
        if int(r["reference_min_moves"]) != mv: issues.append(f"ref moves {r['reference_min_moves']} != {mv}")
        if int(r["path_found"]) != 1: issues.append(f"{alg} missed path")
        pc, pl = int(r["path_cost"]), int(r["path_length"])
        if alg in ("dijkstra", "astar") and pc != mc: issues.append(f"{alg} cost {pc} != optimal {mc}")
        if alg == "bfs" and pl != mv: issues.append(f"bfs moves {pl} != {mv}")
        if pc < mc or pl < mv: issues.append(f"{alg} beats the optimum (impossible)")
        if cfg["terrain"] == 0 and pc != pl: issues.append(f"{alg} unit grid cost != length")
    return (cfg["csv"], gi, issues)

if __name__ == "__main__":
    jobs, summary = [], {}
    for cfg in configs():
        df = pd.read_csv(os.path.join(RAW, cfg["csv"]), dtype={"grid_hash": str})
        summary[cfg["csv"]] = {"grids_csv": df.grid_index.nunique(), "grids_cmd": cfg["grids"],
                               "rows_per_grid": len(df) // max(df.grid_index.nunique(), 1)}
        for gi, part in df.groupby("grid_index"):
            jobs.append((cfg, int(gi), part.to_dict("records")))
    print("grids to verify:", len(jobs), flush=True)
    with Pool(10) as p:
        results = p.map(check_grid, jobs, chunksize=4)
    bad = [(c, g, i) for c, g, i in results if i]
    per = {}
    for c, g, i in results:
        per.setdefault(c, [0, 0]); per[c][0] += 1; per[c][1] += bool(i)
    for c in sorted(per): print(f"{c:42s} grids={per[c][0]:4d} with_issues={per[c][1]}")
    print("TOTAL grids:", len(results), "| grids with any discrepancy:", len(bad))
    for b in bad[:20]: print("  ", b)
    sys.exit(1 if bad else 0)
