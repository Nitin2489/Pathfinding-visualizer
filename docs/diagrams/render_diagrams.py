#!/usr/bin/env python3
"""Renders docs/images/architecture.png and docs/images/benchmark-pipeline.png.

The diagrams are laid out by hand (fixed coordinates) and written as SVG, so
the output is deterministic and every label and arrow is reviewable here.
Their content follows the source: module boundaries from the #include graph
and CMakeLists.txt, control flow from main.cpp and runBenchmark().

Usage:  python3 docs/diagrams/render_diagrams.py [--svg-only]
PNG conversion needs CairoSVG (pip install cairosvg). Fonts: Liberation Sans
or DejaVu Sans.
"""
import html
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.normpath(os.path.join(HERE, "..", "images"))

FONT = "Liberation Sans, DejaVu Sans, Arial, Helvetica, sans-serif"
MONO = "DejaVu Sans Mono, Liberation Mono, monospace"
INK = "#1F2933"
MUTED = "#52606D"
LINE = "#7B8794"
EXE = "#243B53"     # header: executable components (main.cpp, visualizer.cpp)
CORE = "#2563EB"    # header: pathfinding_core library components
TIMED = "#D97706"   # the timed region
FAIL = "#B91C1C"


class Svg:
    def __init__(self, w, h):
        self.w, self.h, self.parts = w, h, []

    def add(self, s):
        self.parts.append(s)

    def text(self, x, y, s, size=14, weight="normal", fill=INK, anchor="start", family=FONT, italic=False,
             rotate=None):
        style = ' font-style="italic"' if italic else ""
        tr = f' transform="rotate({rotate} {x} {y})"' if rotate is not None else ""
        self.add(f'<text x="{x}" y="{y}" font-family="{family}" font-size="{size}" font-weight="{weight}" '
                 f'fill="{fill}" text-anchor="{anchor}"{style}{tr}>{html.escape(s)}</text>')

    def rect(self, x, y, w, h, fill="#FFFFFF", stroke="#CBD2D9", sw=1.2, rx=10, dash=None, opacity=1.0):
        d = f' stroke-dasharray="{dash}"' if dash else ""
        self.add(f'<rect x="{x}" y="{y}" width="{w}" height="{h}" rx="{rx}" fill="{fill}" fill-opacity="{opacity}" '
                 f'stroke="{stroke}" stroke-width="{sw}"{d}/>')

    def box(self, x, y, w, h, title, lines=(), header=CORE, tag=None, size=13.5, border="#CBD2D9", sw=1.2):
        """Card with a coloured title band and body lines."""
        self.add(f'<rect x="{x+2}" y="{y+3}" width="{w}" height="{h}" rx="10" fill="#0F172A" fill-opacity="0.06"/>')
        self.rect(x, y, w, h, stroke=border, sw=sw)
        self.add(f'<path d="M{x},{y+10} a10,10 0 0 1 10,-10 h{w-20} a10,10 0 0 1 10,10 v20 h-{w} z" fill="{header}"/>')
        self.text(x + 12, y + 20, title, size=14.5, weight="bold", fill="#FFFFFF")
        if tag:
            self.text(x + w - 10, y + 20, tag, size=11.5, fill="#DBEAFE", anchor="end", family=MONO)
        for i, line in enumerate(lines):
            if isinstance(line, tuple):
                txt, opts = line
            else:
                txt, opts = line, {}
            self.text(x + 12, y + 50 + i * 19, txt, size=opts.get("size", size), fill=opts.get("fill", INK),
                      weight=opts.get("weight", "normal"), family=opts.get("family", FONT),
                      italic=opts.get("italic", False))

    def chip(self, x, y, w, h, label, fill, ink="#FFFFFF", size=13.5):
        self.rect(x, y, w, h, fill=fill, stroke=fill, rx=h / 2)
        self.text(x + w / 2, y + h / 2 + 5, label, size=size, weight="bold", fill=ink, anchor="middle")

    def group(self, x, y, w, h, label, fill, stroke):
        self.rect(x, y, w, h, fill=fill, stroke=stroke, sw=1.4, rx=14, dash="6 5")
        self.text(x + 14, y + 20, label, size=12, weight="bold", fill=stroke)

    def arrow(self, pts, color=LINE, dashed=False, width=1.8, label=None, label_at=None, label_anchor="start",
              label_rotate=None, label_color=None):
        d = "M" + " L".join(f"{px},{py}" for px, py in pts)
        dash = ' stroke-dasharray="7 5"' if dashed else ""
        marker = "arrowDark" if color == LINE else ("arrowTimed" if color == TIMED else
                                                    "arrowFail" if color == FAIL else "arrowDark")
        self.add(f'<path d="{d}" fill="none" stroke="{color}" stroke-width="{width}"{dash} '
                 f'stroke-linejoin="round" marker-end="url(#{marker})"/>')
        if label:
            lx, ly = label_at
            self.text(lx, ly, label, size=12.5, fill=label_color or MUTED, anchor=label_anchor, italic=True,
                      rotate=label_rotate)

    def render(self):
        defs = "".join(
            f'<marker id="{name}" viewBox="0 0 10 10" refX="9" refY="5" markerWidth="8" markerHeight="8" '
            f'orient="auto-start-reverse"><path d="M0,0 L10,5 L0,10 z" fill="{c}"/></marker>'
            for name, c in (("arrowDark", LINE), ("arrowTimed", TIMED), ("arrowFail", FAIL)))
        return (f'<svg xmlns="http://www.w3.org/2000/svg" width="{self.w}" height="{self.h}" '
                f'viewBox="0 0 {self.w} {self.h}"><defs>{defs}</defs>'
                f'<rect width="{self.w}" height="{self.h}" fill="#FFFFFF"/>' + "".join(self.parts) + "</svg>")


# --------------------------------------------------------------------------- architecture

def architecture():
    s = Svg(1700, 1090)
    s.text(40, 46, "Pathfinding Visualizer — system architecture", size=26, weight="bold")
    s.text(40, 72, "Components, dependencies and data flow of the pathfinding executable (C++17)", size=15,
           fill=MUTED)

    # Layer 1-2: user, entry point, CLI
    s.box(480, 96, 320, 62, "User · terminal", ["interactive menu or command-line flags"], header="#475569")
    s.box(400, 192, 480, 88, "pathfinding executable", [
        "main.cpp: parses arguments, selects the mode,",
        "wires benchmark outputs, sets exit codes 0/1/2/3"], header=EXE, tag="main.cpp")
    s.box(1200, 192, 460, 88, "Command-line parsing", [
        "flags, suites, defaults, config validation,",
        "output-path collision preflight"], header=CORE, tag="cli.cpp")
    s.arrow([(640, 158), (640, 190)])
    s.arrow([(880, 236), (1198, 236)], label="parse & validate argv", label_at=(960, 228))

    # Layer 3: modes
    s.group(40, 316, 480, 232, "INTERACTIVE MODE (default)", "#F0FDFA", "#0F766E")
    s.box(60, 350, 440, 182, "Interactive visualizer", [
        "menu: run algorithm, random walls, maze,",
        "random terrain, clear grid, animation speed",
        "runs the search first, then replays its",
        "expansion trace as an ANSI animation",
        "prints metrics incl. core & end-to-end time",
        ("rendering never overlaps the timed search", {"italic": True, "fill": MUTED})],
        header=EXE, tag="visualizer.cpp")

    s.group(560, 316, 600, 232, "BENCHMARK MODE (--benchmark)", "#EEF2FF", "#4338CA")
    s.text(1146, 336, "benchmark.cpp", size=11.5, fill="#4338CA", anchor="end", family=MONO)
    s.box(578, 350, 276, 88, "Scenario generation", [
        "seeded grids, separate RNG streams",
        "placement: corners · random · reachable"], size=12.5)
    s.box(866, 350, 276, 88, "Run scheduler", [
        "warm-up, then interleaved timed runs",
        "algorithm order rotated every run"], size=12.5)
    s.box(866, 448, 276, 88, "Per-grid statistics", [
        "over valid runs: median, mean, stddev,",
        "CV, timer-floor flag (level 1)"], size=12.5)
    s.box(578, 448, 276, 88, "Aggregation", [
        "per-grid medians across grids (level 2)",
        "paired per-grid ratios between algorithms"], size=12.5)
    s.arrow([(854, 394), (864, 394)])
    s.arrow([(1004, 438), (1004, 446)])
    s.arrow([(866, 492), (856, 492)])

    s.arrow([(470, 280), (470, 348)], label="default", label_at=(478, 304))
    s.arrow([(780, 280), (780, 314)], label="--benchmark", label_at=(788, 302))

    # Layer 4: search + validation
    s.box(40, 610, 540, 172, "Search layer", [
        "", "",
        "Dijkstra and A* share one best-first core (priority",
        "snapshots, stale entries skipped); 4-directional moves",
        "returns SearchResult: path + SearchMetrics (discovered,",
        "expanded, length, cost, peak frontier, core & e2e time)"],
        header=CORE, tag="search.cpp", size=13)
    for i, (name, col) in enumerate((("Dijkstra", "#1D4ED8"), ("A*", "#7C3AED"), ("BFS", "#0E7490"),
                                     ("DFS", "#B45309"))):
        s.chip(56 + i * 128, 648, 116, 28, name, col)

    s.box(620, 610, 540, 172, "Independent validation", [
        "reference BFS on (row, col): reachability, fewest moves",
        "Dial bucket queue: minimum weighted cost",
        "per-run checks: path structure, recomputed length",
        "and cost, optimality per algorithm, metric invariants",
        "component labels for reachable endpoint placement",
        ("no calls into search.cpp — shares only Grid and result types", {"italic": True, "fill": MUTED})],
        header=CORE, tag="validation.cpp", size=13)

    s.arrow([(280, 532), (280, 608)], label="runSearch + trace", label_at=(290, 584))
    s.arrow([(586, 548), (586, 566), (560, 566), (560, 608)], label="timed runSearch", label_at=(552, 590),
            label_anchor="end")
    s.arrow([(600, 548), (600, 870)], label="builds seeded grids", label_at=(611, 840), label_rotate=None)
    s.arrow([(900, 548), (900, 608)], label="reference + validateRun (untimed)", label_at=(910, 584))

    # Layer 5: grid model (foundation)
    s.box(40, 872, 1120, 96, "Grid & terrain model", [
        "cells · walls · terrain costs (empty 1, sand 3, water 5, mountain 10) · start / end · bounds-checked access",
        "platform-independent seeded RNG · random walls · recursive-division maze · random terrain · FNV-1a fingerprint"],
        header=CORE, tag="grid.cpp", size=13.5)
    s.arrow([(300, 782), (300, 870)], label="reads cells & costs", label_at=(310, 832))
    s.arrow([(900, 782), (900, 870)], label="reads cells", label_at=(910, 832))
    # visualizer -> grid generation via the left channel
    s.arrow([(60, 470), (24, 470), (24, 928), (38, 928)])
    s.text(17, 700, "generates walls / maze / terrain", size=12.5, fill=MUTED, anchor="middle", italic=True,
           rotate=-90)

    # Right column: results & persistence
    s.group(1200, 316, 460, 466, "RESULTS & PERSISTENCE (benchmark mode)", "#F8FAFC", "#475569")
    s.box(1220, 350, 420, 108, "Reporting", [
        "per-run, per-grid, summary and paired CSV",
        "RFC 4180, empty fields for missing values",
        "console summary tables"], header=CORE, tag="report.cpp")
    s.box(1220, 472, 420, 108, "Metadata", [
        "timer calibration (overhead, granularity)",
        "build, git commit, OS/CPU, configuration",
        "JSON sidecar per CSV"], header=CORE, tag="metadata.cpp")
    s.box(1220, 594, 420, 168, "Output files", [
        "write PATH.partial, rename on success",
        "per-run CSV streamed, flushed per grid",
        "refuse overwrite without --force",
        "clean up staging files on failure",
        ("no multi-file transaction", {"italic": True, "fill": MUTED})], header=CORE, tag="output.cpp")
    s.arrow([(860, 280), (860, 300), (1430, 300), (1430, 314)], label="results → reports, metadata, files",
            label_at=(1000, 294))
    s.arrow([(1160, 400), (1198, 400)], dashed=True)
    s.arrow([(1652, 280), (1652, 678), (1642, 678)])
    s.text(1676, 500, "path-collision preflight", size=12.5, fill=MUTED, anchor="middle", italic=True, rotate=90)

    s.box(1220, 872, 420, 96, "Artifacts", [
        ("runs.csv · grid.csv · summary.csv · paired.csv", {"family": MONO, "size": 12.5}),
        ("*.csv.meta.json sidecars", {"family": MONO, "size": 12.5}),
        "+ summary tables on the terminal"], header="#475569")
    s.arrow([(1430, 762), (1430, 870)], label="commit", label_at=(1440, 822))

    # Legend
    y = 1018
    s.rect(40, y - 18, 22, 16, fill=EXE, stroke=EXE, rx=4)
    s.text(70, y - 5, "executable: main.cpp, visualizer.cpp", size=13)
    s.rect(370, y - 18, 22, 16, fill=CORE, stroke=CORE, rx=4)
    s.text(400, y - 5, "pathfinding_core static library (also linked by the unit tests)", size=13)
    s.arrow([(860, y - 10), (920, y - 10)])
    s.text(930, y - 5, "calls / control flow", size=13)
    s.arrow([(1110, y - 10), (1170, y - 10)], dashed=True)
    s.text(1180, y - 5, "data passed back through callbacks", size=13)
    s.text(40, 1060, "Unit tests (tests/test_main.cpp) and end-to-end CLI checks (tests/cli_checks.cmake) "
                     "exercise the library and the executable through CTest.", size=12.5, fill=MUTED)
    return s


# --------------------------------------------------------------------------- benchmark pipeline

def pipeline():
    s = Svg(1700, 900)
    s.text(40, 46, "Benchmark pipeline", size=26, weight="bold")
    s.text(40, 72, "What one `pathfinding --benchmark` invocation does, in order (runBenchmark() and main.cpp)",
           size=15, fill=MUTED)

    cols = [(40, "1 · SETUP (once)"), (370, "2 · PER GRID"), (700, "3 · PER RUN × ALGORITHM"),
            (1030, "4 · PER GRID (after its runs)"), (1360, "5 · AGGREGATE & REPORT (once)")]
    for x, label in cols:
        s.text(x, 112, label, size=13, weight="bold", fill="#4338CA")

    # nested loop frames
    s.group(352, 126, 984, 660, "", "#F5F7FF", "#6366F1")
    s.text(366, 776, "loop: every grid of every scenario  (seed = base seed + grid index)", size=12.5,
           weight="bold", fill="#4338CA")
    s.group(684, 140, 322, 592, "", "#FFFFFF", "#94A3B8")
    s.text(698, 722, "loop: runs × algorithms, order rotated each run", size=12, weight="bold", fill="#475569")

    w = 300
    # 1 setup
    s.box(40, 140, w, 120, "Configuration", [
        "flags or --suite, defaults:",
        "10 grids × 12 runs, 1 warm-up",
        "seed-range check, methodology warnings"], header=CORE, size=13)
    s.box(40, 280, w, 120, "Output preflight", [
        "destination, staging and metadata",
        "paths must not collide",
        "no overwrite without --force"], header=CORE, size=13)
    s.box(40, 420, w, 100, "Stage outputs", [
        ("open *.partial staging files", {}),
        "(final files untouched until commit)"], header=CORE, size=13)
    s.box(40, 540, w, 100, "Timer calibration", [
        "steady_clock overhead & granularity",
        "→ near-floor threshold; environment info"], header=CORE, size=13)
    for y0, y1 in ((260, 278), (400, 418), (520, 538)):
        s.arrow([(190, y0), (190, y1)])

    # 2 per grid
    s.box(370, 150, w, 190, "Generate grid", [
        "stream 0: walls or maze",
        "stream 1: endpoints",
        "stream 2: terrain",
        "placement: corners · random ·",
        "reachable (connected pairs only,",
        "≤ 10,000 draws)"], header=CORE, size=13)
    s.box(370, 360, w, 140, "Fingerprint & reference", [
        "FNV-1a grid hash",
        "independent BFS: reachability,",
        "fewest moves; Dial: minimum cost",
        ("untimed", {"italic": True, "fill": MUTED})], header=CORE, size=13)
    s.box(370, 520, w, 90, "Warm-up", ["each algorithm, untimed,", "results discarded"], header=CORE, size=13)
    s.arrow([(520, 340), (520, 358)])
    s.arrow([(520, 500), (520, 518)])
    s.box(370, 640, w, 96, "Generation failure", [
        "no connected pair found:", "grid skipped, reported → exit 3"], header=FAIL, size=13)
    s.arrow([(370, 250), (361, 250), (361, 688), (368, 688)], color=FAIL, dashed=True)

    # 3 per run
    s.box(700, 156, 290, 150, "runSearch()", [
        ("ONLY TIMED REGION", {"weight": "bold", "fill": TIMED}),
        "core: after per-cell setup →",
        "path rebuilt",
        "end-to-end: whole call incl.",
        "allocation and freeing"], header=TIMED, size=13, border=TIMED, sw=2.2)
    s.box(700, 326, 290, 150, "validateRun()", [
        "path, recomputed length & cost,",
        "reachability, optimality per",
        "algorithm, metric invariants,",
        "determinism vs first valid run",
        ("untimed", {"italic": True, "fill": MUTED})], header=CORE, size=13)
    s.box(700, 496, 290, 110, "RunRecord", [
        ("→ runs.csv.partial", {"family": MONO, "size": 12.5}),
        "streamed; flushed after each grid"], header=CORE, size=13)
    s.box(700, 622, 290, 88, "Invalid run", [
        "excluded from statistics,", "row kept, reported → exit 2"], header=FAIL, size=13)
    s.arrow([(845, 306), (845, 324)])
    s.arrow([(845, 476), (845, 494)])
    s.arrow([(700, 400), (692, 400), (692, 666), (698, 666)], color=FAIL, dashed=True)
    # warm-up -> first timed run
    s.arrow([(670, 565), (679, 565), (679, 230), (698, 230)])

    # 4 per grid
    s.box(1030, 200, w, 196, "GridSummary (level 1)", [
        "per algorithm, valid runs only:",
        "core & e2e median, mean, stddev,",
        "min, max; core CV (noise)",
        "near-timer-floor flag",
        "cost ratio vs reference minimum",
        "grid fingerprint re-checked"], header=CORE, size=13)
    s.arrow([(1006, 300), (1028, 300)])

    # 5 aggregate & report
    s.box(1360, 140, w, 160, "Across-grid (level 2)", [
        "distribution of per-grid medians:",
        "median, mean, Q1, Q3, IQR (n = grids)",
        "also solvable grids only",
        "solvability & success rates"], header=CORE, size=13)
    s.box(1360, 320, w, 128, "Paired comparisons", [
        "per grid: median time a ÷ b",
        "median, IQR, geomean, share faster",
        "invalid pairs excluded & counted"], header=CORE, size=13)
    s.box(1360, 468, w, 128, "Reports & metadata", [
        ("grid.csv · summary.csv · paired.csv", {"family": MONO, "size": 12}),
        ("*.meta.json", {"family": MONO, "size": 12}),
        "console tables"], header=CORE, size=13)
    s.box(1360, 616, w, 120, "Commit & exit", [
        "rename each .partial → final",
        "exit 0 ok · 1 I/O · 2 check failed",
        "3 generation failures"], header="#475569", size=13)
    for y0, y1 in ((300, 318), (448, 466), (596, 614)):
        s.arrow([(1510, y0), (1510, y1)])
    s.arrow([(1336, 220), (1358, 220)])

    # setup (after timer calibration) -> first grid
    s.arrow([(340, 590), (346, 590), (346, 136), (520, 136), (520, 148)])

    # legend
    y = 836
    s.rect(40, y - 14, 22, 16, fill=TIMED, stroke=TIMED, rx=4)
    s.text(70, y, "timed region (the only code inside the clock)", size=13)
    s.rect(420, y - 14, 22, 16, fill=FAIL, stroke=FAIL, rx=4)
    s.text(450, y, "failure path (reported, never silently dropped)", size=13)
    s.rect(820, y - 14, 22, 16, fill=CORE, stroke=CORE, rx=4)
    s.text(850, y, "pathfinding_core library code", size=13)
    s.text(40, 870, "Details: docs/benchmark-methodology.md", size=12.5, fill=MUTED)
    return s


def main():
    os.makedirs(OUT, exist_ok=True)
    svg_only = "--svg-only" in sys.argv
    for name, fn in (("architecture", architecture), ("benchmark-pipeline", pipeline)):
        svg = fn().render()
        if svg_only:
            path = os.path.join(OUT, name + ".svg")
            with open(path, "w", encoding="utf-8") as f:
                f.write(svg)
        else:
            import cairosvg  # imported lazily so --svg-only needs no dependencies
            path = os.path.join(OUT, name + ".png")
            cairosvg.svg2png(bytestring=svg.encode("utf-8"), write_to=path, output_width=fn().w * 3 // 2)
        print("wrote", os.path.relpath(path))


if __name__ == "__main__":
    main()
