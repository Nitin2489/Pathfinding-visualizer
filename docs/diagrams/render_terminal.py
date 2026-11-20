#!/usr/bin/env python3
"""Renders docs/images/visualizer.png and docs/images/menu.png from the REAL
output of the interactive program.

The script runs the built executable with scripted keyboard input, captures
its ANSI output, and draws exactly those characters and background colours
(Windows Terminal "Campbell" palette). Nothing is drawn by hand.

Usage:  python3 docs/diagrams/render_terminal.py build/pathfinding
Needs CairoSVG for PNG output (pip install cairosvg).
"""
import html
import os
import re
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.normpath(os.path.join(HERE, "..", "images"))

SEED = "7"
# maze (full grid), random terrain, Dijkstra, Enter, quit
SCRIPT = "9\n0 0\n10\n1\n\n7\n"

BG = "#0C0C0C"
FG = "#CCCCCC"
PALETTE = {41: "#C50F1F", 42: "#13A10E", 43: "#C19C00", 44: "#0037DA", 45: "#881798", 46: "#3A96DD",
           47: "#CCCCCC", 103: "#F9F1A5"}
GLYPH_ON = {"#CCCCCC": "#0C0C0C", "#F9F1A5": "#0C0C0C"}  # dark glyphs on light backgrounds
MONO = "DejaVu Sans Mono, Liberation Mono, monospace"
SIZE = 16
CW = SIZE * 0.602  # DejaVu Sans Mono advance width
LH = 21


def parse(text):
    """ANSI text -> list of lines, each a list of (char, background or None)."""
    lines, cur, bg = [], [], None
    i = 0
    while i < len(text):
        if text[i] == "\x1b":
            m = re.match(r"\x1b\[([0-9;]*)([A-Za-z])", text[i:])
            if m:
                if m.group(2) == "m":
                    for code in (m.group(1) or "0").split(";"):
                        c = int(code or 0)
                        bg = None if c == 0 else PALETTE.get(c, bg)
                i += m.end()
                continue
        if text[i] == "\n":
            lines.append(cur)
            cur = []
        else:
            cur.append((text[i], bg))
        i += 1
    if cur:
        lines.append(cur)
    return lines


def render(lines, title, path):
    import cairosvg
    cols = max(len(l) for l in lines)
    pad, bar = 18, 34
    w = int(pad * 2 + cols * CW)
    h = int(bar + pad * 2 + len(lines) * LH)
    parts = [f'<rect width="{w}" height="{h}" rx="10" fill="{BG}"/>',
             f'<path d="M0,10 a10,10 0 0 1 10,-10 h{w-20} a10,10 0 0 1 10,10 v{bar-10} h-{w} z" fill="#2B2B2B"/>']
    for k, c in enumerate(("#FF5F57", "#FEBC2E", "#28C840")):
        parts.append(f'<circle cx="{18 + k * 20}" cy="17" r="6" fill="{c}"/>')
    parts.append(f'<text x="{w/2}" y="22" font-family="{MONO}" font-size="12.5" fill="#9AA5B1" '
                 f'text-anchor="middle">{html.escape(title)}</text>')
    for r, line in enumerate(lines):
        y0 = bar + pad + r * LH
        for c, (ch, bg) in enumerate(line):
            x0 = pad + c * CW
            if bg:
                parts.append(f'<rect x="{x0:.2f}" y="{y0}" width="{CW + 0.4:.2f}" height="{LH}" fill="{bg}"/>')
            if ch != " ":
                fill = "#F2F2F2" if bg in ("#C50F1F", "#13A10E") else GLYPH_ON.get(bg, FG)
                parts.append(f'<text x="{x0:.2f}" y="{y0 + LH - 5}" font-family="{MONO}" font-size="{SIZE}" '
                             f'fill="{fill}">{html.escape(ch)}</text>')
    svg = (f'<svg xmlns="http://www.w3.org/2000/svg" width="{w}" height="{h}" viewBox="0 0 {w} {h}">'
           + "".join(parts) + "</svg>")
    cairosvg.svg2png(bytestring=svg.encode(), write_to=path, output_width=w * 2)
    print("wrote", os.path.relpath(path))


def main():
    exe = sys.argv[1] if len(sys.argv) > 1 else "build/pathfinding"
    out = subprocess.run([exe, "--seed", SEED], input=SCRIPT.encode(), capture_output=True, timeout=120,
                         check=True).stdout.decode("utf-8", "replace")

    # Menu: everything before the first prompt.
    menu = out[: out.index("Enter your choice: ") + len("Enter your choice: ")]
    render(parse(menu), f"pathfinding --seed {SEED}", os.path.join(OUT, "menu.png"))

    # Final Dijkstra frame: last clear-screen frame, up to the Enter prompt.
    frames = out.split("\x1b[H\x1b[J")
    final = next(f for f in reversed(frames) if "Dijkstra's Algorithm\nPath found!" in f)
    final = final[: final.index("Press Enter to continue...") + len("Press Enter to continue...")]
    render(parse(final), f"pathfinding --seed {SEED}  ·  maze + random terrain + Dijkstra",
           os.path.join(OUT, "visualizer.png"))


if __name__ == "__main__":
    main()
