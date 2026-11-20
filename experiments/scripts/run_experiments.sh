#!/usr/bin/env bash
# Runs the benchmark experiments defined in experiments/plan.md using the
# real `pathfinding --benchmark` CLI. Nothing here changes the benchmark;
# it only invokes it with fixed, recorded configurations.
#
# Usage (from the repository root):
#   BIN=build/pathfinding experiments/scripts/run_experiments.sh [A B C D E F G]
# Environment:
#   BIN    path to a Release build of pathfinding   (default: build/pathfinding)
#   CPU    logical CPU for taskset pinning           (default: 2)
#   OUT    raw output directory                      (default: experiments/raw)
#   FORCE  1 = delete and re-run an experiment that already has raw data
set -uo pipefail

BIN=${BIN:-build/pathfinding}
CPU=${CPU:-2}
OUT=${OUT:-experiments/raw}
FORCE=${FORCE:-0}
SEED=1000

[ -x "$BIN" ] || { echo "error: $BIN is not an executable (build it first)" >&2; exit 1; }
command -v taskset >/dev/null || { echo "error: taskset not found" >&2; exit 1; }
mkdir -p "$OUT"
LOG="$OUT/commands.log"

# bench EXP NAME PIN args...   (PIN = pin | nopin)
bench() {
    local exp=$1 name=$2 pin=$3
    shift 3
    local dir="$OUT/$exp"
    mkdir -p "$dir"
    local base="$dir/$name"
    local -a cmd=("$BIN" --benchmark "$@" --quiet
                  --csv "$base.runs.csv" --grid-csv "$base.grid.csv"
                  --summary-csv "$base.summary.csv" --paired-csv "$base.paired.csv")
    [ "$pin" = pin ] && cmd=(taskset -c "$CPU" "${cmd[@]}")
    local start end rc
    start=$(date +%s.%N)
    "${cmd[@]}" 2> "$base.stderr.txt"
    rc=$?
    end=$(date +%s.%N)
    printf '%s\texit=%d\twall_s=%.2f\t%s\n' "$(date -u +%FT%TZ)" "$rc" "$(echo "$end - $start" | bc)" \
        "${cmd[*]}" >> "$LOG"
    if [ $rc -eq 2 ] || [ $rc -eq 1 ]; then
        echo "FATAL: $exp/$name exited $rc (see $base.stderr.txt)" >&2
        exit $rc
    fi
    [ -s "$base.stderr.txt" ] || rm -f "$base.stderr.txt"
    gzip -9 -f "$base.runs.csv"
    printf '  %-34s exit=%d\n' "$exp/$name" "$rc"
}

prepare() {   # prepare EXP -> 0 to run, 1 to skip
    local dir="$OUT/$1"
    if [ -d "$dir" ]; then
        if [ "$FORCE" = 1 ]; then rm -rf "$dir"; else
            echo "skip $1: $dir exists (FORCE=1 to re-run)"; return 1; fi
    fi
    echo "== experiment $1"
    return 0
}

ALL="--algorithm all"

expA() {
    prepare A || return 0
    bench A baseline pin $ALL --size 100x100 --density 30 --placement reachable --seed $SEED --grids 50 --runs 12 --warmup 1
}

expB() {
    prepare B || return 0
    for d in 0 10 20 30 35 40 45 50; do
        bench B "reachable_d$d" pin $ALL --size 100x100 --density $d --placement reachable --seed $SEED --grids 50 --runs 12 --warmup 1
    done
    for d in 0 10 20 30 35 40 45 50; do
        bench B "corners_d$d" pin $ALL --size 100x100 --density $d --placement corners --seed $SEED --grids 50 --runs 4 --warmup 1
    done
}

expC() {
    prepare C || return 0
    for t in 0 10 30 50 100; do
        bench C "terrain_p$t" pin $ALL --size 100x100 --density 20 --terrain $t --placement reachable --seed $SEED --grids 50 --runs 12 --warmup 1
    done
}

expD() {
    prepare D || return 0
    # size grids runs
    for spec in "50 50 12" "100 50 12" "200 30 12" "400 20 8" "800 10 4" "1600 5 4"; do
        set -- $spec
        bench D "reachable_n$1" pin $ALL --size "$1x$1" --density 20 --placement reachable --seed $SEED --grids "$2" --runs "$3" --warmup 1
    done
    for spec in "50 20" "100 20" "200 20" "400 20" "800 20" "1600 5"; do
        set -- $spec
        bench D "corners_n$1" pin $ALL --size "$1x$1" --density 20 --placement corners --seed $SEED --grids "$2" --runs 4 --warmup 1
    done
}

expE() {
    prepare E || return 0
    for t in 0 30; do
        bench E "astar_dijkstra_p$t" pin --algorithm dijkstra,astar --size 200x200 --density 20 --terrain $t --placement reachable --seed $SEED --grids 100 --runs 12 --warmup 1
    done
}

expF() {
    prepare F || return 0
    for d in 10 20 30 40; do
        for p in corners random reachable; do
            bench F "${p}_d$d" pin $ALL --size 100x100 --density $d --placement $p --seed $SEED --grids 100 --runs 4 --warmup 1
        done
    done
}

expG() {
    prepare G || return 0
    local a=(--size 100x100 --density 30 --placement reachable --grids 50 --runs 12 --warmup 1)
    bench G rep1 pin $ALL "${a[@]}" --seed $SEED
    bench G rep2 pin $ALL "${a[@]}" --seed $SEED
    bench G unpinned nopin $ALL "${a[@]}" --seed $SEED
    bench G seed5000 pin $ALL "${a[@]}" --seed 5000
}

EXPS=${*:-A B C D E F G}
echo "binary: $BIN   pinned CPU: $CPU   output: $OUT"
for e in $EXPS; do
    case $e in
        A) expA ;; B) expB ;; C) expC ;; D) expD ;; E) expE ;; F) expF ;; G) expG ;;
        *) echo "unknown experiment $e" >&2; exit 1 ;;
    esac
done
echo "done; command log: $LOG"
