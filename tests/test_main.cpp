// Self-contained test suite (no external framework). Run via `ctest` or
// directly: ./build/pathfinding_tests [name-filter]
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <functional>
#include <iostream>
#include <limits>
#include <set>
#include <stdexcept>
#include <sstream>
#include <string>
#include <vector>

#include <cmath>
#include <filesystem>
#include <map>

#include "benchmark.h"
#include "cli.h"
#include "grid.h"
#include "metadata.h"
#include "output.h"
#include "report.h"
#include "search.h"
#include "validation.h"

// ------------------------------------------------------------- harness
namespace {

struct TestCase {
    const char* name;
    std::function<void()> fn;
};

std::vector<TestCase>& registry() {
    static std::vector<TestCase> tests;
    return tests;
}

int g_failures = 0;

struct Registrar {
    Registrar(const char* name, std::function<void()> fn) { registry().push_back({name, std::move(fn)}); }
};

#define TEST(name)                                       \
    static void name();                                  \
    static const Registrar registrar_##name(#name, name); \
    static void name()

#define CHECK(cond)                                                                       \
    do {                                                                                  \
        if (!(cond)) {                                                                    \
            ++g_failures;                                                                 \
            std::cerr << "  " << __FILE__ << ":" << __LINE__ << ": CHECK(" #cond ") failed\n"; \
        }                                                                                 \
    } while (0)

#define CHECK_EQ(a, b)                                                                              \
    do {                                                                                            \
        const auto va_ = (a);                                                                       \
        const auto vb_ = (b);                                                                       \
        if (!(va_ == vb_)) {                                                                        \
            ++g_failures;                                                                           \
            std::cerr << "  " << __FILE__ << ":" << __LINE__ << ": CHECK_EQ(" #a ", " #b ") failed: " << va_ \
                      << " != " << vb_ << "\n";                                                     \
        }                                                                                           \
    } while (0)

// ------------------------------------------------------------- helpers

// Independent oracle: minimum entry cost from start to every cell by
// repeated relaxation (Bellman-Ford style). Slow but obviously correct.
long long bruteForceCost(const Grid& g, bool unitCost) {
    const long long inf = std::numeric_limits<long long>::max() / 4;
    std::vector<long long> d(static_cast<std::size_t>(g.cellCount()), inf);
    const Cell s = g.getStartPos();
    const Cell e = g.getEndPos();
    if (!g.isPassable(s.row, s.col) || !g.isPassable(e.row, e.col)) return -1;
    d[g.index(s.row, s.col)] = 0;
    bool changed = true;
    while (changed) {
        changed = false;
        for (int r = 0; r < g.rows(); ++r) {
            for (int c = 0; c < g.cols(); ++c) {
                if (!g.isPassable(r, c)) continue;
                const int dr[] = {-1, 0, 1, 0};
                const int dc[] = {0, 1, 0, -1};
                for (int k = 0; k < 4; ++k) {
                    const int pr = r + dr[k];
                    const int pc = c + dc[k];
                    if (!g.isPassable(pr, pc) || d[g.index(pr, pc)] >= inf) continue;
                    const long long cand = d[g.index(pr, pc)] + (unitCost ? 1 : g.costAt(g.index(r, c)));
                    if (cand < d[g.index(r, c)]) {
                        d[g.index(r, c)] = cand;
                        changed = true;
                    }
                }
            }
        }
    }
    const long long v = d[g.index(e.row, e.col)];
    return v >= inf ? -1 : v;
}

int reachableCount(const Grid& g) {
    std::vector<char> seen(static_cast<std::size_t>(g.cellCount()), 0);
    std::vector<Cell> stack{g.getStartPos()};
    seen[g.index(g.getStartPos().row, g.getStartPos().col)] = 1;
    int n = 0;
    while (!stack.empty()) {
        const Cell c = stack.back();
        stack.pop_back();
        ++n;
        const Cell nbs[] = {{c.row - 1, c.col}, {c.row, c.col + 1}, {c.row + 1, c.col}, {c.row, c.col - 1}};
        for (const Cell& nb : nbs) {
            if (g.isPassable(nb.row, nb.col) && !seen[g.index(nb.row, nb.col)]) {
                seen[g.index(nb.row, nb.col)] = 1;
                stack.push_back(nb);
            }
        }
    }
    return n;
}

// Invariants every result must satisfy, for any algorithm and grid.
void checkInvariants(const Grid& g, Algorithm a, const SearchResult& r, const ExpansionTrace& trace) {
    const SearchMetrics& m = r.metrics;
    CHECK(m.executionTimeMs >= 0.0);
    CHECK(m.searchTimeMs >= 0.0);
    CHECK(m.searchTimeMs <= m.executionTimeMs);  // core is inside end-to-end
    CHECK(m.nodesExpanded <= m.nodesDiscovered);
    CHECK(m.nodesDiscovered <= g.passableCount());
    CHECK(m.nodesDiscovered >= 1);
    CHECK(m.peakFrontierSize >= 1);
    CHECK_EQ(static_cast<int>(trace.size()), m.nodesExpanded);
    std::set<int> unique;
    for (const Cell& c : trace) unique.insert(g.index(c.row, c.col));
    CHECK_EQ(unique.size(), trace.size());  // no cell expanded twice (stale entries skipped)
    if (m.pathFound) {
        std::string why;
        const bool ok = validatePath(g, r.path, &why);
        if (!ok) std::cerr << "  " << algorithmName(a) << ": " << why << "\n";
        CHECK(ok);
        CHECK_EQ(m.pathLength, static_cast<int>(r.path.size()) - 1);
        CHECK_EQ(m.pathCost, computePathCost(g, r.path));
        std::set<int> cells;
        for (const Cell& c : r.path) cells.insert(g.index(c.row, c.col));
        CHECK_EQ(cells.size(), r.path.size());  // simple path
    } else {
        CHECK(r.path.empty());
        CHECK_EQ(m.pathLength, -1);
        CHECK_EQ(m.pathCost, -1LL);
        // Exhaustive: every reachable cell was expanded.
        CHECK_EQ(m.nodesExpanded, reachableCount(g));
    }
}

SearchResult run(Algorithm a, const Grid& g) {
    ExpansionTrace trace;
    SearchResult r = runSearch(a, g, &trace);
    checkInvariants(g, a, r, trace);
    return r;
}

}  // namespace

// ------------------------------------------------------------- algorithms

TEST(start_equals_destination) {
    const Grid g = Grid::fromStrings({"...", ".S.", "..."});
    for (Algorithm a : allAlgorithms()) {
        const SearchResult r = run(a, g);
        CHECK(r.metrics.pathFound);
        CHECK_EQ(r.metrics.pathLength, 0);
        CHECK_EQ(r.metrics.pathCost, 0LL);
        CHECK_EQ(r.path.size(), std::size_t{1});
        CHECK_EQ(r.metrics.nodesDiscovered, 1);
        CHECK_EQ(r.metrics.nodesExpanded, 0);
        CHECK_EQ(r.metrics.peakFrontierSize, 1);
    }
}

TEST(single_cell_grid) {
    const Grid g = Grid::fromStrings({"S"});
    for (Algorithm a : allAlgorithms()) CHECK_EQ(run(a, g).metrics.pathLength, 0);
}

TEST(straight_corridor_exact_metrics) {
    // S . . . E in a 1x5 corridor: each algorithm discovers all 5 cells and
    // expands the 4 before the goal; the frontier never holds more than 1.
    const Grid g = Grid::fromStrings({"S...E"});
    for (Algorithm a : allAlgorithms()) {
        const SearchResult r = run(a, g);
        CHECK(r.metrics.pathFound);
        CHECK_EQ(r.metrics.pathLength, 4);
        CHECK_EQ(r.metrics.nodesDiscovered, 5);
        CHECK_EQ(r.metrics.nodesExpanded, 4);
        CHECK_EQ(r.metrics.peakFrontierSize, 1);
        CHECK_EQ(r.path.front().col, 0);
        CHECK_EQ(r.path.back().col, 4);
    }
}

TEST(empty_grid_shortest_is_manhattan) {
    for (int rows : {3, 7, 20}) {
        for (int cols : {3, 11, 50}) {
            const Grid g(rows, cols);  // border walls, corners start/end
            const int manhattan = (rows - 3) + (cols - 3);
            for (Algorithm a : allAlgorithms()) {
                const SearchResult r = run(a, g);
                CHECK(r.metrics.pathFound);
                if (a != Algorithm::DFS) CHECK_EQ(r.metrics.pathLength, manhattan);
                else CHECK(r.metrics.pathLength >= manhattan);
            }
        }
    }
}

TEST(path_requiring_turns) {
    const Grid g = Grid::fromStrings({
        "S.#....",
        ".##.##.",
        "......#",
        "#####.E",
    });
    // S(0,0) down to (2,0), right to (2,5), down (3,5), right (3,6) = 2+5+1+1
    for (Algorithm a : allAlgorithms()) {
        const SearchResult r = run(a, g);
        CHECK(r.metrics.pathFound);
        if (a != Algorithm::DFS) CHECK_EQ(r.metrics.pathLength, 9);
    }
}

TEST(narrow_corridor) {
    const Grid g = Grid::fromStrings({
        "#########",
        "#S#.....#",
        "#.#.###.#",
        "#.#.#E#.#",
        "#...#.#.#",
        "#####...#",
        "#########",
    });
    const long long expected = bruteForceCost(g, true);
    CHECK(expected > 0);
    for (Algorithm a : allAlgorithms()) {
        const SearchResult r = run(a, g);
        CHECK(r.metrics.pathFound);
        // Only one route exists, so even DFS must find the optimal one.
        CHECK_EQ(static_cast<long long>(r.metrics.pathLength), expected);
    }
}

TEST(unreachable_destination) {
    const Grid g = Grid::fromStrings({
        "S..#...",
        "...#.E.",
        "...#...",
    });
    for (Algorithm a : allAlgorithms()) {
        const SearchResult r = run(a, g);
        CHECK(!r.metrics.pathFound);
        CHECK_EQ(r.metrics.nodesDiscovered, 9);
        CHECK_EQ(r.metrics.nodesExpanded, 9);
    }
}

TEST(destination_surrounded_by_walls) {
    const Grid g = Grid::fromStrings({"S....", "...#.", "..#E#", "...#."});
    for (Algorithm a : allAlgorithms()) CHECK(!run(a, g).metrics.pathFound);
}

TEST(start_surrounded_by_walls) {
    const Grid g = Grid::fromStrings({".#...", "#S#..", ".#..E"});
    for (Algorithm a : allAlgorithms()) {
        const SearchResult r = run(a, g);
        CHECK(!r.metrics.pathFound);
        CHECK_EQ(r.metrics.nodesDiscovered, 1);
        CHECK_EQ(r.metrics.nodesExpanded, 1);
        CHECK_EQ(r.metrics.peakFrontierSize, 1);
    }
}

TEST(weighted_terrain_dijkstra_astar_avoid_mountains) {
    // Straight line is 4 moves through mountains (cost 1+10+10+10... );
    // the detour is 6 moves of cost 1.
    const Grid g = Grid::fromStrings({
        "SMMME",
        ".....",
    });
    const long long optimal = bruteForceCost(g, false);
    CHECK_EQ(optimal, 6LL);
    for (Algorithm a : {Algorithm::Dijkstra, Algorithm::AStar}) {
        const SearchResult r = run(a, g);
        CHECK_EQ(r.metrics.pathCost, 6LL);
        CHECK_EQ(r.metrics.pathLength, 6);
    }
    // BFS minimises moves, not cost.
    const SearchResult b = run(Algorithm::BFS, g);
    CHECK_EQ(b.metrics.pathLength, 4);
    CHECK_EQ(b.metrics.pathCost, 31LL);  // 10 + 10 + 10 + 1
}

TEST(dijkstra_handles_stale_entries) {
    // Water near the start makes Dijkstra push some cells twice with a
    // cheaper distance later; invariants check no double expansion.
    const Grid g = Grid::fromStrings({
        "SWW.",
        "...W",
        "W..E",
    });
    for (Algorithm a : {Algorithm::Dijkstra, Algorithm::AStar}) {
        const SearchResult r = run(a, g);
        CHECK_EQ(r.metrics.pathCost, bruteForceCost(g, false));
    }
}

TEST(randomized_against_oracle) {
    // Many random grids, sizes and densities: BFS/Dijkstra/A* must match the
    // brute-force oracle exactly; DFS must find a valid path iff one exists.
    int reachable = 0;
    int total = 0;
    for (std::uint64_t seed = 1; seed <= 150; ++seed) {
        Scenario s;
        s.rows = 3 + static_cast<int>(seed % 13);
        s.cols = 3 + static_cast<int>((seed * 7) % 17);
        s.obstacleDensity = static_cast<double>(seed % 6) * 0.1;
        s.terrainDensity = (seed % 3 == 0) ? 0.4 : 0.0;
        s.placement = (seed % 2 == 0) ? Placement::Random : Placement::Corners;
        const Grid g = buildScenarioGrid(s, seed);
        const long long unitOpt = bruteForceCost(g, true);
        const long long costOpt = bruteForceCost(g, false);
        ++total;
        if (unitOpt >= 0) ++reachable;
        for (Algorithm a : allAlgorithms()) {
            const SearchResult r = run(a, g);
            CHECK_EQ(r.metrics.pathFound, unitOpt >= 0);
            if (!r.metrics.pathFound) continue;
            if (a == Algorithm::BFS) CHECK_EQ(static_cast<long long>(r.metrics.pathLength), unitOpt);
            if (isCostOptimal(a)) CHECK_EQ(r.metrics.pathCost, costOpt);
            if (a == Algorithm::DFS) CHECK(r.metrics.pathCost >= costOpt);
        }
    }
    // Make sure the sample actually exercised both outcomes.
    CHECK(reachable > 20);
    CHECK(total - reachable > 5);
}

TEST(astar_expands_no_more_than_dijkstra_on_open_grid) {
    const Grid g(30, 30);
    CHECK(run(Algorithm::AStar, g).metrics.nodesExpanded < run(Algorithm::Dijkstra, g).metrics.nodesExpanded);
}

TEST(validate_path_rejects_bad_paths) {
    const Grid g = Grid::fromStrings({"S.#", "..E"});
    CHECK(validatePath(g, {{0, 0}, {1, 0}, {1, 1}, {1, 2}}));
    std::string why;
    CHECK(!validatePath(g, {}, &why));
    CHECK(!validatePath(g, {{0, 1}, {1, 1}, {1, 2}}, &why));          // wrong start
    CHECK(!validatePath(g, {{0, 0}, {0, 1}, {1, 1}}, &why));          // wrong end
    CHECK(!validatePath(g, {{0, 0}, {0, 1}, {0, 2}, {1, 2}}, &why));  // crosses wall
    CHECK(!validatePath(g, {{0, 0}, {1, 1}, {1, 2}}, &why));          // diagonal
    CHECK(!validatePath(g, {{0, 0}, {1, 0}, {0, 0}, {1, 0}, {1, 1}, {1, 2}}, &why));  // revisits
    CHECK(!validatePath(g, {{0, 0}, {0, 1}, {1, 1}, {1, 2}, {1, 3}}, &why));  // out of bounds / wrong end
}

// ------------------------------------------------------------- grid generation

TEST(seeded_generation_is_reproducible) {
    Scenario s;
    s.rows = 60;
    s.cols = 80;
    s.terrainDensity = 0.2;
    s.placement = Placement::Random;
    CHECK_EQ(buildScenarioGrid(s, 7).fingerprint(), buildScenarioGrid(s, 7).fingerprint());
    CHECK(buildScenarioGrid(s, 7).fingerprint() != buildScenarioGrid(s, 8).fingerprint());
    s.type = ScenarioType::Maze;
    CHECK_EQ(buildScenarioGrid(s, 7).fingerprint(), buildScenarioGrid(s, 7).fingerprint());
}

TEST(seeded_generation_golden_value) {
    // Guards cross-platform/cross-version reproducibility: the RNG uses only
    // the standard-specified mt19937_64 output, never std::*_distribution.
    // If this value changes, previously published seeds no longer
    // reproduce the same grids.
    Scenario s;  // random_100x100_obs30_ter0_corners
    const std::uint64_t fp = buildScenarioGrid(s, 42).fingerprint();
    std::cout << "    fingerprint(random_100x100_obs30, seed 42) = " << std::hex << fp << std::dec << "\n";
    CHECK_EQ(fp, 0x5651a74ac8070debULL);
}

TEST(random_wall_density_is_close_to_requested) {
    for (double d : {0.1, 0.3, 0.5}) {
        Scenario s;
        s.rows = 202;
        s.cols = 202;
        s.obstacleDensity = d;
        const Grid g = buildScenarioGrid(s, 99);
        const int interior = 200 * 200;
        const int walls = interior - g.passableCount();  // passable count is interior-only (border is wall)
        const double observed = static_cast<double>(walls) / interior;
        CHECK(observed > d - 0.02 && observed < d + 0.02);
    }
}

TEST(maze_is_connected_and_keeps_border) {
    for (int n : {5, 11, 20, 51}) {
        for (std::uint64_t seed = 1; seed <= 10; ++seed) {
            Grid g(n, n + 3);
            Rng rng(seed);
            g.generateMaze(rng);
            CHECK(reachableCount(g) == g.passableCount());  // every open cell connected
            for (int c = 0; c < g.cols(); ++c) CHECK(g.isWall(0, c) && g.isWall(n - 1, c));
            CHECK(bfsSearch(g).metrics.pathFound);
        }
    }
}

TEST(walls_never_cover_start_or_end) {
    Grid g(10, 10);
    Rng rng(3);
    g.generateRandomWalls(1.0, rng);
    CHECK(g.isPassable(1, 1));
    CHECK(g.isPassable(8, 8));
    g.addWall(1, 1);
    CHECK(g.isPassable(1, 1));
    CHECK_EQ(g.passableCount(), 2);
}

TEST(rng_uniform_int_in_range) {
    Rng rng(123);
    for (int i = 0; i < 10000; ++i) {
        const auto v = rng.uniformInt(7);
        CHECK(v < 7);
        const double x = rng.uniformReal();
        CHECK(x >= 0.0 && x < 1.0);
    }
}

// ------------------------------------------------------------- grid API safety

TEST(add_terrain_accepts_every_valid_type) {
    struct Expect {
        char type;
        int cost;
    };
    const Expect valid[] = {{EMPTY, 1}, {SAND, 3}, {WATER, 5}, {MOUNTAIN, 10}, {WALL, 0}};
    for (const Expect& e : valid) {
        Grid g(5, 5);
        CHECK(isValidTerrain(e.type));
        CHECK(g.addTerrain(2, 2, e.type));
        CHECK_EQ(g.cellAt(2, 2), e.type);
        if (e.type == WALL) CHECK(!g.isPassable(2, 2));
        else CHECK_EQ(g.costAt(g.index(2, 2)), e.cost);
    }
    Grid g(5, 5);
    CHECK(g.addWall(2, 3));
    CHECK(g.isWall(2, 3));
    CHECK(g.addTerrain(2, 3, EMPTY));  // terrain can be cleared again
    CHECK(g.isPassable(2, 3));
}

TEST(add_terrain_rejects_invalid_input_without_modifying) {
    Grid g(6, 7);
    Rng rng(5);
    g.generateRandomTerrain(0.5, rng);
    const std::uint64_t before = g.fingerprint();
    // Invalid terrain characters, including the start/end/overlay markers.
    for (char bad : {'S', 'E', '.', '*', 'x', 'w', '\0', '\n', static_cast<char>(0xFF)}) {
        CHECK(!isValidTerrain(bad));
        CHECK(!g.addTerrain(2, 2, bad));
    }
    // Out of bounds, and start/end, with otherwise valid terrain.
    CHECK(!g.addTerrain(-1, 2, WALL));
    CHECK(!g.addTerrain(2, -1, WALL));
    CHECK(!g.addTerrain(6, 2, WALL));
    CHECK(!g.addTerrain(2, 7, WALL));
    CHECK(!g.addWall(1, 1));  // start
    CHECK(!g.addWall(4, 5));  // end
    CHECK_EQ(g.fingerprint(), before);
}

namespace {
template <typename F>
bool throwsInvalidArgument(F&& f) {
    try {
        f();
    } catch (const std::invalid_argument&) {
        return true;
    }
    return false;
}
}  // namespace

TEST(grid_dimension_validation) {
    static_assert(Grid::kMaxCells * 4 + 1 <= std::numeric_limits<int>::max(), "frontier counts must fit in int");
    const int big = std::numeric_limits<int>::max();
    CHECK(throwsInvalidArgument([] { Grid g(2, 3); }));
    CHECK(throwsInvalidArgument([] { Grid g(3, 2); }));
    CHECK(throwsInvalidArgument([] { Grid g(0, 0); }));
    CHECK(throwsInvalidArgument([] { Grid g(-5, 10); }));
    CHECK(throwsInvalidArgument([] { Grid g(10, -5); }));
    // rows * cols overflows a 32-bit int: must be rejected before allocating.
    CHECK(throwsInvalidArgument([] { Grid g(std::numeric_limits<int>::max(), std::numeric_limits<int>::max()); }));
    CHECK(throwsInvalidArgument([] { Grid g(std::numeric_limits<int>::max(), 3); }));
    CHECK(throwsInvalidArgument([] { Grid g(46341, 46341); }));  // 2,147,488,281 > INT_MAX
    // Just above the limit without overflowing.
    CHECK(throwsInvalidArgument([] { Grid g(static_cast<int>(Grid::kMaxCells / 3 + 1), 3); }));

    const Grid smallest(3, 3);
    CHECK_EQ(smallest.cellCount(), 9);
    CHECK(smallest.getStartPos() == smallest.getEndPos());
    CHECK_EQ(smallest.passableCount(), 1);
    const Grid wide(3, 1000);
    CHECK_EQ(wide.cellCount(), 3000);
    CHECK(wide.getEndPos() == (Cell{1, 998}));
    CHECK_EQ(wide.index(2, 999), 2999);
    CHECK(wide.cellOf(2999) == (Cell{2, 999}));
    // Bounds-safe accessors on out-of-range coordinates.
    CHECK(!wide.isValidCell(-1, 0));
    CHECK(!wide.isValidCell(3, 0));
    CHECK(!wide.isPassable(0, 1000));
    CHECK(!wide.isPassable(big, big));
}

TEST(from_strings_rejects_malformed_input) {
    using Lines = std::vector<std::string>;
    const Lines bad[] = {
        {},                       // no rows
        {""},                     // empty row
        {"", "S"},                // empty first row
        {"S.", "."},              // ragged
        {"S.", "..", "E"},        // ragged last row
        {"S.", "..E"},            // longer later row (E would be silently dropped)
        {"S.E", "....", "..."},   // longer middle row
        {"S?E"},                  // unknown character
        {"s.E"},                  // lowercase start
        {"..E"},                  // no start
        {"S.S"},                  // two starts
        {"SEE"},                  // two ends
        {"S", "E", "E"},          // two ends across rows
        {std::string("S\0E", 3)}, // embedded NUL
    };
    for (const Lines& lines : bad) CHECK(throwsInvalidArgument([&lines] { (void)Grid::fromStrings(lines); }));
}

TEST(from_strings_accepts_unusual_dimensions) {
    const Grid one = Grid::fromStrings({"S"});
    CHECK_EQ(one.cellCount(), 1);
    CHECK(one.getStartPos() == one.getEndPos());

    const Grid row = Grid::fromStrings({"S.W~M#E"});
    CHECK_EQ(row.rows(), 1);
    CHECK_EQ(row.cols(), 7);
    CHECK(row.getEndPos() == (Cell{0, 6}));
    CHECK_EQ(row.cellAt(0, 0), EMPTY);  // 'S' is a position, the cell is empty
    CHECK_EQ(row.cellAt(0, 1), EMPTY);
    CHECK_EQ(row.cellAt(0, 2), WATER);
    CHECK_EQ(row.cellAt(0, 3), SAND);
    CHECK_EQ(row.cellAt(0, 4), MOUNTAIN);
    CHECK_EQ(row.cellAt(0, 5), WALL);

    const Grid column = Grid::fromStrings({"E", " ", "S"});
    CHECK_EQ(column.rows(), 3);
    CHECK_EQ(column.cols(), 1);
    CHECK(column.getStartPos() == (Cell{2, 0}));
    CHECK(column.getEndPos() == (Cell{0, 0}));
    CHECK_EQ(bfsSearch(column).metrics.pathLength, 2);

    // Grid operations that assume a border are harmless on border-less grids.
    Grid tiny = Grid::fromStrings({"SE"});
    Rng rng(1);
    tiny.clear();
    tiny.generateRandomWalls(1.0, rng);
    tiny.generateMaze(rng);
    CHECK_EQ(tiny.passableCount(), 2);
}

namespace {
// Reference FNV-1a 64 written independently of Grid.
std::uint64_t referenceFnv1a(const std::vector<unsigned char>& bytes) {
    std::uint64_t h = 0xcbf29ce484222325ULL;
    for (unsigned char b : bytes) {
        h ^= b;
        h *= 0x100000001b3ULL;
    }
    return h;
}
std::vector<unsigned char> bytesOf(const std::string& s) { return std::vector<unsigned char>(s.begin(), s.end()); }
}  // namespace

TEST(fingerprint_is_standard_fnv1a) {
    // Published FNV-1a 64-bit test vectors anchor the reference.
    CHECK_EQ(referenceFnv1a({}), 0xcbf29ce484222325ULL);
    CHECK_EQ(referenceFnv1a(bytesOf("a")), 0xaf63dc4c8601ec8cULL);
    CHECK_EQ(referenceFnv1a(bytesOf("foobar")), 0x85944171f73967e8ULL);

    // fingerprint() == FNV-1a over the documented serialization.
    const Grid g = Grid::fromStrings({"S.#", "W~M", "..E"});
    std::vector<unsigned char> bytes;
    auto put64 = [&bytes](std::uint64_t v) {
        for (int i = 0; i < 8; ++i) bytes.push_back(static_cast<unsigned char>((v >> (8 * i)) & 0xFFu));
    };
    put64(3);  // rows
    put64(3);  // cols
    put64(0);  // start index
    put64(8);  // end index
    for (char ch : std::string("  #W~M   ")) bytes.push_back(static_cast<unsigned char>(ch));
    CHECK_EQ(g.fingerprint(), referenceFnv1a(bytes));
}

TEST(fingerprint_consistency_and_sensitivity) {
    const Grid base = Grid::fromStrings({"S...", ".W..", "...E"});
    const std::uint64_t h = base.fingerprint();
    CHECK_EQ(h, base.fingerprint());                                                 // stable
    CHECK_EQ(h, Grid(base).fingerprint());                                           // copies
    CHECK_EQ(h, Grid::fromStrings({"S...", ".W..", "...E"}).fingerprint());           // rebuilt
    CHECK_EQ(h, Grid::fromStrings({"S   ", " W  ", "   E"}).fingerprint());           // ' ' == '.'

    std::vector<std::uint64_t> variants;
    {
        Grid g = base;
        g.addWall(0, 2);  // one cell becomes a wall
        variants.push_back(g.fingerprint());
        g.addTerrain(0, 2, EMPTY);  // reverting restores the hash
        CHECK_EQ(g.fingerprint(), h);
    }
    {
        Grid g = base;
        g.addTerrain(1, 1, SAND);  // terrain type change only
        variants.push_back(g.fingerprint());
    }
    {
        Grid g = base;
        g.setStart(Cell{0, 1});
        variants.push_back(g.fingerprint());
    }
    {
        Grid g = base;
        g.setEnd(Cell{2, 2});
        variants.push_back(g.fingerprint());
    }
    variants.push_back(Grid::fromStrings({"E...", ".W..", "...S"}).fingerprint());  // start/end swapped
    // Same 12 cells, same start/end indices, different shape.
    variants.push_back(Grid::fromStrings({"S..", ".W.", "...", "..E"}).fingerprint());
    variants.push_back(Grid::fromStrings({"S..", "...", "...", "W.E"}).fingerprint());

    std::set<std::uint64_t> unique(variants.begin(), variants.end());
    unique.insert(h);
    CHECK_EQ(unique.size(), variants.size() + 1);  // all pairwise different
}

TEST(compute_path_cost_rejects_invalid_cells) {
    const Grid g = Grid::fromStrings({"SM#", "..E"});
    CHECK_EQ(computePathCost(g, {{0, 0}, {0, 1}}), 10LL);
    CHECK_EQ(computePathCost(g, {{0, 0}}), 0LL);
    CHECK_EQ(computePathCost(g, {}), -1LL);
    CHECK_EQ(computePathCost(g, {{0, 0}, {-1, 0}}), -1LL);  // out of bounds: no read
    CHECK_EQ(computePathCost(g, {{0, 0}, {0, 3}}), -1LL);
    CHECK_EQ(computePathCost(g, {{0, 0}, {5, 5}}), -1LL);
    CHECK_EQ(computePathCost(g, {{0, 1}, {0, 2}}), -1LL);   // wall
}

// ------------------------------------------------------------- DFS behaviour

namespace {
// Plain recursive DFS, neighbors in up/right/down/left order. Records
// expansion order (goal excluded) and the path held by the recursion.
bool recursiveDfs(const Grid& g, Cell cur, std::vector<char>& visited, std::vector<Cell>& order,
                  std::vector<Cell>& path) {
    visited[g.index(cur.row, cur.col)] = 1;
    path.push_back(cur);
    if (cur == g.getEndPos()) return true;
    order.push_back(cur);
    const Cell nbs[] = {{cur.row - 1, cur.col}, {cur.row, cur.col + 1}, {cur.row + 1, cur.col}, {cur.row, cur.col - 1}};
    for (const Cell& nb : nbs) {
        if (g.isPassable(nb.row, nb.col) && !visited[g.index(nb.row, nb.col)]) {
            if (recursiveDfs(g, nb, visited, order, path)) return true;
        }
    }
    path.pop_back();
    return false;
}
bool sameCells(const std::vector<Cell>& a, const std::vector<Cell>& b) {
    if (a.size() != b.size()) return false;
    for (std::size_t i = 0; i < a.size(); ++i) {
        if (a[i] != b[i]) return false;
    }
    return true;
}
}  // namespace

TEST(dfs_matches_recursive_dfs_exactly) {
    // Same expansion order and same path (parent links) as textbook
    // recursive DFS: the stale-duplicate approach is genuine DFS.
    int compared = 0;
    for (std::uint64_t seed = 1; seed <= 120; ++seed) {
        Scenario s;
        s.type = (seed % 4 == 0) ? ScenarioType::Maze : ScenarioType::Random;
        s.rows = 3 + static_cast<int>(seed % 17);
        s.cols = 3 + static_cast<int>((seed * 5) % 19);
        s.obstacleDensity = (s.type == ScenarioType::Maze) ? 0.0 : static_cast<double>(seed % 5) * 0.1;
        s.placement = (seed % 3 == 0) ? Placement::Random : Placement::Corners;
        const Grid g = buildScenarioGrid(s, seed);

        ExpansionTrace trace;
        const SearchResult r = dfsSearch(g, &trace);
        std::vector<char> visited(static_cast<std::size_t>(g.cellCount()), 0);
        std::vector<Cell> order;
        std::vector<Cell> path;
        const bool found = recursiveDfs(g, g.getStartPos(), visited, order, path);
        CHECK_EQ(r.metrics.pathFound, found);
        CHECK(sameCells(trace, order));
        if (found) CHECK(sameCells(r.path, path));
        ++compared;
    }
    CHECK_EQ(compared, 120);
}

TEST(dfs_two_by_two_exact_metrics) {
    // S at (0,0), E at (1,0). DFS goes right, down, left: 3 moves. The cell
    // E is pushed twice (by S and by (1,1)); the stack peaks at 2 entries.
    // Marking on insertion would instead return S->E directly.
    const Grid g = Grid::fromStrings({"S.", "E."});
    const SearchResult r = run(Algorithm::DFS, g);
    CHECK(sameCells(r.path, {{0, 0}, {0, 1}, {1, 1}, {1, 0}}));
    CHECK_EQ(r.metrics.pathLength, 3);
    CHECK_EQ(r.metrics.nodesDiscovered, 4);
    CHECK_EQ(r.metrics.nodesExpanded, 3);
    CHECK_EQ(r.metrics.peakFrontierSize, 2);
}

TEST(dfs_frontier_metric_bounds) {
    for (std::uint64_t seed = 1; seed <= 60; ++seed) {
        Scenario s;
        s.rows = 10 + static_cast<int>(seed % 30);
        s.cols = 10 + static_cast<int>((seed * 3) % 30);
        s.obstacleDensity = static_cast<double>(seed % 4) * 0.1;
        const Grid g = buildScenarioGrid(s, seed);
        const SearchMetrics m = run(Algorithm::DFS, g).metrics;
        // Pushes: 1 for the start, <= 4 when expanding the start and <= 3 for
        // every other expansion (its parent is already visited).
        CHECK(m.peakFrontierSize <= 2 + 3 * m.nodesExpanded);
        // A discovered cell is expanded, is the goal, or is still stacked.
        CHECK(m.nodesDiscovered <= m.nodesExpanded + 1 + m.peakFrontierSize);
    }
}

// ------------------------------------------------------------- benchmark engine

namespace {
// Runs the benchmark and collects every per-run record.
BenchmarkResult runCollecting(const BenchmarkConfig& cfg, std::vector<RunRecord>& records) {
    BenchmarkCallbacks cb;
    cb.onRun = [&records](const RunRecord& r) { records.push_back(r); };
    return runBenchmark(cfg, cb);
}
}  // namespace

TEST(benchmark_runs_all_algorithms_on_identical_grids) {
    BenchmarkConfig cfg;
    Scenario s;
    s.rows = 40;
    s.cols = 40;
    s.obstacleDensity = 0.25;
    s.placement = Placement::Random;
    cfg.scenarios = {s};
    cfg.grids = 3;
    cfg.runs = 4;
    cfg.warmup = 0;
    std::vector<RunRecord> records;
    const BenchmarkResult result = runCollecting(cfg, records);
    CHECK(result.failures.empty());
    CHECK_EQ(records.size(), std::size_t{3 * 4 * 4});
    for (const RunRecord& r : records) {
        CHECK(r.valid);
        CHECK_EQ(r.seed, 42 + static_cast<std::uint64_t>(r.gridIndex));
        // Same seed -> same grid hash for every algorithm and run.
        for (const RunRecord& o : records) {
            if (o.seed == r.seed) CHECK_EQ(o.gridHash, r.gridHash);
        }
        CHECK_EQ(r.gridHash, buildScenarioGrid(s, r.seed).fingerprint());
    }
    // Re-running reproduces every non-timing value.
    std::vector<RunRecord> again;
    runCollecting(cfg, again);
    CHECK_EQ(again.size(), records.size());
    for (std::size_t i = 0; i < records.size() && i < again.size(); ++i) {
        CHECK_EQ(again[i].metrics.nodesExpanded, records[i].metrics.nodesExpanded);
        CHECK_EQ(again[i].metrics.pathCost, records[i].metrics.pathCost);
        CHECK(again[i].algorithm == records[i].algorithm);
        CHECK_EQ(again[i].gridHash, records[i].gridHash);
    }
    // Level-1 summaries: one per (grid, algorithm), each over 4 runs.
    CHECK_EQ(result.grids.size(), std::size_t{3 * 4});
    for (const GridSummary& g : result.grids) {
        CHECK_EQ(g.runs, 4);
        CHECK_EQ(g.validRuns, 4);
        CHECK(g.searchMs.has_value() && g.endToEndMs.has_value());
    }
}

TEST(benchmark_validates_every_suite) {
    BenchmarkConfig cfg;
    CHECK(buildSuite("all", cfg.scenarios));
    // 1 baseline + 5 density + 5 density-corners + 4 size + 4 size-corners
    // + 3 maze + 1 terrain, minus 3 duplicates of the 100x100 30% scenarios.
    CHECK_EQ(cfg.scenarios.size(), std::size_t{20});
    cfg.grids = 2;
    cfg.runs = 1;
    cfg.warmup = 0;
    const BenchmarkResult result = runBenchmark(cfg);
    for (const std::string& f : result.failures) std::cerr << "  " << f << "\n";
    CHECK(result.failures.empty());
    CHECK_EQ(result.invalidRuns, 0);
    CHECK_EQ(result.generationFailures, 0);
}

TEST(stats_median_mean_stddev) {
    const Stats odd = computeStats({5, 1, 3});
    CHECK_EQ(odd.median, 3.0);
    CHECK_EQ(odd.min, 1.0);
    CHECK_EQ(odd.max, 5.0);
    CHECK_EQ(odd.mean, 3.0);
    CHECK_EQ(odd.stddev, 2.0);
    const Stats even = computeStats({4, 1, 3, 2});
    CHECK_EQ(even.median, 2.5);
    const Stats one = computeStats({7});
    CHECK_EQ(one.stddev, 0.0);
    CHECK_EQ(one.median, 7.0);
    const Stats none = computeStats({});
    CHECK_EQ(none.mean, 0.0);
}

TEST(quantiles_use_linear_interpolation) {
    const std::vector<double> v = {1, 2, 3, 4};
    CHECK_EQ(quantileSorted(v, 0.25), 1.75);  // NumPy default / R type 7
    CHECK_EQ(quantileSorted(v, 0.5), 2.5);
    CHECK_EQ(quantileSorted(v, 0.75), 3.25);
    CHECK_EQ(quantileSorted({5}, 0.25), 5.0);
    const auto d = distributionOf({10, 1, 4, 2, 3});
    CHECK(d.has_value());
    CHECK_EQ(d->n, 5);
    CHECK_EQ(d->median, 3.0);
    CHECK_EQ(d->q1, 2.0);
    CHECK_EQ(d->q3, 4.0);
    CHECK_EQ(d->iqr(), 2.0);
    CHECK_EQ(d->mean, 4.0);
    CHECK(!distributionOf({}).has_value());
}

namespace {
GridSummary makeGrid(int gridIndex, Algorithm a, double medianMs, bool found, std::uint64_t hash = 7) {
    GridSummary g;
    g.scenarioId = "x";
    g.algorithm = a;
    g.gridIndex = gridIndex;
    g.seed = 100 + static_cast<std::uint64_t>(gridIndex);
    g.gridHash = hash + static_cast<std::uint64_t>(gridIndex);
    g.runs = 10;
    g.validRuns = 10;
    g.reference.reachable = found;
    g.reference.passableCells = 100;
    g.reference.minimumCost = found ? 10 : -1;
    SearchMetrics m;
    m.pathFound = found;
    m.pathLength = found ? 12 : -1;
    m.pathCost = found ? 12 : -1;
    m.nodesExpanded = found ? 50 : 30;
    m.nodesDiscovered = 60;
    m.peakFrontierSize = 5;
    g.metrics = m;
    Stats st;
    st.median = st.mean = st.min = st.max = medianMs;
    g.searchMs = st;
    g.endToEndMs = st;
    if (found) g.costRatio = 1.2;
    return g;
}
}  // namespace

TEST(aggregate_success_rate_and_ratios) {
    std::vector<GridSummary> grids = {makeGrid(0, Algorithm::BFS, 2.0, true), makeGrid(1, Algorithm::BFS, 0.0, false)};
    const std::vector<Aggregate> agg = aggregate(grids);
    CHECK_EQ(agg.size(), std::size_t{1});
    const Aggregate& a = agg[0];
    CHECK_EQ(a.gridsMeasured, 2);
    CHECK_EQ(a.gridsGenerated, 2);
    CHECK_EQ(a.runsPerGrid, 10);
    CHECK(a.successRate && *a.successRate == 0.5);
    CHECK(a.solvabilityRate && *a.solvabilityRate == 0.5);
    CHECK(a.meanPathLength && *a.meanPathLength == 12.0);  // over the solved grid only
    CHECK(a.meanCostRatio && *a.meanCostRatio == 1.2);
    CHECK_EQ(a.costRatioGrids, 1);
    // 50 expanded / 2 ms; the 0 ms grid is skipped (no division by zero).
    CHECK(a.medianExpansionsPerSearchMs && *a.medianExpansionsPerSearchMs == 25.0);
    CHECK(a.meanExplorationRatio && *a.meanExplorationRatio == 0.4);  // (0.5 + 0.3) / 2
    CHECK(a.searchMsSolvable && a.searchMsSolvable->n == 1 && a.searchMsSolvable->median == 2.0);

    const std::vector<Aggregate> none = aggregate({makeGrid(1, Algorithm::BFS, 0.0, false)});
    CHECK(!none[0].meanPathLength);
    CHECK(!none[0].meanCostRatio);
    CHECK(!none[0].medianExpansionsPerSearchMs);
    CHECK(aggregate({}).empty());
}

TEST(aggregation_counts_grids_not_runs) {
    // Three grids x 10 runs each: n is 3, and the statistics are over the
    // three per-grid medians.
    std::vector<GridSummary> grids = {makeGrid(0, Algorithm::DFS, 1.0, true), makeGrid(1, Algorithm::DFS, 2.0, true),
                                      makeGrid(2, Algorithm::DFS, 10.0, true)};
    GridSummary broken = makeGrid(3, Algorithm::DFS, 99.0, true);
    broken.validRuns = 0;  // every run failed validation
    broken.metrics.reset();
    broken.searchMs.reset();
    broken.endToEndMs.reset();
    grids.push_back(broken);
    const Aggregate a = aggregate(grids)[0];
    CHECK_EQ(a.gridsMeasured, 3);
    CHECK_EQ(a.invalidRuns, 10);
    CHECK(a.searchMs && a.searchMs->n == 3);
    CHECK_EQ(a.searchMs->median, 2.0);
    CHECK(std::fabs(a.searchMs->mean - 13.0 / 3.0) < 1e-12);
    CHECK_EQ(a.searchMs->q1, 1.5);
    CHECK_EQ(a.searchMs->q3, 6.0);

    // With scenario status, requested/generated come from generation.
    ScenarioStatus st;
    st.scenarioId = "x";
    st.gridsRequested = 5;
    st.gridsGenerated = 4;
    const Aggregate b = aggregate(grids, {st})[0];
    CHECK_EQ(b.gridsRequested, 5);
    CHECK_EQ(b.gridsGenerated, 4);
}

TEST(paired_comparisons_use_same_grid_ratios) {
    std::vector<GridSummary> grids;
    const double ta[] = {1, 2, 4};
    for (int i = 0; i < 3; ++i) {
        grids.push_back(makeGrid(i, Algorithm::AStar, ta[i], true));
        grids.push_back(makeGrid(i, Algorithm::Dijkstra, 2.0, true));
    }
    // Grid 3: Dijkstra has no valid runs -> excluded, not substituted.
    grids.push_back(makeGrid(3, Algorithm::AStar, 1.0, true));
    GridSummary invalid = makeGrid(3, Algorithm::Dijkstra, 1.0, true);
    invalid.validRuns = 0;
    invalid.searchMs.reset();
    invalid.endToEndMs.reset();
    grids.push_back(invalid);
    // Grid 4: zero median -> excluded.
    grids.push_back(makeGrid(4, Algorithm::AStar, 0.0, true));
    grids.push_back(makeGrid(4, Algorithm::Dijkstra, 1.0, true));
    // Grid 5: different grid hashes -> not comparable -> excluded.
    grids.push_back(makeGrid(5, Algorithm::AStar, 1.0, true, 7));
    grids.push_back(makeGrid(5, Algorithm::Dijkstra, 1.0, true, 999));
    // Grid 6: only one algorithm present -> excluded.
    grids.push_back(makeGrid(6, Algorithm::AStar, 1.0, true));

    const auto pairs = pairedComparisons(grids);
    CHECK_EQ(pairs.size(), std::size_t{2});  // one pair x {search, end_to_end}
    const PairedComparison& p = pairs[0];
    CHECK(p.a == Algorithm::AStar && p.b == Algorithm::Dijkstra);
    CHECK(p.timing == TimingKind::Search);
    CHECK_EQ(p.gridsCompared, 3);
    CHECK_EQ(p.gridsExcluded, 4);
    CHECK(p.ratio && p.ratio->median == 1.0);  // ratios 0.5, 1, 2
    CHECK(p.geomeanRatio && std::fabs(*p.geomeanRatio - 1.0) < 1e-12);
    CHECK(p.aFasterFraction && std::fabs(*p.aFasterFraction - 1.0 / 3.0) < 1e-12);
    CHECK(pairs[1].timing == TimingKind::EndToEnd);
    CHECK(pairedComparisons({}).empty());

    // Nothing comparable: no ratio, no geomean.
    const auto none = pairedComparisons({grids.back()});
    CHECK(none.empty());  // a single algorithm forms no pair
}

TEST(benchmark_paired_astar_dijkstra_on_same_grids) {
    BenchmarkConfig cfg;
    Scenario s;
    s.rows = 60;
    s.cols = 60;
    s.obstacleDensity = 0.2;
    s.placement = Placement::Reachable;
    cfg.scenarios = {s};
    cfg.algorithms = {Algorithm::Dijkstra, Algorithm::AStar};
    cfg.grids = 4;
    cfg.runs = 2;
    cfg.warmup = 0;
    const BenchmarkResult r = runBenchmark(cfg);
    CHECK(r.failures.empty());
    const auto pairs = pairedComparisons(r.grids);
    CHECK_EQ(pairs.size(), std::size_t{2});
    CHECK_EQ(pairs[0].gridsCompared + pairs[0].gridsExcluded, 4);
    const auto agg = aggregate(r.grids, r.scenarios);
    CHECK_EQ(agg.size(), std::size_t{2});
    CHECK_EQ(agg[0].gridsMeasured, 4);
    CHECK(agg[0].solvabilityRate && *agg[0].solvabilityRate == 1.0);
    // A* never expands more than Dijkstra on the same grid.
    CHECK(*agg[1].meanNodesExpanded <= *agg[0].meanNodesExpanded);
}

TEST(run_order_is_rotated_and_balanced) {
    BenchmarkConfig cfg;
    Scenario s;
    s.rows = 12;
    s.cols = 12;
    cfg.scenarios = {s};
    cfg.grids = 1;
    cfg.runs = 8;
    cfg.warmup = 0;
    std::vector<RunRecord> records;
    runCollecting(cfg, records);
    std::map<std::pair<int, int>, int> counts;  // (algorithm, position) -> runs
    for (const RunRecord& r : records) {
        CHECK_EQ(r.orderPosition, (static_cast<int>(r.algorithm) - r.runIndex % 4 + 4) % 4);
        ++counts[{static_cast<int>(r.algorithm), r.orderPosition}];
    }
    CHECK_EQ(counts.size(), std::size_t{16});
    for (const auto& kv : counts) CHECK_EQ(kv.second, 2);  // 8 runs / 4 positions
}

TEST(config_validation_and_warnings) {
    BenchmarkConfig c;
    CHECK(validateConfig(c).empty());
    CHECK(configWarnings(c).empty());  // defaults: 10 grids, 12 runs, 4 algorithms
    c.grids = 1;
    CHECK_EQ(configWarnings(c).size(), std::size_t{1});
    c.grids = 10;
    c.runs = 10;  // not a multiple of 4
    CHECK_EQ(configWarnings(c).size(), std::size_t{1});
    c.runs = 12;
    c.baseSeed = std::uint64_t{1} << 60;
    CHECK_EQ(configWarnings(c).size(), std::size_t{1});  // beyond 2^53

    c.baseSeed = std::numeric_limits<std::uint64_t>::max();
    c.grids = 1;
    CHECK(validateConfig(c).empty());
    c.grids = 2;  // seed max + 1 would wrap
    CHECK(!validateConfig(c).empty());
    c.baseSeed = std::numeric_limits<std::uint64_t>::max() - 1;
    CHECK(validateConfig(c).empty());

    BenchmarkConfig bad;
    bad.runs = 0;
    CHECK(!validateConfig(bad).empty());
    bad = BenchmarkConfig{};
    bad.algorithms = {Algorithm::BFS, Algorithm::BFS};
    CHECK(!validateConfig(bad).empty());
    bad = BenchmarkConfig{};
    bad.scenarios[0].obstacleDensity = 1.5;
    CHECK(!validateConfig(bad).empty());
    bad = BenchmarkConfig{};
    bad.scenarios[0].rows = 2;
    CHECK(!validateConfig(bad).empty());
    bool threw = false;
    try {
        runBenchmark(bad);
    } catch (const std::invalid_argument&) {
        threw = true;
    }
    CHECK(threw);
}

// ------------------------------------------------------------- CSV

TEST(csv_escaping) {
    CHECK_EQ(csvEscape("plain"), std::string("plain"));
    CHECK_EQ(csvEscape("a,b"), std::string("\"a,b\""));
    CHECK_EQ(csvEscape("say \"hi\""), std::string("\"say \"\"hi\"\"\""));
    CHECK_EQ(csvEscape("line\nbreak"), std::string("\"line\nbreak\""));
}

namespace {
// Minimal RFC 4180 parser for the tests.
std::vector<std::vector<std::string>> parseCsv(const std::string& text) {
    std::vector<std::vector<std::string>> rows(1);
    std::string field;
    bool quoted = false;
    for (std::size_t i = 0; i < text.size(); ++i) {
        const char c = text[i];
        if (quoted) {
            if (c == '"' && i + 1 < text.size() && text[i + 1] == '"') {
                field += '"';
                ++i;
            } else if (c == '"') {
                quoted = false;
            } else {
                field += c;
            }
        } else if (c == '"') {
            quoted = true;
        } else if (c == ',') {
            rows.back().push_back(field);
            field.clear();
        } else if (c == '\n') {
            rows.back().push_back(field);
            field.clear();
            rows.emplace_back();
        } else {
            field += c;
        }
    }
    if (rows.back().empty()) rows.pop_back();
    return rows;
}

// Every row has the header's width; every field outside `textColumns` is
// empty or a complete number.
bool csvIsNumericClean(const std::vector<std::vector<std::string>>& rows, const std::set<std::string>& textColumns) {
    bool ok = !rows.empty();
    for (const auto& row : rows) {
        if (row.size() != rows[0].size()) {
            std::cerr << "  ragged row\n";
            ok = false;
        }
    }
    for (std::size_t r = 1; r < rows.size() && ok; ++r) {
        for (std::size_t c = 0; c < rows[r].size(); ++c) {
            const std::string& f = rows[r][c];
            if (textColumns.count(rows[0][c]) || f.empty()) continue;
            char* end = nullptr;
            std::strtod(f.c_str(), &end);
            if (end != f.c_str() + f.size()) {
                std::cerr << "  non-numeric '" << f << "' in column " << rows[0][c] << "\n";
                ok = false;
            }
        }
    }
    return ok;
}

std::size_t column(const std::vector<std::string>& header, const std::string& name) {
    for (std::size_t i = 0; i < header.size(); ++i) {
        if (header[i] == name) return i;
    }
    std::cerr << "  missing column " << name << "\n";
    return header.size();
}

const std::set<std::string> kTextColumns = {"scenario_id", "scenario_type", "placement", "algorithm", "grid_hash",
                                            "failure", "algorithm_a", "algorithm_b", "timing"};
}  // namespace

TEST(csv_files_are_rectangular_and_numeric) {
    BenchmarkConfig cfg;
    Scenario s;
    s.rows = 20;
    s.cols = 30;
    s.terrainDensity = 0.25;
    cfg.scenarios = {s};
    cfg.grids = 3;
    cfg.runs = 4;
    cfg.warmup = 0;
    std::vector<RunRecord> records;
    const BenchmarkResult result = runCollecting(cfg, records);

    std::ostringstream runs;
    writeRunsCsv(runs, records);
    const auto runRows = parseCsv(runs.str());
    CHECK_EQ(runRows.size(), records.size() + 1);
    CHECK(runs.str().rfind("scenario_id,scenario_type,rows,columns,obstacle_density,terrain_density,placement,algorithm,", 0) == 0);
    CHECK(csvIsNumericClean(runRows, kTextColumns));
    CHECK_EQ(runRows[1][0], std::string("random_20x30_obs0.3_ter0.25_corners"));

    std::ostringstream grid, summary, paired;
    writeGridCsv(grid, result.grids);
    const auto agg = aggregate(result.grids, result.scenarios);
    writeSummaryCsv(summary, agg);
    const auto pairs = pairedComparisons(result.grids);
    writePairedCsv(paired, pairs);
    const auto gridRows = parseCsv(grid.str());
    const auto sumRows = parseCsv(summary.str());
    const auto pairRows = parseCsv(paired.str());
    CHECK_EQ(gridRows.size(), result.grids.size() + 1);
    CHECK_EQ(sumRows.size(), agg.size() + 1);
    CHECK_EQ(pairRows.size(), pairs.size() + 1);
    CHECK(csvIsNumericClean(gridRows, kTextColumns));
    CHECK(csvIsNumericClean(sumRows, kTextColumns));
    CHECK(csvIsNumericClean(pairRows, kTextColumns));
    CHECK_EQ(pairs.size(), std::size_t{12});  // 6 pairs x 2 timing kinds
}

TEST(csv_missing_values_are_empty_not_sentinels) {
    // Corner placement at 100% walls: start and end are walled in, so no
    // path and no reference cost exist.
    BenchmarkConfig cfg;
    Scenario s;
    s.rows = 8;
    s.cols = 8;
    s.obstacleDensity = 1.0;
    cfg.scenarios = {s};
    cfg.grids = 1;
    cfg.runs = 1;
    cfg.warmup = 0;
    std::vector<RunRecord> records;
    const BenchmarkResult result = runCollecting(cfg, records);
    CHECK(result.failures.empty());
    std::ostringstream runs;
    writeRunsCsv(runs, records);
    const auto rows = parseCsv(runs.str());
    for (const char* name : {"path_length", "path_cost", "reference_min_cost", "reference_min_moves"}) {
        const std::size_t c = column(rows[0], name);
        for (std::size_t r = 1; r < rows.size(); ++r) CHECK(c < rows[r].size() && rows[r][c].empty());
    }
    CHECK_EQ(rows[1][column(rows[0], "reachable")], std::string("0"));
    CHECK_EQ(rows[1][column(rows[0], "path_found")], std::string("0"));

    std::ostringstream summary;
    writeSummaryCsv(summary, aggregate(result.grids, result.scenarios));
    const auto sum = parseCsv(summary.str());
    for (const char* name : {"mean_path_length", "mean_cost_ratio", "search_ms_solvable_median"}) {
        CHECK(sum[1][column(sum[0], name)].empty());
    }
    // No "-1" anywhere in any field.
    for (const auto& table : {rows, sum}) {
        for (std::size_t r = 1; r < table.size(); ++r) {
            for (const std::string& f : table[r]) CHECK(f != "-1");
        }
    }
}

TEST(csv_hash_and_density_formats) {
    CHECK_EQ(formatHash(0), std::string("0x0000000000000000"));
    CHECK_EQ(formatHash(0xabc), std::string("0x0000000000000abc"));
    CHECK_EQ(formatHash(std::numeric_limits<std::uint64_t>::max()), std::string("0xffffffffffffffff"));
    CHECK_EQ(formatRoundTrip(0.3), std::string("0.3"));
    CHECK_EQ(formatRoundTrip(0.1 + 0.2), std::string("0.30000000000000004"));
    CHECK_EQ(formatRoundTrip(0.12345), std::string("0.12345"));
    CHECK_EQ(formatRoundTrip(0.0), std::string("0"));
    CHECK_EQ(formatRoundTrip(1.0), std::string("1"));
    Rng rng(9);
    for (int i = 0; i < 2000; ++i) {
        const double v = rng.uniformReal();
        CHECK_EQ(std::strtod(formatRoundTrip(v).c_str(), nullptr), v);
    }
}

TEST(csv_failure_column_is_escaped) {
    RunRecord r;
    r.scenarioId = "random_5x5_obs0_ter0_corners";
    r.valid = false;
    r.failure = "pathFound mismatch; cost 3, expected 2 \"quoted\"";
    std::ostringstream out;
    writeRunsCsv(out, {r});
    const auto rows = parseCsv(out.str());
    CHECK_EQ(rows.size(), std::size_t{2});
    CHECK_EQ(rows[1].size(), rows[0].size());
    CHECK_EQ(rows[1][column(rows[0], "failure")], r.failure);
    CHECK_EQ(rows[1][column(rows[0], "checks_passed")], std::string("0"));
}

// ------------------------------------------------------------- scenario identity

TEST(scenario_ids_are_exact_and_distinct) {
    std::set<std::string> ids;
    const double densities[] = {0.0, 0.004, 0.0041, 0.3, 0.1 + 0.2, 0.5, 1.0};
    for (double d : densities) {
        Scenario s;
        s.obstacleDensity = d;
        ids.insert(scenarioId(s));
    }
    CHECK_EQ(ids.size(), std::size_t{7});
    Scenario s;
    CHECK_EQ(scenarioId(s), std::string("random_100x100_obs0.3_ter0_corners"));
    s.placement = Placement::Reachable;
    s.terrainDensity = 0.004;
    CHECK_EQ(scenarioId(s), std::string("random_100x100_obs0.3_ter0.004_reachable"));
    Scenario m;
    m.type = ScenarioType::Maze;
    CHECK_EQ(scenarioId(m), std::string("maze_100x100_ter0_corners"));
}

TEST(aggregation_keeps_close_densities_apart) {
    BenchmarkConfig cfg;
    Scenario a;
    a.rows = 10;
    a.cols = 10;
    a.obstacleDensity = 0.0;
    Scenario b = a;
    b.obstacleDensity = 0.004;
    cfg.scenarios = {a, b};
    cfg.algorithms = {Algorithm::BFS};
    cfg.grids = 2;
    cfg.runs = 1;
    cfg.warmup = 0;
    const BenchmarkResult r = runBenchmark(cfg);
    const auto agg = aggregate(r.grids, r.scenarios);
    CHECK_EQ(agg.size(), std::size_t{2});
    for (const Aggregate& x : agg) CHECK_EQ(x.gridsMeasured, 2);
}

// ------------------------------------------------------------- generation modes

namespace {
bool sameWalls(const Grid& a, const Grid& b) {
    if (a.rows() != b.rows() || a.cols() != b.cols()) return false;
    for (int r = 0; r < a.rows(); ++r) {
        for (int c = 0; c < a.cols(); ++c) {
            if (a.isWall(r, c) != b.isWall(r, c)) return false;
        }
    }
    return true;
}
}  // namespace

TEST(placement_modes_share_identical_obstacles) {
    for (std::uint64_t seed = 1; seed <= 30; ++seed) {
        for (ScenarioType type : {ScenarioType::Random, ScenarioType::Maze}) {
            Scenario s;
            s.type = type;
            s.rows = 31;
            s.cols = 41;
            s.obstacleDensity = type == ScenarioType::Random ? 0.3 : 0.0;
            const Grid corners = buildScenarioGrid(s, seed);
            for (Placement p : {Placement::Random, Placement::Reachable}) {
                for (double terrain : {0.0, 0.4}) {
                    Scenario t = s;
                    t.placement = p;
                    t.terrainDensity = terrain;
                    const ScenarioInstance inst = generateScenario(t, seed);
                    if (!inst.ok) continue;  // (reachable may legitimately fail; covered elsewhere)
                    CHECK(sameWalls(corners, inst.grid));
                }
            }
        }
    }
}

TEST(terrain_does_not_move_endpoints) {
    for (std::uint64_t seed = 1; seed <= 20; ++seed) {
        Scenario s;
        s.rows = 30;
        s.cols = 30;
        s.placement = Placement::Reachable;
        const ScenarioInstance plain = generateScenario(s, seed);
        s.terrainDensity = 0.5;
        const ScenarioInstance weighted = generateScenario(s, seed);
        CHECK(plain.ok && weighted.ok);
        CHECK(plain.grid.getStartPos() == weighted.grid.getStartPos());
        CHECK(plain.grid.getEndPos() == weighted.grid.getEndPos());
        CHECK_EQ(plain.placementAttempts, weighted.placementAttempts);
    }
    CHECK_EQ(deriveStreamSeed(123, 0), std::uint64_t{123});
    CHECK(deriveStreamSeed(123, 1) != deriveStreamSeed(123, 2));
    CHECK(deriveStreamSeed(123, 1) != std::uint64_t{123});
    CHECK(deriveStreamSeed(123, 1) != deriveStreamSeed(124, 1));
}

TEST(reachable_placement_is_connected_and_deterministic) {
    int instances = 0;
    for (double d : {0.1, 0.3, 0.45, 0.6}) {
        for (std::uint64_t seed = 1; seed <= 25; ++seed) {
            Scenario s;
            s.rows = 40;
            s.cols = 40;
            s.obstacleDensity = d;
            s.placement = Placement::Reachable;
            const ScenarioInstance a = generateScenario(s, seed);
            const ScenarioInstance b = generateScenario(s, seed);
            CHECK_EQ(a.ok, b.ok);
            if (!a.ok) continue;
            ++instances;
            CHECK_EQ(a.grid.fingerprint(), b.grid.fingerprint());
            CHECK_EQ(a.placementAttempts, b.placementAttempts);
            CHECK(a.placementAttempts >= 1 && a.placementAttempts <= kMaxPlacementAttempts);
            CHECK(a.grid.getStartPos() != a.grid.getEndPos());
            const Cell st = a.grid.getStartPos();
            const Cell en = a.grid.getEndPos();
            CHECK(st.row >= 1 && st.row <= 38 && st.col >= 1 && st.col <= 38);
            CHECK(en.row >= 1 && en.row <= 38 && en.col >= 1 && en.col <= 38);
            CHECK(computeReference(a.grid).reachable);
        }
    }
    CHECK(instances >= 90);  // generation essentially always succeeds at these densities
}

TEST(unrestricted_placements_keep_unreachable_instances) {
    int unreachable = 0;
    for (std::uint64_t seed = 1; seed <= 40; ++seed) {
        Scenario s;
        s.rows = 50;
        s.cols = 50;
        s.obstacleDensity = 0.4;
        const ScenarioInstance inst = generateScenario(s, seed);  // corners
        CHECK(inst.ok);
        CHECK_EQ(inst.placementAttempts, 0);
        if (!computeReference(inst.grid).reachable) ++unreachable;
    }
    CHECK(unreachable > 20);  // corners at 40% are mostly disconnected
}

TEST(reachable_placement_failure_is_explicit) {
    // 100% walls: only the two protected corner cells are open and they are
    // not adjacent, so no connected pair exists.
    Scenario s;
    s.rows = 6;
    s.cols = 6;
    s.obstacleDensity = 1.0;
    s.placement = Placement::Reachable;
    const ScenarioInstance inst = generateScenario(s, 1);
    CHECK(!inst.ok);
    CHECK_EQ(inst.placementAttempts, kMaxPlacementAttempts);
    CHECK(!inst.error.empty());
    bool threw = false;
    try {
        buildScenarioGrid(s, 1);
    } catch (const std::runtime_error&) {
        threw = true;
    }
    CHECK(threw);

    // 3x3: one interior cell. Reachable needs two cells; Random allows start == end.
    Scenario tiny;
    tiny.rows = 3;
    tiny.cols = 3;
    tiny.placement = Placement::Reachable;
    CHECK(!generateScenario(tiny, 1).ok);
    tiny.placement = Placement::Random;
    const ScenarioInstance one = generateScenario(tiny, 1);
    CHECK(one.ok);
    CHECK(one.grid.getStartPos() == one.grid.getEndPos());

    // The benchmark reports failures per scenario instead of dropping them.
    BenchmarkConfig cfg;
    cfg.scenarios = {s};
    cfg.grids = 3;
    cfg.runs = 1;
    cfg.warmup = 0;
    const BenchmarkResult r = runBenchmark(cfg);
    CHECK_EQ(r.generationFailures, 3);
    CHECK(r.grids.empty());
    CHECK(r.failures.empty());  // not a correctness failure
    CHECK_EQ(r.scenarios.size(), std::size_t{1});
    CHECK_EQ(r.scenarios[0].gridsRequested, 3);
    CHECK_EQ(r.scenarios[0].gridsGenerated, 0);
    CHECK_EQ(r.scenarios[0].generationErrors.size(), std::size_t{3});
}

TEST(reachable_generation_golden_value) {
    // Pins the endpoint stream derivation, like seeded_generation_golden_value.
    Scenario s;
    s.placement = Placement::Reachable;
    s.terrainDensity = 0.2;
    const ScenarioInstance inst = generateScenario(s, 42);
    CHECK(inst.ok);
    std::cout << "    fingerprint(random_100x100_obs0.3_ter0.2_reachable, seed 42) = " << std::hex
              << inst.grid.fingerprint() << std::dec << ", attempts " << inst.placementAttempts << "\n";
    CHECK_EQ(inst.grid.fingerprint(), 0x6f9b95b9731475a6ULL);
}

// ------------------------------------------------------------- independent validation

TEST(reference_entry_costs_match_cost_model) {
    for (char t : {EMPTY, SAND, WATER, MOUNTAIN}) CHECK_EQ(referenceEntryCost(t), getTerrainCost(t));
    CHECK_EQ(referenceEntryCost(WALL), -1);
}

TEST(reference_matches_bruteforce_oracle) {
    for (std::uint64_t seed = 1; seed <= 120; ++seed) {
        Scenario s;
        s.rows = 3 + static_cast<int>(seed % 23);
        s.cols = 3 + static_cast<int>((seed * 7) % 29);
        s.obstacleDensity = static_cast<double>(seed % 6) * 0.1;
        s.terrainDensity = (seed % 2 == 0) ? 0.5 : 0.0;
        s.placement = (seed % 3 == 0) ? Placement::Random : Placement::Corners;
        const Grid g = buildScenarioGrid(s, seed);
        const ReferenceSolution ref = computeReference(g);
        CHECK_EQ(ref.minimumCost, bruteForceCost(g, false));
        CHECK_EQ(static_cast<long long>(ref.shortestMoves), bruteForceCost(g, true));
        CHECK_EQ(ref.reachable, bruteForceCost(g, true) >= 0);
        CHECK_EQ(ref.startComponentSize, reachableCount(g));
        CHECK_EQ(ref.passableCells, g.passableCount());
        const std::vector<int> label = labelComponents(g);
        const Cell st = g.getStartPos();
        const Cell en = g.getEndPos();
        CHECK_EQ(label[static_cast<std::size_t>(g.index(st.row, st.col))] ==
                     label[static_cast<std::size_t>(g.index(en.row, en.col))],
                 ref.reachable);
    }
}

TEST(validate_run_accepts_correct_results_on_large_grids) {
    for (std::uint64_t seed = 1; seed <= 6; ++seed) {
        Scenario s;
        s.rows = 250;
        s.cols = 250;
        s.obstacleDensity = 0.1 * static_cast<double>(seed % 5);
        s.terrainDensity = seed % 2 ? 0.3 : 0.0;
        s.placement = seed % 3 ? Placement::Reachable : Placement::Corners;
        const Grid g = buildScenarioGrid(s, seed);
        const ReferenceSolution ref = computeReference(g);
        for (Algorithm a : allAlgorithms()) {
            const auto problems = validateRun(g, ref, a, runSearch(a, g));
            for (const auto& p : problems) std::cerr << "  " << algorithmName(a) << ": " << p << "\n";
            CHECK(problems.empty());
        }
    }
}

TEST(validate_run_detects_tampered_results) {
    // Weighted grid where DFS is neither move- nor cost-optimal.
    Scenario s;
    s.rows = 30;
    s.cols = 30;
    s.obstacleDensity = 0.15;
    s.terrainDensity = 0.4;
    s.placement = Placement::Reachable;
    const Grid g = buildScenarioGrid(s, 3);
    const ReferenceSolution ref = computeReference(g);
    const SearchResult good = dijkstraSearch(g);
    CHECK(validateRun(g, ref, Algorithm::Dijkstra, good).empty());
    CHECK(good.path.size() >= 4);

    auto rejected = [&](Algorithm a, SearchResult r) { return !validateRun(g, ref, a, r).empty(); };
    SearchResult r = good;
    r.metrics.pathLength += 1;
    CHECK(rejected(Algorithm::Dijkstra, r));
    r = good;
    r.metrics.pathCost += 1;
    CHECK(rejected(Algorithm::Dijkstra, r));
    r = good;
    r.path.erase(r.path.begin() + 1);  // non-adjacent jump
    CHECK(rejected(Algorithm::Dijkstra, r));
    r = good;
    r.path.front() = g.getEndPos();  // wrong start
    CHECK(rejected(Algorithm::Dijkstra, r));
    r = good;
    r.path.insert(r.path.begin() + 1, r.path[0]);  // repeated cell
    CHECK(rejected(Algorithm::Dijkstra, r));
    r = good;
    r.path[1] = Cell{-1, 0};  // out of bounds
    CHECK(rejected(Algorithm::Dijkstra, r));
    r = good;
    r.metrics.pathFound = false;
    CHECK(rejected(Algorithm::Dijkstra, r));
    r = good;
    r.metrics.nodesExpanded = r.metrics.nodesDiscovered + 1;
    CHECK(rejected(Algorithm::Dijkstra, r));
    r = good;
    r.metrics.nodesDiscovered = ref.startComponentSize + 1;
    CHECK(rejected(Algorithm::Dijkstra, r));
    r = good;
    r.metrics.peakFrontierSize = 0;
    CHECK(rejected(Algorithm::Dijkstra, r));
    r = good;
    r.metrics.searchTimeMs = r.metrics.executionTimeMs + 1.0;
    CHECK(rejected(Algorithm::Dijkstra, r));
    r = good;
    r.metrics.executionTimeMs = std::nan("");
    CHECK(rejected(Algorithm::Dijkstra, r));

    // Optimality is algorithm-specific: a DFS path is fine for DFS but not for
    // Dijkstra/A* (cost) or BFS (moves).
    const SearchResult dfs = dfsSearch(g);
    CHECK(validateRun(g, ref, Algorithm::DFS, dfs).empty());
    CHECK(dfs.metrics.pathCost > ref.minimumCost);
    CHECK(rejected(Algorithm::Dijkstra, dfs));
    CHECK(rejected(Algorithm::AStar, dfs));
    CHECK(dfs.metrics.pathLength > ref.shortestMoves);
    CHECK(rejected(Algorithm::BFS, dfs));
    // BFS is move-optimal but not cost-optimal on terrain: valid as BFS only.
    const SearchResult bfs = bfsSearch(g);
    CHECK(validateRun(g, ref, Algorithm::BFS, bfs).empty());
    if (bfs.metrics.pathCost != ref.minimumCost) CHECK(rejected(Algorithm::Dijkstra, bfs));

    // Unreachable grid: an "unsuccessful" search must exhaust the component.
    const Grid closed = Grid::fromStrings({"S..#..", "...#.E"});
    const ReferenceSolution cref = computeReference(closed);
    SearchResult miss = bfsSearch(closed);
    CHECK(validateRun(closed, cref, Algorithm::BFS, miss).empty());
    miss.metrics.nodesExpanded -= 1;
    CHECK(!validateRun(closed, cref, Algorithm::BFS, miss).empty());
}

// ------------------------------------------------------------- output files and metadata

TEST(staged_output_preserves_existing_file_until_commit) {
    namespace fs = std::filesystem;
    const fs::path dir = fs::temp_directory_path() / ("pathfinding_tests_" + std::to_string(std::random_device{}()));
    fs::create_directories(dir);
    const std::string path = (dir / "out.csv").string();
    {
        std::ofstream(path) << "OLD\n";
    }
    auto read = [](const std::string& p) {
        std::ifstream in(p);
        std::stringstream ss;
        ss << in.rdbuf();
        return ss.str();
    };
    std::string error;
    CHECK(!checkOutputsWritable({path}, false, error));
    CHECK(checkOutputsWritable({path}, true, error));
    {
        StagedFile f;
        CHECK(f.open(path, error));
        f.stream() << "NEW\n";
        f.stream().flush();
        // Not committed: the original survives and the partial holds the data.
        CHECK_EQ(read(path), std::string("OLD\n"));
        CHECK_EQ(read(f.partialPath()), std::string("NEW\n"));
    }
    CHECK_EQ(read(path), std::string("OLD\n"));
    {
        StagedFile f;
        CHECK(f.open(path, error));
        f.stream() << "NEW\n";
        CHECK(f.commit(error));
        CHECK(!fs::exists(f.partialPath()));
    }
    CHECK_EQ(read(path), std::string("NEW\n"));
    // A metadata sidecar alone also blocks overwriting.
    const std::string other = (dir / "other.csv").string();
    {
        std::ofstream(metadataPathFor(other)) << "{}";
    }
    CHECK(!checkOutputsWritable({other}, false, error));
    // Unwritable location fails at open time.
    StagedFile bad;
    CHECK(!bad.open((dir / "missing" / "x.csv").string(), error));
    fs::remove_all(dir);
}

namespace {
bool collides(const std::vector<OutputRequest>& requests, std::string* message = nullptr) {
    std::string error;
    const bool ok = checkOutputPathCollisions(requests, error);
    if (message) *message = error;
    return !ok && !error.empty();
}
}  // namespace

TEST(output_path_collisions_are_detected) {
    std::string msg;
    // CSV destination vs CSV destination, including relative spellings.
    CHECK(collides({{"--csv", "a.csv"}, {"--grid-csv", "a.csv"}}));
    CHECK(collides({{"--csv", "a.csv"}, {"--grid-csv", "./a.csv"}}));
    CHECK(collides({{"--csv", "a.csv"}, {"--grid-csv", "sub/../a.csv"}}));
    CHECK(collides({{"--csv", "out/a.csv"}, {"--grid-csv", "out//./a.csv"}}));
    const std::string absolute = (std::filesystem::current_path() / "a.csv").string();
    CHECK(collides({{"--csv", "a.csv"}, {"--grid-csv", absolute}}));
    // CSV destination vs another output's metadata destination.
    CHECK(collides({{"--csv", "results.csv"}, {"--grid-csv", "results.csv.meta.json"}}, &msg));
    CHECK(msg.find("--csv metadata 'results.csv.meta.json'") != std::string::npos);
    CHECK(msg.find("--grid-csv destination 'results.csv.meta.json'") != std::string::npos);
    CHECK(collides({{"--csv", "results.csv.meta.json"}, {"--summary-csv", "results.csv"}}));
    // Metadata destination/staging vs another output's staging file.
    CHECK(collides({{"--csv", "r.csv"}, {"--paired-csv", "r.csv.meta.json"}}, &msg));
    CHECK(collides({{"--csv", "r.csv"}, {"--grid-csv", "r.csv.meta.json.partial"}}, &msg));
    CHECK(msg.find("metadata staging file") != std::string::npos);
    // Staging file vs another requested destination.
    CHECK(collides({{"--csv", "r.csv"}, {"--summary-csv", "r.csv.partial"}}, &msg));
    CHECK(msg.find("--csv staging file 'r.csv.partial'") != std::string::npos);
    CHECK(collides({{"--csv", "r.csv"}, {"--summary-csv", "./r.csv.partial"}}));

    // No false positives for merely similar names.
    std::string error;
    CHECK(checkOutputPathCollisions({{"--csv", "a.csv"}, {"--grid-csv", "a.csv2"}, {"--summary-csv", "b/a.csv"},
                                     {"--paired-csv", "a.csv.meta"}},
                                    error));
    CHECK(checkOutputPathCollisions({}, error));
    CHECK_EQ(plannedOutputFiles({{"--csv", "x.csv"}}).size(), std::size_t{4});
}

TEST(cli_rejects_output_path_collisions) {
    const std::vector<std::vector<std::string>> bad = {
        {"--benchmark", "--csv", "results.csv", "--grid-csv", "results.csv.meta.json"},
        {"--benchmark", "--csv", "results.csv", "--paired-csv", "results.csv.partial"},
        {"--benchmark", "--csv", "results.csv", "--summary-csv", "results.csv.meta.json.partial"},
        {"--benchmark", "--grid-csv", "g.csv", "--summary-csv", "./g.csv"},
    };
    for (const auto& args : bad) {
        CliOptions opt;
        std::string err;
        CHECK(!parseCommandLine(args, opt, err));
        CHECK(err.find("output path collision") != std::string::npos);
    }
    CliOptions opt;
    std::string err;
    CHECK(parseCommandLine({"--benchmark", "--csv", "r.csv", "--grid-csv", "g.csv", "--summary-csv", "s.csv",
                            "--paired-csv", "p.csv"},
                           opt, err));
}

TEST(staged_files_are_cleaned_up_on_failure_paths) {
    namespace fs = std::filesystem;
    const fs::path dir = fs::temp_directory_path() / ("pathfinding_cleanup_" + std::to_string(std::random_device{}()));
    fs::create_directories(dir);
    const std::string final = (dir / "out.csv").string();
    {
        std::ofstream(final) << "FINAL\n";
    }
    auto read = [](const std::string& p) {
        std::ifstream in(p);
        std::stringstream ss;
        ss << in.rdbuf();
        return ss.str();
    };
    std::string error;

    // Destroyed without commit: staging file removed, final untouched.
    {
        StagedFile f;
        CHECK(f.open(final, error));
        f.stream() << "rows\n";
        CHECK(fs::exists(f.partialPath()));
    }
    CHECK(!fs::exists(partialPathFor(final)));
    CHECK_EQ(read(final), std::string("FINAL\n"));

    // Explicit discard() is idempotent and never touches the final file.
    {
        StagedFile f;
        CHECK(f.open(final, error));
        f.discard();
        f.discard();
        CHECK(!fs::exists(f.partialPath()));
    }
    CHECK_EQ(read(final), std::string("FINAL\n"));

    // retain(): staging file kept (aborted benchmark keeps completed rows).
    const std::string aborted = (dir / "aborted.csv").string();  // no final file yet
    {
        StagedFile f;
        CHECK(f.open(aborted, error));
        f.stream() << "completed rows\n";
        f.stream().flush();
        f.retain();
    }
    CHECK_EQ(read(partialPathFor(aborted)), std::string("completed rows\n"));
    CHECK(!fs::exists(aborted));
    // The leftover staging file blocks a new run unless --force.
    CHECK(!checkOutputsWritable({aborted}, false, error));
    CHECK(error.find("aborted.csv.partial") != std::string::npos);
    CHECK(checkOutputsWritable({aborted}, true, error));

    // Committed: no staging file left, and the destructor does not remove
    // the final file.
    {
        StagedFile f;
        CHECK(f.open(final, error));
        f.stream() << "NEW\n";
        CHECK(f.commit(error));
    }
    CHECK_EQ(read(final), std::string("NEW\n"));
    CHECK(!fs::exists(partialPathFor(final)));

    // Open failure (staging path is a directory): nothing created, nothing
    // removed, and earlier staged files are cleaned up by their destructors.
    const std::string blocked = (dir / "blocked.csv").string();
    fs::create_directories(partialPathFor(blocked));
    {
        StagedFile first;
        StagedFile second;
        CHECK(first.open((dir / "first.csv").string(), error));
        CHECK(!second.open(blocked, error));
    }
    CHECK(!fs::exists(dir / "first.csv.partial"));
    CHECK(fs::is_directory(partialPathFor(blocked)));  // not ours: untouched
    fs::remove_all(dir);
}

TEST(metadata_json_contains_reproducibility_fields) {
    CHECK_EQ(jsonString("a\"b\\c\nd"), std::string("\"a\\\"b\\\\c\\nd\""));
    CHECK_EQ(jsonString(std::string("\x01", 1)), std::string("\"\\u0001\""));
    const TimerInfo t = calibrateTimer();
    CHECK(t.overheadNs > 0 || t.granularityNs > 0);
    CHECK(t.floorMs > 0);
    const EnvironmentInfo env = collectEnvironment();
    CHECK(!env.compiler.empty() && !env.timestampUtc.empty());
    BenchmarkConfig cfg;
    RunOutcome outcome;
    outcome.status = "complete";
    const std::string json = metadataJson(env, t, cfg, "baseline", {"a.csv"}, outcome);
    for (const char* key : {"\"benchmark_format_version\"", "\"git_commit\"", "\"git_dirty_at_build\"", "\"compiler\"",
                            "\"cxx_standard\"", "\"build_type\"", "\"os\"", "\"cpu_model\"", "\"timestamp_utc\"",
                            "\"base_seed\"", "\"grids_per_scenario\"", "\"runs_per_grid\"", "\"warmup_runs\"",
                            "\"scenarios\"", "\"algorithms\"", "\"timing_methodology\"", "\"validation\"",
                            "\"measured_overhead_ns\"", "\"status\""}) {
        if (json.find(key) == std::string::npos) std::cerr << "  missing " << key << "\n";
        CHECK(json.find(key) != std::string::npos);
    }
    int depth = 0;
    bool inString = false;
    for (std::size_t i = 0; i < json.size(); ++i) {
        if (json[i] == '\\' && inString) {
            ++i;
            continue;
        }
        if (json[i] == '"') inString = !inString;
        if (!inString && (json[i] == '{' || json[i] == '[')) ++depth;
        if (!inString && (json[i] == '}' || json[i] == ']')) --depth;
        CHECK(depth >= 0);
    }
    CHECK_EQ(depth, 0);
}

// ------------------------------------------------------------- CLI

namespace {
bool parses(const std::vector<std::string>& args, CliOptions& opt) {
    std::string err;
    return parseCommandLine(args, opt, err);
}
bool rejects(const std::vector<std::string>& args) {
    CliOptions opt;
    std::string err;
    const bool ok = parseCommandLine(args, opt, err);
    if (ok) std::cerr << "  expected rejection of:";
    if (ok) for (const auto& a : args) std::cerr << ' ' << a;
    if (ok) std::cerr << '\n';
    return !ok && !err.empty();
}
}  // namespace

TEST(cli_defaults_to_interactive) {
    CliOptions opt;
    CHECK(parses({}, opt));
    CHECK(opt.mode == Mode::Interactive);
    CHECK(parses({"--seed", "5"}, opt));
    CHECK(opt.mode == Mode::Interactive);
    CHECK(opt.seedGiven);
    CHECK_EQ(opt.bench.baseSeed, std::uint64_t{5});
    CHECK(parses({"--help"}, opt));
    CHECK(opt.mode == Mode::Help);
    CHECK(parses({"-h"}, opt));
    CHECK(opt.mode == Mode::Help);
}

TEST(cli_parses_full_benchmark_command) {
    CliOptions opt;
    // Largest base seed for which seeds base .. base+3 do not overflow.
    CHECK(parses({"--benchmark", "--algorithm", "astar,bfs", "--size", "50x120", "--density", "12.5", "--seed",
                  "18446744073709551612", "--runs", "30", "--grids", "4", "--warmup", "0", "--terrain", "10",
                  "--placement", "random", "--csv", "out.csv", "--summary-csv", "sum.csv", "--grid-csv", "g.csv",
                  "--paired-csv", "p.csv", "--force"},
                 opt));
    CHECK(opt.mode == Mode::Benchmark);
    CHECK_EQ(opt.bench.algorithms.size(), std::size_t{2});
    CHECK(opt.bench.algorithms[0] == Algorithm::AStar);
    CHECK_EQ(opt.bench.scenarios.size(), std::size_t{1});
    CHECK_EQ(opt.bench.scenarios[0].rows, 50);
    CHECK_EQ(opt.bench.scenarios[0].cols, 120);
    CHECK_EQ(opt.bench.scenarios[0].obstacleDensity, 0.125);
    CHECK_EQ(opt.bench.scenarios[0].terrainDensity, 0.1);
    CHECK(opt.bench.scenarios[0].placement == Placement::Random);
    CHECK_EQ(opt.bench.baseSeed, std::numeric_limits<std::uint64_t>::max() - 3);
    CHECK_EQ(opt.gridCsvPath, std::string("g.csv"));
    CHECK_EQ(opt.pairedPath, std::string("p.csv"));
    CHECK_EQ(opt.outputPaths().size(), std::size_t{4});
    CHECK_EQ(opt.bench.runs, 30);
    CHECK_EQ(opt.bench.grids, 4);
    CHECK_EQ(opt.bench.warmup, 0);
    CHECK_EQ(opt.csvPath, std::string("out.csv"));
    CHECK(opt.force);

    CHECK(parses({"--benchmark", "--suite", "density"}, opt));
    CHECK_EQ(opt.bench.scenarios.size(), std::size_t{5});
    for (const Scenario& sc : opt.bench.scenarios) CHECK(sc.placement == Placement::Reachable);
    CHECK(parses({"--benchmark", "--suite", "density-corners"}, opt));
    for (const Scenario& sc : opt.bench.scenarios) CHECK(sc.placement == Placement::Corners);
    // Defaults: reachable placement, 10 grids, 12 runs.
    CHECK(parses({"--benchmark"}, opt));
    CHECK(opt.bench.scenarios[0].placement == Placement::Reachable);
    CHECK_EQ(opt.bench.grids, 10);
    CHECK_EQ(opt.bench.runs, 12);
    CHECK(parses({"--benchmark", "--placement", "corners"}, opt));
    CHECK(opt.bench.scenarios[0].placement == Placement::Corners);
    CHECK(parses({"--benchmark", "--scenario", "maze", "--size", "51x51"}, opt));
    CHECK(opt.bench.scenarios[0].type == ScenarioType::Maze);
}

TEST(cli_rejects_invalid_input) {
    CHECK(rejects({"--benchmark", "--size", "100"}));
    CHECK(rejects({"--benchmark", "--size", "2x100"}));
    CHECK(rejects({"--benchmark", "--size", "100x-5"}));
    CHECK(rejects({"--benchmark", "--size", "10001x10"}));
    CHECK(rejects({"--benchmark", "--size", "5000x5000"}));  // > kMaxCells
    CHECK(rejects({"--benchmark", "--size", "10x10abc"}));
    CHECK(rejects({"--benchmark", "--density", "-1"}));
    CHECK(rejects({"--benchmark", "--density", "100.5"}));
    CHECK(rejects({"--benchmark", "--density", "abc"}));
    CHECK(rejects({"--benchmark", "--density", "nan"}));
    CHECK(rejects({"--benchmark", "--runs", "0"}));
    CHECK(rejects({"--benchmark", "--runs", "3.5"}));
    CHECK(rejects({"--benchmark", "--grids", "0"}));
    CHECK(rejects({"--benchmark", "--seed", "-1"}));
    CHECK(rejects({"--benchmark", "--seed", "18446744073709551616"}));  // 2^64
    CHECK(rejects({"--benchmark", "--seed", "18446744073709551613", "--grids", "4"}));  // seed range overflows
    CHECK(rejects({"--benchmark", "--placement", "anywhere"}));
    CHECK(rejects({"--benchmark", "--csv", "a.csv", "--paired-csv", "a.csv"}));
    CHECK(rejects({"--benchmark", "--grid-csv", "g.csv", "--summary-csv", "g.csv"}));
    CHECK(rejects({"--benchmark", "--algorithm", "greedy"}));
    CHECK(rejects({"--benchmark", "--algorithm", "bfs,"}));
    CHECK(rejects({"--benchmark", "--scenario", "spiral"}));
    CHECK(rejects({"--benchmark", "--suite", "nope"}));
    CHECK(rejects({"--benchmark", "--suite", "size", "--size", "10x10"}));
    CHECK(rejects({"--benchmark", "--scenario", "maze", "--density", "20"}));
    CHECK(rejects({"--benchmark", "--csv", "a.csv", "--summary-csv", "a.csv"}));
    CHECK(rejects({"--benchmark", "--runs"}));
    CHECK(rejects({"--benchmark", "--runs", "--csv"}));
    CHECK(rejects({"--benchmark", "--runs", "3", "--runs", "4"}));
    CHECK(rejects({"--benchmark", "--bogus"}));
    CHECK(rejects({"--benchmark", "--grids", "100000", "--runs", "100000"}));  // total work cap
    CHECK(rejects({"stray"}));
    CHECK(rejects({"--csv", "x.csv"}));  // benchmark-only option without --benchmark
}

// ------------------------------------------------------------- main

int main(int argc, char* argv[]) {
    const std::string filter = argc > 1 ? argv[1] : "";
    int ran = 0;
    int failedTests = 0;
    for (const TestCase& t : registry()) {
        if (!filter.empty() && std::string(t.name).find(filter) == std::string::npos) continue;
        const int before = g_failures;
        std::cout << "[ RUN  ] " << t.name << "\n";
        t.fn();
        const bool ok = g_failures == before;
        std::cout << (ok ? "[  OK  ] " : "[ FAIL ] ") << t.name << "\n";
        ++ran;
        if (!ok) ++failedTests;
    }
    std::cout << "\n" << ran - failedTests << "/" << ran << " tests passed, " << g_failures << " failed checks\n";
    return (failedTests == 0 && ran > 0) ? 0 : 1;
}
