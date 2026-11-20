// Grid / domain model: cell contents, start and end positions, neighbor
// access, traversal costs and reproducible (seeded) grid generation.
//
// The grid knows nothing about rendering, searching or benchmarking.
#pragma once

#include <cassert>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <random>
#include <string>
#include <vector>

// Default dimensions of the interactive grid. The benchmark engine and the
// search algorithms accept any size; these only apply to `Grid()`.
constexpr int ROWS = 20;
constexpr int COLS = 50;

// Cell contents. Start/end are stored as positions, not as cell contents,
// so they never hide terrain and can never be overwritten by a wall.
constexpr char EMPTY = ' ';
constexpr char WALL = '#';
constexpr char WATER = 'W';
constexpr char SAND = '~';   // was 'S' in v1, which collided with the start marker
constexpr char MOUNTAIN = 'M';

struct Cell {
    int row = 0;
    int col = 0;
};

inline bool operator==(const Cell& a, const Cell& b) { return a.row == b.row && a.col == b.col; }
inline bool operator!=(const Cell& a, const Cell& b) { return !(a == b); }

// True for the five cell contents a grid may hold: EMPTY, WALL, WATER,
// SAND, MOUNTAIN.
bool isValidTerrain(char terrain);

// Cost of *entering* a cell of the given type (walls are impassable and
// have no cost). All costs are >= 1, which keeps Manhattan distance an
// admissible and consistent A* heuristic.
int getTerrainCost(char terrain);

// Deterministic, platform-independent random source.
//
// std::mt19937_64's output sequence is fixed by the C++ standard, but the
// std::*_distribution classes are implementation-defined (libstdc++, libc++
// and MSVC produce different values). We therefore derive integers and
// probabilities directly from the raw engine output so that the same seed
// generates the same grid on every platform.
class Rng {
public:
    explicit Rng(std::uint64_t seed) : engine_(seed) {}
    // Uniform integer in [0, n). Requires n > 0. Uses rejection sampling to
    // avoid modulo bias.
    std::uint64_t uniformInt(std::uint64_t n);
    // Uniform double in [0, 1) with 53 bits of precision.
    double uniformReal();
    // True with probability p.
    bool chance(double p) { return uniformReal() < p; }

private:
    std::mt19937_64 engine_;
};

// Bounds and preconditions
// -------------------------
// Every grid satisfies 1 <= rows, 1 <= cols and rows * cols <= kMaxCells
// (enforced by both constructors and fromStrings). Hence cellCount(),
// every valid flat index, and every count the search layer derives from
// them (frontier entries, at most ~4 per cell) fit in an int.
//
// The accessors below marked "unchecked" are on the search hot path and do
// not bounds-check in release builds. Their preconditions are asserted, so
// Debug and sanitizer builds trap violations. Use isValidCell() /
// isPassable() (always safe) when a coordinate may be out of range.
class Grid {
public:
    // Largest accepted rows * cols: a quarter of INT_MAX, so that up to four
    // frontier entries per cell still fit in an int.
    static constexpr long long kMaxCells = (std::numeric_limits<int>::max() - 1) / 4;

    // Interactive default: ROWS x COLS, border walls, start (1,1),
    // end (ROWS-2, COLS-2).
    Grid();
    // rows x cols grid with border walls; start at (1,1) and end at
    // (rows-2, cols-2). Throws std::invalid_argument unless rows >= 3,
    // cols >= 3 and rows * cols <= kMaxCells (checked before allocating).
    Grid(int rows, int cols);

    // Builds a grid from text, one string per row (all rows equal length).
    // Legend: 'S' start, 'E' end, '#' wall, '.' or ' ' empty,
    // 'W' water, '~' sand, 'M' mountain. No border is added, so any size
    // from 1x1 up to kMaxCells cells is allowed. Exactly one 'S' is
    // required and at most one 'E'; with no 'E', end == start.
    // Throws std::invalid_argument on an empty input, an empty or ragged
    // row, an unknown character, a bad S/E count or too many cells.
    static Grid fromStrings(const std::vector<std::string>& lines);

    int rows() const { return rows_; }
    int cols() const { return cols_; }
    int cellCount() const { return rows_ * cols_; }  // <= kMaxCells, no overflow
    // Unchecked. Requires isValidCell(row, col).
    int index(int row, int col) const {
        assert(isValidCell(row, col));
        return row * cols_ + col;
    }
    // Unchecked. Requires 0 <= idx < cellCount().
    Cell cellOf(int idx) const {
        assert(idx >= 0 && idx < cellCount());
        return Cell{idx / cols_, idx % cols_};
    }

    // Always safe.
    bool isValidCell(int row, int col) const {
        return row >= 0 && row < rows_ && col >= 0 && col < cols_;
    }
    // Unchecked. Requires isValidCell(row, col).
    bool isWall(int row, int col) const { return cells_[index(row, col)] == WALL; }
    // Always safe: false for out-of-range coordinates.
    bool isPassable(int row, int col) const {
        return isValidCell(row, col) && cells_[index(row, col)] != WALL;
    }
    // Unchecked. Requires isValidCell(row, col).
    char cellAt(int row, int col) const { return cells_[index(row, col)]; }
    // Unchecked. Cost of entering the cell at flat index `idx`. Requires
    // 0 <= idx < cellCount(); meaningful only for passable cells.
    int costAt(int idx) const {
        assert(idx >= 0 && idx < cellCount());
        return getTerrainCost(cells_[static_cast<std::size_t>(idx)]);
    }
    // Unchecked. Requires 0 <= idx < cellCount().
    bool isPassableIndex(int idx) const {
        assert(idx >= 0 && idx < cellCount());
        return cells_[static_cast<std::size_t>(idx)] != WALL;
    }

    Cell getStartPos() const { return start_; }
    Cell getEndPos() const { return end_; }
    // Moves start/end. The target cell is cleared to EMPTY. Returns false
    // (and changes nothing) if the cell is out of bounds.
    bool setStart(Cell c);
    bool setEnd(Cell c);

    // Sets the cell to `terrainType`. Returns true if the cell was set;
    // returns false and leaves the grid unchanged if the coordinates are
    // out of bounds, the cell is the start or end, or `terrainType` is not
    // one of EMPTY, WALL, WATER, SAND, MOUNTAIN (see isValidTerrain()).
    // v1 callers that ignore the result keep working.
    bool addTerrain(int row, int col, char terrainType);
    // addTerrain(row, col, WALL).
    bool addWall(int row, int col);

    // Removes all interior walls and terrain; border walls are kept if the
    // grid was created with them (i.e. the outermost ring is left untouched).
    void clear();

    // Each interior cell (except start/end) becomes a wall with the given
    // probability in [0, 1]. Existing contents are kept otherwise.
    void generateRandomWalls(double probability, Rng& rng);
    // Each interior non-wall cell (except start/end) becomes WATER, SAND or
    // MOUNTAIN (equally likely) with the given probability.
    void generateRandomTerrain(double probability, Rng& rng);
    // Clears the interior, then builds a recursive-division maze in the
    // region rows [1, min(height, rows-2)] x cols [1, min(width, cols-2)]
    // (the whole interior by default). Walls are placed on even
    // coordinates and passages on odd ones, so every open cell of the maze
    // stays connected. Start/end are never walled.
    void generateMaze(Rng& rng, int height = -1, int width = -1);

    // True if any passable cell has a cost other than 1.
    bool hasWeightedTerrain() const;
    // Number of non-wall cells.
    int passableCount() const;
    // Standard 64-bit FNV-1a (offset basis 0xcbf29ce484222325, prime
    // 0x100000001b3) over: rows, cols, start index, end index (each as 8
    // little-endian bytes), then one byte per cell in row-major order. Two grids
    // with the same fingerprint are (with overwhelming probability)
    // identical; used to prove algorithms were compared on the same grid.
    std::uint64_t fingerprint() const;

private:
    struct NoBorder {};
    // rows x cols of EMPTY with no border; validates the dimensions.
    Grid(NoBorder, long long rows, long long cols);
    bool isStartOrEnd(int row, int col) const {
        return (row == start_.row && col == start_.col) || (row == end_.row && col == end_.col);
    }
    void divide(Rng& rng, int r0, int c0, int r1, int c1);

    int rows_ = 0;
    int cols_ = 0;
    std::vector<char> cells_;
    Cell start_;
    Cell end_;
};
