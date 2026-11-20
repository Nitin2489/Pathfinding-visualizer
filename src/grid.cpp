#include "grid.h"

#include <stdexcept>

namespace {

// Throws unless minSide <= rows, minSide <= cols and rows * cols <= kMaxCells.
// Works in 64-bit before anything is narrowed to int or allocated.
void validateDimensions(long long rows, long long cols, long long minSide) {
    if (rows < minSide || cols < minSide) {
        throw std::invalid_argument("grid dimensions " + std::to_string(rows) + "x" + std::to_string(cols) +
                                    " are too small (minimum " + std::to_string(minSide) + "x" +
                                    std::to_string(minSide) + ")");
    }
    // rows, cols >= 1 here; dividing avoids overflow in rows * cols.
    if (rows > Grid::kMaxCells / cols) {
        throw std::invalid_argument("grid dimensions " + std::to_string(rows) + "x" + std::to_string(cols) +
                                    " exceed the maximum of " + std::to_string(Grid::kMaxCells) + " cells");
    }
}

}  // namespace

bool isValidTerrain(char terrain) {
    switch (terrain) {
        case EMPTY:
        case WALL:
        case WATER:
        case SAND:
        case MOUNTAIN: return true;
        default: return false;
    }
}

int getTerrainCost(char terrain) {
    switch (terrain) {
        case WATER: return 5;
        case SAND: return 3;
        case MOUNTAIN: return 10;
        case EMPTY:
        default: return 1;
    }
}

std::uint64_t Rng::uniformInt(std::uint64_t n) {
    // 2^64 mod n; values below this threshold would bias the modulo.
    const std::uint64_t threshold = (0 - n) % n;
    for (;;) {
        const std::uint64_t r = engine_();
        if (r >= threshold) return r % n;
    }
}

double Rng::uniformReal() {
    return static_cast<double>(engine_() >> 11) * (1.0 / 9007199254740992.0);  // 2^-53
}

Grid::Grid() : Grid(ROWS, COLS) {}

Grid::Grid(NoBorder, long long rows, long long cols) {
    validateDimensions(rows, cols, 1);
    rows_ = static_cast<int>(rows);
    cols_ = static_cast<int>(cols);
    cells_.assign(static_cast<std::size_t>(rows) * static_cast<std::size_t>(cols), EMPTY);
}

Grid::Grid(int rows, int cols) {
    // 3x3 = border walls plus one interior cell.
    validateDimensions(rows, cols, 3);
    rows_ = rows;
    cols_ = cols;
    cells_.assign(static_cast<std::size_t>(rows) * static_cast<std::size_t>(cols), EMPTY);
    for (int r = 0; r < rows_; ++r) {
        cells_[index(r, 0)] = WALL;
        cells_[index(r, cols_ - 1)] = WALL;
    }
    for (int c = 0; c < cols_; ++c) {
        cells_[index(0, c)] = WALL;
        cells_[index(rows_ - 1, c)] = WALL;
    }
    start_ = Cell{1, 1};
    end_ = Cell{rows_ - 2, cols_ - 2};
}

Grid Grid::fromStrings(const std::vector<std::string>& lines) {
    if (lines.empty()) throw std::invalid_argument("grid text has no rows");
    const std::size_t width = lines[0].size();
    if (width == 0) throw std::invalid_argument("grid text has an empty first row");
    // Reject oversized input before any size is narrowed to long long / int.
    const auto maxCells = static_cast<std::size_t>(kMaxCells);
    if (lines.size() > maxCells || width > maxCells) {
        throw std::invalid_argument("grid text exceeds the maximum of " + std::to_string(kMaxCells) + " cells");
    }
    for (std::size_t r = 0; r < lines.size(); ++r) {
        if (lines[r].size() != width) {
            throw std::invalid_argument("grid row " + std::to_string(r) + " has length " +
                                        std::to_string(lines[r].size()) + ", expected " + std::to_string(width));
        }
    }
    Grid g(NoBorder{}, static_cast<long long>(lines.size()), static_cast<long long>(width));

    int starts = 0;
    int ends = 0;
    for (int r = 0; r < g.rows_; ++r) {
        for (int c = 0; c < g.cols_; ++c) {
            const char ch = lines[r][c];
            char cell = EMPTY;
            switch (ch) {
                case 'S': g.start_ = Cell{r, c}; ++starts; break;
                case 'E': g.end_ = Cell{r, c}; ++ends; break;
                case '.':
                case ' ': break;
                case WALL:
                case WATER:
                case SAND:
                case MOUNTAIN: cell = ch; break;
                default:
                    throw std::invalid_argument("unknown grid character (code " +
                                                std::to_string(static_cast<int>(static_cast<unsigned char>(ch))) +
                                                ") at row " + std::to_string(r) + ", column " + std::to_string(c));
            }
            g.cells_[static_cast<std::size_t>(g.index(r, c))] = cell;
        }
    }
    if (starts != 1) throw std::invalid_argument("grid text needs exactly one 'S', found " + std::to_string(starts));
    if (ends > 1) throw std::invalid_argument("grid text allows at most one 'E', found " + std::to_string(ends));
    if (ends == 0) g.end_ = g.start_;
    return g;
}

bool Grid::setStart(Cell c) {
    if (!isValidCell(c.row, c.col)) return false;
    start_ = c;
    cells_[index(c.row, c.col)] = EMPTY;
    return true;
}

bool Grid::setEnd(Cell c) {
    if (!isValidCell(c.row, c.col)) return false;
    end_ = c;
    cells_[index(c.row, c.col)] = EMPTY;
    return true;
}

bool Grid::addWall(int row, int col) { return addTerrain(row, col, WALL); }

bool Grid::addTerrain(int row, int col, char terrainType) {
    if (!isValidTerrain(terrainType) || !isValidCell(row, col) || isStartOrEnd(row, col)) return false;
    cells_[static_cast<std::size_t>(index(row, col))] = terrainType;
    return true;
}

void Grid::clear() {
    for (int r = 1; r < rows_ - 1; ++r) {
        for (int c = 1; c < cols_ - 1; ++c) {
            cells_[index(r, c)] = EMPTY;
        }
    }
}

void Grid::generateRandomWalls(double probability, Rng& rng) {
    for (int r = 1; r < rows_ - 1; ++r) {
        for (int c = 1; c < cols_ - 1; ++c) {
            // Draw for every cell (even start/end) so the random stream does
            // not depend on where start and end are.
            const bool wall = rng.chance(probability);
            if (wall && !isStartOrEnd(r, c)) cells_[index(r, c)] = WALL;
        }
    }
}

void Grid::generateRandomTerrain(double probability, Rng& rng) {
    static const char kTypes[] = {WATER, SAND, MOUNTAIN};
    for (int r = 1; r < rows_ - 1; ++r) {
        for (int c = 1; c < cols_ - 1; ++c) {
            if (cells_[index(r, c)] == WALL) continue;
            if (!rng.chance(probability)) continue;
            const char t = kTypes[rng.uniformInt(3)];
            if (!isStartOrEnd(r, c)) cells_[index(r, c)] = t;
        }
    }
}

void Grid::generateMaze(Rng& rng, int height, int width) {
    clear();
    const int r1 = (height <= 0 || height > rows_ - 2) ? rows_ - 2 : height;
    const int c1 = (width <= 0 || width > cols_ - 2) ? cols_ - 2 : width;
    if (r1 >= 1 && c1 >= 1) divide(rng, 1, 1, r1, c1);
}

// Recursive division over the inclusive region [r0, r1] x [c0, c1], where
// r0 and c0 are always odd. Walls go on even rows/cols strictly inside the
// region, and the single passage through each wall is on an odd coordinate.
// Since sub-walls are again on even coordinates, they can never cover a
// parent passage or the cells next to it, so each region stays connected.
void Grid::divide(Rng& rng, int r0, int c0, int r1, int c1) {
    // Even rows in [r0+1, r1-1] and even cols in [c0+1, c1-1].
    const int rowChoices = (r1 - 1 >= r0 + 1) ? ((r1 - 1) - (r0 + 1)) / 2 + 1 : 0;
    const int colChoices = (c1 - 1 >= c0 + 1) ? ((c1 - 1) - (c0 + 1)) / 2 + 1 : 0;
    if (rowChoices == 0 && colChoices == 0) return;

    const int height = r1 - r0 + 1;
    const int width = c1 - c0 + 1;
    bool horizontal;
    if (rowChoices == 0) horizontal = false;
    else if (colChoices == 0) horizontal = true;
    else if (width < height) horizontal = true;
    else if (height < width) horizontal = false;
    else horizontal = rng.uniformInt(2) == 0;

    if (horizontal) {
        const int wr = r0 + 1 + 2 * static_cast<int>(rng.uniformInt(static_cast<std::uint64_t>(rowChoices)));
        const int passages = (c1 - c0) / 2 + 1;  // odd cols in [c0, c1]
        const int pc = c0 + 2 * static_cast<int>(rng.uniformInt(static_cast<std::uint64_t>(passages)));
        for (int c = c0; c <= c1; ++c) {
            if (c != pc) addWall(wr, c);
        }
        divide(rng, r0, c0, wr - 1, c1);
        divide(rng, wr + 1, c0, r1, c1);
    } else {
        const int wc = c0 + 1 + 2 * static_cast<int>(rng.uniformInt(static_cast<std::uint64_t>(colChoices)));
        const int passages = (r1 - r0) / 2 + 1;  // odd rows in [r0, r1]
        const int pr = r0 + 2 * static_cast<int>(rng.uniformInt(static_cast<std::uint64_t>(passages)));
        for (int r = r0; r <= r1; ++r) {
            if (r != pr) addWall(r, wc);
        }
        divide(rng, r0, c0, r1, wc - 1);
        divide(rng, r0, wc + 1, r1, c1);
    }
}

bool Grid::hasWeightedTerrain() const {
    for (char ch : cells_) {
        if (ch != WALL && getTerrainCost(ch) != 1) return true;
    }
    return false;
}

int Grid::passableCount() const {
    int n = 0;
    for (char ch : cells_) n += (ch != WALL) ? 1 : 0;
    return n;
}

std::uint64_t Grid::fingerprint() const {
    // Standard FNV-1a 64-bit offset basis and prime.
    std::uint64_t h = 14695981039346656037ULL;
    auto mix = [&h](std::uint64_t v) {
        for (int i = 0; i < 8; ++i) {
            h ^= (v >> (8 * i)) & 0xFFu;
            h *= 1099511628211ULL;
        }
    };
    mix(static_cast<std::uint64_t>(rows_));
    mix(static_cast<std::uint64_t>(cols_));
    mix(static_cast<std::uint64_t>(index(start_.row, start_.col)));
    mix(static_cast<std::uint64_t>(index(end_.row, end_.col)));
    for (char ch : cells_) {
        h ^= static_cast<unsigned char>(ch);
        h *= 1099511628211ULL;  // 0x100000001b3
    }
    return h;
}
