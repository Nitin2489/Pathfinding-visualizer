#include "validation.h"

#include <cmath>
#include <deque>
#include <limits>
#include <utility>

namespace {

constexpr int kDr[4] = {-1, 0, 1, 0};
constexpr int kDc[4] = {0, 1, 0, -1};
constexpr int kMaxEntryCost = 10;  // largest value referenceEntryCost() returns

bool open(const Grid& g, int r, int c) { return g.isValidCell(r, c) && g.cellAt(r, c) != WALL; }

std::size_t at(const Grid& g, int r, int c) {
    return static_cast<std::size_t>(r) * static_cast<std::size_t>(g.cols()) + static_cast<std::size_t>(c);
}

}  // namespace

int referenceEntryCost(char cell) {
    switch (cell) {
        case EMPTY: return 1;
        case SAND: return 3;
        case WATER: return 5;
        case MOUNTAIN: return 10;
        default: return -1;  // WALL or unknown
    }
}

std::vector<int> labelComponents(const Grid& g) {
    std::vector<int> label(static_cast<std::size_t>(g.rows()) * static_cast<std::size_t>(g.cols()), -1);
    int next = 0;
    std::deque<std::pair<int, int>> frontier;
    for (int r = 0; r < g.rows(); ++r) {
        for (int c = 0; c < g.cols(); ++c) {
            if (!open(g, r, c) || label[at(g, r, c)] != -1) continue;
            label[at(g, r, c)] = next;
            frontier.emplace_back(r, c);
            while (!frontier.empty()) {
                const auto [cr, cc] = frontier.front();
                frontier.pop_front();
                for (int k = 0; k < 4; ++k) {
                    const int nr = cr + kDr[k];
                    const int nc = cc + kDc[k];
                    if (open(g, nr, nc) && label[at(g, nr, nc)] == -1) {
                        label[at(g, nr, nc)] = next;
                        frontier.emplace_back(nr, nc);
                    }
                }
            }
            ++next;
        }
    }
    return label;
}

ReferenceSolution computeReference(const Grid& g) {
    ReferenceSolution ref;
    for (int r = 0; r < g.rows(); ++r) {
        for (int c = 0; c < g.cols(); ++c) ref.passableCells += open(g, r, c) ? 1 : 0;
    }
    const Cell s = g.getStartPos();
    const Cell e = g.getEndPos();
    if (!open(g, s.row, s.col) || !open(g, e.row, e.col)) return ref;

    // Fewest moves and start-component size: plain BFS on (row, col) pairs.
    std::vector<int> moves(static_cast<std::size_t>(g.rows()) * static_cast<std::size_t>(g.cols()), -1);
    std::deque<std::pair<int, int>> queue;
    moves[at(g, s.row, s.col)] = 0;
    queue.emplace_back(s.row, s.col);
    while (!queue.empty()) {
        const auto [r, c] = queue.front();
        queue.pop_front();
        ++ref.startComponentSize;
        for (int k = 0; k < 4; ++k) {
            const int nr = r + kDr[k];
            const int nc = c + kDc[k];
            if (open(g, nr, nc) && moves[at(g, nr, nc)] == -1) {
                moves[at(g, nr, nc)] = moves[at(g, r, c)] + 1;
                queue.emplace_back(nr, nc);
            }
        }
    }
    ref.shortestMoves = moves[at(g, e.row, e.col)];
    ref.reachable = ref.shortestMoves >= 0;
    if (!ref.reachable) return ref;

    // Least cost: Dial's algorithm. Entry costs are integers in [1, 10],
    // so kMaxEntryCost + 1 circular buckets suffice.
    const long long inf = std::numeric_limits<long long>::max();
    std::vector<long long> cost(moves.size(), inf);
    std::vector<std::vector<std::pair<int, int>>> buckets(kMaxEntryCost + 1);
    cost[at(g, s.row, s.col)] = 0;
    buckets[0].emplace_back(s.row, s.col);
    std::size_t pending = 1;
    for (long long d = 0; pending > 0; ++d) {
        auto& bucket = buckets[static_cast<std::size_t>(d % (kMaxEntryCost + 1))];
        while (!bucket.empty()) {
            const auto [r, c] = bucket.back();
            bucket.pop_back();
            --pending;
            if (cost[at(g, r, c)] != d) continue;  // superseded entry
            for (int k = 0; k < 4; ++k) {
                const int nr = r + kDr[k];
                const int nc = c + kDc[k];
                if (!open(g, nr, nc)) continue;
                const long long nd = d + referenceEntryCost(g.cellAt(nr, nc));
                if (nd < cost[at(g, nr, nc)]) {
                    cost[at(g, nr, nc)] = nd;
                    buckets[static_cast<std::size_t>(nd % (kMaxEntryCost + 1))].emplace_back(nr, nc);
                    ++pending;
                }
            }
        }
    }
    ref.minimumCost = cost[at(g, e.row, e.col)];
    return ref;
}

std::vector<std::string> validateRun(const Grid& g, const ReferenceSolution& ref, Algorithm algorithm,
                                     const SearchResult& result) {
    std::vector<std::string> problems;
    auto fail = [&problems](std::string msg) { problems.push_back(std::move(msg)); };
    const SearchMetrics& m = result.metrics;
    const std::vector<Cell>& path = result.path;

    // Timing sanity.
    if (!std::isfinite(m.executionTimeMs) || !std::isfinite(m.searchTimeMs) || m.executionTimeMs < 0 ||
        m.searchTimeMs < 0) {
        fail("timing is negative or not finite");
    } else if (m.searchTimeMs > m.executionTimeMs) {
        fail("core search time exceeds end-to-end time");
    }

    // Reachability.
    if (m.pathFound != ref.reachable) {
        fail(std::string("pathFound=") + (m.pathFound ? "true" : "false") + " but reference says " +
             (ref.reachable ? "reachable" : "unreachable"));
    }

    if (m.pathFound) {
        // Path structure, recomputed independently.
        bool structural = true;
        if (path.empty()) {
            fail("pathFound but path is empty");
            structural = false;
        } else {
            if (path.front() != g.getStartPos()) {
                fail("path does not begin at the start");
                structural = false;
            }
            if (path.back() != g.getEndPos()) {
                fail("path does not end at the destination");
                structural = false;
            }
            std::vector<char> seen(static_cast<std::size_t>(g.rows()) * static_cast<std::size_t>(g.cols()), 0);
            for (std::size_t i = 0; i < path.size() && structural; ++i) {
                const Cell c = path[i];
                if (!g.isValidCell(c.row, c.col)) {
                    fail("path leaves the grid at step " + std::to_string(i));
                    structural = false;
                } else if (g.cellAt(c.row, c.col) == WALL) {
                    fail("path crosses a wall at step " + std::to_string(i));
                    structural = false;
                } else if (seen[at(g, c.row, c.col)]) {
                    fail("path revisits a cell at step " + std::to_string(i));
                    structural = false;
                } else if (i > 0 && std::abs(c.row - path[i - 1].row) + std::abs(c.col - path[i - 1].col) != 1) {
                    fail("non-adjacent move at step " + std::to_string(i));
                    structural = false;
                } else {
                    seen[at(g, c.row, c.col)] = 1;
                }
            }
        }
        if (structural) {
            const long long moves = static_cast<long long>(path.size()) - 1;
            long long cost = 0;
            for (std::size_t i = 1; i < path.size(); ++i) cost += referenceEntryCost(g.cellAt(path[i].row, path[i].col));
            if (moves != m.pathLength) {
                fail("reported pathLength " + std::to_string(m.pathLength) + " != path moves " + std::to_string(moves));
            }
            if (cost != m.pathCost) {
                fail("reported pathCost " + std::to_string(m.pathCost) + " != recomputed " + std::to_string(cost));
            }
            if (ref.reachable) {
                if (moves < ref.shortestMoves) fail("path is shorter than the reference minimum (reference bug?)");
                if (cost < ref.minimumCost) fail("path is cheaper than the reference minimum (reference bug?)");
                if (isCostOptimal(algorithm) && cost != ref.minimumCost) {
                    fail("not cost-optimal: cost " + std::to_string(cost) + ", reference minimum " +
                         std::to_string(ref.minimumCost));
                }
                if (algorithm == Algorithm::BFS && moves != ref.shortestMoves) {
                    fail("BFS not move-optimal: " + std::to_string(moves) + " moves, reference minimum " +
                         std::to_string(ref.shortestMoves));
                }
            }
            // Every path cell except the goal was expanded; all were discovered.
            if (m.nodesExpanded < moves) fail("nodesExpanded is smaller than the number of expanded path cells");
            if (m.nodesDiscovered < static_cast<long long>(path.size())) fail("nodesDiscovered is smaller than the path");
        }
    } else {
        if (!path.empty()) fail("no path reported but path is not empty");
        if (m.pathLength != -1 || m.pathCost != -1) fail("no path reported but pathLength/pathCost are not -1");
        // Without a goal to stop at, every reachable cell is discovered and expanded.
        if (m.nodesExpanded != ref.startComponentSize || m.nodesDiscovered != ref.startComponentSize) {
            fail("unsuccessful search did not exhaust the start component (" + std::to_string(ref.startComponentSize) +
                 " cells; discovered " + std::to_string(m.nodesDiscovered) + ", expanded " +
                 std::to_string(m.nodesExpanded) + ")");
        }
    }

    // Metric invariants (start is always passable in benchmark grids).
    if (m.nodesDiscovered < 1 || m.nodesDiscovered > ref.startComponentSize) {
        fail("nodesDiscovered " + std::to_string(m.nodesDiscovered) + " outside [1, " +
             std::to_string(ref.startComponentSize) + "]");
    }
    if (m.nodesExpanded < 0 || m.nodesExpanded > m.nodesDiscovered) fail("nodesExpanded exceeds nodesDiscovered");
    if (m.peakFrontierSize < 1 || static_cast<long long>(m.peakFrontierSize) > 1 + 4LL * m.nodesExpanded) {
        fail("peakFrontierSize " + std::to_string(m.peakFrontierSize) + " outside [1, 1 + 4 * nodesExpanded]");
    }
    if (algorithm == Algorithm::BFS && m.peakFrontierSize > m.nodesDiscovered) {
        fail("BFS frontier larger than nodesDiscovered (BFS never holds duplicates)");
    }
    return problems;
}
