#include "search.h"

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cstdlib>
#include <limits>
#include <queue>

namespace {

using Clock = std::chrono::steady_clock;

constexpr long long kInf = std::numeric_limits<long long>::max();

// Calls f(neighborIndex) for each in-bounds passable 4-neighbor of idx, in
// the order up, right, down, left. No allocation.
template <typename F>
inline void forEachNeighbor(const Grid& g, int idx, F&& f) {
    const int cols = g.cols();
    const int r = idx / cols;
    const int c = idx % cols;
    if (r > 0 && g.isPassableIndex(idx - cols)) f(idx - cols);
    if (c + 1 < cols && g.isPassableIndex(idx + 1)) f(idx + 1);
    if (r + 1 < g.rows() && g.isPassableIndex(idx + cols)) f(idx + cols);
    if (c > 0 && g.isPassableIndex(idx - 1)) f(idx - 1);
}

bool endpointsUsable(const Grid& g) {
    const Cell s = g.getStartPos();
    const Cell e = g.getEndPos();
    return g.isPassable(s.row, s.col) && g.isPassable(e.row, e.col);
}

// Builds the path from the parent links and fills the path metrics.
void reconstruct(const Grid& g, const std::vector<int>& parent, int startIdx, int goalIdx, SearchResult& out) {
    for (int at = goalIdx; at != -1; at = (at == startIdx) ? -1 : parent[at]) {
        out.path.push_back(g.cellOf(at));
    }
    std::reverse(out.path.begin(), out.path.end());
    out.metrics.pathFound = true;
    out.metrics.pathLength = static_cast<int>(out.path.size()) - 1;
    out.metrics.pathCost = computePathCost(g, out.path);
}

double elapsedMs(Clock::time_point since) {
    return std::chrono::duration<double, std::milli>(Clock::now() - since).count();
}

// Runs `body(out)` and records the end-to-end time: everything from entering
// the search to the moment its working state has been freed. `body` keeps
// its per-cell arrays as locals, so they are destroyed before the clock
// stops. The body records the core time itself (see SearchMetrics).
template <typename Body>
SearchResult timedSearch(Body&& body) {
    const auto t0 = Clock::now();
    SearchResult out;
    body(out);
    out.metrics.executionTimeMs = elapsedMs(t0);
    return out;
}

// Heap entry carrying a *snapshot* of its priority, so the heap ordering
// never depends on mutable external state. `seq` (insertion order) makes
// tie-breaking deterministic and FIFO among equal priorities.
struct HeapEntry {
    long long key;     // g (Dijkstra) or f = g + h (A*)
    long long tie;     // 0 (Dijkstra) or h (A*): prefer cells closer to the goal
    long long g;       // g at insertion time; used to detect stale entries
    std::uint64_t seq;
    int idx;
};

struct HeapGreater {
    bool operator()(const HeapEntry& a, const HeapEntry& b) const {
        if (a.key != b.key) return a.key > b.key;
        if (a.tie != b.tie) return a.tie > b.tie;
        return a.seq > b.seq;
    }
};

// Shared best-first search: Dijkstra when useHeuristic is false, A* otherwise.
SearchResult bestFirst(const Grid& g, ExpansionTrace* trace, bool useHeuristic) {
    // The end-to-end clock in timedSearch() also covers allocating and
    // freeing the per-cell arrays declared inside this lambda.
    return timedSearch([&](SearchResult& out) {
        if (!endpointsUsable(g)) return;
        const int n = g.cellCount();
        const Cell end = g.getEndPos();
        const int startIdx = g.index(g.getStartPos().row, g.getStartPos().col);
        const int goalIdx = g.index(end.row, end.col);
        const int cols = g.cols();

        auto heuristic = [&](int idx) -> long long {
            if (!useHeuristic) return 0;
            // Minimum entry cost is 1, so Manhattan distance never overestimates.
            return std::abs(idx / cols - end.row) + std::abs(idx % cols - end.col);
        };

        std::vector<long long> dist(static_cast<std::size_t>(n), kInf);
        std::vector<int> parent(static_cast<std::size_t>(n), -1);
        std::vector<char> discovered(static_cast<std::size_t>(n), 0);
        std::vector<char> closed(static_cast<std::size_t>(n), 0);
        std::priority_queue<HeapEntry, std::vector<HeapEntry>, HeapGreater> open;
        std::uint64_t seq = 0;
        std::size_t peak = 0;

        // Per-cell state is allocated and initialised above; the core
        // (search-only) clock starts here.
        const auto coreStart = Clock::now();
        dist[startIdx] = 0;
        discovered[startIdx] = 1;
        out.metrics.nodesDiscovered = 1;
        const long long h0 = heuristic(startIdx);
        open.push({h0, h0, 0, seq++, startIdx});
        peak = 1;

        bool found = false;
        while (!open.empty()) {
            const HeapEntry top = open.top();
            open.pop();
            // Stale entry: the cell was already finalised, or a cheaper route
            // was found after this entry was pushed.
            if (closed[top.idx] || top.g != dist[top.idx]) continue;
            if (top.idx == goalIdx) {
                found = true;
                break;
            }
            closed[top.idx] = 1;
            ++out.metrics.nodesExpanded;
            if (trace) trace->push_back(g.cellOf(top.idx));

            const long long gCur = top.g;
            forEachNeighbor(g, top.idx, [&](int nb) {
                if (closed[nb]) return;  // safe: the heuristic is consistent
                const long long cand = gCur + g.costAt(nb);
                if (cand < dist[nb]) {
                    dist[nb] = cand;
                    parent[nb] = top.idx;
                    if (!discovered[nb]) {
                        discovered[nb] = 1;
                        ++out.metrics.nodesDiscovered;
                    }
                    const long long h = heuristic(nb);
                    open.push({cand + h, h, cand, seq++, nb});
                    peak = std::max(peak, open.size());
                }
            });
        }

        out.metrics.peakFrontierSize = static_cast<int>(peak);
        if (found) reconstruct(g, parent, startIdx, goalIdx, out);
        out.metrics.searchTimeMs = elapsedMs(coreStart);
    });
}

}  // namespace

SearchResult dijkstraSearch(const Grid& grid, ExpansionTrace* trace) { return bestFirst(grid, trace, false); }

SearchResult astarSearch(const Grid& grid, ExpansionTrace* trace) { return bestFirst(grid, trace, true); }

SearchResult bfsSearch(const Grid& g, ExpansionTrace* trace) {
    // The end-to-end clock in timedSearch() also covers allocating and
    // freeing the per-cell arrays declared inside this lambda.
    return timedSearch([&](SearchResult& out) {
        if (!endpointsUsable(g)) return;
        const int n = g.cellCount();
        const int startIdx = g.index(g.getStartPos().row, g.getStartPos().col);
        const int goalIdx = g.index(g.getEndPos().row, g.getEndPos().col);

        std::vector<int> parent(static_cast<std::size_t>(n), -1);
        std::vector<char> discovered(static_cast<std::size_t>(n), 0);
        // FIFO queue as a vector with a moving head: each cell is enqueued at
        // most once, so the vector never exceeds n entries.
        std::vector<int> queue;
        queue.reserve(64);
        std::size_t head = 0;
        std::size_t peak = 1;

        // Per-cell state is allocated and initialised above; the core
        // (search-only) clock starts here.
        const auto coreStart = Clock::now();
        queue.push_back(startIdx);
        discovered[startIdx] = 1;
        out.metrics.nodesDiscovered = 1;

        bool found = false;
        while (head < queue.size()) {
            const int cur = queue[head++];
            if (cur == goalIdx) {
                found = true;
                break;
            }
            ++out.metrics.nodesExpanded;
            if (trace) trace->push_back(g.cellOf(cur));
            forEachNeighbor(g, cur, [&](int nb) {
                if (discovered[nb]) return;
                discovered[nb] = 1;
                ++out.metrics.nodesDiscovered;
                parent[nb] = cur;
                queue.push_back(nb);
            });
            peak = std::max(peak, queue.size() - head);
        }

        out.metrics.peakFrontierSize = static_cast<int>(peak);
        if (found) reconstruct(g, parent, startIdx, goalIdx, out);
        out.metrics.searchTimeMs = elapsedMs(coreStart);
    });
}

SearchResult dfsSearch(const Grid& g, ExpansionTrace* trace) {
    // The end-to-end clock in timedSearch() also covers allocating and
    // freeing the per-cell arrays declared inside this lambda.
    return timedSearch([&](SearchResult& out) {
        if (!endpointsUsable(g)) return;
        const int n = g.cellCount();
        const int startIdx = g.index(g.getStartPos().row, g.getStartPos().col);
        const int goalIdx = g.index(g.getEndPos().row, g.getEndPos().col);

        struct Entry {
            int idx;
            int from;
        };
        std::vector<int> parent(static_cast<std::size_t>(n), -1);
        std::vector<char> discovered(static_cast<std::size_t>(n), 0);
        std::vector<char> visited(static_cast<std::size_t>(n), 0);
        // A cell may sit on the stack several times (pushed from different
        // parents); only the first pop is processed, the rest are stale.
        std::vector<Entry> stack;
        stack.reserve(64);
        std::size_t peak = 1;

        // Per-cell state is allocated and initialised above; the core
        // (search-only) clock starts here.
        const auto coreStart = Clock::now();
        stack.push_back({startIdx, -1});
        discovered[startIdx] = 1;
        out.metrics.nodesDiscovered = 1;

        bool found = false;
        while (!stack.empty()) {
            const Entry e = stack.back();
            stack.pop_back();
            if (visited[e.idx]) continue;  // stale duplicate
            visited[e.idx] = 1;
            parent[e.idx] = e.from;  // tree edge: e.from was expanded earlier
            if (e.idx == goalIdx) {
                found = true;
                break;
            }
            ++out.metrics.nodesExpanded;
            if (trace) trace->push_back(g.cellOf(e.idx));

            // Push in reverse so the first neighbor (up) is explored first.
            int nbs[4];
            int count = 0;
            forEachNeighbor(g, e.idx, [&](int nb) { nbs[count++] = nb; });
            for (int i = count - 1; i >= 0; --i) {
                const int nb = nbs[i];
                if (visited[nb]) continue;
                if (!discovered[nb]) {
                    discovered[nb] = 1;
                    ++out.metrics.nodesDiscovered;
                }
                stack.push_back({nb, e.idx});
            }
            peak = std::max(peak, stack.size());
        }

        out.metrics.peakFrontierSize = static_cast<int>(peak);
        if (found) reconstruct(g, parent, startIdx, goalIdx, out);
        out.metrics.searchTimeMs = elapsedMs(coreStart);
    });
}

SearchResult runSearch(Algorithm algorithm, const Grid& grid, ExpansionTrace* trace) {
    switch (algorithm) {
        case Algorithm::Dijkstra: return dijkstraSearch(grid, trace);
        case Algorithm::AStar: return astarSearch(grid, trace);
        case Algorithm::BFS: return bfsSearch(grid, trace);
        case Algorithm::DFS: return dfsSearch(grid, trace);
    }
    return SearchResult{};
}

const char* algorithmName(Algorithm algorithm) {
    switch (algorithm) {
        case Algorithm::Dijkstra: return "dijkstra";
        case Algorithm::AStar: return "astar";
        case Algorithm::BFS: return "bfs";
        case Algorithm::DFS: return "dfs";
    }
    return "unknown";
}

const char* algorithmDisplayName(Algorithm algorithm) {
    switch (algorithm) {
        case Algorithm::Dijkstra: return "Dijkstra's Algorithm";
        case Algorithm::AStar: return "A* Search";
        case Algorithm::BFS: return "Breadth-First Search";
        case Algorithm::DFS: return "Depth-First Search";
    }
    return "Unknown";
}

bool parseAlgorithm(const std::string& text, Algorithm& out) {
    std::string s;
    for (char ch : text) s += static_cast<char>(std::tolower(static_cast<unsigned char>(ch)));
    if (s == "dijkstra") out = Algorithm::Dijkstra;
    else if (s == "astar" || s == "a*" || s == "a-star") out = Algorithm::AStar;
    else if (s == "bfs") out = Algorithm::BFS;
    else if (s == "dfs") out = Algorithm::DFS;
    else return false;
    return true;
}

const std::vector<Algorithm>& allAlgorithms() {
    static const std::vector<Algorithm> all = {Algorithm::Dijkstra, Algorithm::AStar, Algorithm::BFS, Algorithm::DFS};
    return all;
}

bool isCostOptimal(Algorithm algorithm) { return algorithm == Algorithm::Dijkstra || algorithm == Algorithm::AStar; }

bool validatePath(const Grid& grid, const std::vector<Cell>& path, std::string* error) {
    auto fail = [error](const std::string& msg) {
        if (error) *error = msg;
        return false;
    };
    if (path.empty()) return fail("path is empty");
    if (path.front() != grid.getStartPos()) return fail("path does not begin at the start");
    if (path.back() != grid.getEndPos()) return fail("path does not end at the destination");
    std::vector<char> seen(static_cast<std::size_t>(grid.cellCount()), 0);
    for (std::size_t i = 0; i < path.size(); ++i) {
        const Cell c = path[i];
        if (!grid.isValidCell(c.row, c.col)) return fail("path leaves the grid at step " + std::to_string(i));
        if (grid.isWall(c.row, c.col)) return fail("path crosses a wall at step " + std::to_string(i));
        char& visited = seen[static_cast<std::size_t>(grid.index(c.row, c.col))];
        if (visited) return fail("path revisits a cell at step " + std::to_string(i));
        visited = 1;
        if (i > 0) {
            const Cell p = path[i - 1];
            if (std::abs(p.row - c.row) + std::abs(p.col - c.col) != 1) {
                return fail("non-adjacent move at step " + std::to_string(i));
            }
        }
    }
    return true;
}

long long computePathCost(const Grid& grid, const std::vector<Cell>& path) {
    if (path.empty()) return -1;
    long long cost = 0;
    for (std::size_t i = 0; i < path.size(); ++i) {
        // Caller-supplied cells: check bounds before indexing.
        if (!grid.isPassable(path[i].row, path[i].col)) return -1;
        if (i > 0) cost += grid.costAt(grid.index(path[i].row, path[i].col));
    }
    return cost;
}
