#include "visualizer.h"

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <iomanip>
#include <iostream>
#include <limits>
#include <string>
#include <thread>

#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>
#endif

namespace {

const char* backgroundFor(char content) {
    switch (content) {
        case WALL: return "\033[47m";
        case WATER: return "\033[46m";
        case SAND: return "\033[103m";
        case MOUNTAIN: return "\033[45m";
        default: return nullptr;
    }
}

char glyphFor(char content) {
    switch (content) {
        case WATER: return '~';
        case SAND: return ':';
        case MOUNTAIN: return '^';
        default: return ' ';
    }
}

void sleepMs(int ms) { std::this_thread::sleep_for(std::chrono::milliseconds(ms)); }

void printMetrics(const SearchMetrics& m) {
    std::cout << std::fixed << std::setprecision(3);
    std::cout << (m.pathFound ? "Path found!" : "No path found!") << '\n';
    std::cout << "Nodes discovered:   " << m.nodesDiscovered << '\n';
    std::cout << "Nodes expanded:     " << m.nodesExpanded << '\n';
    if (m.pathFound) {
        std::cout << "Path length:        " << m.pathLength << " moves\n";
        std::cout << "Path cost:          " << m.pathCost << '\n';
    }
    std::cout << "Peak frontier size: " << m.peakFrontierSize << '\n';
    std::cout << "Search time:        " << m.searchTimeMs << " ms core, " << m.executionTimeMs
              << " ms end-to-end (animation excluded)\n";
    std::cout.unsetf(std::ios::floatfield);
}

// Searches first (timed, no rendering), then animates the recorded result.
bool runAnimated(Grid& grid, Algorithm algorithm, int animationSpeed) {
    ExpansionTrace trace;
    const SearchResult result = runSearch(algorithm, grid, &trace);

    std::vector<char> overlay(static_cast<std::size_t>(grid.cellCount()), 0);
    for (std::size_t i = 0; i < trace.size(); ++i) {
        overlay[grid.index(trace[i].row, trace[i].col)] = VISITED;
        if (i % 5 == 0) {
            renderGrid(std::cout, grid, &overlay);
            sleepMs(animationSpeed);
        }
    }
    // Draw the path from the destination back to the start, as in v1.
    for (std::size_t i = result.path.size(); i-- > 0;) {
        overlay[grid.index(result.path[i].row, result.path[i].col)] = PATH;
        renderGrid(std::cout, grid, &overlay);
        sleepMs(animationSpeed);
    }
    renderGrid(std::cout, grid, &overlay);
    std::cout << algorithmDisplayName(algorithm) << '\n';
    printMetrics(result.metrics);
    return result.metrics.pathFound;
}

void waitForEnter() {
    std::cout << "Press Enter to continue..." << std::flush;
    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
    std::cin.get();
}

}  // namespace

void renderGrid(std::ostream& out, const Grid& grid, const std::vector<char>* overlay) {
    // Build the whole frame first and write it once to reduce flicker.
    std::string frame = "\033[H\033[J";
    frame.reserve(static_cast<std::size_t>(grid.cellCount()) * 8 + 16);
    const Cell s = grid.getStartPos();
    const Cell e = grid.getEndPos();
    for (int r = 0; r < grid.rows(); ++r) {
        for (int c = 0; c < grid.cols(); ++c) {
            const char content = grid.cellAt(r, c);
            const char mark = overlay ? (*overlay)[grid.index(r, c)] : 0;
            if (r == s.row && c == s.col) {
                frame += "\033[42mS\033[0m";
            } else if (r == e.row && c == e.col) {
                frame += "\033[41mE\033[0m";
            } else if (mark == PATH) {
                frame += "\033[43m";
                frame += glyphFor(content);
                frame += "\033[0m";
            } else if (mark == VISITED) {
                frame += "\033[44m";
                frame += glyphFor(content);
                frame += "\033[0m";
            } else if (const char* bg = backgroundFor(content)) {
                frame += bg;
                frame += glyphFor(content);
                frame += "\033[0m";
            } else {
                frame += ' ';
            }
        }
        frame += '\n';
    }
    out << frame << std::flush;
}

bool dijkstra(Grid& grid, int animationSpeed) { return runAnimated(grid, Algorithm::Dijkstra, animationSpeed); }
bool astar(Grid& grid, int animationSpeed) { return runAnimated(grid, Algorithm::AStar, animationSpeed); }
bool bfs(Grid& grid, int animationSpeed) { return runAnimated(grid, Algorithm::BFS, animationSpeed); }
bool dfs(Grid& grid, int animationSpeed) { return runAnimated(grid, Algorithm::DFS, animationSpeed); }

int runInteractive(std::uint64_t seed) {
    std::ios::sync_with_stdio(false);

#ifdef _WIN32
    system("cls");
    // Enable ANSI escape sequences on Windows 10+ consoles.
    HANDLE hOut = GetStdHandle(STD_OUTPUT_HANDLE);
    DWORD dwMode = 0;
    if (GetConsoleMode(hOut, &dwMode)) SetConsoleMode(hOut, dwMode | ENABLE_VIRTUAL_TERMINAL_PROCESSING);
#endif

    Rng rng(seed);
    Grid grid;
    int animationSpeed = 50;  // milliseconds
    bool running = true;

    while (running) {
        std::cout << "Pathfinding Visualizer (seed " << seed << ")\n"
                  << "1. Dijkstra's Algorithm\n"
                  << "2. A* Search\n"
                  << "3. Breadth-First Search\n"
                  << "4. Depth-First Search\n"
                  << "5. Generate Random Walls\n"
                  << "6. Clear Grid\n"
                  << "7. Quit\n"
                  << "8. Adjust Animation Speed\n"
                  << "9. Generate Maze\n"
                  << "10. Generate Random Terrain (cost: ':' sand 3, '~' water 5, '^' mountain 10)\n"
                  << "Enter your choice: " << std::flush;

        int choice = 0;
        while (!(std::cin >> choice) || choice < 1 || choice > 10) {
            if (std::cin.eof()) return 0;
            std::cin.clear();
            std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
            std::cout << "Invalid input. Enter a number between 1-10: " << std::flush;
        }

        switch (choice) {
            case 1:
            case 2:
            case 3:
            case 4: {
                static const Algorithm kByChoice[] = {Algorithm::Dijkstra, Algorithm::AStar, Algorithm::BFS,
                                                      Algorithm::DFS};
                const Algorithm a = kByChoice[choice - 1];
                std::cout << "Running " << algorithmDisplayName(a) << "..." << std::endl;
                switch (a) {
                    case Algorithm::Dijkstra: dijkstra(grid, animationSpeed); break;
                    case Algorithm::AStar: astar(grid, animationSpeed); break;
                    case Algorithm::BFS: bfs(grid, animationSpeed); break;
                    case Algorithm::DFS: dfs(grid, animationSpeed); break;
                }
                if (grid.hasWeightedTerrain() && (a == Algorithm::BFS || a == Algorithm::DFS)) {
                    std::cout << "Note: " << algorithmName(a) << " ignores terrain costs.\n";
                }
                waitForEnter();
                break;
            }
            case 5:
                grid.clear();
                std::cout << "Generating random walls..." << std::endl;
                grid.generateRandomWalls(0.3, rng);
                renderGrid(std::cout, grid);
                break;
            case 6:
                grid.clear();
                renderGrid(std::cout, grid);
                break;
            case 7:
                running = false;
                break;
            case 8:
                std::cout << "Enter animation delay (ms) [1-200]: " << std::flush;
                if (!(std::cin >> animationSpeed)) {
                    if (std::cin.eof()) return 0;
                    std::cin.clear();
                    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
                    animationSpeed = 50;
                    std::cout << "Invalid input; using 50 ms.\n";
                }
                animationSpeed = std::max(1, std::min(200, animationSpeed));
                break;
            case 9: {
                std::cout << "Enter maze dimensions (width height), 0 0 for the full grid: " << std::flush;
                int width = 0;
                int height = 0;
                if (!(std::cin >> width >> height)) {
                    if (std::cin.eof()) return 0;
                    std::cin.clear();
                    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
                    width = height = 0;
                }
                grid.generateMaze(rng, height, width);
                renderGrid(std::cout, grid);
                break;
            }
            case 10:
                grid.generateRandomTerrain(0.3, rng);
                renderGrid(std::cout, grid);
                break;
            default:
                break;
        }
    }
    return 0;
}
