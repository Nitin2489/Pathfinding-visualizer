#include <iostream>
#include <vector>
#include <queue>
#include <stack>
#include <cmath>
#include <algorithm>
#include <limits>
#include <chrono>
#include <thread>
#include <cstdlib> // For rand(), srand()
#include <ctime>   // For time()
#include <windows.h> // For Windows compatibility
using namespace std;

// Constants
const int ROWS = 20;
const int COLS = 50;
const char EMPTY = ' ';
const char WALL = '#';
const char START = 'S';
const char END = 'E';
const char VISITED = '.';
const char PATH = '*';
const char WATER = 'W';
const char SAND = 'S';
const char MOUNTAIN = 'M';

// Node structure for pathfinding
struct Node {
    int row, col;
    double f, g, h; // For A*
    int distance;   // For Dijkstra
    pair<int, int> parent;
    
    Node(int r, int c) : row(r), col(c), f(numeric_limits<double>::infinity()),
                         g(numeric_limits<double>::infinity()), 
                         h(numeric_limits<double>::infinity()),
                         distance(numeric_limits<int>::max()),
                         parent({-1, -1}) {}
};

// Grid class
class Grid {
private:
    vector<vector<char>> grid;
    pair<int, int> startPos;
    pair<int, int> endPos;
    
public:
    Grid() : startPos({1, 1}), endPos({ROWS-2, COLS-2}) {
        // Initialize grid with empty cells
        grid.resize(ROWS, vector<char>(COLS, EMPTY));
        
        // Create border walls
        for (int i = 0; i < ROWS; ++i) {
            grid[i][0] = WALL;
            grid[i][COLS-1] = WALL;
        }
        for (int j = 0; j < COLS; ++j) {
            grid[0][j] = WALL;
            grid[ROWS-1][j] = WALL;
        }
        
        // Set start and end positions
        grid[startPos.first][startPos.second] = START;
        grid[endPos.first][endPos.second] = END;
    }
    
    // Print the grid
    void print() {
        cout << "\033[H\033[J"; // Clear screen (ANSI escape code)
        for (int i = 0; i < ROWS; ++i) {
            for (int j = 0; j < COLS; ++j) {
                // Color codes for different cell types
                if (grid[i][j] == START) {
                    cout << "\033[42m" << 'S' << "\033[0m"; // Green background
                } else if (grid[i][j] == END) {
                    cout << "\033[41m" << 'E' << "\033[0m"; // Red background
                } else if (grid[i][j] == WALL) {
                    cout << "\033[47m" << ' ' << "\033[0m"; // White background
                } else if (grid[i][j] == VISITED) {
                    cout << "\033[44m" << ' ' << "\033[0m"; // Blue background
                } else if (grid[i][j] == PATH) {
                    cout << "\033[43m" << ' ' << "\033[0m"; // Yellow background
                } else if (grid[i][j] == WATER) {
                    cout << "\033[46m" << ' ' << "\033[0m"; // Cyan background
                } else if (grid[i][j] == SAND) {
                    cout << "\033[43m" << ' ' << "\033[0m"; // Yellow background
                } else if (grid[i][j] == MOUNTAIN) {
                    cout << "\033[45m" << ' ' << "\033[0m"; // Magenta background
                } else {
                    cout << ' ';
                }
            }
            cout << endl;
        }
    }
    
    // Add a wall at given position
    void addWall(int row, int col) {
        if (isValidCell(row, col) && grid[row][col] != START && grid[row][col] != END) {
            grid[row][col] = WALL;
        }
    }
    
    // Reset the grid (keeping walls)
    void reset() {
        for (int i = 0; i < ROWS; ++i) {
            for (int j = 0; j < COLS; ++j) {
                if (grid[i][j] == VISITED || grid[i][j] == PATH) {
                    grid[i][j] = EMPTY;
                }
            }
        }
        grid[startPos.first][startPos.second] = START;
        grid[endPos.first][endPos.second] = END;
    }
    
    // Clear entire grid (including walls)
    void clear() {
        for (int i = 1; i < ROWS-1; ++i) {
            for (int j = 1; j < COLS-1; ++j) {
                grid[i][j] = EMPTY;
            }
        }
        grid[startPos.first][startPos.second] = START;
        grid[endPos.first][endPos.second] = END;
    }
    
    // Check if a cell is valid for movement
    bool isValidCell(int row, int col) {
        return row >= 0 && row < ROWS && col >= 0 && col < COLS;
    }
    
    // Check if a cell is a wall
    bool isWall(int row, int col) {
        return grid[row][col] == WALL;
    }
    
    // Mark a cell as visited
    void markVisited(int row, int col) {
        if (grid[row][col] != START && grid[row][col] != END) {
            grid[row][col] = VISITED;
        }
    }
    
    // Mark a cell as part of the path
    void markPath(int row, int col) {
        if (grid[row][col] != START && grid[row][col] != END) {
            grid[row][col] = PATH;
        }
    }
    
    // Getters for start and end positions
    pair<int, int> getStartPos() const { return startPos; }
    pair<int, int> getEndPos() const { return endPos; }
    
    // Generate random walls
    void generateRandomWalls(double probability = 0.3) {
        for (int i = 1; i < ROWS-1; ++i) {
            for (int j = 1; j < COLS-1; ++j) {
                if ((i != startPos.first || j != startPos.second) && 
                    (i != endPos.first || j != endPos.second) && 
                    ((double)rand() / RAND_MAX) < probability) {
                    grid[i][j] = WALL;
                }
            }
        }
    }
    
    // Add terrain type at given position
    void addTerrain(int row, int col, char terrainType) {
        if (isValidCell(row, col) && grid[row][col] != START && grid[row][col] != END) {
            grid[row][col] = terrainType;
        }
    }
};

// Function to get neighbors of a cell
vector<pair<int, int>> getNeighbors(int row, int col) {
    vector<pair<int, int>> neighbors;
    // 4-directional movement (up, right, down, left)
    const int dr[] = {-1, 0, 1, 0};
    const int dc[] = {0, 1, 0, -1};
    
    for (int i = 0; i < 4; ++i) {
        int newRow = row + dr[i];
        int newCol = col + dc[i];
        neighbors.push_back({newRow, newCol});
    }
    
    return neighbors;
}

// Manhattan distance heuristic for A*
double manhattanDistance(int r1, int c1, int r2, int c2) {
    return abs(r1 - r2) + abs(c1 - c2);
}

// Dijkstra's Algorithm
bool dijkstra(Grid& grid, int animationSpeed = 50) {
    pair<int, int> start = grid.getStartPos();
    pair<int, int> end = grid.getEndPos();
    
    // Initialize nodes
    vector<vector<Node>> nodes(ROWS, vector<Node>(COLS, Node(0, 0)));
    for (int i = 0; i < ROWS; ++i) {
        for (int j = 0; j < COLS; ++j) {
            nodes[i][j] = Node(i, j);
        }
    }
    
    // Set start node distance to 0
    nodes[start.first][start.second].distance = 0;
    
    // Priority queue for Dijkstra's algorithm
    auto compare = [&nodes](pair<int, int> a, pair<int, int> b) {
        return nodes[a.first][a.second].distance > nodes[b.first][b.second].distance;
    };
    priority_queue<pair<int, int>, vector<pair<int, int>>, decltype(compare)> pq(compare);
    
    pq.push(start);
    
    bool finalPath = false;
    int visitedCount = 0;
    
    // Track these in algorithm functions
    int nodesExplored = 0;
    auto startTime = chrono::high_resolution_clock::now();
    
    while (!pq.empty()) {
        auto current = pq.top();
        pq.pop();
        
        int row = current.first;
        int col = current.second;
        
        // Mark current cell as visited
        grid.markVisited(row, col);
        nodesExplored++;
        if (visitedCount % 5 == 0 || finalPath) { 
            grid.print();
            this_thread::sleep_for(chrono::milliseconds(animationSpeed));
        }
        visitedCount++;
        
        // If we've reached the end
        if (row == end.first && col == end.second) {
            // Reconstruct path
            pair<int, int> pathCell = end;
            int pathLength = 0;
            while (pathCell != start) {
                pathCell = nodes[pathCell.first][pathCell.second].parent;
                grid.markPath(pathCell.first, pathCell.second);
                pathLength++;
                if (pathCell != start) { // Don't mark start as path
                    grid.print();
                    this_thread::sleep_for(chrono::milliseconds(animationSpeed));
                }
            }
            finalPath = true;
            
            // After algorithm completes:
            auto endTime = chrono::high_resolution_clock::now();
            auto duration = chrono::duration_cast<chrono::milliseconds>(endTime - startTime).count();
            
            cout << "Path found!" << endl;
            cout << "Nodes explored: " << nodesExplored << endl;
            cout << "Path length: " << pathLength << endl;
            cout << "Time taken: " << duration << " ms" << endl;
            return true;
        }
        
        // Check neighbors
        for (auto& neighbor : getNeighbors(row, col)) {
            int r = neighbor.first;
            int c = neighbor.second;
            
            if (!grid.isValidCell(r, c) || grid.isWall(r, c)) {
                continue;
            }
            
            int newDist = nodes[row][col].distance + 1;
            if (newDist < nodes[r][c].distance) {
                nodes[r][c].distance = newDist;
                nodes[r][c].parent = {row, col};
                pq.push({r, c});
            }
        }
    }
    
    // No path found
    return false;
}

// A* Search Algorithm
bool astar(Grid& grid, int animationSpeed = 50) {
    pair<int, int> start = grid.getStartPos();
    pair<int, int> end = grid.getEndPos();
    
    // Initialize nodes
    vector<vector<Node>> nodes(ROWS, vector<Node>(COLS, Node(0, 0)));
    for (int i = 0; i < ROWS; ++i) {
        for (int j = 0; j < COLS; ++j) {
            nodes[i][j] = Node(i, j);
        }
    }
    
    // Set start node values
    nodes[start.first][start.second].g = 0;
    nodes[start.first][start.second].h = manhattanDistance(start.first, start.second, end.first, end.second);
    nodes[start.first][start.second].f = nodes[start.first][start.second].h;
    
    // Open set (nodes to be evaluated)
    auto compare = [&nodes](pair<int, int> a, pair<int, int> b) {
        return nodes[a.first][a.second].f > nodes[b.first][b.second].f;
    };
    priority_queue<pair<int, int>, vector<pair<int, int>>, decltype(compare)> openSet(compare);
    
    // Closed set (nodes already evaluated)
    vector<vector<bool>> closedSet(ROWS, vector<bool>(COLS, false));
    
    openSet.push(start);
    
    bool finalPath = false;
    int visitedCount = 0;
    
    // Track these in algorithm functions
    int nodesExplored = 0;
    auto startTime = chrono::high_resolution_clock::now();
    
    while (!openSet.empty()) {
        auto current = openSet.top();
        openSet.pop();
        
        int row = current.first;
        int col = current.second;
        
        // Mark current cell as visited
        grid.markVisited(row, col);
        nodesExplored++;
        if (visitedCount % 5 == 0 || finalPath) { 
            grid.print();
            this_thread::sleep_for(chrono::milliseconds(animationSpeed));
        }
        visitedCount++;
        
        // If we've reached the end
        if (row == end.first && col == end.second) {
            // Reconstruct path
            pair<int, int> pathCell = end;
            int pathLength = 0;
            while (pathCell != start) {
                pathCell = nodes[pathCell.first][pathCell.second].parent;
                grid.markPath(pathCell.first, pathCell.second);
                pathLength++;
                if (pathCell != start) { // Don't mark start as path
                    grid.print();
                    this_thread::sleep_for(chrono::milliseconds(animationSpeed));
                }
            }
            finalPath = true;
            
            // After algorithm completes:
            auto endTime = chrono::high_resolution_clock::now();
            auto duration = chrono::duration_cast<chrono::milliseconds>(endTime - startTime).count();
            
            cout << "Path found!" << endl;
            cout << "Nodes explored: " << nodesExplored << endl;
            cout << "Path length: " << pathLength << endl;
            cout << "Time taken: " << duration << " ms" << endl;
            return true;
        }
        
        closedSet[row][col] = true;
        
        // Check neighbors
        for (auto& neighbor : getNeighbors(row, col)) {
            int r = neighbor.first;
            int c = neighbor.second;
            
            if (!grid.isValidCell(r, c) || grid.isWall(r, c) || closedSet[r][c]) {
                continue;
            }
            
            double tentativeG = nodes[row][col].g + 1;
            
            bool isNewNode = false;
            if (nodes[r][c].g == numeric_limits<double>::infinity()) {
                isNewNode = true;
            } else if (tentativeG < nodes[r][c].g) {
                isNewNode = true;
            } else {
                isNewNode = false;
            }
            
            if (isNewNode) {
                nodes[r][c].parent = {row, col};
                nodes[r][c].g = tentativeG;
                nodes[r][c].h = manhattanDistance(r, c, end.first, end.second);
                nodes[r][c].f = nodes[r][c].g + nodes[r][c].h;
                openSet.push({r, c});
            }
        }
    }
    
    // No path found
    return false;
}

// Breadth-First Search Algorithm
bool bfs(Grid& grid, int animationSpeed = 50) {
    pair<int, int> start = grid.getStartPos();
    pair<int, int> end = grid.getEndPos();
    
    // Initialize nodes
    vector<vector<Node>> nodes(ROWS, vector<Node>(COLS, Node(0, 0)));
    for (int i = 0; i < ROWS; ++i) {
        for (int j = 0; j < COLS; ++j) {
            nodes[i][j] = Node(i, j);
        }
    }
    
    // Queue for BFS
    queue<pair<int, int>> q;
    q.push(start);
    
    // Keep track of visited cells
    vector<vector<bool>> visited(ROWS, vector<bool>(COLS, false));
    visited[start.first][start.second] = true;
    
    bool finalPath = false;
    int visitedCount = 0;
    
    // Track these in algorithm functions
    int nodesExplored = 0;
    auto startTime = chrono::high_resolution_clock::now();
    
    while (!q.empty()) {
        auto current = q.front();
        q.pop();
        
        int row = current.first;
        int col = current.second;
        
        // Mark current cell as visited
        grid.markVisited(row, col);
        nodesExplored++;
        if (visitedCount % 5 == 0 || finalPath) { 
            grid.print();
            this_thread::sleep_for(chrono::milliseconds(animationSpeed));
        }
        visitedCount++;
        
        // If we've reached the end
        if (row == end.first && col == end.second) {
            // Reconstruct path
            pair<int, int> pathCell = end;
            int pathLength = 0;
            while (pathCell != start) {
                pathCell = nodes[pathCell.first][pathCell.second].parent;
                grid.markPath(pathCell.first, pathCell.second);
                pathLength++;
                if (pathCell != start) { // Don't mark start as path
                    grid.print();
                    this_thread::sleep_for(chrono::milliseconds(animationSpeed));
                }
            }
            finalPath = true;
            
            // After algorithm completes:
            auto endTime = chrono::high_resolution_clock::now();
            auto duration = chrono::duration_cast<chrono::milliseconds>(endTime - startTime).count();
            
            cout << "Path found!" << endl;
            cout << "Nodes explored: " << nodesExplored << endl;
            cout << "Path length: " << pathLength << endl;
            cout << "Time taken: " << duration << " ms" << endl;
            return true;
        }
        
        // Check neighbors
        for (auto& neighbor : getNeighbors(row, col)) {
            int r = neighbor.first;
            int c = neighbor.second;
            
            if (!grid.isValidCell(r, c) || grid.isWall(r, c) || visited[r][c]) {
                continue;
            }
            
            visited[r][c] = true;
            nodes[r][c].parent = {row, col};
            q.push({r, c});
        }
    }
    
    // No path found
    return false;
}

// Depth-First Search Algorithm
bool dfs(Grid& grid, int animationSpeed = 50) {
    pair<int, int> start = grid.getStartPos();
    pair<int, int> end = grid.getEndPos();
    
    // Initialize nodes
    vector<vector<Node>> nodes(ROWS, vector<Node>(COLS, Node(0, 0)));
    for (int i = 0; i < ROWS; ++i) {
        for (int j = 0; j < COLS; ++j) {
            nodes[i][j] = Node(i, j);
        }
    }
    
    // Stack for DFS
    stack<pair<int, int>> s;
    s.push(start);
    
    // Keep track of visited cells
    vector<vector<bool>> visited(ROWS, vector<bool>(COLS, false));
    visited[start.first][start.second] = true;
    
    bool finalPath = false;
    int visitedCount = 0;
    
    // Track these in algorithm functions
    int nodesExplored = 0;
    auto startTime = chrono::high_resolution_clock::now();
    
    while (!s.empty()) {
        auto current = s.top();
        s.pop();
        
        int row = current.first;
        int col = current.second;
        
        // Mark current cell as visited
        grid.markVisited(row, col);
        nodesExplored++;
        if (visitedCount % 5 == 0 || finalPath) { 
            grid.print();
            this_thread::sleep_for(chrono::milliseconds(animationSpeed));
        }
        visitedCount++;
        
        // If we've reached the end
        if (row == end.first && col == end.second) {
            // Reconstruct path
            pair<int, int> pathCell = end;
            int pathLength = 0;
            while (pathCell != start) {
                pathCell = nodes[pathCell.first][pathCell.second].parent;
                grid.markPath(pathCell.first, pathCell.second);
                pathLength++;
                if (pathCell != start) { // Don't mark start as path
                    grid.print();
                    this_thread::sleep_for(chrono::milliseconds(animationSpeed));
                }
            }
            finalPath = true;
            
            // After algorithm completes:
            auto endTime = chrono::high_resolution_clock::now();
            auto duration = chrono::duration_cast<chrono::milliseconds>(endTime - startTime).count();
            
            cout << "Path found!" << endl;
            cout << "Nodes explored: " << nodesExplored << endl;
            cout << "Path length: " << pathLength << endl;
            cout << "Time taken: " << duration << " ms" << endl;
            return true;
        }
        
        // Check neighbors
        for (auto& neighbor : getNeighbors(row, col)) {
            int r = neighbor.first;
            int c = neighbor.second;
            
            if (!grid.isValidCell(r, c) || grid.isWall(r, c) || visited[r][c]) {
                continue;
            }
            
            visited[r][c] = true;
            nodes[r][c].parent = {row, col};
            s.push({r, c});
        }
    }
    
    // No path found
    return false;
}

// Add weights for each terrain
int getTerrainCost(char terrain) {
    switch(terrain) {
        case EMPTY: return 1;
        case WATER: return 5;
        case SAND: return 3;
        case MOUNTAIN: return 10;
        default: return 1;
    }
}

// Function declarations
void generateMaze(Grid& grid, int startX, int startY, int width, int height, bool horizontal);
void smoothPath(vector<pair<int, int>>& path);

// Main function
int main() {
    // For smoother output
    cout.sync_with_stdio(false);
    setvbuf(stdout, nullptr, _IOFBF, 1024);
    
    // At the beginning of main, add this for Windows compatibility
    #ifdef _WIN32
        system("cls"); // Clear screen on Windows
        // Enable ANSI colors on Windows
        HANDLE hOut = GetStdHandle(STD_OUTPUT_HANDLE);
        DWORD dwMode = 0;
        GetConsoleMode(hOut, &dwMode);
        SetConsoleMode(hOut, dwMode | ENABLE_VIRTUAL_TERMINAL_PROCESSING);
    #endif
    
    srand(static_cast<unsigned int>(time(nullptr))); // Seed for random wall generation
    
    Grid grid;
    bool running = true;
    
    // Add this to your main variables
    int animationSpeed = 50; // milliseconds
    
    while (running) {
        cout << "Pathfinding Visualizer" << endl;
        cout << "1. Dijkstra's Algorithm" << endl;
        cout << "2. A* Search" << endl;
        cout << "3. Breadth-First Search" << endl;
        cout << "4. Depth-First Search" << endl;
        cout << "5. Generate Random Walls" << endl;
        cout << "6. Clear Grid" << endl;
        cout << "7. Quit" << endl;
        cout << "8. Adjust Animation Speed" << endl;
        cout << "9. Generate Maze" << endl;
        cout << "Enter your choice: ";
        
        int choice;
        while(!(cin >> choice) || choice < 1 || choice > 9) {
            cin.clear();
            cin.ignore(numeric_limits<streamsize>::max(), '\n');
            cout << "Invalid input. Enter a number between 1-9: ";
        }
        
        bool pathFound = false;
        
        switch (choice) {
            case 1:
                grid.reset();
                cout << "Running Dijkstra's Algorithm..." << endl;
                pathFound = dijkstra(grid, animationSpeed);
                break;
            case 2:
                grid.reset();
                cout << "Running A* Search..." << endl;
                pathFound = astar(grid, animationSpeed);
                break;
            case 3:
                grid.reset();
                cout << "Running Breadth-First Search..." << endl;
                pathFound = bfs(grid, animationSpeed);
                break;
            case 4:
                grid.reset();
                cout << "Running Depth-First Search..." << endl;
                pathFound = dfs(grid, animationSpeed);
                break;
            case 5:
                grid.clear();
                cout << "Generating random walls..." << endl;
                grid.generateRandomWalls();
                grid.print();
                break;
            case 6:
                grid.clear();
                grid.print();
                break;
            case 7:
                running = false;
                break;
            case 8:
                cout << "Enter animation delay (ms) [1-200]: ";
                cin >> animationSpeed;
                animationSpeed = max(1, min(200, animationSpeed)); // Ensure valid range
                break;
            case 9:
                cout << "Enter maze dimensions (width height): ";
                int width, height;
                cin >> width >> height;
                generateMaze(grid, 0, 0, width, height, true);
                grid.print();
                break;
            default:
                cout << "Invalid choice" << endl;
        }
        
        if (choice >= 1 && choice <= 4) {
            if (pathFound) {
                cout << "Path found!" << endl;
            } else {
                cout << "No path found!" << endl;
            }
            cout << "Press Enter to continue...";
            cin.ignore();
            cin.get();
        }
    }
    
    return 0;
}

// Create a recursive division maze generator
void generateMaze(Grid& grid, int startX, int startY, int width, int height, bool horizontal) {
    if (width <= 2 || height <= 2) return;
    
    // Generate a horizontal or vertical wall
    bool isHorizontal = width < height ? false : (height < width ? true : (rand() % 2 == 0));
    
    // Wall position and passage position
    int wallX = startX + (isHorizontal ? 0 : (rand() % (width - 2) + 1));
    int wallY = startY + (isHorizontal ? (rand() % (height - 2) + 1) : 0);
    
    // Passage position
    int passageX = wallX + (isHorizontal ? (rand() % width) : 0);
    int passageY = wallY + (isHorizontal ? 0 : (rand() % height));
    
    // Direction of wall
    int dx = isHorizontal ? 1 : 0;
    int dy = isHorizontal ? 0 : 1;
    
    // Wall length
    int length = isHorizontal ? width : height;
    
    // Create the wall with a passage
    for (int i = 0; i < length; i++) {
        int x = wallX + i * dx;
        int y = wallY + i * dy;
        
        if (x != passageX || y != passageY) {
            grid.addWall(y, x); // Note: grid uses row, col which is y, x
        }
    }
    
    // Calculate new dimensions
    int newX1 = startX;
    int newY1 = startY;
    int newW1 = isHorizontal ? width : wallX - startX;
    int newH1 = isHorizontal ? wallY - startY : height;
    
    int newX2 = isHorizontal ? startX : wallX + 1;
    int newY2 = isHorizontal ? wallY + 1 : startY;
    int newW2 = isHorizontal ? width : startX + width - wallX - 1;
    int newH2 = isHorizontal ? startY + height - wallY - 1 : height;
    
    // Recursively divide the regions
    if (newW1 >= 2 && newH1 >= 2)
        generateMaze(grid, newX1, newY1, newW1, newH1, !isHorizontal);
    if (newW2 >= 2 && newH2 >= 2)
        generateMaze(grid, newX2, newY2, newW2, newH2, !isHorizontal);
}

void smoothPath(vector<pair<int, int>>& path) {
    if (path.size() < 3) return;
    
    vector<pair<int, int>> smoothedPath;
    smoothedPath.push_back(path.front());
    
    // Add remaining points that couldn't be smoothed
    smoothedPath.push_back(path.back());
    
    path = smoothedPath;
} 