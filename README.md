# Pathfinding Algorithm Visualizer

A C++ console-based application that visualizes common pathfinding algorithms on a 2D grid.

## Features

- Interactive console visualization with ANSI color codes
- Multiple pathfinding algorithms:
  - Dijkstra's Algorithm
  - A* Search Algorithm
  - Breadth-First Search (BFS)
  - Depth-First Search (DFS)
- Random maze generation
- Visual representation of:
  - Start and end points
  - Walls/obstacles
  - Visited nodes
  - Final path

## Preview

```
Pathfinding Visualizer
1. Dijkstra's Algorithm
2. A* Search
3. Breadth-First Search
4. Depth-First Search
5. Generate Random Walls
6. Clear Grid
7. Quit
```

The visualizer shows the progress of algorithms with colored cells:
- Green (S): Start point
- Red (E): End point
- White blocks: Walls/obstacles
- Blue blocks: Visited nodes
- Yellow blocks: Final path

Requirements

- C++11 or later
- Terminal that supports ANSI color codes

## Installation

1. Clone the repository:
```bash
git clone https://github.com/Nitin2489/pathfinding-visualizer.git
cd pathfinding-visualizer
```

2. Compile the code:
```bash
g++ -std=c++17 -o pathfinding pathfinding.cpp
```

3. Run the program:
```bash
./pathfinding
```

## How to Use

1. Start the program
2. Choose an algorithm by entering its corresponding number
3. Watch the algorithm explore the grid
4. The final path will be highlighted once found
5. Press Enter to return to the menu
6. Generate random walls or clear the grid as needed

## Algorithm Details

### Dijkstra's Algorithm
A weighted graph algorithm that guarantees the shortest path by exploring nodes in order of their distance from the start.

### A* Search
An informed search algorithm that uses a heuristic (Manhattan distance) to guide its search toward the goal, typically more efficient than Dijkstra's.

### Breadth-First Search
A simple unweighted algorithm that explores all neighbors at the current depth before moving to nodes at the next depth level.

### Depth-First Search
An algorithm that explores as far as possible along a branch before backtracking, which may not find the shortest path but uses less memory.

## License

[MIT License](LICENSE)

## Contributing

Contributions are welcome! Please feel free to submit a Pull Request. 
