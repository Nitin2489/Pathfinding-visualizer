cmake_minimum_required(VERSION 3.10)
project(PathfindingVisualizer VERSION 1.0)

# Specify C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Add executable
add_executable(pathfinding pathfinding.cpp)

# Add compiler options
if(MSVC)
  target_compile_options(pathfinding PRIVATE /W4)
else()
  target_compile_options(pathfinding PRIVATE -Wall -Wextra)
endif()

# Install target
install(TARGETS pathfinding DESTINATION bin)

# Create package
set(CPACK_PACKAGE_NAME "PathfindingVisualizer")
set(CPACK_PACKAGE_VERSION "1.0.0")
set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "Visualizer for pathfinding algorithms")
include(CPack) 