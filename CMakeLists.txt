cmake_minimum_required(VERSION 3.10)
project(PathfindingVisualizer VERSION 2.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Benchmarks are meaningless without optimization: default single-config
# generators to Release unless the user chose otherwise.
if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

option(PATHFINDING_SANITIZE "Build with AddressSanitizer and UndefinedBehaviorSanitizer (GCC/Clang)" OFF)
option(PATHFINDING_WERROR "Treat compiler warnings as errors" OFF)

function(pathfinding_warnings target)
  if(MSVC)
    target_compile_options(${target} PRIVATE /W4 /permissive-)
    if(PATHFINDING_WERROR)
      target_compile_options(${target} PRIVATE /WX)
    endif()
  else()
    target_compile_options(${target} PRIVATE -Wall -Wextra -Wpedantic -Wshadow -Wconversion -Wno-sign-conversion)
    if(PATHFINDING_WERROR)
      target_compile_options(${target} PRIVATE -Werror)
    endif()
  endif()
  if(PATHFINDING_SANITIZE AND NOT MSVC)
    target_compile_options(${target} PRIVATE -fsanitize=address,undefined -fno-omit-frame-pointer -fno-sanitize-recover=all)
    target_link_libraries(${target} PRIVATE -fsanitize=address,undefined)
  endif()
endfunction()

# Core library: grid model, search algorithms, benchmark engine, reporting
# and CLI parsing. No console rendering.
add_library(pathfinding_core STATIC
  src/grid.cpp
  src/search.cpp
  src/validation.cpp
  src/benchmark.cpp
  src/report.cpp
  src/metadata.cpp
  src/output.cpp
  src/cli.cpp
)
target_include_directories(pathfinding_core PUBLIC src PRIVATE ${CMAKE_CURRENT_BINARY_DIR}/generated)
target_compile_definitions(pathfinding_core PRIVATE
  PATHFINDING_BUILD_TYPE="$<CONFIG>"
  PATHFINDING_VERSION="${PROJECT_VERSION}")
pathfinding_warnings(pathfinding_core)

# Git commit / dirty state, refreshed on every build (see cmake/GitInfo.cmake).
set(PATHFINDING_GIT_INFO ${CMAKE_CURRENT_BINARY_DIR}/generated/git_info.h)
set(PATHFINDING_GIT_INFO_CMD ${CMAKE_COMMAND} -DSRC_DIR=${CMAKE_CURRENT_SOURCE_DIR}
    -DOUT_FILE=${PATHFINDING_GIT_INFO} -P ${CMAKE_CURRENT_SOURCE_DIR}/cmake/GitInfo.cmake)
execute_process(COMMAND ${PATHFINDING_GIT_INFO_CMD})
add_custom_target(pathfinding_git_info COMMAND ${PATHFINDING_GIT_INFO_CMD} BYPRODUCTS ${PATHFINDING_GIT_INFO}
                  COMMENT "Recording git revision" VERBATIM)
add_dependencies(pathfinding_core pathfinding_git_info)

add_executable(pathfinding src/main.cpp src/visualizer.cpp)
target_link_libraries(pathfinding PRIVATE pathfinding_core)
pathfinding_warnings(pathfinding)

install(TARGETS pathfinding DESTINATION bin)

# ---------------------------------------------------------------- tests
include(CTest)
if(BUILD_TESTING)
  add_executable(pathfinding_tests tests/test_main.cpp)
  target_link_libraries(pathfinding_tests PRIVATE pathfinding_core)
  pathfinding_warnings(pathfinding_tests)
  add_test(NAME unit_tests COMMAND pathfinding_tests)

  # End-to-end checks of the real executable.
  add_test(NAME cli_help COMMAND pathfinding --help)
  add_test(NAME cli_rejects_bad_density COMMAND pathfinding --benchmark --density 150)
  set_tests_properties(cli_rejects_bad_density PROPERTIES WILL_FAIL TRUE)
  add_test(NAME cli_rejects_unknown_algorithm COMMAND pathfinding --benchmark --algorithm foo)
  set_tests_properties(cli_rejects_unknown_algorithm PROPERTIES WILL_FAIL TRUE)
  add_test(NAME benchmark_smoke
    COMMAND pathfinding --benchmark --suite all --grids 2 --runs 4 --warmup 0 --quiet --force
            --csv ${CMAKE_CURRENT_BINARY_DIR}/smoke_runs.csv
            --grid-csv ${CMAKE_CURRENT_BINARY_DIR}/smoke_grid.csv
            --summary-csv ${CMAKE_CURRENT_BINARY_DIR}/smoke_summary.csv
            --paired-csv ${CMAKE_CURRENT_BINARY_DIR}/smoke_paired.csv)
  add_test(NAME cli_end_to_end
    COMMAND ${CMAKE_COMMAND} -DEXE=$<TARGET_FILE:pathfinding> -DWORK=${CMAKE_CURRENT_BINARY_DIR}/cli_checks
            -P ${CMAKE_CURRENT_SOURCE_DIR}/tests/cli_checks.cmake)
endif()

set(CPACK_PACKAGE_NAME "PathfindingVisualizer")
set(CPACK_PACKAGE_VERSION "${PROJECT_VERSION}")
set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "Pathfinding visualizer and benchmarking framework")
include(CPack)
