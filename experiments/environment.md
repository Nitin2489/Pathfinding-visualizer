# Environment

Every value below was recorded during the study, either by the benchmark's `.meta.json` sidecars (identical in all 52 invocations except for the timer calibration and timestamps) or by the commands noted.

## Software under test

| Item | Value | Source |
|---|---|---|
| Commit | `3d42fc8175d67c8b0efb6ef3bf8d432580777eba` | `program.git_commit` |
| Working tree at build | clean (`git_dirty_at_build=false`). Built from a dedicated `git worktree` of `HEAD`, so untracked files in the main checkout did not count. | metadata |
| Program version | 2.1.0 | metadata |
| Compiler | GCC 15.2.0 | `build.compiler` |
| Standard | C++17 (`__cplusplus` = 201703) | `build.cxx_standard` |
| Build type | Release, `-O3 -DNDEBUG` (CMake default Release flags) | metadata; `CMAKE_CXX_FLAGS_RELEASE` |
| CMake | 4.2.3 | `cmake --version` |
| Pre-run check | `ctest`: 6/6 passed on this exact build | ctest output |

## Platform

| Item | Value | Source |
|---|---|---|
| CPU | 12th Gen Intel Core i5-1235U: hybrid, 2 performance cores with hyper-threading + 8 efficiency cores | `environment.cpu_model`; Intel specification |
| Logical CPUs visible | 12 | metadata, `nproc` |
| Caches visible to the guest | L2 7.5 MiB (6 instances), L3 12 MiB | `lscpu` |
| Memory | 7.6 GiB visible to the guest | `/proc/meminfo` |
| OS | Linux 6.6.87.2-microsoft-standard-WSL2 (WSL2 virtual machine on Windows, Hyper-V) | `environment.os_release` |
| CPU frequency governor | not exposed inside WSL2 (`/sys/.../cpufreq` absent), so frequency is uncontrolled | sysfs |
| Pinning | `taskset -c 2` (logical CPU 2 of the guest) for every invocation except `G/unpinned`. The Hyper-V host still decides which physical core, performance or efficiency, runs that virtual CPU. | `raw/commands.log` |
| Time of runs | 2026-11-20 21:33:36Z → 21:34:44Z (UTC) | `environment.timestamp_utc` |

## Timer (`std::chrono::steady_clock`)

| Measured at startup of each invocation | min | median | max |
|---|---|---|---|
| Back-to-back overhead (ns) | 27 | 29 | 38 |
| Smallest observable step (ns) | 26 | 26 | 33 |
| Near-floor threshold (µs) = 100 × max of the two | 2.7 | 2.9 | 3.8 |

The nominal tick is 1 ns, but the effective resolution is about 26–33 ns. Per-grid medians below the threshold are flagged `near_timer_floor` and are not used for timing conclusions.

## Analysis tools

| Tool | Version |
|---|---|
| Python | 3.14.4 |
| pandas | 3.1.1 |
| NumPy | 2.5.5 |
| SciPy | 1.18.1 (Wilcoxon signed-rank, linear regression) |
| Matplotlib | 3.11.2 |

These were installed in a throwaway virtual environment outside the repository. No package was installed system-wide.
