# End-to-end checks of the pathfinding executable: exit codes, staged
# outputs, metadata sidecars and overwrite protection.
# Usage: cmake -DEXE=<path> -DWORK=<dir> -P cli_checks.cmake
file(REMOVE_RECURSE "${WORK}")
file(MAKE_DIRECTORY "${WORK}")

function(run expected_code)
  execute_process(COMMAND "${EXE}" ${ARGN} WORKING_DIRECTORY "${WORK}"
                  RESULT_VARIABLE code OUTPUT_VARIABLE out ERROR_VARIABLE err)
  if(NOT code EQUAL expected_code)
    message(FATAL_ERROR "expected exit ${expected_code}, got ${code}: ${ARGN}\nstdout:\n${out}\nstderr:\n${err}")
  endif()
  set(LAST_STDOUT "${out}" PARENT_SCOPE)
  set(LAST_STDERR "${err}" PARENT_SCOPE)
endfunction()

function(require_contains file text)
  file(READ "${WORK}/${file}" content)
  string(FIND "${content}" "${text}" pos)
  if(pos EQUAL -1)
    message(FATAL_ERROR "${file} does not contain '${text}'")
  endif()
endfunction()

set(outputs runs.csv grid.csv summary.csv paired.csv)
set(common --benchmark --suite baseline --grids 2 --runs 4 --warmup 0 --quiet)

# 1. Successful run: four CSVs + four sidecars, no leftover .partial files,
#    metadata written although --quiet suppressed the terminal metadata.
run(0 ${common} --csv runs.csv --grid-csv grid.csv --summary-csv summary.csv --paired-csv paired.csv)
foreach(f ${outputs})
  if(NOT EXISTS "${WORK}/${f}" OR NOT EXISTS "${WORK}/${f}.meta.json")
    message(FATAL_ERROR "missing ${f} or its metadata")
  endif()
  if(EXISTS "${WORK}/${f}.partial")
    message(FATAL_ERROR "leftover ${f}.partial")
  endif()
  require_contains(${f}.meta.json "\"status\": \"complete\"")
endforeach()
require_contains(runs.csv "scenario_id,scenario_type,rows,columns,obstacle_density")
require_contains(runs.csv ",0x")

# 2. Existing outputs are refused without --force and left byte-identical.
file(SHA256 "${WORK}/runs.csv" before)
run(1 ${common} --seed 7 --csv runs.csv)
file(SHA256 "${WORK}/runs.csv" after)
if(NOT before STREQUAL after)
  message(FATAL_ERROR "runs.csv changed although the run was refused")
endif()
if(EXISTS "${WORK}/runs.csv.partial")
  message(FATAL_ERROR "refused run created a partial file")
endif()

# 3. --force replaces them.
run(0 ${common} --seed 7 --force --csv runs.csv)
file(SHA256 "${WORK}/runs.csv" forced)
if(forced STREQUAL before)
  message(FATAL_ERROR "--force did not replace runs.csv")
endif()

# 4. Reachable placement that cannot succeed: exit 3, failure recorded.
run(3 --benchmark --size 6x6 --density 100 --grids 2 --runs 1 --warmup 0 --quiet --csv gen.csv)
require_contains(gen.csv.meta.json "\"status\": \"generation_failures\"")
require_contains(gen.csv.meta.json "no connected start/end pair")

# 5. Unwritable output: exit 1 before any work.
run(1 ${common} --csv "${WORK}/no/such/dir/x.csv")

# 6. Methodology warnings go to stderr even with --quiet.
run(0 --benchmark --grids 1 --runs 3 --warmup 0 --quiet)
string(FIND "${LAST_STDERR}" "warning: only one grid" w1)
string(FIND "${LAST_STDERR}" "not a multiple of the number of algorithms" w2)
if(w1 EQUAL -1 OR w2 EQUAL -1)
  message(FATAL_ERROR "expected warnings missing:\n${LAST_STDERR}")
endif()

# 7. Path collisions are rejected before any file is opened; existing
#    final files stay byte-identical and no staging files appear.
file(WRITE "${WORK}/results.csv" "EXISTING CSV\n")
file(WRITE "${WORK}/results.csv.meta.json" "{\"existing\": true}\n")
file(SHA256 "${WORK}/results.csv" csv_before)
file(SHA256 "${WORK}/results.csv.meta.json" meta_before)
run(1 ${common} --force --csv results.csv --grid-csv results.csv.meta.json)
string(FIND "${LAST_STDERR}" "output path collision" c1)
string(FIND "${LAST_STDERR}" "results.csv.meta.json" c2)
if(c1 EQUAL -1 OR c2 EQUAL -1)
  message(FATAL_ERROR "collision message missing:\n${LAST_STDERR}")
endif()
run(1 ${common} --force --csv results.csv --summary-csv ./results.csv.partial)
run(1 ${common} --force --csv results.csv --paired-csv results.csv.meta.json.partial)
file(SHA256 "${WORK}/results.csv" csv_after)
file(SHA256 "${WORK}/results.csv.meta.json" meta_after)
if(NOT csv_before STREQUAL csv_after OR NOT meta_before STREQUAL meta_after)
  message(FATAL_ERROR "existing files changed although preflight failed")
endif()
file(GLOB leftovers "${WORK}/*.partial")
if(leftovers)
  message(FATAL_ERROR "staging files left after preflight failure: ${leftovers}")
endif()

# 8. A later output failing to open removes the staging files already
#    opened; the existing final file is untouched.
file(WRITE "${WORK}/keep.csv" "KEEP\n")
file(SHA256 "${WORK}/keep.csv" keep_before)
run(1 ${common} --force --csv keep.csv --grid-csv "${WORK}/missing/dir/grid.csv")
file(SHA256 "${WORK}/keep.csv" keep_after)
if(NOT keep_before STREQUAL keep_after)
  message(FATAL_ERROR "keep.csv changed after an open failure")
endif()
if(EXISTS "${WORK}/keep.csv.partial")
  message(FATAL_ERROR "keep.csv.partial was not cleaned up")
endif()

# 9. A leftover staging file (e.g. from an interrupted run) is not
#    truncated without --force.
file(WRITE "${WORK}/left.csv.partial" "RECOVERABLE ROWS\n")
run(1 ${common} --csv left.csv)
file(READ "${WORK}/left.csv.partial" left)
if(NOT left STREQUAL "RECOVERABLE ROWS\n")
  message(FATAL_ERROR "leftover staging file was modified")
endif()

message(STATUS "all CLI checks passed")
