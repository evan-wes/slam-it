cmake_minimum_required(VERSION 3.26)
project(slamlab LANGUAGES CXX)

# C++23 for std::expected, the cores' error contract (slamlab/foundations/error.hpp).
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# The C++ cores build standalone (no Python) for the edge budgets and native tests;
# the nanobind module is added only when built through scikit-build-core.
option(SLAMLAB_BUILD_TESTS "Build the native C++ tests" OFF)
option(SLAMLAB_BUILD_BENCHMARKS "Build the native C++ benchmarks (Google Benchmark)" OFF)

# Eigen is the cores' linear algebra: fixed-size types may appear in core headers, so it is
# a PUBLIC dependency of every rung library. Provided by the conda env (setup_slamlab.sh).
find_package(Eigen3 5.0 CONFIG REQUIRED)

# One static library per rung, mirroring the Python packages: slamlab_<rung> from
# cpp/src/<rung>/, linking only the rungs below it. DEPENDS must agree with the ladder's
# layers in pyproject.toml; scripts/check_layers.py enforces that, and that every
# #include "slamlab/<rung>/..." stays within a rung's allowed set.
function(slamlab_add_rung rung)
  cmake_parse_arguments(ARG "" "" "DEPENDS" ${ARGN})
  file(GLOB_RECURSE sources CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/cpp/src/${rung}/*.cpp)
  add_library(slamlab_${rung} STATIC ${sources})
  target_include_directories(slamlab_${rung} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/cpp/include)
  target_link_libraries(slamlab_${rung} PUBLIC Eigen3::Eigen ${ARG_DEPENDS})
  set_target_properties(slamlab_${rung} PROPERTIES POSITION_INDEPENDENT_CODE ON)
  target_compile_options(slamlab_${rung} PRIVATE -Wall -Wextra -Wpedantic)
  if(SLAMLAB_BUILD_TESTS)
    # One executable per test file: cpp/tests/<rung>/test_<name>.cpp -> <rung>_test_<name>.
    file(GLOB_RECURSE tests CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/cpp/tests/${rung}/*.cpp)
    foreach(test_source IN LISTS tests)
      get_filename_component(test_stem ${test_source} NAME_WE)
      set(test_name ${rung}_${test_stem})
      add_executable(${test_name} ${test_source})
      target_link_libraries(${test_name} PRIVATE slamlab_${rung} nlohmann_json::nlohmann_json)
      target_compile_definitions(${test_name}
        PRIVATE SLAMLAB_FIXTURES_DIR="${CMAKE_CURRENT_SOURCE_DIR}/tests/fixtures")
      # Tests may include a rung's internal headers (cpp/src/<rung>/...) to unit-test them.
      target_include_directories(${test_name} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/cpp/src)
      target_compile_options(${test_name} PRIVATE -Wall -Wextra -Wpedantic)
      add_test(NAME ${test_name} COMMAND ${test_name})
    endforeach()
  endif()
  if(SLAMLAB_BUILD_BENCHMARKS)
    # One executable per benchmark file: cpp/bench/<rung>/bench_<name>.cpp ->
    # <rung>_bench_<name>, in build/<dir>/bench/.
    file(GLOB benches CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/cpp/bench/${rung}/*.cpp)
    foreach(bench_source IN LISTS benches)
      get_filename_component(bench_stem ${bench_source} NAME_WE)
      set(bench_name ${rung}_${bench_stem})
      add_executable(${bench_name} ${bench_source})
      target_link_libraries(${bench_name} PRIVATE slamlab_${rung} benchmark::benchmark)
      target_compile_options(${bench_name} PRIVATE -Wall -Wextra -Wpedantic)
      set_target_properties(${bench_name} PROPERTIES RUNTIME_OUTPUT_DIRECTORY bench)
    endforeach()
  endif()
endfunction()

if(SLAMLAB_BUILD_TESTS)
  enable_testing()
  # Native tests read the shared parity fixtures (tests/fixtures/) with nlohmann_json, a
  # test-only dependency from the conda env.
  find_package(nlohmann_json 3 CONFIG REQUIRED)
endif()
if(SLAMLAB_BUILD_BENCHMARKS)
  # Google Benchmark, a benchmark-only dependency from the conda env.
  find_package(benchmark CONFIG REQUIRED)
endif()

slamlab_add_rung(foundations)

if(SKBUILD)
  find_package(Python 3.11 REQUIRED COMPONENTS Interpreter Development.Module)
  find_package(nanobind CONFIG REQUIRED)
  # Bindings mirror the rungs: cpp/bindings/<rung>/<module>.cpp, wired by module.cpp.
  file(GLOB_RECURSE SLAMLAB_BINDING_SOURCES CONFIGURE_DEPENDS cpp/bindings/*.cpp)
  nanobind_add_module(_cpp NB_STATIC ${SLAMLAB_BINDING_SOURCES})
  target_include_directories(_cpp PRIVATE cpp/bindings)
  target_link_libraries(_cpp PRIVATE slamlab_foundations)
  install(TARGETS _cpp LIBRARY DESTINATION slamlab)
endif()
