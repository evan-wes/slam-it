cmake_minimum_required(VERSION 3.26)
project(slam_it LANGUAGES CXX)

# C++23 for std::expected, the cores' error contract (slam_it/foundations/error.hpp).
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# The C++ cores build standalone (no Python) for the edge budgets and native tests;
# the nanobind module is added only when built through scikit-build-core.
option(SLAM_IT_BUILD_TESTS "Build the native C++ tests" OFF)
option(SLAM_IT_BUILD_BENCHMARKS "Build the native C++ benchmarks (Google Benchmark)" OFF)

# Eigen is the cores' linear algebra: fixed-size types may appear in core headers, so it is
# a PUBLIC dependency of every rung library. Provided by the conda env (setup_slam_it.sh).
find_package(Eigen3 5.0 CONFIG REQUIRED)

# One static library per rung, mirroring the Python packages: slam_it_<rung> from
# cpp/src/<rung>/, linking only the rungs below it. DEPENDS must agree with the ladder's
# layers in pyproject.toml; scripts/check_layers.py enforces that, and that every
# #include "slam_it/<rung>/..." stays within a rung's allowed set.
function(slam_it_add_rung rung)
  cmake_parse_arguments(ARG "" "" "DEPENDS" ${ARGN})
  file(GLOB_RECURSE sources CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/cpp/src/${rung}/*.cpp)
  add_library(slam_it_${rung} STATIC ${sources})
  target_include_directories(slam_it_${rung} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/cpp/include)
  target_link_libraries(slam_it_${rung} PUBLIC Eigen3::Eigen ${ARG_DEPENDS})
  set_target_properties(slam_it_${rung} PROPERTIES POSITION_INDEPENDENT_CODE ON)
  target_compile_options(slam_it_${rung} PRIVATE -Wall -Wextra -Wpedantic)
  # Exact parity needs every float operation rounded on its own: no fused multiply-add,
  # which GCC and Clang would otherwise form on ARM. PUBLIC, so code inlining a core
  # header's arithmetic (tests, bindings) is compiled the same way.
  target_compile_options(slam_it_${rung} PUBLIC -ffp-contract=off)
  # A repeatable run needs every product summed in one order. Eigen blocks a large product by
  # the cache sizes of the core that runs its first one, and a hybrid CPU's core types report
  # different caches; without CPUID it uses its own defaults, the same on every core. PUBLIC,
  # so every program, test, benchmark and the extension is compiled the same way.
  target_compile_definitions(slam_it_${rung} PUBLIC EIGEN_NO_CPUID)
  if(SLAM_IT_BUILD_TESTS)
    # One executable per test file: cpp/tests/<rung>/test_<name>.cpp -> <rung>_test_<name>.
    file(GLOB_RECURSE tests CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/cpp/tests/${rung}/*.cpp)
    foreach(test_source IN LISTS tests)
      get_filename_component(test_stem ${test_source} NAME_WE)
      set(test_name ${rung}_${test_stem})
      add_executable(${test_name} ${test_source})
      target_link_libraries(${test_name} PRIVATE slam_it_${rung} nlohmann_json::nlohmann_json)
      target_compile_definitions(${test_name}
        PRIVATE SLAM_IT_FIXTURES_DIR="${CMAKE_CURRENT_SOURCE_DIR}/tests/fixtures")
      # Tests may include a rung's internal headers (cpp/src/<rung>/...) to unit-test them.
      target_include_directories(${test_name} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/cpp/src)
      target_compile_options(${test_name} PRIVATE -Wall -Wextra -Wpedantic)
      add_test(NAME ${test_name} COMMAND ${test_name})
    endforeach()
  endif()
  # One program per app file: cpp/apps/<rung>/<name>.cpp -> <name>, installed with the
  # package (slam_it/bin/) when built through scikit-build, where slam_it.backends finds it.
  file(GLOB apps CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/cpp/apps/${rung}/*.cpp)
  foreach(app_source IN LISTS apps)
    get_filename_component(app_name ${app_source} NAME_WE)
    add_executable(${app_name} ${app_source})
    target_link_libraries(${app_name} PRIVATE slam_it_${rung})
    target_compile_options(${app_name} PRIVATE -Wall -Wextra -Wpedantic)
    if(SKBUILD)
      install(TARGETS ${app_name} RUNTIME DESTINATION slam_it/bin)
    endif()
  endforeach()
  if(SLAM_IT_BUILD_BENCHMARKS)
    # One executable per benchmark file: cpp/bench/<rung>/bench_<name>.cpp ->
    # <rung>_bench_<name>, in build/<dir>/bench/.
    file(GLOB benches CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/cpp/bench/${rung}/*.cpp)
    foreach(bench_source IN LISTS benches)
      get_filename_component(bench_stem ${bench_source} NAME_WE)
      set(bench_name ${rung}_${bench_stem})
      add_executable(${bench_name} ${bench_source})
      target_link_libraries(${bench_name} PRIVATE slam_it_${rung} benchmark::benchmark)
      target_compile_options(${bench_name} PRIVATE -Wall -Wextra -Wpedantic)
      set_target_properties(${bench_name} PROPERTIES RUNTIME_OUTPUT_DIRECTORY bench)
    endforeach()
  endif()
endfunction()

if(SLAM_IT_BUILD_TESTS)
  enable_testing()
  # Native tests read the shared parity fixtures (tests/fixtures/) with nlohmann_json, a
  # test-only dependency from the conda env.
  find_package(nlohmann_json 3 CONFIG REQUIRED)
endif()
if(SLAM_IT_BUILD_BENCHMARKS)
  # Google Benchmark, a benchmark-only dependency from the conda env.
  find_package(benchmark CONFIG REQUIRED)
endif()

slam_it_add_rung(foundations)
slam_it_add_rung(edge DEPENDS slam_it_foundations)
# Evaluation: dataset readers and the run writer, outside the algorithm cores. PNGs decode
# with the vendored stb_image (third_party/stb), included as a system header so its own
# warnings stay out. The Python extension never links it; the native programs do.
slam_it_add_rung(eval DEPENDS slam_it_foundations)
target_include_directories(slam_it_eval SYSTEM PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/third_party/stb)
# Orchestration: the native programs that run a whole sequence (cpp/apps/orchestration/).
slam_it_add_rung(orchestration DEPENDS slam_it_edge slam_it_eval)

if(SKBUILD)
  find_package(Python 3.11 REQUIRED COMPONENTS Interpreter Development.Module)
  find_package(nanobind CONFIG REQUIRED)
  # Bindings mirror the rungs: cpp/bindings/<rung>/<module>.cpp, wired by module.cpp.
  file(GLOB_RECURSE SLAM_IT_BINDING_SOURCES CONFIGURE_DEPENDS cpp/bindings/*.cpp)
  nanobind_add_module(_cpp NB_STATIC ${SLAM_IT_BINDING_SOURCES})
  target_include_directories(_cpp PRIVATE cpp/bindings)
  target_link_libraries(_cpp PRIVATE slam_it_foundations slam_it_edge)
  install(TARGETS _cpp LIBRARY DESTINATION slam_it)
endif()
