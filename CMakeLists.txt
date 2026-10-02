cmake_minimum_required(VERSION 3.26)
project(slamlab LANGUAGES CXX)

# C++23 for std::expected, the cores' error contract (slamlab/foundations/error.hpp).
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# The C++ cores build standalone (no Python) for the edge budgets and native tests;
# the nanobind module is added only when built through scikit-build-core.
option(SLAMLAB_BUILD_TESTS "Build the native C++ tests" OFF)

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
      target_link_libraries(${test_name} PRIVATE slamlab_${rung})
      target_compile_options(${test_name} PRIVATE -Wall -Wextra -Wpedantic)
      add_test(NAME ${test_name} COMMAND ${test_name})
    endforeach()
  endif()
endfunction()

if(SLAMLAB_BUILD_TESTS)
  enable_testing()
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
