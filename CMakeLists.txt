cmake_minimum_required(VERSION 3.26)
project(slamlab LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# The C++ core builds standalone (no Python) for the edge budgets and native tests;
# the nanobind module is added only when built through scikit-build-core.
option(SLAMLAB_BUILD_TESTS "Build the native C++ tests" OFF)

# Sources are found recursively so new rung directories (cpp/src/<rung>/) need no edit here.
file(GLOB_RECURSE SLAMLAB_CORE_SOURCES CONFIGURE_DEPENDS cpp/src/*.cpp)
add_library(slamlab_core STATIC ${SLAMLAB_CORE_SOURCES})
target_include_directories(slamlab_core PUBLIC cpp/include)
set_target_properties(slamlab_core PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_compile_options(slamlab_core PRIVATE -Wall -Wextra -Wpedantic)

if(SKBUILD)
  find_package(Python 3.11 REQUIRED COMPONENTS Interpreter Development.Module)
  find_package(nanobind CONFIG REQUIRED)
  nanobind_add_module(_cpp NB_STATIC cpp/bindings/module.cpp)
  target_link_libraries(_cpp PRIVATE slamlab_core)
  install(TARGETS _cpp LIBRARY DESTINATION slamlab)
endif()

if(SLAMLAB_BUILD_TESTS)
  enable_testing()
  # One executable per test file: cpp/tests/<rung>/test_<name>.cpp -> <rung>_test_<name>.
  file(GLOB_RECURSE SLAMLAB_TEST_SOURCES CONFIGURE_DEPENDS cpp/tests/*.cpp)
  foreach(test_source IN LISTS SLAMLAB_TEST_SOURCES)
    file(RELATIVE_PATH test_rel ${CMAKE_CURRENT_SOURCE_DIR}/cpp/tests ${test_source})
    string(REGEX REPLACE "\\.cpp$" "" test_name ${test_rel})
    string(REPLACE "/" "_" test_name ${test_name})
    add_executable(${test_name} ${test_source})
    target_link_libraries(${test_name} PRIVATE slamlab_core)
    target_compile_options(${test_name} PRIVATE -Wall -Wextra -Wpedantic)
    add_test(NAME ${test_name} COMMAND ${test_name})
  endforeach()
endif()
