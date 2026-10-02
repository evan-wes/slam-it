cmake_minimum_required(VERSION 3.26)
project(slamlab LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# The C++ core builds standalone (no Python) for the edge budgets and native tests;
# the nanobind module is added only when built through scikit-build-core.
option(SLAMLAB_BUILD_TESTS "Build the native C++ tests" OFF)

add_library(slamlab_core STATIC cpp/src/so3.cpp)
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
  add_executable(test_so3 cpp/tests/test_so3.cpp)
  target_link_libraries(test_so3 PRIVATE slamlab_core)
  add_test(NAME so3 COMMAND test_so3)
endif()
