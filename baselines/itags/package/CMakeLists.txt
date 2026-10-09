cmake_minimum_required(VERSION 3.20)
project(itags_cpsat VERSION 0.1 LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_BUILD_TYPE Release)
find_package(ortools CONFIG REQUIRED)
find_package(fmt REQUIRED)
find_package(spdlog REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(ompl 1.5.2 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Threads REQUIRED)
configure_file(upstream/include/grstapse/config.hpp.in include/grstapse/config.hpp @ONLY)
file(GLOB_RECURSE CORE_SOURCES CONFIGURE_DEPENDS upstream/src/*.cpp)
# This target links no Gurobi headers or libraries. DeterministicSchedule is a
# solver-independent result container despite its location in the milp folder.
list(FILTER CORE_SOURCES EXCLUDE REGEX "/scheduling/milp/")
list(APPEND CORE_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/upstream/src/scheduling/milp/deterministic/deterministic_schedule.cpp)
add_library(itags_core STATIC ${CORE_SOURCES} cpsat_scheduler.cpp cpsat_model.cpp)
target_include_directories(itags_core PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/upstream/include
  ${CMAKE_CURRENT_BINARY_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/upstream/extern/robin_hood/include
  ${CMAKE_CURRENT_SOURCE_DIR}/upstream/extern/magic_enum/include
  ${OMPL_INCLUDE_DIRS})
target_link_libraries(itags_core PUBLIC ortools::ortools fmt::fmt spdlog::spdlog
  Eigen3::Eigen ompl yaml-cpp geos geos_c Threads::Threads)
add_executable(itags_cpsat itags_cli.cpp)
target_link_libraries(itags_cpsat PRIVATE itags_core)
