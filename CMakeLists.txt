cmake_minimum_required(VERSION 3.16)
project(CPSChallenge CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

# ---------------------------------------------------------------------------
# raylib: use a system install if present, otherwise fetch and build it.
# ---------------------------------------------------------------------------
find_package(raylib 4.5 QUIET)
if(NOT raylib_FOUND)
    include(FetchContent)
    set(BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
    set(BUILD_GAMES    OFF CACHE BOOL "" FORCE)
    FetchContent_Declare(raylib
        GIT_REPOSITORY https://github.com/raysan5/raylib.git
        GIT_TAG        5.5)
    FetchContent_MakeAvailable(raylib)
endif()

# ---------------------------------------------------------------------------
# Locations baked in at compile time (overridable, but handy defaults).
# ---------------------------------------------------------------------------
set(FMU_DYLIB "${CMAKE_SOURCE_DIR}/LateralMotionControl/binaries/darwin64/LateralMotionControl.dylib")
set(EXAMPLES_PATH "${CMAKE_SOURCE_DIR}/examples")

# ---------------------------------------------------------------------------
# cps_core: headless simulation library (FMU + trajectory + scheduler + sim).
# No raylib dependency, so it can build and run without a GUI.
# New scheduling policies dropped into src/sched/policies/*.cpp are picked up
# automatically (CONFIGURE_DEPENDS re-globs on rebuild).
# ---------------------------------------------------------------------------
file(GLOB POLICY_SOURCES CONFIGURE_DEPENDS "${CMAKE_SOURCE_DIR}/src/sched/policies/*.cpp")

add_library(cps_core STATIC
    src/fmu/Fmu.cpp
    src/trace/Trajectory.cpp
    src/sched/TaskModel.cpp
    src/sched/PolicyScheduler.cpp
    ${POLICY_SOURCES}
    src/sim/Recording.cpp
    src/sim/Simulation.cpp)

target_include_directories(cps_core PUBLIC
    "${CMAKE_SOURCE_DIR}/src"
    "${CMAKE_SOURCE_DIR}/LateralMotionControl/sources")

target_compile_definitions(cps_core PUBLIC
    FMU_DYLIB_PATH="${FMU_DYLIB}"
    EXAMPLES_DIR="${EXAMPLES_PATH}")

target_link_libraries(cps_core PUBLIC ${CMAKE_DL_LIBS})

# ---------------------------------------------------------------------------
# cps_viz: the raylib visualizer (depends on the core).
# ---------------------------------------------------------------------------
add_library(cps_viz STATIC src/viz/Visualizer.cpp)
target_link_libraries(cps_viz PUBLIC cps_core raylib)

# ---------------------------------------------------------------------------
# cps: the application. Edit main.cpp to pick / add a scheduler.
# ---------------------------------------------------------------------------
add_executable(cps main.cpp)
target_link_libraries(cps PRIVATE cps_viz)  # cps_viz publicly pulls in cps_core + raylib
