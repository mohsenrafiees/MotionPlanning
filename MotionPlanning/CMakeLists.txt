cmake_minimum_required(VERSION 3.10)

# Project name
project(PlannerTests)

# Set C++ standard
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Include directories
include_directories(include)

# Find OpenCV
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

find_package(Python3 COMPONENTS Interpreter Development NumPy REQUIRED)

target_link_libraries(your_target PRIVATE
    ${Python3_LIBRARIES}
)


# Find Eigen
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

# Find spdlog
find_package(spdlog REQUIRED)
target_link_libraries(your_executable PRIVATE spdlog::spdlog)

# Find Python libraries
find_package(PythonLibs 3 REQUIRED)
include_directories(${PYTHON_INCLUDE_DIRS})

# Add the executable
add_executable(planner_tests
    src/AStarPlanner.cpp
    src/ThetaStarPlanner.cpp
    src/SBPLPlanner.cpp
    src/RRTStarPlanner.cpp
    src/CHOMPPlanner.cpp
    src/STOMPPlanner.cpp
    src/OccupancyGrid.cpp
    src/PathProfiler.cpp
    src/TrajectoryPlanner.cpp
    src/plotting_utils.cpp
    src/Visualizer.cpp
    tests/PlannerTests.cpp
)

# Link libraries
target_link_libraries(planner_tests ${OpenCV_LIBS} Eigen3::Eigen ${PYTHON_LIBRARIES})


target_include_directories(your_target PRIVATE
    ${Python3_INCLUDE_DIRS}
    ${Python3_NumPy_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)