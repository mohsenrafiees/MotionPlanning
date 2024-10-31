CXX = g++
CXXFLAGS = -std=c++17 -O2 -g 

# Manually set Python, NumPy, and OpenCV paths
PYTHON_INCLUDES = -I/usr/include/python3.12 -I/usr/lib/python3/dist-packages/numpy/core/include
PYTHON_LIBS = -lpython3.12  # Adjust based on your Python version
OPENCV_INCLUDES = -I/usr/include/opencv4
OPENCV_LIBS = -lopencv_core -lopencv_imgproc -lopencv_highgui -lopencv_imgcodecs  # Adjust libraries as necessary
ADDITIONAL_LIBS = -lstdc++fs

INCLUDES = -Iinclude -I/usr/include/eigen3 $(PYTHON_INCLUDES) $(OPENCV_INCLUDES)
LDFLAGS = $(PYTHON_LIBS) $(OPENCV_LIBS) $(ADDITIONAL_LIBS)

SOURCES = src/AStarPlanner.cpp src/ThetaStarPlanner.cpp src/PathProfiler.cpp src/STOMPPlanner.cpp src/plotting_utils.cpp src/SBPLPlanner.cpp src/CHOMPPlanner.cpp src/RRTStarPlanner.cpp src/TrajectoryPlanner.cpp src/PlannerUtils.cpp src/main.cpp src/OccupancyGrid.cpp
OBJECTS = $(SOURCES:.cpp=.o)
EXECUTABLE = planner_program

all: $(EXECUTABLE)

$(EXECUTABLE): $(OBJECTS)
	$(CXX) $(OBJECTS) -o $@ $(LDFLAGS)

.cpp.o:
	$(CXX) $(CXXFLAGS) $(INCLUDES) -c $< -o $@

clean:
	rm -f $(OBJECTS) $(EXECUTABLE)
