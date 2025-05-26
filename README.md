## Visualizations

Visualizations of the convex hull computation steps can be found in the `visualizations/hull_charts` directory. These images show the state of the hull at various steps of the algorithm.

# C++ Pennant Feature (Sliding-Window Convex Hull)

## Tools & Dependencies
- **C++17** (or later)
- **CMake** (version  3.14)
- **Google Test** (via `find_package(GTest)`)
- **clang-format** (for code formatting)
- Any C++-capable IDE/editor with CMake support

## Build & Run
```bash
# Configure and build
mkdir build && cd build
cmake ..
make

# Resulting binaries:
# - pennant_exe    CLI tool
# - pennant_tests  Test runner

### Run CLI Tool
```bash
./pennant_exe <input.csv> <window_size> <output.csv>

# Example:
./pennant_exe data/input.csv 10 data/output.csv

### Run Tests
```bash
# Using CTest:
ctest

# Or directly:  
./pennant_tests
```

### Code Formatting
```bash
clang-format -i include/*.hpp src/*.cpp tests/*.cpp
```

