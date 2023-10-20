cmake_minimum_required(VERSION 3.0)


# Set the C++ standard and version
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(AStarBinaryGrid)


# Find required packages
find_package(OpenCV REQUIRED)


# Include directories for header files
include_directories(
   ${CMAKE_SOURCE_DIR}/include
   ${SDL2_INCLUDE_DIRS}
   ${OpenCV_INCLUDE_DIRS}
)


# Add your source files
add_executable(${PROJECT_NAME}
   src/main.cpp
   src/a_star.cpp
)


# Link your project with external libraries
target_link_libraries(${PROJECT_NAME}
    ${OpenCV_LIBS}
)
