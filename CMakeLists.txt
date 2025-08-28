cmake_minimum_required(VERSION 3.20)
project(NBodySimulation)

set(CMAKE_CXX_STANDARD 17)

# Set the vcpkg toolchain
set(CMAKE_TOOLCHAIN_FILE "${CMAKE_CURRENT_SOURCE_DIR}/vcpkg/scripts/buildsystems/vcpkg.cmake"
    CACHE STRING "Vcpkg toolchain file")

# Find packages
find_package(glfw3 CONFIG REQUIRED)
find_package(OpenGL REQUIRED)

# Create executable for console version
add_executable(nbody_console nbody_simulator.cpp)

# Create executable for visualization version
add_executable(nbody_visual nbody_visual.cpp)
target_link_libraries(nbody_visual PRIVATE glfw OpenGL::GL)

# Set working directory for Visual Studio
set_property(TARGET nbody_visual PROPERTY VS_DEBUGGER_WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}")
