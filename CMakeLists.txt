cmake_minimum_required(VERSION 3.15)
project(GalaxySimulator)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find packages
find_package(OpenGL REQUIRED)
find_package(glfw3 CONFIG REQUIRED)
find_package(GLEW REQUIRED)
find_package(glm CONFIG REQUIRED)

# Add executable
add_executable(test_build
    src/main.cpp
)

# Link libraries
target_link_libraries(test_build PRIVATE
    OpenGL::GL
    glfw
    GLEW::GLEW
    glm::glm
)

# Copy shaders to build directory
file(COPY shaders DESTINATION ${CMAKE_BINARY_DIR})