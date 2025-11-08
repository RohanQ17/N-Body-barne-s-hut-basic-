# Development Log

## November 8, 2025

### Project Setup & Configuration

**Environment Setup:**
- Installed vcpkg package manager at `C:\Users\ROHAN\vcpkg`
- Installed dependencies via vcpkg:
  - `glfw3:x64-windows` - Window management
  - `glew:x64-windows` - OpenGL extension loader
  - `glm:x64-windows` - Math library for graphics
- Ran `vcpkg integrate install` for system-wide integration

**Compiler Configuration:**
- Identified existing compilers: MSYS2 (ucrt64) and old MinGW
- Decided to use MSVC (Microsoft Visual C++) for better Windows/vcpkg compatibility
- Installed Visual Studio Build Tools 2022 with "Desktop development with C++"
- Configured VS Code to use MSVC compiler via CMake Tools extension

**VS Code Setup:**
- Installed required extensions:
  - CMake Tools (ms-vscode.cmake-tools)
  - CMake (twxs.cmake)
  - C/C++ (ms-vscode.cpptools)
- Created `.vscode/settings.json` with vcpkg toolchain configuration

**Project Structure Created:**
```
N body/
├── src/
│   └── main.cpp
├── shaders/
│   ├── particle.vert
│   └── particle.frag
├── Docs/
│   └── logs.md
├── .vscode/
│   └── settings.json
├── CMakeLists.txt
├── .gitignore
└── README.md
```

**Initial Code:**
- Created test program to verify OpenGL, GLFW, GLEW, and GLM integration
- Successfully compiled and ran test (OpenGL 3.3.0, NVIDIA 581.29 driver)
- Confirmed all dependencies working correctly

**Git Setup:**
- Initialized Git repository
- Created comprehensive `.gitignore` for C++/CMake projects
- Created `README.md` with project description
- Removed old remote and prepared for GitHub push

### Phase 1 Progress: Foundation ✅

**Completed:**
- ✅ Project structure setup
- ✅ CMake configuration
- ✅ Dependency installation (OpenGL, GLFW, GLEW, GLM)
- ✅ Window creation with OpenGL context
- ✅ Basic shader file structure
- ✅ Camera movement system (mouse look + WASD)
- ✅ Particle data structure
- ✅ Random particle generation (100 particles)
- ✅ VAO/VBO setup for rendering
- ✅ Basic vertex and fragment shaders

**Next Steps:**
- Build and test particle rendering system
- Verify camera controls work properly
- Begin Phase 1: Foundation


