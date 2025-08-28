# N-Body Simulation (C++/OpenGL)

This workspace contains two real-time N‑body simulators:

- A “normal” O(N²) solver for small N (good for learning/visual clarity)
- A Barnes–Hut O(N log N) solver with a Quadtree for ~1000 bodies

Both use GLFW + OpenGL for visualization.

## Files

- `nbody_simulator.cpp` — Minimal console demo (SI units example)
- `nbody_visual.cpp` — Visual O(N²) simulator (small N)
- `nbody_barneshut.cpp` — Visual Barnes–Hut simulator (~1000 bodies)

---

## Normal N‑Body (O(N²)) — approach

Core idea: compute gravitational forces by direct pairwise summation and step forward in time.

- State per body: position (x, y), velocity (vx, vy), mass
- Force model: Newtonian gravity with softening
  - Softening ε avoids singularities when particles get very close
  - Typical: add ε² inside distance squared, or `dist = sqrt(dx²+dy²+ε)`
- Integrator:
  - Visual version uses a simple, stable scheme (semi‑implicit/Leapfrog‑style)
  - Optional light damping to keep the scene tidy for long runs
- Units:
  - Console example (`nbody_simulator.cpp`) demonstrates SI units with real G
  - Visuals use normalized units (G = 1) for convenient scales and visible motion
- Initialization:
  - Small N arranged in rings or random disks with tangential velocities to suggest orbits
- Rendering:
  - GLFW window + OpenGL orthographic projection: `glOrtho(-1, 1, -1, 1, -1, 1)`
  - Bodies rendered as small filled disks (triangle fan)
  - Optional glow via blending and a second, larger, faint disk
  - MSAA enabled (when available)

Complexity: O(N²) per step — great for N ≤ ~1–2k on modern CPUs only if very optimized; here intended for dozens of bodies.

---

## Barnes–Hut (O(N log N)) with Quadtree — approach

Goal: approximate distant groups of bodies as a single mass at their center of mass to reduce complexity from O(N²) to ~O(N log N).

### Data structure

- Quadtree node (`QuadNode`): axis‑aligned region with up to 4 children (NW, NE, SW, SE)
- For each node we maintain:
  - Bounding box (x, y, width, height)
  - Total mass and center of mass (COM)
  - Leaf nodes contain a tiny list of body pointers; internal nodes aggregate children
- Insertion rules:
  - Reject bodies outside bounds
  - Subdivide once a leaf already has a body and positions are not identical
  - Guard against identical positions to avoid infinite subdivision

### Force calculation

- Barnes–Hut criterion: if `node_width / distance_to_node_COM < theta`, approximate subtree by its COM
  - Typical `theta ≈ 0.5` used here
- Otherwise recurse into children
- Softening is used in distance to avoid singularities

### Time stepping

- Build dynamic bounds around current bodies each frame to create the root node
- Insert all bodies into the tree
- Accumulate forces for each body using the BH traversal
- Integrate velocities and positions with a small `dt`
- Stability features used here:
  - Acceleration clamp (prevents blow‑ups)
  - Velocity clamp
  - Very light velocity damping
- Collision merging:
  - Implemented with conservation of momentum and mass
  - Disabled in the current visual build to keep ~1000 stars visible (otherwise clusters quickly collapse into a few massive bodies)

### Initialization (galaxy‑like)

- Central cluster sampled from a narrow Gaussian
- Multiple spiral arms (parametric radius and angle with noise)
- Tangential (approximate orbital) initial velocities
- Masses sampled from a small range for slight brightness variation

### Rendering & GL state

- Explicit black clear each frame; viewport synced to framebuffer size (HiDPI aware)
- Depth/scissor tests disabled; MODELVIEW reset per frame
- Bodies: small filled disks; colors vary with galactic region (bulge/disk/halo palette)
- MSAA enabled when supported

### Parameters (defaults in code)

- Gravitational constant G: ~0.1 (normalized units for visuals)
- Time step dt: ~0.01
- Theta (BH accuracy): ~0.5
- Softening ε: ~1e‑6 (inside sqrt)

Complexity: O(N log N) per step (tree build + queries), suitable for ~1000+ bodies interactively.

---

## Build & Run (Windows, MinGW g++, GLFW via vcpkg)

Assumptions:
- Using MSYS2 MinGW g++ or similar
- GLFW installed via vcpkg and available at `./vcpkg/packages/glfw3_x64-windows/`

Example build commands:

```sh
# Normal visual (O(N²))
g++ nbody_visual.cpp -std=c++17 \
  -I./vcpkg/packages/glfw3_x64-windows/include \
  -L./vcpkg/packages/glfw3_x64-windows/lib \
  -lglfw3dll -lopengl32 -lgdi32 -luser32 -lkernel32 \
  -o nbody_visual.exe

# Barnes–Hut (Quadtree)
g++ nbody_barneshut.cpp -std=c++17 \
  -I./vcpkg/packages/glfw3_x64-windows/include \
  -L./vcpkg/packages/glfw3_x64-windows/lib \
  -lglfw3dll -lopengl32 -lgdi32 -luser32 -lkernel32 \
  -o nbody_barneshut.exe
```

Runtime notes:
- Ensure `glfw3.dll` is alongside the `.exe` or on your PATH
- If you get a white window, verify the app sets the viewport to the actual framebuffer size; this repo’s code calls `glfwGetFramebufferSize` and `glViewport` each frame
- On HiDPI displays the framebuffer can be larger than the window size

---

## Tips & tweaks

- Re‑enable collision merging if you want more realistic clustering (expect visible body count to drop over time)
- Adjust `theta` for accuracy/speed trade‑off (smaller = more accurate, slower)
- Tune `G`, `dt`, and damping for stability vs. dynamism
- Colors and disk sizes are cosmetic; try blending with a subtle outer glow for a nebula‑like look

## References
- [Wikipedia: N-body problem](https://en.wikipedia.org/wiki/N-body_problem)
- [Numerical Recipes](https://numerical.recipes/)

---
