// N-body simulation visualization using GLFW and OpenGL
// Compile with: g++ nbody_visual.cpp -I./vcpkg/packages/glfw3_x64-windows/include -L./vcpkg/packages/glfw3_x64-windows/lib -lglfw3dll -lopengl32 -lgdi32 -luser32 -lkernel32 -o nbody_visual.exe
#include <GLFW/glfw3.h>
#include <vector>
#include <cmath>
#include <random>
#include <algorithm>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// Fallback define for MSAA token on some Windows headers
#ifndef GL_MULTISAMPLE
#define GL_MULTISAMPLE 0x809D
#endif

struct Body {
    double x, y;
    double vx, vy;
    double mass;
};

class NBodySimulator {
public:
    NBodySimulator(std::vector<Body>& bodies, double G = 0.5, double dt = 0.005)
        : bodies(bodies), G(G), dt(dt) {}
    
    void step() {
        // Use Leapfrog integration for better energy conservation
        std::vector<std::pair<double, double>> forces(bodies.size(), {0.0, 0.0});
        
        // Calculate forces
        for (size_t i = 0; i < bodies.size(); ++i) {
            double fx = 0.0, fy = 0.0;
            for (size_t j = 0; j < bodies.size(); ++j) {
                if (i == j) continue;
                double dx = bodies[j].x - bodies[i].x;
                double dy = bodies[j].y - bodies[i].y;
                double dist_sq = dx * dx + dy * dy + 1e-6; // softening parameter
                double dist = std::sqrt(dist_sq);
                double force = G * bodies[i].mass * bodies[j].mass / dist_sq;
                fx += force * dx / dist;
                fy += force * dy / dist;
            }
            forces[i] = {fx, fy};
        }
        
        // Update positions and velocities
        for (size_t i = 0; i < bodies.size(); ++i) {
            double ax = forces[i].first / bodies[i].mass;
            double ay = forces[i].second / bodies[i].mass;
            
            // Leapfrog integration
            bodies[i].vx += ax * dt;
            bodies[i].vy += ay * dt;
            bodies[i].x += bodies[i].vx * dt;
            bodies[i].y += bodies[i].vy * dt;
        }
    }
private:
    std::vector<Body>& bodies;
    double G;
    double dt;
};

// Draw a filled disk at (cx, cy) with radius r (all in NDC units [-1,1])
static void drawDisk(float cx, float cy, float r, int segments = 28) {
    glBegin(GL_TRIANGLE_FAN);
    glVertex2f(cx, cy); // center
    for (int i = 0; i <= segments; ++i) {
        float ang = (float)i * 2.0f * (float)M_PI / (float)segments;
        float x = cx + r * std::cos(ang);
        float y = cy + r * std::sin(ang);
        glVertex2f(x, y);
    }
    glEnd();
}

// Map simulation coordinates to screen coordinates
void drawBodies(const std::vector<Body>& bodies, double scale, int width, int height) {
    glClear(GL_COLOR_BUFFER_BIT);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    // Convert desired pixel radius to NDC (so sizes are true screen-pixel sizes)
    const float pxToNdc = 2.0f / (float)std::min(width, height);

    for (size_t i = 0; i < bodies.size(); ++i) {
        float px = (float)(bodies[i].x / scale);
        float py = (float)(bodies[i].y / scale);

        // Choose color by distance from center for a galaxy-like palette
        double dist = std::sqrt(bodies[i].x * bodies[i].x + bodies[i].y * bodies[i].y);
        float dist_ratio = (float)(dist / 0.8);

        float cr = 1.0f, cg = 1.0f, cb = 0.8f; // default central star
        if (i != 0) {
            if (dist_ratio < 0.3f) {
                cr = 0.7f + 0.3f * dist_ratio;
                cg = 0.8f + 0.2f * dist_ratio;
                cb = 1.0f;
            } else if (dist_ratio < 0.7f) {
                float mid_ratio = (dist_ratio - 0.3f) / 0.4f;
                cr = 1.0f;
                cg = 1.0f - 0.2f * mid_ratio;
                cb = 0.9f - 0.4f * mid_ratio;
            } else {
                float outer_ratio = (dist_ratio - 0.7f) / 0.3f;
                cr = 1.0f;
                cg = 0.6f - 0.3f * outer_ratio;
                cb = 0.3f - 0.3f * outer_ratio;
            }
        }

        // Big, screen-pixel-based radii
        float baseRadiusPx = (i == 0) ? 30.0f : 12.0f; // smaller sizes
        float r = baseRadiusPx * pxToNdc;

        // Soft glow: draw from large transparent to solid core
        glColor4f(cr, cg, cb, 0.20f); drawDisk(px, py, r * 2.2f, 36);
        glColor4f(cr, cg, cb, 0.35f); drawDisk(px, py, r * 1.6f, 36);
        glColor4f(cr, cg, cb, 0.70f); drawDisk(px, py, r * 1.2f, 36);
        glColor4f(cr, cg, cb, 1.00f); drawDisk(px, py, r,        36);
    }
}

int main() {
    if (!glfwInit()) return -1;
    int width = 800, height = 800;
    // Request MSAA for smoother visuals
    glfwWindowHint(GLFW_SAMPLES, 4);
    GLFWwindow* window = glfwCreateWindow(width, height, "N-Body Simulation", NULL, NULL);
    if (!window) {
        glfwTerminate();
        return -1;
    }
    glfwMakeContextCurrent(window);
    glEnable(GL_MULTISAMPLE);
    glClearColor(0, 0, 0, 1);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glOrtho(-1, 1, -1, 1, -1, 1);
    glMatrixMode(GL_MODELVIEW);

    // Initialize bodies in a more stable configuration
    std::vector<Body> bodies;
    std::random_device rd;
    std::mt19937 gen(rd());
    
    // Add a central massive body to provide stability
    bodies.push_back({0.0, 0.0, 0.0, 0.0, 10.0}); // Central massive body
    
    // Add orbiting bodies
    for (int i = 1; i < 20; ++i) {
        double angle = (2.0 * M_PI * i) / 19.0; // evenly spaced angles
        double radius = 0.2 + 0.4 * (i / 19.0); // increasing radius
        double x = radius * cos(angle);
        double y = radius * sin(angle);
        
        // Calculate circular orbital velocity for stability
        double central_mass = 10.0;
        double orbital_speed = sqrt(0.5 * central_mass / radius); // circular orbit velocity
        double vx = -orbital_speed * sin(angle);
        double vy = orbital_speed * cos(angle);
        
        double mass = 0.5 + 0.5 * ((double)rand() / RAND_MAX); // smaller masses
        bodies.push_back({x, y, vx, vy, mass});
    }
    NBodySimulator sim(bodies);
    double scale = 1.0; // for mapping positions to screen

    while (!glfwWindowShouldClose(window)) {
        sim.step();
        drawBodies(bodies, scale, width, height);
        glfwSwapBuffers(window);
        glfwPollEvents();
    }
    glfwDestroyWindow(window);
    glfwTerminate();
    return 0;
}
