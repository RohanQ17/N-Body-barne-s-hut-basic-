// N-body simulation using Barnes-Hut algorithm with spatial partitioning
// Compile with: g++ nbody_barneshut.cpp -I./vcpkg/packages/glfw3_x64-windows/include -L./vcpkg/packages/glfw3_x64-windows/lib -lglfw3dll -lopengl32 -lgdi32 -luser32 -lkernel32 -o nbody_barneshut.exe
#include <GLFW/glfw3.h>
#include <vector>
#include <cmath>
#include <random>
#include <algorithm>
#include <memory>

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
    
    Body(double x = 0, double y = 0, double vx = 0, double vy = 0, double mass = 1.0)
        : x(x), y(y), vx(vx), vy(vy), mass(mass) {}
};

// QuadTree node for Barnes-Hut algorithm
struct QuadNode {
    double x, y, width, height;     // Bounding box
    double centerX, centerY, totalMass; // Center of mass and total mass
    std::vector<Body*> bodies;      // Bodies in this node (for leaf nodes)
    std::unique_ptr<QuadNode> children[4]; // NW, NE, SW, SE
    bool isLeaf;
    
    QuadNode(double x, double y, double width, double height)
        : x(x), y(y), width(width), height(height), centerX(0), centerY(0), 
          totalMass(0), isLeaf(true) {}
    
    void insert(Body* body) {
        // Bounds check to prevent infinite recursion
        if (body->x < x || body->x >= x + width || body->y < y || body->y >= y + height) {
            return; // Body is outside this node's bounds
        }
        
        if (isLeaf && bodies.empty()) {
            // First body in empty leaf
            bodies.push_back(body);
            updateCenterOfMass();
        } else if (isLeaf && bodies.size() == 1) {
            // Prevent infinite subdivision for bodies at same position
            if (std::abs(bodies[0]->x - body->x) < 1e-10 && 
                std::abs(bodies[0]->y - body->y) < 1e-10) {
                bodies.push_back(body); // Just add to same leaf
                updateCenterOfMass();
                return;
            }
            
            // Split the node
            subdivide();
            // Re-insert existing body and new body
            insertIntoChild(bodies[0]);
            bodies.clear();
            insertIntoChild(body);
            isLeaf = false;
            updateCenterOfMass();
        } else if (!isLeaf) {
            // Internal node - insert into appropriate child
            insertIntoChild(body);
            updateCenterOfMass();
        }
    }
    
private:
    void subdivide() {
        double halfW = width / 2.0;
        double halfH = height / 2.0;
        children[0] = std::make_unique<QuadNode>(x, y, halfW, halfH); // NW
        children[1] = std::make_unique<QuadNode>(x + halfW, y, halfW, halfH); // NE
        children[2] = std::make_unique<QuadNode>(x, y + halfH, halfW, halfH); // SW
        children[3] = std::make_unique<QuadNode>(x + halfW, y + halfH, halfW, halfH); // SE
    }
    
    void insertIntoChild(Body* body) {
        // Ensure we don't go outside bounds
        if (body->x < x || body->x >= x + width || body->y < y || body->y >= y + height) {
            return;
        }
        
        int index = 0;
        if (body->x >= x + width / 2.0) index += 1;
        if (body->y >= y + height / 2.0) index += 2;
        
        if (children[index]) {
            children[index]->insert(body);
        }
    }
    
    void updateCenterOfMass() {
        totalMass = 0;
        centerX = 0;
        centerY = 0;
        
        if (isLeaf) {
            for (Body* body : bodies) {
                totalMass += body->mass;
                centerX += body->x * body->mass;
                centerY += body->y * body->mass;
            }
        } else {
            for (int i = 0; i < 4; i++) {
                if (children[i] && children[i]->totalMass > 0) {
                    totalMass += children[i]->totalMass;
                    centerX += children[i]->centerX * children[i]->totalMass;
                    centerY += children[i]->centerY * children[i]->totalMass;
                }
            }
        }
        
        if (totalMass > 0) {
            centerX /= totalMass;
            centerY /= totalMass;
        }
    }
    
public:
    void calculateForce(Body* body, double& fx, double& fy, double theta = 0.5) {
        if (totalMass == 0) return;
        
        double dx = centerX - body->x;
        double dy = centerY - body->y;
        double dist = std::sqrt(dx * dx + dy * dy + 1e-6);
        
        if (isLeaf) {
            // Leaf node - calculate direct forces
            for (Body* other : bodies) {
                if (other == body) continue;
                double odx = other->x - body->x;
                double ody = other->y - body->y;
                double odist = std::sqrt(odx * odx + ody * ody + 1e-6);
                double force = body->mass * other->mass / (odist * odist * odist);
                fx += force * odx;
                fy += force * ody;
            }
        } else {
            // Internal node - check if we can use approximation
            if (width / dist < theta) {
                // Use center of mass approximation
                double force = body->mass * totalMass / (dist * dist * dist);
                fx += force * dx;
                fy += force * dy;
            } else {
                // Recursively calculate forces from children
                for (int i = 0; i < 4; i++) {
                    if (children[i]) {
                        children[i]->calculateForce(body, fx, fy, theta);
                    }
                }
            }
        }
    }
};

class BarnesHutSimulator {
public:
    BarnesHutSimulator(std::vector<Body>& bodies, double G = 0.1, double dt = 0.01)
        : bodies(bodies), G(G), dt(dt) {}
    
    void step() {
        // Build QuadTree with expanded bounds to contain all bodies
        double minX = -3.0, maxX = 3.0, minY = -3.0, maxY = 3.0;
        
        // Find actual bounds of bodies
        for (const Body& body : bodies) {
            minX = std::min(minX, body.x - 0.1);
            maxX = std::max(maxX, body.x + 0.1);
            minY = std::min(minY, body.y - 0.1);
            maxY = std::max(maxY, body.y + 0.1);
        }
        
        QuadNode root(minX, minY, maxX - minX, maxY - minY);
        
        // Insert all bodies into QuadTree
        for (Body& body : bodies) {
            root.insert(&body);
        }
        
    // Calculate forces using Barnes-Hut
        std::vector<std::pair<double, double>> forces(bodies.size(), {0.0, 0.0});
        for (size_t i = 0; i < bodies.size(); ++i) {
            double fx = 0, fy = 0;
            root.calculateForce(&bodies[i], fx, fy);
            forces[i] = {G * fx, G * fy};
        }
    // NOTE: Collision merging disabled for visualization clarity.
    // If you want merging, perform it either before force computation or recompute forces after compaction.

        // Update velocities and positions
        for (size_t i = 0; i < bodies.size(); ++i) {
            double ax = forces[i].first / bodies[i].mass;
            double ay = forces[i].second / bodies[i].mass;
            
            // Limit acceleration to prevent instability
            double accel_mag = std::sqrt(ax*ax + ay*ay);
            if (accel_mag > 5.0) {
                ax = ax / accel_mag * 5.0;
                ay = ay / accel_mag * 5.0;
            }
            
            bodies[i].vx += ax * dt;
            bodies[i].vy += ay * dt;
            
            // Limit velocity to prevent runaway
            double vel_mag = std::sqrt(bodies[i].vx*bodies[i].vx + bodies[i].vy*bodies[i].vy);
            if (vel_mag > 2.0) {
                bodies[i].vx = bodies[i].vx / vel_mag * 2.0;
                bodies[i].vy = bodies[i].vy / vel_mag * 2.0;
            }
            
            // Very light damping for long-term stability
            bodies[i].vx *= 0.99995;
            bodies[i].vy *= 0.99995;
            
            bodies[i].x += bodies[i].vx * dt;
            bodies[i].y += bodies[i].vy * dt;
        }
    }
    
private:
    std::vector<Body>& bodies;
    double G;
    double dt;
};

// Draw a small filled disk
static void drawSmallDisk(float cx, float cy, float r, int segments = 12) {
    glBegin(GL_TRIANGLE_FAN);
    glVertex2f(cx, cy);
    for (int i = 0; i <= segments; ++i) {
        float ang = (float)i * 2.0f * (float)M_PI / (float)segments;
        float x = cx + r * std::cos(ang);
        float y = cy + r * std::sin(ang);
        glVertex2f(x, y);
    }
    glEnd();
}

void drawBodies(const std::vector<Body>& bodies, double scale, int width, int height) {
    // Ensure correct viewport and state
    glDisable(GL_SCISSOR_TEST);
    glDisable(GL_DEPTH_TEST);
    glViewport(0, 0, width, height);

    // Clear with explicit black background
    glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    
    // Disable blending initially to debug
    glDisable(GL_BLEND);
    
    const float pxToNdc = 2.0f / (float)std::min(width, height);
    
    for (size_t i = 0; i < bodies.size(); ++i) {
        float px = (float)(bodies[i].x / scale);
        float py = (float)(bodies[i].y / scale);
        
        // Simple bright colors for debugging
        float r = 1.0f, g = 1.0f, b = 0.0f; // Start with bright yellow
        
        // Real galaxy coloring based on distance from center and mass
        double dist = std::sqrt(bodies[i].x * bodies[i].x + bodies[i].y * bodies[i].y);
        
        if (dist < 0.3) {
            // Central bulge - bright red
            r = 1.0f; g = 0.3f; b = 0.1f;
        } else if (dist < 0.8) {
            // Disk region - bright blue/white
            r = 0.8f; g = 0.9f; b = 1.0f;
        } else {
            // Outer halo - bright orange
            r = 1.0f; g = 0.6f; b = 0.2f;
        }
        
    // Particle size
    float baseSize = 3.5f; // Smaller to see many bodies at once
        float radius = baseSize * pxToNdc;
        
        // Draw simple solid disk without transparency
        glColor3f(r, g, b);
        drawSmallDisk(px, py, radius, 12);
    }
}

int main() {
    if (!glfwInit()) return -1;
    int width = 1200, height = 900;
    glfwWindowHint(GLFW_SAMPLES, 4);
    GLFWwindow* window = glfwCreateWindow(width, height, "Barnes-Hut N-Body Simulation (1000 bodies)", NULL, NULL);
    if (!window) {
        glfwTerminate();
        return -1;
    }
    glfwMakeContextCurrent(window);
    
    // Explicitly set clear color to black and test
    glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    
    // Ensure viewport matches framebuffer size (handles HiDPI scaling too)
    int fbw = 0, fbh = 0;
    glfwGetFramebufferSize(window, &fbw, &fbh);
    if (fbw <= 0 || fbh <= 0) { fbw = width; fbh = height; }
    glViewport(0, 0, fbw, fbh);
    width = fbw; height = fbh;
    
    glDisable(GL_SCISSOR_TEST);
    glDisable(GL_DEPTH_TEST);
    
    glEnable(GL_MULTISAMPLE);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glOrtho(-1, 1, -1, 1, -1, 1);
    glMatrixMode(GL_MODELVIEW);
    
    // Initialize 1000 bodies in aesthetic distributions
    std::vector<Body> bodies;
    std::random_device rd;
    std::mt19937 gen(rd());
    
    // Create multiple spiral galaxy arms
    const int numArms = 4;
    const int bodiesPerArm = 200;
    const int centralCluster = 200;
    
    // Central cluster
    std::normal_distribution<> central_pos(0.0, 0.1);
    std::uniform_real_distribution<> mass_dist(0.8, 1.5);
    for (int i = 0; i < centralCluster; ++i) {
        double x = central_pos(gen);
        double y = central_pos(gen);
        double r = std::sqrt(x*x + y*y);
        double orbital_speed = 0.8 * std::sqrt(1.0 / (r + 0.1));
        double angle = std::atan2(y, x);
        double vx = -orbital_speed * std::sin(angle);
        double vy = orbital_speed * std::cos(angle);
        bodies.emplace_back(x, y, vx, vy, mass_dist(gen));
    }
    
    // Spiral arms
    for (int arm = 0; arm < numArms; ++arm) {
        double armAngle = arm * 2.0 * M_PI / numArms;
        for (int i = 0; i < bodiesPerArm; ++i) {
            double t = (double)i / bodiesPerArm;
            double r = 0.2 + 1.2 * t;
            double theta = armAngle + 3.0 * t; // Spiral
            
            // Add some randomness
            std::normal_distribution<> noise(0.0, 0.05);
            r += noise(gen);
            theta += noise(gen);
            
            double x = r * std::cos(theta);
            double y = r * std::sin(theta);
            
            // Orbital velocity with some tangential component
            double orbital_speed = 0.6 * std::sqrt(1.0 / (r + 0.1));
            double vx = -orbital_speed * std::sin(theta);
            double vy = orbital_speed * std::cos(theta);
            
            bodies.emplace_back(x, y, vx, vy, mass_dist(gen));
        }
    }
    
    BarnesHutSimulator sim(bodies);
    double scale = 2.0;
    
    while (!glfwWindowShouldClose(window)) {
        // Keep viewport in sync if window resized
        int curFbw = 0, curFbh = 0;
        glfwGetFramebufferSize(window, &curFbw, &curFbh);
        if (curFbw > 0 && curFbh > 0 && (curFbw != width || curFbh != height)) {
            width = curFbw; height = curFbh;
            glViewport(0, 0, width, height);
        }

        sim.step();
        drawBodies(bodies, scale, width, height);
        glfwSwapBuffers(window);
        glfwPollEvents();
    }
    
    glfwDestroyWindow(window);
    glfwTerminate();
    return 0;
}
