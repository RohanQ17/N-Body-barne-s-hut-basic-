#include <iostream>
#include <vector>
#include <cmath>
#include <random>

struct Body {
    double x, y;
    double vx, vy;
    double mass;
};

// For realistic simulation, the actual gravitational constant (SI units):
// G = 6.67430e-11 m^3 kg^-1 s^-2
// SI units for mass (kg), position (meters), and time (seconds).
class NBodySimulator {
public:
    NBodySimulator(std::vector<Body>& bodies, double G = 6.67430e-11, double dt = 1.0)
        : bodies(bodies), G(G), dt(dt) {}

    void step() {
        std::vector<std::pair<double, double>> forces(bodies.size(), {0.0, 0.0});
        for (size_t i = 0; i < bodies.size(); ++i) {
            double fx = 0.0, fy = 0.0;
            for (size_t j = 0; j < bodies.size(); ++j) {
                if (i == j) continue;
                double dx = bodies[j].x - bodies[i].x;
                double dy = bodies[j].y - bodies[i].y;
                double dist_sq = dx * dx + dy * dy + 1e-10;
                double dist = std::sqrt(dist_sq);
                double force = G * bodies[i].mass * bodies[j].mass / dist_sq;
                fx += force * dx / dist;
                fy += force * dy / dist;
            }
            forces[i] = {fx, fy};
        }
        for (size_t i = 0; i < bodies.size(); ++i) {
            double ax = forces[i].first / bodies[i].mass;
            double ay = forces[i].second / bodies[i].mass;
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

int main() {
    // Example: 3 bodies with random positions and masses (SI units)
    // Positions in meters, masses in kilograms
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> pos_dist(-1e11, 1e11); // meters
    std::uniform_real_distribution<> mass_dist(1e24, 1e25); // kg
    std::vector<Body> bodies;
    for (int i = 0; i < 3; ++i) {
        bodies.push_back({pos_dist(gen), pos_dist(gen), 0.0, 0.0, mass_dist(gen)});
    }
    NBodySimulator sim(bodies);
    int steps = 100;
    for (int step = 0; step < steps; ++step) {
        sim.step();
        std::cout << "Step " << step + 1 << "\n";
        for (const auto& b : bodies) {
            std::cout << "Body(x=" << b.x << ", y=" << b.y << ", vx=" << b.vx << ", vy=" << b.vy << ", mass=" << b.mass << ")\n";
        }
        std::cout << std::endl;
    }
    return 0;
}
