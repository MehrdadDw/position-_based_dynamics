#include <SFML/Graphics.hpp>
#include <vector>
#include <cmath>

struct Vec2 {
    float x, y;
    Vec2(float x_ = 0.0f, float y_ = 0.0f) : x(x_), y(y_) {}
    Vec2 operator+(const Vec2& v) const { return Vec2(x + v.x, y + v.y); }
    Vec2 operator-(const Vec2& v) const { return Vec2(x - v.x, y - v.y); }
    Vec2 operator*(float s) const { return Vec2(x * s, y * s); }
    Vec2 operator/(float s) const { return Vec2(x / s, y / s); }
    float length() const { return std::sqrt(x * x + y * y); }
};

struct Particle {
    Vec2 pos;
    Vec2 prev_pos;
    Vec2 vel;
    float inv_mass;
    bool fixed;
    bool has_shadow;  // Added to track if particle has a shadow
    size_t shadow_index;  // Index into shadow_particles vector
    Particle() : has_shadow(false), shadow_index(0) {}  // Constructor to initialize new members
};

struct DistanceConstraint {
    int p1, p2;
    float rest_length;
};

class PBDSolver {
public:
    std::vector<Particle> particles;
    std::vector<Particle> shadow_particles;
    std::vector<DistanceConstraint> constraints;
    Vec2 gravity = Vec2(0.0f, 9.81f/100.0f);
    float time_step = 10.0f / 60.0f;
    int num_iterations = 5;

    void addParticle(float x, float y, float mass, bool fixed = false) {
        Particle p;
        p.pos = Vec2(x, y);
        p.prev_pos = p.pos;
        p.vel = Vec2(0.0f, 0.0f);
        p.inv_mass = (mass > 0.0f) ? 1.0f / mass : 0.0f;
        p.fixed = fixed;
        p.has_shadow = true;
        p.shadow_index = shadow_particles.size();  // Index of corresponding shadow
        particles.push_back(p);

        // Shadow particle
        Particle shadow_p;
        shadow_p.pos = p.pos;
        shadow_p.prev_pos = shadow_p.pos;
        shadow_p.vel = Vec2(0.0f, 0.0f);
        shadow_p.inv_mass = (mass > 0.0f) ? 1.0f / mass : 0.0f;
        shadow_p.fixed = fixed;
        shadow_p.has_shadow = false;  // Shadow doesn't have its own shadow
        shadow_particles.push_back(shadow_p);
    }

    void addDistanceConstraint(int p1, int p2) {
        DistanceConstraint c;
        c.p1 = p1;
        c.p2 = p2;
        c.rest_length = (particles[p1].pos - particles[p2].pos).length();
        constraints.push_back(c);
    }

    void simulate() {
        for (auto& p : particles) {
            if (!p.fixed) {
                p.vel = p.vel + gravity * (time_step * 50.0f);
                p.prev_pos = p.pos;
                p.pos = p.pos + p.vel * time_step;
                if (p.has_shadow) {
                    shadow_particles[p.shadow_index].pos = p.pos; 
                }
            }
        }
        for (int iter = 0; iter < num_iterations; ++iter) {
            for (const auto& c : constraints) {
                Particle& p1 = particles[c.p1];
                Particle& p2 = particles[c.p2];
                if (p1.fixed && p2.fixed) continue;
                Vec2 delta = p2.pos - p1.pos;
                float current_length = delta.length();
                float error = current_length - c.rest_length;
                if (current_length > 1e-6f) delta = delta / current_length;
                else delta = Vec2(0.0f, 0.0f);
                float w1 = p1.inv_mass, w2 = p2.inv_mass;
                float total_weight = w1 + w2;
                if (total_weight < 1e-6f) continue;
                float correction = error / total_weight;
                Vec2 correction_vec = delta * correction;
                if (!p1.fixed) p1.pos = p1.pos + correction_vec * w1;
                if (!p2.fixed) p2.pos = p2.pos - correction_vec * w2;
            }
        }
        for (auto& p : particles) {
            if (!p.fixed) p.vel = (p.pos - p.prev_pos) / time_step;
        }
    }
};

int main() {
    sf::RenderWindow window(sf::VideoMode(800, 600), "Simple PBD with SFML");
    window.setFramerateLimit(60);

    std::vector<PBDSolver> solvers(3);
    std::vector<Vec2> centers = {
        Vec2(200.0f, 250.0f),
        Vec2(5 * 50.0f + 200.0f, 250.0f),
        Vec2((5+4) * 50.0f + 200.0f, 250.0f)
    };
    std::vector<int> particle_counts = {5, 2, 5};
    std::vector<std::vector<float>> Ws = {{1.0f, 1.0f, 1.0f,1.0f,3.0f},{1.0f,1.0f},{1.0f, 1.5f, 2.0f,2.5f,3.0f}};

    for (int s = 0; s < solvers.size(); ++s) {
        for (int i = 0; i < particle_counts[s]; ++i) {
            solvers[s].addParticle(centers[s].x + i * 50.0f, centers[s].y, Ws[s][i], i == 0);
            if (i > 0) {
                solvers[s].addDistanceConstraint(i - 1, i);
            }
        }
    }

    while (window.isOpen()) {
        sf::Event event;
        while (window.pollEvent(event)) {
            if (event.type == sf::Event::Closed) window.close();
        }

        for (auto& solver : solvers) {
            solver.simulate();
        }

        window.clear(sf::Color::Black);

        sf::CircleShape particle_shape(5.0f);
        sf::CircleShape particle_shape_shadow(5.0f);
        particle_shape.setFillColor(sf::Color::White);
        particle_shape_shadow.setFillColor(sf::Color::Green);



    for (int s = 0; s < solvers.size(); ++s) {
            // Draw shadow particles first
            for (const auto& shadow : solvers[s].shadow_particles) {
                 particle_shape.setRadius(5.0f);
                particle_shape_shadow.setPosition(shadow.pos.x - 5.0f, shadow.pos.y - 5.0f);
                window.draw(particle_shape_shadow);
            }
            // Draw regular particles
                for (int i = 0; i < particle_counts[s]; ++i) {
                 particle_shape.setRadius(Ws[s][i]*5.0f);
                particle_shape.setPosition(solvers[s].particles[i].pos.x - (Ws[s][i]*5.0f), solvers[s].particles[i].pos.y - (Ws[s][i]*5.0f));
                window.draw(particle_shape);
            }
    }

        for (const auto& solver : solvers) {
            for (const auto& c : solver.constraints) {
                Vec2 p1_pos = solver.particles[c.p1].pos;
                Vec2 p2_pos = solver.particles[c.p2].pos;
                sf::Vertex line[] = {
                    sf::Vertex(sf::Vector2f(p1_pos.x, p1_pos.y), sf::Color::Red),
                    sf::Vertex(sf::Vector2f(p2_pos.x, p2_pos.y), sf::Color::Red)
                };
                window.draw(line, 2, sf::Lines);
            }
        }

        window.display();
    }

    return 0;
}