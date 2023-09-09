
#include "../scene/particles.h"
#include "../rays/pathtracer.h"

bool Particle::update(const PT::BVH<PT::Object>& scene, float dt, float radius) {
    float time_remaining = dt;
    while (time_remaining > 0) {
        Ray ray(pos, velocity);
        PT::Trace trace = scene.hit(ray);
        if (trace.hit) {
            float cos_theta = dot(trace.normal, ray.dir);
            if (cos_theta < 0) {
                cos_theta = -cos_theta;
            }
            float d = trace.distance - radius / cos_theta;
            float t = d / velocity.norm();
            if (t >= time_remaining) {
                pos += time_remaining * velocity;
                time_remaining = 0;
            } else {
                pos += t * velocity;
                if (t >= 0) {
                    time_remaining -= t;
                }
                velocity -= 2 * dot(velocity, trace.normal) * trace.normal;
            }
        } else {
            pos += time_remaining * velocity;
            time_remaining = 0;
        }
    }
    velocity += dt * acceleration;
    age -= dt;
    return age > 0;
}
