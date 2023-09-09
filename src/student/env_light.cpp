
#include "../rays/env_light.h"
#include "debug.h"

#include <limits>
#include <cmath>

namespace PT {

Light_Sample Env_Map::sample() const {

    Light_Sample ret;
    ret.distance = std::numeric_limits<float>::infinity();

    // TODO (PathTracer): Task 7
    // Uniformly sample the sphere. Tip: implement Samplers::Sphere::Uniform
    // Samplers::Sphere::Uniform uniform;
    // ret.direction = uniform.sample(ret.pdf);

    // Once you've implemented Samplers::Sphere::Image, remove the above and
    // uncomment this line to use importance sampling instead.
    ret.direction = sampler.sample(ret.pdf);

    ret.radiance = sample_direction(ret.direction);
    return ret;
}

Spectrum Env_Map::sample_direction(Vec3 dir) const {

    // TODO (PathTracer): Task 7
    // Find the incoming light along a given direction by finding the corresponding
    // place in the enviornment image. You should bi-linearly interpolate the value
    // between the 4 image pixels nearest to the exact direction.
    float phi;
    if (dir.x == 0 && dir.z == 0) {
        phi = 0;
    } else {
        phi = std::atan2(dir.z, dir.x);
        if (phi < 0) {
            phi += 2 * PI_F;
        }
    }
    float theta = std::acos(dir.y);

    size_t w = image.dimension().first;
    size_t h = image.dimension().second;
    size_t x_min = (size_t)(w / 2 + w * phi / (2 * PI_F)) % w, y_min = h * (PI_F - theta) / PI_F;
    size_t x_max = x_min + 1, y_max = y_min + 1;

    if (x_min >= w) {
        x_min = w - 1;
    }
    if (x_max >= w) {
        x_max = w - 1;
    }
    if (y_min >= h) {
        y_min = h - 1;
    }
    if (y_max >= h) {
        y_max = h - 1;
    }

    Spectrum upper_left = image.at(x_min, y_max);
    Spectrum upper_right = image.at(x_max, y_max);
    Spectrum lower_left = image.at(x_min, y_min);
    Spectrum lower_right = image.at(x_max, y_min);

    return ((upper_left + upper_right) / 2 + (lower_left + lower_right) / 2) / 2;
}

Light_Sample Env_Hemisphere::sample() const {
    Light_Sample ret;
    ret.direction = sampler.sample(ret.pdf);
    ret.radiance = radiance;
    ret.distance = std::numeric_limits<float>::infinity();
    return ret;
}

Spectrum Env_Hemisphere::sample_direction(Vec3 dir) const {
    if(dir.y > 0.0f) return radiance;
    return {};
}

Light_Sample Env_Sphere::sample() const {
    Light_Sample ret;
    ret.direction = sampler.sample(ret.pdf);
    ret.radiance = radiance;
    ret.distance = std::numeric_limits<float>::infinity();
    return ret;
}

Spectrum Env_Sphere::sample_direction(Vec3) const {
    return radiance;
}

} // namespace PT
