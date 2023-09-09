
#include <cmath>

#include "../util/camera.h"
#include "../rays/samplers.h"
#include "debug.h"

Ray Camera::generate_ray(Vec2 screen_coord) const {

    // TODO (PathTracer): Task 1
    // compute position of the input sensor sample coordinate on the
    // canonical sensor plane one unit away from the pinhole.
    // Tip: compute the ray direction in view space and use
    // the camera transform to transform it back into world space.
    float sensor_z = -focal_dist;

    float half_sh = std::tan(Radians(vert_fov / 2)) * std::abs(sensor_z); // Half of the screen height.
    float half_sw = half_sh * aspect_ratio;
    float sh = 2 * half_sh;
    float sw = 2 * half_sw;
    Vec2 sensor_xy = Vec2(-half_sw, -half_sh) + screen_coord * Vec2(sw, sh);

    Vec3 sensor(sensor_xy.x, sensor_xy.y, sensor_z);

    Vec2 ray_origin_xy(0, 0);
    if (aperture > 0) {
        // Randomly choose the ray origin from an aperture * aperture square centered at the origin and facing the camera direction (-Z).
        float half_aperture = aperture / 2;
        float pdf; // Only used to fill in the argument list of Samplers::Rect::Uniform::sample(float&).
        ray_origin_xy = Vec2(-half_aperture, -half_aperture) + Samplers::Rect::Uniform(Vec2(aperture, aperture)).sample(pdf);
    }

    Vec3 ray_origin(ray_origin_xy.x, ray_origin_xy.y, 0.0);

    Ray ray(ray_origin, sensor);
    ray.transform(iview); // Transform ray to world space.

    return ray;
}
