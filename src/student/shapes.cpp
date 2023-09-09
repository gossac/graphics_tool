
#include <cmath>

#include "../rays/shapes.h"
#include "debug.h"

namespace PT {

const char* Shape_Type_Names[(int)Shape_Type::count] = {"None", "Sphere"};

BBox Sphere::bbox() const {

    BBox box;
    box.enclose(Vec3(-radius));
    box.enclose(Vec3(radius));
    return box;
}

Trace Sphere::hit(const Ray& ray) const {
    // TODO (PathTracer): Task 2
    // Intersect this ray with a sphere of radius Sphere::radius centered at the origin.

    // If the ray intersects the sphere twice, ret should
    // represent the first intersection, but remember to respect
    // ray.dist_bounds! For example, if there are two intersections,
    // but only the _later_ one is within ray.dist_bounds, you should
    // return that one!
    Vec3 o = ray.point;
    Vec3 d = ray.dir;
    float dot_prod = dot(o, d); // Precompute this dot product.

    float discriminant = dot_prod * dot_prod - o.norm_squared() + radius * radius;

    if (discriminant >= 0) { // Hit the sphere.
        float sqr_root = std::sqrt(discriminant); // Precompute this square root.

        float t1 = -dot_prod - sqr_root;
        Vec3 path1 = t1 * d;
        float dist1 = t1;

        if (dist1 >= ray.dist_bounds.x && dist1 <= ray.dist_bounds.y) { // If the first intersection point on the ray is within the ray’s dist_bounds, it must be closer than the second intersection point.
            Trace ret;
            ret.origin = o;
            ret.hit = true;
            ret.distance = dist1;
            ret.position = o + path1;
            ret.normal = ret.position.unit();

            ray.dist_bounds.y = dist1;

            return ret;
        }

        float t2 = -dot_prod + sqr_root;
        Vec3 path2 = t2 * d;
        float dist2 = t2;

        if (dist2 >= ray.dist_bounds.x && dist2 <= ray.dist_bounds.y) { // If the first intersection point on the ray is out of the ray’s dist_bounds, check the second intersection point.
            Trace ret;
            ret.origin = o;
            ret.hit = true;
            ret.distance = dist2;
            ret.position = o + path2;
            ret.normal = ret.position.unit();

            ray.dist_bounds.y = dist2;

            return ret;
        }
    }

    Trace ret;
    ret.origin = ray.point;
    ret.hit = false;       // was there an intersection?
    ret.distance = 0.0f;   // at what distance did the intersection occur?
    ret.position = Vec3{}; // where was the intersection?
    ret.normal = Vec3{};   // what was the surface normal at the intersection?
    return ret;
}

} // namespace PT
