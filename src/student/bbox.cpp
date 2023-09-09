
#include "../lib/mathlib.h"
#include "debug.h"
#include <iostream>

bool BBox::hit(const Ray& ray, Vec2& times) const {
    // TODO (PathTracer):
    // Implement ray - bounding box intersection test
    // If the ray intersected the bounding box within the range given by
    // [times.x,times.y], update times with the new intersection times.
    float tmin = ray.dist_bounds.x, tmax = ray.dist_bounds.y;

    if (tmin < times.x) {
        tmin = times.x;
    }
    if (tmax > times.y) {
        tmax = times.y;
    }

    float txmin, txmax;
    if (ray.invdir.x < 0) {
        txmin = (max.x - ray.point.x) * ray.invdir.x;
        txmax = (min.x - ray.point.x) * ray.invdir.x;
    } else {
        txmin = (min.x - ray.point.x) * ray.invdir.x;
        txmax = (max.x - ray.point.x) * ray.invdir.x;
    }
    if (txmax < tmin || txmin > tmax) {
        return false;
    }
    if (tmin < txmin) {
        tmin = txmin;
    }
    if (tmax > txmax) {
        tmax = txmax;
    }

    float tymin, tymax;
    if (ray.invdir.y < 0) {
        tymin = (max.y - ray.point.y) * ray.invdir.y;
        tymax = (min.y - ray.point.y) * ray.invdir.y;
    } else {
        tymin = (min.y - ray.point.y) * ray.invdir.y;
        tymax = (max.y - ray.point.y) * ray.invdir.y;
    }
    if (tymax < tmin || tymin > tmax) {
        return false;
    }
    if (tmin < tymin) {
        tmin = tymin;
    }
    if (tmax > tymax) {
        tmax = tymax;
    }

    float tzmin, tzmax;
    if (ray.invdir.z < 0) {
        tzmin = (max.z - ray.point.z) * ray.invdir.z;
        tzmax = (min.z - ray.point.z) * ray.invdir.z;
    } else {
        tzmin = (min.z - ray.point.z) * ray.invdir.z;
        tzmax = (max.z - ray.point.z) * ray.invdir.z;
    }
    if (tzmax < tmin || tzmin > tmax) {
        return false;
    }
    if (tmin < tzmin) {
        tmin = tzmin;
    }
    if (tmax > tzmax) {
        tmax = tzmax;
    }

    times.x = tmin;
    times.y = tmax;

    return true;
}
