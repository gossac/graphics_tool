
#include "../rays/samplers.h"
#include "../util/rand.h"
#include "debug.h"
#include <cmath>
#include <algorithm>

namespace Samplers {

Vec2 Rect::Uniform::sample(float& pdf) const {

    // TODO (PathTracer): Task 1
    // Generate a uniformly random point on a rectangle of size size.x * size.y
    // Tip: RNG::unit()
    float random_x = RNG::unit() * size.x; // RNG::unit() generates a random float in [0.0, 1.0).
    float random_y = RNG::unit() * size.y;

    pdf = 1.f / (size.x * size.y); // the PDF should integrate to 1 over the whole rectangle
    return Vec2(random_x, random_y);
}

Vec3 Hemisphere::Cosine::sample(float& pdf) const {

    // TODO (PathTracer): Task 6
    // You may implement this, but don't have to.
    return Vec3();
}

Vec3 Sphere::Uniform::sample(float& pdf) const {

    // TODO (PathTracer): Task 7
    // Generate a uniformly random point on the unit sphere (or equivalently, direction)
    // Tip: start with Hemisphere::Uniform

    Vec3 result = hemi.sample(pdf);

    if (RNG::coin_flip(0.5)) {
        result.y = -result.y;
    }

    pdf /= 2; // what was the PDF at the chosen direction?

    return result;
}

Sphere::Image::Image(const HDR_Image& image) {

    // TODO (PathTracer): Task 7
    // Set up importance sampling for a spherical environment map image.

    // You may make use of the pdf, cdf, and total members, or create your own
    // representation.

    const auto [_w, _h] = image.dimension();
    w = _w;
    h = _h;

    // Precompute cdf and pdf for each pixel.

    total = 0;

    for (size_t y = 0; y < h; y++) {
        for (size_t x = 0; x < w; x++) {
            float luminosity = image.at(x, y).luma();

            float center_y = y + 0.5;
            float sin_theta = std::sin(PI_F * (h - center_y) / h);

            float probability = luminosity * sin_theta;

            pdf.push_back(probability);
            total += probability;
        }
    }

    // Normalize pdf and populate cdf.
    for (size_t i = 0; i < pdf.size(); i++) {
        pdf[i] /= total;

        if (i == 0) {
            cdf.push_back(pdf[i]);
        } else {
            cdf.push_back(cdf.back() + pdf[i]);
        }
    }
}

Vec3 Sphere::Image::sample(float& out_pdf) const {

    // TODO (PathTracer): Task 7
    // Use your importance sampling data structure to generate a sample direction.
    // Tip: std::upper_bound can easily binary search your CDF

    float random_cdf = RNG::unit();

    size_t cdf_idx = std::distance(cdf.begin(), std::upper_bound(cdf.begin(), cdf.end(), random_cdf));

    if (cdf_idx >= cdf.size()) {
        cdf_idx = cdf.size() - 1;
    }

    size_t x = cdf_idx % w;
    size_t y = cdf_idx / w;

    float theta = (h - y) * PI_F / h;
    float phi = ((x + w / 2) % w) * 2 * PI_F / w;

    float xs = std::sin(theta) * std::cos(phi);
    float ys = std::cos(theta);
    float zs = std::sin(theta) * std::sin(phi);

    out_pdf = pdf[cdf_idx] * w * h / (2 * PI_F * PI_F * std::sin(theta)); // what was the PDF (again, PMF here) of your chosen sample?
    return Vec3(xs, ys, zs);
}

Vec3 Point::sample(float& pmf) const {

    pmf = 1.0f;
    return point;
}

Vec3 Two_Points::sample(float& pmf) const {
    if(RNG::unit() < prob) {
        pmf = prob;
        return p1;
    }
    pmf = 1.0f - prob;
    return p2;
}

Vec3 Hemisphere::Uniform::sample(float& pdf) const {

    float Xi1 = RNG::unit();
    float Xi2 = RNG::unit();

    float theta = std::acos(Xi1);
    float phi = 2.0f * PI_F * Xi2;

    float xs = std::sin(theta) * std::cos(phi);
    float ys = std::cos(theta);
    float zs = std::sin(theta) * std::sin(phi);

    pdf = 1.0f / (2.0f * PI_F);
    return Vec3(xs, ys, zs);
}

} // namespace Samplers
