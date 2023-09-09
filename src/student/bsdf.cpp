
#include "../rays/bsdf.h"
#include "../util/rand.h"
#include "debug.h"
#include <cmath>

namespace PT {

Vec3 reflect(Vec3 dir) {

    // TODO (PathTracer): Task 6
    // Return reflection of dir about the surface normal (0,1,0).

    return Vec3(-dir.x, dir.y, -dir.z).unit();
}

Vec3 refract(Vec3 out_dir, float index_of_refraction, bool& was_internal) {

    // TODO (PathTracer): Task 6
    // Use Snell's Law to refract out_dir through the surface
    // Return the refracted direction. Set was_internal to false if
    // refraction does not occur due to total internal reflection,
    // and true otherwise.

    // When dot(out_dir,normal=(0,1,0)) is positive, then out_dir corresponds to a
    // ray exiting the surface into vaccum (ior = 1). However, note that
    // you should actually treat this case as _entering_ the surface, because
    // you want to compute the 'input' direction that would cause this output,
    // and to do so you can simply find the direction that out_dir would refract
    // _to_, as refraction is symmetric.

    was_internal = false;

    out_dir.normalize();
    float ratio; // eta_t / eta_i
    float y;
    if (out_dir.y > 0) { // From inside to vaccum.
        ratio = 1 / index_of_refraction;
        float sqr_sin_theta = (1 - out_dir.y * out_dir.y) * ratio * ratio;

        y = -std::sqrt(1 - sqr_sin_theta);
    } else { // From vaccum to inside.
        ratio = index_of_refraction;
        float sqr_sin_theta = (1 - out_dir.y * out_dir.y) * ratio * ratio;

        if (sqr_sin_theta >= 1) { // Total internal reflection.
            was_internal = true;
            return reflect(out_dir);
        }

        y = std::sqrt(1 - sqr_sin_theta);
    }

    return Vec3(-out_dir.x * ratio, y, -out_dir.z * ratio).unit();
}

BSDF_Sample BSDF_Lambertian::sample(Vec3 out_dir) const {

    // TODO (PathTracer): Task 5
    // Implement lambertian BSDF. Use of BSDF_Lambertian::sampler may be useful

    BSDF_Sample ret;
    ret.direction = sampler.sample(ret.pdf); // What direction should we sample incoming light from?
                                             // Was was the PDF of the sampled direction?
    ret.attenuation = evaluate(ret.direction, out_dir); // What is the ratio of reflected/incoming light?
    return ret;
}

Spectrum BSDF_Lambertian::evaluate(Vec3 out_dir, Vec3 in_dir) const {
    return albedo * (1.0f / PI_F);
}

BSDF_Sample BSDF_Mirror::sample(Vec3 out_dir) const {

    // TODO (PathTracer): Task 6
    // Implement mirror BSDF

    BSDF_Sample ret;
    ret.direction = reflect(out_dir); // What direction should we sample incoming light from?
    ret.pdf = 1.0f; // Was was the PDF of the sampled direction? (In this case, the PMF)
    ret.attenuation = ret.pdf * reflectance / fabs(ret.direction.y); // What is the ratio of reflected/incoming light?
    return ret;
}

Spectrum BSDF_Mirror::evaluate(Vec3 out_dir, Vec3 in_dir) const {
    // Technically, we would return the proper reflectance
    // if in_dir was the perfectly reflected out_dir, but given
    // that we assume these are single exact directions in a
    // continuous space, just assume that we never hit them
    // _exactly_ and always return 0.
    return {};
}

BSDF_Sample BSDF_Glass::sample(Vec3 out_dir) const {

    // TODO (PathTracer): Task 6

    // Implement glass BSDF.

    BSDF_Sample ret;
    bool was_internal = false;
    Vec3 in_dir = refract(out_dir, index_of_refraction, was_internal);
    if (was_internal) {
        ret.direction = in_dir;
        ret.pdf = 1;
        ret.attenuation = ret.pdf * transmittance / std::fabs(ret.direction.y);
        return ret;
    }

    // (1) Compute Fresnel coefficient. Tip: use Schlick's approximation.
    float r0 = (1 - index_of_refraction) * (1 - index_of_refraction) / ((1 + index_of_refraction) * (1 + index_of_refraction));
    float fresnel = r0 + (1 - r0) * std::pow(1 - std::fabs(in_dir.y), 5);

    // (2) Reflect or refract probabilistically based on Fresnel coefficient. Tip: RNG::coin_flip
    // (3) Compute attenuation based on reflectance or transmittance
    if (RNG::coin_flip(fresnel)) { // Reflect.
        ret.direction = reflect(out_dir);
        ret.pdf = fresnel;
        ret.attenuation = ret.pdf * reflectance / std::fabs(ret.direction.y);
    } else { // Refract.
        ret.direction = in_dir;
        ret.pdf = 1 - fresnel;

        if (out_dir.y >= 0) {
            ret.attenuation = ret.pdf / (index_of_refraction * index_of_refraction) * transmittance / std::fabs(ret.direction.y);
        } else {
            ret.attenuation = ret.pdf * index_of_refraction * index_of_refraction * transmittance / std::fabs(ret.direction.y);
        }
    }

    // Be wary of your eta1/eta2 ratio - are you entering or leaving the surface?

    return ret;
}

Spectrum BSDF_Glass::evaluate(Vec3 out_dir, Vec3 in_dir) const {
    // As with BSDF_Mirror, just assume that we never hit the correct
    // directions _exactly_ and always return 0.
    return {};
}

BSDF_Sample BSDF_Diffuse::sample(Vec3 out_dir) const {
    BSDF_Sample ret;
    ret.direction = sampler.sample(ret.pdf);
    ret.emissive = radiance;
    ret.attenuation = {};
    return ret;
}

Spectrum BSDF_Diffuse::evaluate(Vec3 out_dir, Vec3 in_dir) const {
    // No incoming light is reflected; only emitted
    return {};
}

BSDF_Sample BSDF_Refract::sample(Vec3 out_dir) const {

    // TODO (PathTracer): Task 6
    // Implement pure refraction BSDF.

    // Be wary of your eta1/eta2 ratio - are you entering or leaving the surface?

    bool was_internal = false;

    BSDF_Sample ret;
    ret.direction = refract(out_dir, index_of_refraction, was_internal); // What direction should we sample incoming light from?
    ret.pdf = 1.0f; // Was was the PDF of the sampled direction? (In this case, the PMF)
    ret.attenuation = ret.pdf * transmittance / std::fabs(ret.direction.y); // What is the ratio of reflected/incoming light?
    return ret;
}

Spectrum BSDF_Refract::evaluate(Vec3 out_dir, Vec3 in_dir) const {
    // As with BSDF_Mirror, just assume that we never hit the correct
    // directions _exactly_ and always return 0.
    return {};
}

} // namespace PT
