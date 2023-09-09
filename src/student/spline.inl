
#include "../geometry/spline.h"
#include "debug.h"
#include <map>

template<typename T>
T Spline<T>::cubic_unit_spline(float time, const T& position0, const T& position1,
                               const T& tangent0, const T& tangent1) {

    // TODO (Animation): Task 1a
    // Given time in [0,1] compute the cubic spline coefficients and use them to compute
    // the interpolated value at time 'time' based on the positions & tangents

    // Note that Spline is parameterized on type T, which allows us to create splines over
    // any type that supports the * and + operators.

    float t = time;
    float sqr_t = t * t;
    float cube_t = sqr_t * t;

    float h00 = 2 *cube_t - 3 * sqr_t + 1;
    float h10 = cube_t - 2 * sqr_t + t;
    float h01 = -2 * cube_t + 3 * sqr_t;
    float h11 = cube_t - sqr_t;

    T res = h00 * position0 + h10 * tangent0 + h01 * position1 + h11 * tangent1;

    return res;
}

template<typename T> T Spline<T>::at(float time) const {

    // TODO (Animation): Task 1b

    // Given a time, find the nearest positions & tangent values
    // defined by the control point map.

    // Transform them for use with cubic_unit_spline

    // Be wary of edge cases! What if time is before the first knot,
    // before the second knot, etc...

    // If there are no knots at all in the spline,
    // interpolation should return the default value for the interpolated type.
    // This value can be computed by simply calling the constructor for the type: T().
    if (!any()) {
        return T();
    }

    // If there is only one knot in the spline,
    // interpolation should always return the value of that knot (independent of the time).
    if (control_points.size() == 1) {
        return control_points.begin()->second;
    }

    // If the query time is greater than or equal to the final knot,
    // return the final knot’s value.
    typename std::map<float, T>::const_iterator final_iterator = control_points.end();
    final_iterator--;
    if (time >= final_iterator->first) {
        return final_iterator->second;
    }

    // If the query time is less than or equal to the initial knot,
    // return the initial knot’s value.
    typename std::map<float, T>::const_iterator initial_iterator = control_points.begin();
    if (time <= initial_iterator->first) {
        return initial_iterator->second;
    }

    // Any query time between the first and last knot will have at least one knot "to the left" (k1) and one "to the right" (k2).
    typename std::map<float, T>::const_iterator iterator_k1 = control_points.lower_bound(time), iterator_k2 = iterator_k1--;
    std::pair<float, T> k1 = *iterator_k1, k2 = *iterator_k2;

    // Precompute delta t and delta p.
    float delta_time = k2.first - k1.first;
    T delta_position = k2.second - k1.second;

    typename std::map<float, T>::const_iterator iterator_k0 = control_points.end();
    std::pair<float, T> k0;
    if (iterator_k1 == control_points.begin()) {
        // Suppose we don’t have a knot "two to the left" (k0).
        // Then we will define a "virtual" knot k0 = k1 - (k2 - k1).

        k0.first = k1.first - delta_time;
        k0.second = k1.second - delta_position;
    } else {
        iterator_k0 = iterator_k1;
        iterator_k0--;
        k0 = *iterator_k0;
    }

    typename std::map<float, T>::const_iterator iterator_k3 = iterator_k2;
    iterator_k3++;
    std::pair<float, T> k3;
    if (iterator_k3 == control_points.end()) {
        // Likewise, if we don’t have a knot "two to the right" (k3),
        // then we will "mirror" the difference to get a "virtual" knot k3 = k2 + (k2 - k1).

        k3.first = k2.first + delta_time;
        k3.second = k2.second + delta_position;
    } else {
        k3 = *iterator_k3;
    }

    // At this point, we have four valid knot values (whether "real" or "virtual"),
    // and can compute our tangents and positions as usual.
    T tangent0 = (k2.second - k0.second) / (k2.first - k0.first);
    T tangent1 = (k3.second - k1.second) / (k3.first - k1.first);
    float normalized_time = (time - k1.first) / delta_time;

    return cubic_unit_spline(normalized_time, k1.second, k2.second, tangent0, tangent1);
}
