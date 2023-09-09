
#include "../rays/tri_mesh.h"
#include "debug.h"

namespace PT {

BBox Triangle::bbox() const {

    // TODO (PathTracer): Task 2
    // compute the bounding box of the triangle
    Tri_Mesh_Vert v_0 = vertex_list[v0];
    Tri_Mesh_Vert v_1 = vertex_list[v1];
    Tri_Mesh_Vert v_2 = vertex_list[v2];

    Vec3 tri_min = hmin(hmin(v_0.position, v_1.position), v_2.position);
    Vec3 tri_max = hmax(hmax(v_0.position, v_1.position), v_2.position);

    // Beware of flat/zero-volume boxes! You may need to
    // account for that here, or later on in BBox::intersect
    if (tri_min.x >= tri_max.x) {
        tri_max.x = tri_min.x + EPS_F;
    }
    if (tri_min.y >= tri_max.y) {
        tri_max.y = tri_min.y + EPS_F;
    }
    if (tri_min.z >= tri_max.z) {
        tri_max.z = tri_min.z + EPS_F;
    }

    BBox box(tri_min, tri_max);
    return box;
}

Trace Triangle::hit(const Ray& ray) const {
    // Vertices of triangle - has position and surface normal
    Tri_Mesh_Vert v_0 = vertex_list[v0];
    Tri_Mesh_Vert v_1 = vertex_list[v1];
    Tri_Mesh_Vert v_2 = vertex_list[v2];
    (void)v_0;
    (void)v_1;
    (void)v_2;

    // TODO (PathTracer): Task 2
    // Intersect this ray with a triangle defined by the three above points.
    Vec3 o = ray.point;
    Vec3 d = ray.dir;
    Vec3 e1 = v_1.position - v_0.position;
    Vec3 e2 = v_2.position - v_0.position;
    Vec3 s = o - v_0.position;

    float denominator = dot(cross(e1, d), e2);

    if (denominator != 0) { // The ray is not parallel to the plane.
        float t = -dot(cross(s, e2), e1) / denominator;
        Vec3 path = t * d; // Path of the ray.
        float dist = t;

        if (dist >= ray.dist_bounds.x && dist <= ray.dist_bounds.y) { //  The intersection point on the ray is within the ray’s dist_bounds.
            float u = -dot(cross(s, e2), d) / denominator;
            float v = dot(cross(e1, d), s) / denominator;
            float w = 1 - u - v; // Weight of v_0.

            if (u >= 0 && v >= 0 && w >= 0) { // The intersection point is inside the triangle.
                Trace ret;
                ret.origin = o;
                ret.hit = true;
                ret.distance = dist;
                ret.position = o + path;
                ret.normal = w * v_0.normal + u * v_1.normal + v * v_2.normal;
                ret.normal.normalize();

                ray.dist_bounds.y = dist; // Shrink the ray's dist_bounds.

                return ret;
            }
        }
    }

    Trace ret;
    ret.origin = ray.point;
    ret.hit = false;       // was there an intersection?
    ret.distance = 0.0f;   // at what distance did the intersection occur?
    ret.position = Vec3{}; // where was the intersection?
    ret.normal = Vec3{};   // what was the surface normal at the intersection?
                           // (this should be interpolated between the three vertex normals)
    return ret;
}

Triangle::Triangle(Tri_Mesh_Vert* verts, unsigned int v0, unsigned int v1, unsigned int v2)
    : vertex_list(verts), v0(v0), v1(v1), v2(v2) {
}

void Tri_Mesh::build(const GL::Mesh& mesh) {

    verts.clear();
    triangles.clear();

    for(const auto& v : mesh.verts()) {
        verts.push_back({v.pos, v.norm});
    }

    const auto& idxs = mesh.indices();

    std::vector<Triangle> tris;
    for(size_t i = 0; i < idxs.size(); i += 3) {
        tris.push_back(Triangle(verts.data(), idxs[i], idxs[i + 1], idxs[i + 2]));
    }

    triangles.build(std::move(tris), 4);
}

Tri_Mesh::Tri_Mesh(const GL::Mesh& mesh) {
    build(mesh);
}

Tri_Mesh Tri_Mesh::copy() const {
    Tri_Mesh ret;
    ret.verts = verts;
    ret.triangles = triangles.copy();
    return ret;
}

BBox Tri_Mesh::bbox() const {
    return triangles.bbox();
}

Trace Tri_Mesh::hit(const Ray& ray) const {
    Trace t = triangles.hit(ray);
    return t;
}

size_t Tri_Mesh::visualize(GL::Lines& lines, GL::Lines& active, size_t level,
                           const Mat4& trans) const {
    return triangles.visualize(lines, active, level, trans);
}

} // namespace PT
