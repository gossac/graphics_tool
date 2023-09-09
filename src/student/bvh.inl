
#include "../rays/bvh.h"
#include "debug.h"
#include <stack>
#include <iostream>
#include <cfloat>
#include <algorithm>
#include <vector>
#include <utility>

namespace PT {

// compute_bucket takes a value that represents a primitive
// (e.g. one dimension of its centroid), as well as bounds of this value
// (low <= val and val <= high), then tells index of the bucket
// it belongs to.
//
// The index should be a non-negative integer smaller than B.
// Otherwise the function will return -1.
template<typename Primitive> int BVH<Primitive>::compute_bucket(float val, float low, float high) {
    // The current implementation considers buckets uniformly distributed.
    if (val == high) {
        return B - 1;
    }

    float interval = (high - low) / B;
    float offset = val - low;
    int bucket_idx = offset / interval;

    if (bucket_idx < 0 || bucket_idx >= B) {
        std::cerr << "Bucket index out of bound.\n";
        return -1;
    }

    return bucket_idx;
}

// Partition the node's primitives into two subtrees until leaves are met.
template<typename Primitive> void BVH<Primitive>::part(size_t node_idx, size_t max_leaf_size) {
    size_t start = nodes[node_idx].start;
    size_t size = nodes[node_idx].size;
    size_t end = start + size;

    if (!nodes[node_idx].is_leaf()) { // The node must be a leaf.
        return;
    }
    if (size <= max_leaf_size) { // If the node is small enough, just return.
        return;
    }

    // -----------------------------------------------------------
    // for each axis x, y, z:
    //     initialize buckets
    //     for each primitive p in node:
    //         b = compute_bucket(p.centroid)
    //         b.bbox.union(p.bbox)
    //         b.prim_count++
    //     for each of the B - 1 possible partitioning planes
    //         evaluate cost, keep track of lowest cost partition
    // recurse on lowest cost partition found (or make node a leaf)
    // ------------------------------------------------------------
    float lowest_cost = FLT_MAX;
    int lowest_cost_axis_idx = -1; // Index of the axis.
    int lowest_cost_bucket_r_idx = -1; // Index of the fisrt bucket on the right side.
    Bucket lowest_cost_part_l; // Partition on the left side.
    Bucket lowest_cost_part_r; // Partition on the right side.
    for (int i = 0; i < 3; i++) {
        Bucket buckets[B];
        for (size_t j = start; j < end; j++) {
            BBox bbox = primitives[j].bbox();

            int bucket_idx = compute_bucket(bbox.center()[i], nodes[node_idx].bbox.min[i], nodes[node_idx].bbox.max[i]);
            if (bucket_idx < 0) { // Error information has been printed out.
                return;
            }

            buckets[bucket_idx].bbox.enclose(bbox);
            buckets[bucket_idx].prim_count++;
        }

        // Precompute every possible partition on the right side.
        std::vector<Bucket> parts; // parts[k] stores buckets[B - 1 - k],... buckets[B - 1] combined.
        parts.push_back(buckets[B - 1]);
        for (int j = 1; j < B - 1; j++) {
            Bucket part_r = parts[j - 1];
            Bucket bucket = buckets[B - 1 - j];
            part_r.bbox.enclose(bucket.bbox);
            part_r.prim_count += bucket.prim_count;

            parts.push_back(part_r);
        }

        Bucket part_l = buckets[0]; // Extended in each loop.
        Bucket part_r = parts[B - 2]; // Fetched from parts.
        for (int j = 0; j < B - 1; j++) {
            if (j > 0) {
                Bucket bucket = buckets[j];
                part_l.bbox.enclose(bucket.bbox);
                part_l.prim_count += bucket.prim_count;

                part_r = parts[B - 2 - j];
            }

            // SA * NA + SB * NB is a good estimate.
            float cost = part_l.bbox.surface_area() * part_l.prim_count +
                    part_r.bbox.surface_area() * part_r.prim_count;

            if (lowest_cost > cost) {
                lowest_cost = cost;
                lowest_cost_axis_idx = i;
                lowest_cost_bucket_r_idx = j + 1;
                lowest_cost_part_l = part_l;
                lowest_cost_part_r = part_r;
            }
        }
    }

    if (lowest_cost_part_l.prim_count <= 0 || lowest_cost_part_r.prim_count <= 0) {
        return;
    }

    // Rearrange primitives so that lowest_cost_part_l and lowest_cost_part_r both have consequtive primitives.
    float low = nodes[node_idx].bbox.min[lowest_cost_axis_idx];
    float high = nodes[node_idx].bbox.max[lowest_cost_axis_idx];
    size_t middle = std::partition(primitives.begin() + start, primitives.begin() + end, [=](Primitive& primitive)->bool {
        return compute_bucket(primitive.bbox().center()[lowest_cost_axis_idx], low, high) < lowest_cost_bucket_r_idx;
    }) - primitives.begin();

    // Create subtrees.
    size_t node_l_idx = new_node(lowest_cost_part_l.bbox, start, lowest_cost_part_l.prim_count, 0, 0);
    size_t node_r_idx = new_node(lowest_cost_part_r.bbox, middle, lowest_cost_part_r.prim_count, 0, 0);

    nodes[node_idx].l = node_l_idx;
    nodes[node_idx].r = node_r_idx;

    part(node_l_idx, max_leaf_size);
    part(node_r_idx, max_leaf_size);
}

template<typename Primitive>
void BVH<Primitive>::build(std::vector<Primitive>&& prims, size_t max_leaf_size) {
    // NOTE (PathTracer):
    // This BVH is parameterized on the type of the primitive it contains. This allows
    // us to build a BVH over any type that defines a certain interface. Specifically,
    // we use this to both build a BVH over triangles within each Tri_Mesh, and over
    // a variety of Objects (which might be Tri_Meshes, Spheres, etc.) in Pathtracer.
    //
    // The Primitive interface must implement these two functions:
    //      BBox bbox() const;
    //      Trace hit(const Ray& ray) const;
    // Hence, you may call bbox() and hit() on any value of type Primitive.
    //
    // Finally, also note that while a BVH is a tree structure, our BVH nodes don't
    // contain pointers to children, but rather indicies. This is because instead
    // of allocating each node individually, the BVH class contains a vector that
    // holds all of the nodes. Hence, to get the child of a node, you have to
    // look up the child index in this vector (e.g. nodes[node.l]). Similarly,
    // to create a new node, don't allocate one yourself - use BVH::new_node, which
    // returns the index of a newly added node.

    // Keep these
    nodes.clear();
    primitives = std::move(prims);

    // TODO (PathTracer): Task 3
    // Construct a BVH from the given vector of primitives and maximum leaf
    // size configuration. The starter code builds a BVH with a
    // single leaf node (which is also the root) that encloses all the
    // primitives.

    // Replace these
    BBox box;
    for(const Primitive& prim : primitives) box.enclose(prim.bbox());

    new_node(box, 0, primitives.size(), 0, 0);
    root_idx = 0;

    part(root_idx, max_leaf_size);
}

template<typename Primitive> void BVH<Primitive>::find_closest_hit(const Ray &ray, size_t node_idx, Trace &closest) const {
    const Node &node = nodes[node_idx];
    if (node.is_leaf()) {
        for (size_t i = node.start; i < node.start + node.size; i++) {
            Trace trace = primitives[i].hit(ray);
            closest = Trace::min(closest, trace);
        }
    } else {
        size_t node_idx_list[2] = {
            node.l,
            node.r,
        };
        Vec2 times_list[2] = {
            Vec2(-FLT_MAX, FLT_MAX),
            Vec2(-FLT_MAX, FLT_MAX),
        };
        bool hit_list[2] = {
            nodes[node_idx_list[0]].bbox.hit(ray, times_list[0]),
            nodes[node_idx_list[1]].bbox.hit(ray, times_list[1]),
        };

        int first_idx = 0, second_idx = 1;
        if (!hit_list[first_idx] || (hit_list[second_idx] && times_list[second_idx].x < times_list[first_idx].x)) {
            std::swap(first_idx, second_idx);
        }

        if (hit_list[first_idx]) {
            find_closest_hit(ray, node_idx_list[first_idx], closest);
            if (hit_list[second_idx] && (!closest.hit || closest.distance > times_list[second_idx].x)) {
                find_closest_hit(ray, node_idx_list[second_idx], closest);
            }
        }
    }
}

template<typename Primitive> Trace BVH<Primitive>::hit(const Ray& ray) const {
    // TODO (PathTracer): Task 3
    // Implement ray - BVH intersection test. A ray intersects
    // with a BVH aggregate if and only if it intersects a primitive in
    // the BVH that is not an aggregate.

    // The starter code simply iterates through all the primitives.
    // Again, remember you can use hit() on any Primitive value.

    Trace ret;
    // for(const Primitive& prim : primitives) {
    //     Trace hit = prim.hit(ray);
    //     ret = Trace::min(ret, hit);
    // }

    Vec2 times(-FLT_MAX, FLT_MAX);
    if (!nodes[root_idx].bbox.hit(ray, times)) {
        return ret;
    }

    find_closest_hit(ray, root_idx, ret);
    return ret;
}

template<typename Primitive>
BVH<Primitive>::BVH(std::vector<Primitive>&& prims, size_t max_leaf_size) {
    build(std::move(prims), max_leaf_size);
}

template<typename Primitive> BVH<Primitive> BVH<Primitive>::copy() const {
    BVH<Primitive> ret;
    ret.nodes = nodes;
    ret.primitives = primitives;
    ret.root_idx = root_idx;
    return ret;
}

template<typename Primitive> bool BVH<Primitive>::Node::is_leaf() const {

    // A node is a leaf if l == r, since all interior nodes must have distinct children
    return l == r;
}

template<typename Primitive>
size_t BVH<Primitive>::new_node(BBox box, size_t start, size_t size, size_t l, size_t r) {
    Node n;
    n.bbox = box;
    n.start = start;
    n.size = size;
    n.l = l;
    n.r = r;
    nodes.push_back(n);
    return nodes.size() - 1;
}

template<typename Primitive> BBox BVH<Primitive>::bbox() const {
    return nodes[root_idx].bbox;
}

template<typename Primitive> std::vector<Primitive> BVH<Primitive>::destructure() {
    nodes.clear();
    return std::move(primitives);
}

template<typename Primitive> void BVH<Primitive>::clear() {
    nodes.clear();
    primitives.clear();
}

template<typename Primitive>
size_t BVH<Primitive>::visualize(GL::Lines& lines, GL::Lines& active, size_t level,
                                 const Mat4& trans) const {

    std::stack<std::pair<size_t, size_t>> tstack;
    tstack.push({root_idx, 0});
    size_t max_level = 0;

    if(nodes.empty()) return max_level;

    while(!tstack.empty()) {

        auto [idx, lvl] = tstack.top();
        max_level = std::max(max_level, lvl);
        const Node& node = nodes[idx];
        tstack.pop();

        Vec3 color = lvl == level ? Vec3(1.0f, 0.0f, 0.0f) : Vec3(1.0f);
        GL::Lines& add = lvl == level ? active : lines;

        BBox box = node.bbox;
        box.transform(trans);
        Vec3 min = box.min, max = box.max;

        auto edge = [&](Vec3 a, Vec3 b) { add.add(a, b, color); };

        edge(min, Vec3{max.x, min.y, min.z});
        edge(min, Vec3{min.x, max.y, min.z});
        edge(min, Vec3{min.x, min.y, max.z});
        edge(max, Vec3{min.x, max.y, max.z});
        edge(max, Vec3{max.x, min.y, max.z});
        edge(max, Vec3{max.x, max.y, min.z});
        edge(Vec3{min.x, max.y, min.z}, Vec3{max.x, max.y, min.z});
        edge(Vec3{min.x, max.y, min.z}, Vec3{min.x, max.y, max.z});
        edge(Vec3{min.x, min.y, max.z}, Vec3{max.x, min.y, max.z});
        edge(Vec3{min.x, min.y, max.z}, Vec3{min.x, max.y, max.z});
        edge(Vec3{max.x, min.y, min.z}, Vec3{max.x, max.y, min.z});
        edge(Vec3{max.x, min.y, min.z}, Vec3{max.x, min.y, max.z});

        if(!node.is_leaf()) {
            tstack.push({node.l, lvl + 1});
            tstack.push({node.r, lvl + 1});
        } else {
            for(size_t i = node.start; i < node.start + node.size; i++) {
                size_t c = primitives[i].visualize(lines, active, level - lvl, trans);
                max_level = std::max(c, max_level);
            }
        }
    }
    return max_level;
}

} // namespace PT
