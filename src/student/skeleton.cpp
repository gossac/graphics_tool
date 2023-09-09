
#include "../scene/skeleton.h"
#include "../util/rand.h"

#define ITERATION (100)
#define TIMESTEP_MAX (0.1)
#define TIMESTEP_MIN (0.001)

Vec3 closest_on_line_segment(Vec3 start, Vec3 end, Vec3 point) {

    // TODO(Animation): Task 3

    // Return the closest point to 'point' on the line segment from start to end
    Vec3 line = end - start;

    float length = line.norm();

    Vec3 unit = line.unit();

    float distance = dot(unit, point - start);

    if (distance < 0) {
        distance = 0;
    } else {
        if (distance > length) {
            distance = length;
        }
    }

    Vec3 closest = distance * unit + start;

    return closest;
}

Mat4 Joint::joint_to_bind() const {

    // TODO(Animation): Task 2

    // Return a matrix transforming points in the space of this joint
    // to points in skeleton space in bind position.

    // Bind position implies that all joints have pose = Vec3{0.0f}

    // You will need to traverse the joint heirarchy. This should
    // not take into account Skeleton::base_pos

    Mat4 res = Mat4::I;

    const Joint *joint = this;
    while(true) {
        if (joint->is_root()) {
            break;
        }
        joint = joint->parent;
        res = res * Mat4::translate(joint->extent);
    }

    return res;
}

Mat4 Joint::joint_to_posed() const {

    // TODO(Animation): Task 2

    // Return a matrix transforming points in the space of this joint
    // to points in skeleton space, taking into account joint poses.

    // You will need to traverse the joint heirarchy. This should
    // not take into account Skeleton::base_pos

    Mat4 res = Mat4::euler(pose);

    const Joint *joint = this;
    while(true) {
        if (joint->is_root()) {
            break;
        }
        joint = joint->parent;
        res = Mat4::euler(joint->pose) * Mat4::translate(joint->extent) * res;
    }

    return res;
}

Vec3 Skeleton::end_of(Joint* j) {

    // TODO(Animation): Task 2

    // Return the bind position of the endpoint of joint j in object space.
    // This should take into account Skeleton::base_pos.

    return base_pos + j->joint_to_bind() * j->extent;
}

Vec3 Skeleton::posed_end_of(Joint* j) {

    // TODO(Animation): Task 2

    // Return the posed position of the endpoint of joint j in object space.
    // This should take into account Skeleton::base_pos.

    return base_pos + j->joint_to_posed() * j->extent;
}

Mat4 Skeleton::joint_to_bind(const Joint* j) const {

    // TODO(Animation): Task 2

    // Return a matrix transforming points in joint j's space to object space in
    // bind position. This should take into account Skeleton::base_pos.

    return Mat4::translate(base_pos) * j->joint_to_bind();
}

Mat4 Skeleton::joint_to_posed(const Joint* j) const {

    // TODO(Animation): Task 2

    // Return a matrix transforming points in joint j's space to object space with
    // poses. This should take into account Skeleton::base_pos.

    return Mat4::translate(base_pos) * j->joint_to_posed();
}

void Skeleton::find_joints(const GL::Mesh& mesh,
                           std::unordered_map<unsigned int, std::vector<Joint*>>& map) {

    // TODO(Animation): Task 3

    // Construct a mapping from vertex indices to lists of joints in this skeleton
    // that should effect the vertex at that index. A joint should effect a vertex
    // if it is within Joint::radius distance of the bone's line segment in bind position.

    const std::vector<GL::Mesh::Vert>& verts = mesh.verts();
    (void)verts;

    // For each i in [0, verts.size()), map[i] should contain the list of joints that
    // effect vertex i. Note that i is NOT Vert::id! i is the index in verts.

    for (unsigned i = 0; i < verts.size(); i++) {
        map[i];
    }

    for_joints([&](Joint* j) {
        // What vertices does joint j effect?
        for (unsigned i = 0; i < verts.size(); i++) {
            Vec3 pos_joint = joint_to_bind(j).inverse() * verts[i].pos;
            Vec3 closest = closest_on_line_segment(Vec3(), j->extent, pos_joint);
            float distance = (pos_joint - closest).norm();
            if (distance < j->radius) {
                map[i].push_back(j);
            }
        }
    });
}

void Skeleton::skin(const GL::Mesh& input, GL::Mesh& output,
                    const std::unordered_map<unsigned int, std::vector<Joint*>>& map) {

    // TODO(Animation): Task 3

    // Apply bone poses & weights to the vertices of the input (bind position) mesh
    // and store the result in the output mesh. See the task description for details.
    // map was computed by find_joints, hence gives a mapping from vertex index to
    // the list of bones the vertex should be effected by.

    // Currently, this just copies the input to the output without modification.

    std::vector<GL::Mesh::Vert> verts = input.verts();
    for(size_t i = 0; i < verts.size(); i++) {

        // Skin vertex i. Note that its position is given in object bind space.
        const std::vector<Joint *> &joints = map.at(i);

        if (!joints.empty()) {
            Vec3 new_pos;
            Vec3 new_norm;
            float total_weight = 0;

            for (size_t j = 0; j < joints.size(); j++) {
                Mat4 mat_bind_to_joint = joint_to_bind(joints[j]).inverse();
                Mat4 mat_joint_to_posed = joint_to_posed(joints[j]);

                Vec3 pos_joint = mat_bind_to_joint * verts[i].pos;
                Vec3 pos_pose = mat_joint_to_posed * pos_joint;

                Vec3 norm_pose = verts[i].norm;
                Joint *current = joints[j];
                while (true) {
                    norm_pose = Mat4::euler(current->pose) * norm_pose;

                    if (current->is_root()) {
                        break;
                    }
                    current = current->parent;
                }
                norm_pose.normalize();

                // Closest point in joint space.
                Vec3 closest = closest_on_line_segment(Vec3(), joints[j]->extent, pos_joint);

                float distance = (pos_joint - closest).norm();
                if (distance == 0) {
                    distance = EPS_F;
                }

                float weight = 1 / distance;

                new_pos += weight * pos_pose;
                new_norm += weight * norm_pose;
                total_weight += weight;
            }

            verts[i].pos = new_pos / total_weight;
            verts[i].norm = (new_norm / total_weight).unit();
        }
    }

    std::vector<GL::Mesh::Index> idxs = input.indices();
    output.recreate(std::move(verts), std::move(idxs));
}

void Joint::compute_gradient(Vec3 target, Vec3 current) {

    // TODO(Animation): Task 2

    // Computes the gradient of IK energy for this joint and, should be called
    // recursively upward in the heirarchy. Each call should storing the result
    // in the angle_gradient for this joint.

    // Target is the position of the IK handle in skeleton space.
    // Current is the end position of the IK'd joint in skeleton space.

    // Precompute (p_theta - q).
    Vec3 factor = current - target;

    Mat4 to_skeleton = joint_to_posed();

    // Axes of rotation.
    Vec3 r1 = to_skeleton * Vec3(0, 0, 1);
    Vec3 r2 = to_skeleton * Vec3(0, 1, 0);
    Vec3 r3 = to_skeleton * Vec3(1, 0, 0);

    // Base of the joint.
    Vec3 base = to_skeleton * Vec3();

    Vec3 p = current - base;

    Vec3 j1 = cross(r1, p);
    Vec3 j2 = cross(r2, p);
    Vec3 j3 = cross(r3, p);

    angle_gradient[0] = dot(j1, factor);
    angle_gradient[1] = dot(j2, factor);
    angle_gradient[2] = dot(j3, factor);

    if (!is_root()) {
        parent->compute_gradient(target, current);
    }
}

void Skeleton::step_ik(std::vector<IK_Handle*> active_handles) {

    // TODO(Animation): Task 2

    // Do several iterations of Jacobian Transpose gradient descent for IK

    for (std::vector<IK_Handle*>::iterator i = active_handles.begin(); i != active_handles.end(); i++) {
        IK_Handle *ik_handle = *i;
        Vec3 target = ik_handle->target;

        for (int j = 0; j < ITERATION; j++) {
            Joint *joint = ik_handle->joint;

            Vec3 current = joint->joint_to_posed() * joint->extent;
            joint->compute_gradient(target, current);

            // Random timstep per loop.
            float timestep = RNG::unit() * (TIMESTEP_MAX - TIMESTEP_MIN) + TIMESTEP_MIN;

            while (true) {
                joint->pose -= timestep * joint->angle_gradient;

                if (joint->is_root()) {
                    break;
                }
                joint = joint->parent;
            }
        }
    }
}
