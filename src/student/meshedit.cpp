
#include <queue>
#include <set>
#include <unordered_map>

#include "../geometry/halfedge.h"
#include "debug.h"

/* Note on local operation return types:

    The local operations all return a std::optional<T> type. This is used so that your
    implementation can signify that it does not want to perform the operation for
    whatever reason (e.g. you don't want to allow the user to erase the last vertex).

    An optional can have two values: std::nullopt, or a value of the type it is
    parameterized on. In this way, it's similar to a pointer, but has two advantages:
    the value it holds need not be allocated elsewhere, and it provides an API that
    forces the user to check if it is null before using the value.

    In your implementation, if you have successfully performed the operation, you can
    simply return the required reference:

            ... collapse the edge ...
            return collapsed_vertex_ref;

    And if you wish to deny the operation, you can return the null optional:

            return std::nullopt;

    Note that the stubs below all reject their duties by returning the null optional.
*/

/*
    This method should replace the given vertex and all its neighboring
    edges and faces with a single face, returning the new face.
 */
std::optional<Halfedge_Mesh::FaceRef> Halfedge_Mesh::erase_vertex(Halfedge_Mesh::VertexRef v) {

    (void)v;
    return std::nullopt;
}

/*
    This method should erase the given edge and return an iterator to the
    merged face.
 */
std::optional<Halfedge_Mesh::FaceRef> Halfedge_Mesh::erase_edge(Halfedge_Mesh::EdgeRef e) {

    (void)e;
    return std::nullopt;
}

/*
    This method should collapse the given edge and return an iterator to
    the new vertex created by the collapse.
*/
std::optional<Halfedge_Mesh::VertexRef> Halfedge_Mesh::collapse_edge(Halfedge_Mesh::EdgeRef e) {
    // before

    std::vector<HalfedgeRef> hu;
    hu.push_back(e->halfedge());
    while (true) {
        HalfedgeRef hi = hu.back()->twin()->next();
        if (hi == hu[0]) {
            break;
        }
        hu.push_back(hi);
    }
    int m = hu.size();
    for (int i = 1; i <= m - 1; i++) {
        hu.push_back(hu[i]->twin());
    }

    std::vector<HalfedgeRef> hd;
    hd.push_back(hu[0]->twin());
    while (true) {
        HalfedgeRef hi = hd.back()->twin()->next();
        if (hi == hd[0]) {
            break;
        }
        hd.push_back(hi);
    }
    int n = hd.size();
    for (int i = 1; i <= n - 1; i++) {
        hd.push_back(hd[i]->twin());
    }

    std::vector<VertexRef> vu;
    vu.push_back(hu[0]->vertex());
    for (int i = m; i <= 2 * m - 2; i++) {
        vu.push_back(hu[i]->vertex());
    }
    std::vector<VertexRef> vd;
    vd.push_back(hd[0]->vertex());
    for (int i = n; i <= 2 * n - 2; i++) {
        vd.push_back(hd[i]->vertex());
    }

    std::vector<EdgeRef> eu;
    for (int i = 1; i <= m - 1; i++) {
        eu.push_back(hu[i]->edge());
    }
    std::vector<EdgeRef> ed;
    for (int i = 1; i <= n - 1; i++) {
        ed.push_back(hd[i]->edge());
    }

    std::vector<FaceRef> fu;
    for (int i = 1; i <= m - 1; i++) {
        fu.push_back(hu[i]->face());
    }
    std::vector<FaceRef> fd;
    for (int i = 1; i <= n - 1; i++) {
        fd.push_back(hd[i]->face());
    }

    bool left_triangle = m >= 1 && hu[1]->next()->next()->vertex() == vd[0];
    bool right_triangle = n >= 1 && hd[1]->next()->next()->vertex() == vu[0];

    // after

    std::vector<HalfedgeRef> new_hu;
    for (int i = 1; i <= m - 1; i++) {
        new_hu.push_back(new_halfedge());
    }
    for (int i = 1; i <= m - 1; i++) {
        new_hu.push_back(new_halfedge());
    }

    std::vector<HalfedgeRef> new_hd;
    if (right_triangle) {
        new_hd.push_back(new_hu[m - 2]);
    } else {
        new_hd.push_back(new_halfedge());
    }
    for (int i = 2; i <= n - 2; i++) {
        new_hd.push_back(new_halfedge());
    }
    if (left_triangle) {
        new_hd.push_back(new_hu[0]);
    } else {
        new_hd.push_back(new_halfedge());
    }
    if (right_triangle) {
        new_hd.push_back(new_hu[2 * m - 3]);
    } else {
        new_hd.push_back(new_halfedge());
    }
    for (int i = 2; i <= n - 2; i++) {
        new_hd.push_back(new_halfedge());
    }
    if (left_triangle) {
        new_hd.push_back(new_hu[m - 1]);
    } else {
        new_hd.push_back(new_halfedge());
    }

    VertexRef new_v = new_vertex();

    std::vector<EdgeRef> new_eu;
    for (int i = 1; i <= m - 1; i++) {
        new_eu.push_back(new_edge());
    }
    std::vector<EdgeRef> new_ed;
    if (right_triangle) {
        new_ed.push_back(new_eu[m - 2]);
    } else {
        new_ed.push_back(new_edge());
    }
    for (int i = 2; i <= n - 2; i++) {
        new_ed.push_back(new_edge());
    }
    if (left_triangle) {
        new_ed.push_back(new_eu[0]);
    } else {
        new_ed.push_back(new_edge());
    }

    std::vector<FaceRef> new_fu;
    FaceRef dummy_f;
    if (left_triangle) {
        new_fu.push_back(dummy_f);
    } else {
        new_fu.push_back(new_face());
    }
    for (int i = 2; i <= m - 1; i++) {
        new_fu.push_back(new_face());
    }

    std::vector<FaceRef> new_fd;
    if (right_triangle) {
        new_fd.push_back(dummy_f);
    } else {
        new_fd.push_back(new_face());
    }
    for (int i = 2; i <= n - 1; i++) {
        new_fd.push_back(new_face());
    }

    for (int i = 1; i <= m - 2; i++) {
        new_hu[i + m - 2]->set_neighbors(new_hu[i], new_hu[i - 1], vu[i], new_eu[i - 1], new_fu[i]);
        HalfedgeRef inside_h = hu[i + 1]->next();
        while (true) {
            inside_h->face() = new_fu[i];
            HalfedgeRef next_h = inside_h->next();
            if (next_h == hu[i + m - 1]) {
                break;
            }
            inside_h = next_h;
        }
        inside_h->next() = new_hu[i + m - 2];
        new_hu[i]->set_neighbors(hu[i + 1]->next(), new_hu[i + m - 1], new_v, new_eu[i], new_fu[i]);
    }
    for (int i = 1; i <= n - 2; i++) {
        new_hd[i + n - 2]->set_neighbors(new_hd[i], new_hd[i - 1], vd[i], new_ed[i - 1], new_fd[i]);
        HalfedgeRef inside_h = hd[i + 1]->next();
        while (true) {
            inside_h->face() = new_fd[i];
            HalfedgeRef next_h = inside_h->next();
            if (next_h == hd[i + n - 1]) {
                break;
            }
            inside_h = next_h;
        }
        inside_h->next() = new_hd[i + n - 2];
        new_hd[i]->set_neighbors(hd[i + 1]->next(), new_hd[i + n - 1], new_v, new_ed[i], new_fd[i]);
    }

    if (!left_triangle) {
        new_hd[2 * n - 3]->set_neighbors(new_hu[0], new_hd[n - 2], vd[n - 1], new_ed[n - 2], new_fu[0]);
        HalfedgeRef inside_h = hu[1]->next();
        while (true) {
            inside_h->face() = new_fu[0];
            HalfedgeRef next_h = inside_h->next();
            if (next_h == hd[2 * n - 2]) {
                break;
            }
            inside_h = next_h;
        }
        inside_h->next() = new_hd[2 * n - 3];
        new_hu[0]->set_neighbors(hu[1]->next(), new_hu[m - 1], new_v, new_eu[0], new_fu[0]);
    }
    if (!right_triangle) {
        new_hu[2 * m - 3]->set_neighbors(new_hd[0], new_hu[m - 2], vu[m - 1], new_eu[m - 2], new_fd[0]);
        HalfedgeRef inside_h = hd[1]->next();
        while (true) {
            inside_h->face() = new_fd[0];
            HalfedgeRef next_h = inside_h->next();
            if (next_h == hu[2 * m - 2]) {
                break;
            }
            inside_h = next_h;
        }
        inside_h->next() = new_hu[2 * m - 3];
        new_hd[0]->set_neighbors(hd[1]->next(), new_hd[n - 1], new_v, new_ed[0], new_fd[0]);
    }
    for (std::vector<HalfedgeRef>::iterator i = hu.begin(); i != hu.end(); i++) {
        erase(*i);
    }
    for (std::vector<HalfedgeRef>::iterator i = hd.begin(); i != hd.end(); i++) {
        erase(*i);
    }

    new_v->halfedge() = new_hu[0];
    for (int i = 1; i <= m - 1; i++) {
        vu[i]->halfedge() = new_hu[i + m - 2];
    }

    for (int i = 1; i <= n - 1; i++) {
        vd[i]->halfedge() = new_hd[i + n - 2];
    }
    erase(vu[0]);
    erase(vd[0]);

    for (int i = 1; i <= m - 1; i++) {
        new_eu[i - 1]->halfedge() = new_hu[i - 1];
    }
    for (int i = 1; i <= n - 1; i++) {
        new_ed[i - 1]->halfedge() = new_hd[i - 1];
    }
    for (std::vector<EdgeRef>::iterator i = eu.begin(); i != eu.end(); i++) {
        erase(*i);
    }
    for (std::vector<EdgeRef>::iterator i = ed.begin(); i != ed.end(); i++) {
        erase(*i);
    }
    erase(e);

    if (!left_triangle) {
        new_fu[0]->halfedge() = new_hu[0];
    }
    for (int i = 1; i <= m - 2; i++) {
        new_fu[i]->halfedge() = new_hu[i];
    }
    if (!right_triangle) {
        new_fd[0]->halfedge() = new_hd[0];
    }
    for (int i = 1; i <= n - 2; i++) {
        new_fd[i]->halfedge() = new_hd[i];
    }
    for (int i = 0; i <= m - 2; i++) {
        erase(fu[i]);
    }
    for (int i = 0; i <= n - 2; i++) {
        erase(fd[i]);
    }

    return new_v;
}

/*
    This method should collapse the given face and return an iterator to
    the new vertex created by the collapse.
*/
std::optional<Halfedge_Mesh::VertexRef> Halfedge_Mesh::collapse_face(Halfedge_Mesh::FaceRef f) {

    (void)f;
    return std::nullopt;
}

/*
    This method should flip the given edge and return an iterator to the
    flipped edge.
*/
std::optional<Halfedge_Mesh::EdgeRef> Halfedge_Mesh::flip_edge(Halfedge_Mesh::EdgeRef e) {
    if (e->on_boundary()) {
        return std::nullopt;
    }

    // before

    HalfedgeRef h0 = e->halfedge();
    HalfedgeRef h1 = h0->next();
    HalfedgeRef h2 = h1->next();
    HalfedgeRef h3 = h2;
    while (true) {
        HalfedgeRef next_h = h3->next();
        if (next_h == h0) {
            break;
        }
        h3 = next_h;
    }
    HalfedgeRef h4 = h0->twin();
    HalfedgeRef h5 = h4->next();
    HalfedgeRef h6 = h5->next();
    HalfedgeRef h7 = h6;
    while (true) {
        HalfedgeRef next_h = h7->next();
        if (next_h == h4) {
            break;
        }
        h7 = next_h;
    }

    // outside
    HalfedgeRef h8 = h1->twin();
    // HalfedgeRef h9 = h2->twin();
    HalfedgeRef h10 = h3->twin();
    HalfedgeRef h11 = h5->twin();
    // HalfedgeRef h12 = h6->twin();
    HalfedgeRef h13 = h7->twin();

    VertexRef v0 = h0->vertex();
    VertexRef v1 = h1->vertex();
    VertexRef v2 = h2->vertex();
    VertexRef v3 = h3->vertex();
    VertexRef v4 = h6->vertex();
    VertexRef v5 = h7->vertex();

    EdgeRef e0 = h0->edge();
    EdgeRef e1 = h1->edge();
    EdgeRef e2 = h2->edge();
    EdgeRef e3 = h3->edge();
    EdgeRef e4 = h5->edge();
    EdgeRef e5 = h6->edge();
    EdgeRef e6 = h7->edge();

    FaceRef f0 = h0->face();
    FaceRef f1 = h4->face();

    // after

    h0->next() = h2;
    h0->twin() = h4;
    h0->vertex() = v4;
    h0->edge() = e0;
    h0->face() = f0;

    h1->next() = h4;
    h1->twin() = h8;
    h1->vertex() = v1;
    h1->edge() = e1;
    h1->face() = f1;

    // h2 remains the same.

    h3->next() = h5;
    h3->twin() = h10;
    h3->vertex() = v3;
    h3->edge() = e3;
    h3->face() = f0;

    h4->next() = h6;
    h4->twin() = h0;
    h4->vertex() = v2;
    h4->edge() = e0;
    h4->face() = f1;

    h5->next() = h0;
    h5->twin() = h11;
    h5->vertex() = v0;
    h5->edge() = e4;
    h5->face() = f0;

    // h6 remains the same.

    h7->next() = h1;
    h7->twin() = h13;
    h7->vertex() = v5;
    h7->edge() = e6;
    h7->face() = f1;

    // h8 remains the same.

    // h9 remains the same.

    // h10 remains the same.

    // h11 remains the same.

    // h12 remains the same.

    // h13 remains the same.

    v0->halfedge() = h5;
    v1->halfedge() = h1;
    v2->halfedge() = h2;
    v3->halfedge() = h3;
    v4->halfedge() = h6;
    v5->halfedge() = h7;

    e0->halfedge() = h0;
    e1->halfedge() = h1;
    e2->halfedge() = h2;
    e3->halfedge() = h3;
    e4->halfedge() = h5;
    e5->halfedge() = h6;
    e6->halfedge() = h7;

    f0->halfedge() = h0;
    f1->halfedge() = h4;

    return e0;
}

/*
    This method should split the given edge and return an iterator to the
    newly inserted vertex. The halfedge of this vertex should point along
    the edge that was split, rather than the new edges.
*/
std::optional<Halfedge_Mesh::VertexRef> Halfedge_Mesh::split_edge(Halfedge_Mesh::EdgeRef e) {
    if (e->on_boundary()) {
        return std::nullopt;
    }

    // before

    std::vector<HalfedgeRef> hr;
    hr.push_back(e->halfedge());
    while (true) {
        HalfedgeRef hi = hr.back()->next();
        if (hi == hr[0]) {
            break;
        }
        hr.push_back(hi);
    }
    int m = hr.size();
    for (int i = 1; i < m; i++) {
        hr.push_back(hr[i]->twin());
    }

    std::vector<HalfedgeRef> hl;
    hl.push_back(hr[0]->twin());
    while (true) {
        HalfedgeRef hi = hl.back()->next();
        if (hi == hl[0]) {
            break;
        }
        hl.push_back(hi);
    }
    int n = hl.size();
    for (int i = 1; i < n; i++) {
        hl.push_back(hl[i]->twin());
    }

    std::vector<VertexRef> vr;
    for (int i = 1; i < m; i++) {
        vr.push_back(hr[i]->vertex());
    }
    std::vector<VertexRef> vl;
    for (int i = 1; i < n; i++) {
        vl.push_back(hl[i]->vertex());
    }

    std::vector<EdgeRef> er;
    for (int i = 1; i < m; i++) {
        er.push_back(hr[i]->edge());
    }
    std::vector<EdgeRef> el;
    for (int i = 1; i < n; i++) {
        el.push_back(hl[i]->edge());
    }

    FaceRef fr = hr[0]->face();
    FaceRef fl = hl[0]->face();

    // after

    std::vector<HalfedgeRef> new_hr;
    for (int i = 0; i <= m - 2; i++) {
        new_hr.push_back(new_halfedge());
    }
    for (int i = 1; i <= m - 2; i++) {
        new_hr.push_back(new_halfedge());
    }

    std::vector<HalfedgeRef> new_hl;
    for (int i = 1; i <= n - 1; i++) {
        new_hl.push_back(new_halfedge());
    }
    for (int i = 1; i <= n - 2; i++) {
        new_hl.push_back(new_halfedge());
    }

    VertexRef new_v = new_vertex();

    std::vector<EdgeRef> new_er;
    for (int i = 0; i <= m - 2; i++) {
        new_er.push_back(new_edge());
    }
    std::vector<EdgeRef> new_el;
    for (int i = 1; i <= n - 2; i++) {
        new_el.push_back(new_edge());
    }

    std::vector<FaceRef> new_fr;
    for (int i = 0; i < m - 2; i++) {
        new_fr.push_back(new_face());
    }
    std::vector<FaceRef> new_fl;
    for (int i = 1; i <= n - 2; i++) {
        new_fl.push_back(new_face());
    }

    hr[0]->next() = new_hr[m - 2];
    for (int i = 1; i < m - 1; i++) {
        hr[i]->next() = new_hr[m - 2 + i];
        hr[i]->face() = new_fr[i - 1];
    }
    hl[0]->vertex() = new_v;
    hl[1]->next() = new_hl[0];
    for (int i = 2; i <= n - 1; i++) {
        hl[i]->next() = new_hl[i - 1];
        hl[i]->face() = new_fl[i - 2];
    }
    new_hr[0]->set_neighbors(hr[1], new_hl[n - 2], new_v, new_er[0], new_fr[0]);
    for (int i = 1; i < m - 2; i++) {
        new_hr[i]->set_neighbors(hr[i + 1], new_hr[m - 2 + i], new_v, new_er[i], new_fr[i]);
    }
    new_hr[m - 2]->set_neighbors(hr[m - 1], new_hr[2 * m - 4], new_v, new_er[m - 2], fr);
    for (int i = m - 1; i <= 2 * m - 4; i++) {
        new_hr[i]->set_neighbors(new_hr[i - m + 1], new_hr[i - m + 2], vr[i - m + 2], new_er[i - m + 2], new_fr[i - m + 1]);
    }

    new_hl[0]->set_neighbors(hl[0], new_hl[n - 1], vl[1], new_el[0], fl);
    for (int i = 1; i < n - 2; i++) {
        new_hl[i]->set_neighbors(new_hl[i + n - 2], new_hl[i + n - 1], vl[i + 1], new_el[i], new_fl[i - 1]);
    }
    new_hl[n - 2]->set_neighbors(new_hl[2 * n - 4], new_hr[0], vr[0], new_er[0], new_fl[n - 3]);
    for (int i = n - 1; i <= 2 * n - 4; i++) {
        new_hl[i]->set_neighbors(hl[i - n + 3], new_hl[i - n + 1], new_v, new_el[i - n + 1], new_fl[i - n + 1]);
    }

    vr[0]->halfedge() = hr[1];
    new_v->halfedge() = hl[0];

    for (int i = 0; i <= m - 2; i++) {
        new_er[i]->halfedge() = new_hr[i];
    }
    for (int i = 0; i <= n - 3; i++) {
        new_el[i]->halfedge() = new_hl[i];
    }

    fr->halfedge() = hr[0];
    fl->halfedge() = hl[0];
    for (int i = 0; i <= m - 3; i++) {
        new_fr[i]->halfedge() = hr[i + 1];
    }
    for (int i = 0; i <= n - 3; i++) {
        new_fl[i]->halfedge() = hl[i + 2];
    }

    return new_v;
}

/* Note on the beveling process:

    Each of the bevel_vertex, bevel_edge, and bevel_face functions do not represent
    a full bevel operation. Instead, they should update the _connectivity_ of
    the mesh, _not_ the positions of newly created vertices. In fact, you should set
    the positions of new vertices to be exactly the same as wherever they "started from."

    When you click on a mesh element while in bevel mode, one of those three functions
    is called. But, because you may then adjust the distance/offset of the newly
    beveled face, we need another method of updating the positions of the new vertices.

    This is where bevel_vertex_positions, bevel_edge_positions, and
    bevel_face_positions come in: these functions are called repeatedly as you
    move your mouse, the position of which determins the normal and tangent offset
    parameters. These functions are also passed an array of the original vertex
    positions: for  bevel_vertex, it has one element, the original vertex position,
    for bevel_edge,  two for the two vertices, and for bevel_face, it has the original
    position of each vertex in halfedge order. You should use these positions, as well
    as the normal and tangent offset fields to assign positions to the new vertices.

    Finally, note that the normal and tangent offsets are not relative values - you
    should compute a particular new position from them, not a delta to apply.
*/

/*
    This method should replace the vertex v with a face, corresponding to
    a bevel operation. It should return the new face.  NOTE: This method is
    responsible for updating the *connectivity* of the mesh only---it does not
    need to update the vertex positions.  These positions will be updated in
    Halfedge_Mesh::bevel_vertex_positions (which you also have to
    implement!)
*/
std::optional<Halfedge_Mesh::FaceRef> Halfedge_Mesh::bevel_vertex(Halfedge_Mesh::VertexRef v) {

    // Reminder: You should set the positions of new vertices (v->pos) to be exactly
    // the same as wherever they "started from."

    (void)v;
    return std::nullopt;
}

/*
    This method should replace the edge e with a face, corresponding to a
    bevel operation. It should return the new face. NOTE: This method is
    responsible for updating the *connectivity* of the mesh only---it does not
    need to update the vertex positions.  These positions will be updated in
    Halfedge_Mesh::bevel_edge_positions (which you also have to
    implement!)
*/
std::optional<Halfedge_Mesh::FaceRef> Halfedge_Mesh::bevel_edge(Halfedge_Mesh::EdgeRef e) {

    // Reminder: You should set the positions of new vertices (v->pos) to be exactly
    // the same as wherever they "started from."

    (void)e;
    return std::nullopt;
}

/*
    This method should replace the face f with an additional, inset face
    (and ring of faces around it), corresponding to a bevel operation. It
    should return the new face.  NOTE: This method is responsible for updating
    the *connectivity* of the mesh only---it does not need to update the vertex
    positions. These positions will be updated in
    Halfedge_Mesh::bevel_face_positions (which you also have to
    implement!)
*/
std::optional<Halfedge_Mesh::FaceRef> Halfedge_Mesh::bevel_face(Halfedge_Mesh::FaceRef f) {

    // Reminder: You should set the positions of new vertices (v->pos) to be exactly
    // the same as wherever they "started from."

    // before

    std::vector<HalfedgeRef> h;
    h.push_back(f->halfedge());
    while (true) {
        HalfedgeRef next_h = h.back()->next();
        if (next_h == h[0]) {
            break;
        }
        h.push_back(next_h);
    }
    int n = h.size();

    std::vector<VertexRef> v;
    for (std::vector<HalfedgeRef>::iterator i = h.begin(); i != h.end(); i++) {
        v.push_back((*i)->vertex());
    }

    std::vector<EdgeRef> e;
    for (std::vector<HalfedgeRef>::iterator i = h.begin(); i != h.end(); i++) {
        e.push_back((*i)->edge());
    }

    // after

    std::vector<HalfedgeRef> new_h;
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < n; j++) {
            new_h.push_back(new_halfedge());
        }
    }

    std::vector<VertexRef> new_v;
    for (int i = 0; i < n; i++) {
        new_v.push_back(new_vertex());
    }

    std::vector<EdgeRef> new_e;
    for (int i = 0; i < 2; i++) {
        for (int j = 0; j < n; j++) {
            new_e.push_back(new_edge());
        }
    }

    std::vector<FaceRef> new_f;
    new_f.push_back(new_face());
    for (int i = 0; i < n; i++) {
        new_f.push_back(new_face());
    }

    for (int i = 0; i < n; i++) {
        int next_i = (i + 1) % n;
        int previous_i = (i + n - 1) % n;

        h[i]->next() = new_h[i];
        h[i]->face() = new_f[i + 1];
        new_h[i]->set_neighbors(new_h[i + n], new_h[next_i + 2 * n], v[next_i], new_e[i], new_f[i + 1]);
        new_h[i + n]->set_neighbors(new_h[i + 2 * n], new_h[i + 3 * n], new_v[next_i], new_e[i + n], new_f[i + 1]);
        new_h[i + 2 * n]->set_neighbors(h[i], new_h[previous_i], new_v[i], new_e[previous_i], new_f[i + 1]);
        new_h[i + 3 * n]->set_neighbors(new_h[next_i + 3 * n], new_h[i + n], new_v[i], new_e[i + n], new_f[0]);

        new_v[i]->halfedge() = new_h[i + 3 * n];
        new_v[i]->pos = v[i]->pos;

        new_e[i]->halfedge() = new_h[i];
        new_e[i + n]->halfedge() = new_h[i + n];

        new_f[i + 1]->halfedge() = h[i];
    }
    new_f[0]->halfedge() = new_h[3 * n];
    erase(f);

    return new_f[0];
}

/*
    Compute new vertex positions for the vertices of the beveled vertex.

    These vertices can be accessed via new_halfedges[i]->vertex()->pos for
    i = 1, ..., new_halfedges.size()-1.

    The basic strategy here is to loop over the list of outgoing halfedges,
    and use the original vertex position and its associated outgoing edge
    to compute a new vertex position along the outgoing edge.
*/
void Halfedge_Mesh::bevel_vertex_positions(const std::vector<Vec3>& start_positions,
                                           Halfedge_Mesh::FaceRef face, float tangent_offset) {

    std::vector<HalfedgeRef> new_halfedges;
    auto h = face->halfedge();
    do {
        new_halfedges.push_back(h);
        h = h->next();
    } while(h != face->halfedge());

    (void)new_halfedges;
    (void)start_positions;
    (void)face;
    (void)tangent_offset;
}

/*
    Compute new vertex positions for the vertices of the beveled edge.

    These vertices can be accessed via new_halfedges[i]->vertex()->pos for
    i = 1, ..., new_halfedges.size()-1.

    The basic strategy here is to loop over the list of outgoing halfedges,
    and use the preceding and next vertex position from the original mesh
    (in the orig array) to compute an offset vertex position.

    Note that there is a 1-to-1 correspondence between halfedges in
    newHalfedges and vertex positions
    in orig.  So, you can write loops of the form

    for(size_t i = 0; i < new_halfedges.size(); i++)
    {
            Vector3D pi = start_positions[i]; // get the original vertex
            position corresponding to vertex i
    }
*/
void Halfedge_Mesh::bevel_edge_positions(const std::vector<Vec3>& start_positions,
                                         Halfedge_Mesh::FaceRef face, float tangent_offset) {

    std::vector<HalfedgeRef> new_halfedges;
    auto h = face->halfedge();
    do {
        new_halfedges.push_back(h);
        h = h->next();
    } while(h != face->halfedge());

    (void)new_halfedges;
    (void)start_positions;
    (void)face;
    (void)tangent_offset;
}

/*
    Compute new vertex positions for the vertices of the beveled face.

    These vertices can be accessed via new_halfedges[i]->vertex()->pos for
    i = 1, ..., new_halfedges.size()-1.

    The basic strategy here is to loop over the list of outgoing halfedges,
    and use the preceding and next vertex position from the original mesh
    (in the start_positions array) to compute an offset vertex
    position.

    Note that there is a 1-to-1 correspondence between halfedges in
    new_halfedges and vertex positions
    in orig. So, you can write loops of the form

    for(size_t i = 0; i < new_halfedges.size(); i++)
    {
            Vec3 pi = start_positions[i]; // get the original vertex
            position corresponding to vertex i
    }
*/
void Halfedge_Mesh::bevel_face_positions(const std::vector<Vec3>& start_positions,
                                         Halfedge_Mesh::FaceRef face, float tangent_offset,
                                         float normal_offset) {

    if(flip_orientation) normal_offset = -normal_offset;
    std::vector<HalfedgeRef> new_halfedges;
    auto h = face->halfedge();
    do {
        new_halfedges.push_back(h);
        h = h->next();
    } while(h != face->halfedge());

    float n = start_positions.size();

    Vec3 center(0, 0, 0);
    for (std::vector<Vec3>::const_iterator i = start_positions.begin(); i != start_positions.end(); i++) {
        center += *i;
    }
    center /= n;

    for (int i = 0; i < n; i++) {
        new_halfedges[i]->vertex()->pos = (start_positions[i] - center) * (1 + tangent_offset) + center + face->normal() * normal_offset;
    }
}

/*
    Splits all non-triangular faces into triangles.
*/
void Halfedge_Mesh::triangulate() {

    // For each face...
    for (FaceRef f = faces_begin(); f != faces_end(); f++) {
        // before

        std::vector<HalfedgeRef> h;
        h.push_back(f->halfedge());
        while (true) {
            HalfedgeRef next_h = h.back()->next();
            if (next_h == h[0]) {
                break;
            }
            h.push_back(next_h);
        }
        int n = h.size();

        if (n <= 3) {
            continue;
        }

        std::vector<VertexRef> v;
        for (std::vector<HalfedgeRef>::iterator i = h.begin(); i != h.end(); i++) {
            v.push_back((*i)->vertex());
        }

        std::vector<EdgeRef> e;
        for (std::vector<HalfedgeRef>::iterator i = h.begin(); i != h.end(); i++) {
            e.push_back((*i)->edge());
        }

        // after

        std::vector<HalfedgeRef> new_h;
        for (int i = 0; i <= 2 * n - 7; i++) {
            new_h.push_back(new_halfedge());
        }

        std::vector<EdgeRef> new_e;
        for (int i = 0; i <= n - 4; i++) {
            new_e.push_back(new_edge());
        }

        std::vector<FaceRef> new_f;
        // only create (n - 3) new faces, reuse f as the (n-2)th face
        for (int i = 0; i <= n - 4; i++) {
            new_f.push_back(new_face());
        }

        h[0]->face() = new_f[0];
        h[1]->next() = new_h[0];
        h[1]->face() = new_f[0];
        new_h[0]->set_neighbors(h[0], new_h[n - 3], v[2], new_e[0], new_f[0]);
        h[n - 1]->next() = new_h[2 * n - 7];
        new_h[2 * n - 7]->set_neighbors(h[n - 2], new_h[n - 4], v[0], new_e[n - 4], f);
        for (int i = 3; i <= n - 2; i++) {
            new_h[i - 2]->set_neighbors(new_h[i + n - 6], new_h[i + n - 5], v[i], new_e[i - 2], new_f[i - 2]);
            new_h[i + n - 6]->set_neighbors(h[i - 1], new_h[i - 3], v[0], new_e[i - 3], new_f[i - 2]);
            h[i - 1]->next() = new_h[i - 2];
            h[i - 1]->face() = new_f[i - 2];
        }

        for (int i = 0; i <= n - 4; i++) {
            new_e[i]->halfedge() = new_h[i];
        }

        for (int i = 0; i <= n - 4; i++) {
            new_f[i]->halfedge() = new_h[i];
        }
        f->halfedge() = h[n - 1];
    }
}

/* Note on the quad subdivision process:

        Unlike the local mesh operations (like bevel or edge flip), we will perform
        subdivision by splitting *all* faces into quads "simultaneously."  Rather
        than operating directly on the halfedge data structure (which as you've
        seen is quite difficult to maintain!) we are going to do something a bit nicer:
           1. Create a raw list of vertex positions and faces (rather than a full-
              blown halfedge mesh).
           2. Build a new halfedge mesh from these lists, replacing the old one.
        Sometimes rebuilding a data structure from scratch is simpler (and even
        more efficient) than incrementally modifying the existing one.  These steps are
        detailed below.

  Step I: Compute the vertex positions for the subdivided mesh.
        Here we're going to do something a little bit strange: since we will
        have one vertex in the subdivided mesh for each vertex, edge, and face in
        the original mesh, we can nicely store the new vertex *positions* as
        attributes on vertices, edges, and faces of the original mesh. These positions
        can then be conveniently copied into the new, subdivided mesh.
        This is what you will implement in linear_subdivide_positions() and
        catmullclark_subdivide_positions().

  Steps II-IV are provided (see Halfedge_Mesh::subdivide()), but are still detailed
  here:

  Step II: Assign a unique index (starting at 0) to each vertex, edge, and
        face in the original mesh. These indices will be the indices of the
        vertices in the new (subdivided mesh).  They do not have to be assigned
        in any particular order, so long as no index is shared by more than one
        mesh element, and the total number of indices is equal to V+E+F, i.e.,
        the total number of vertices plus edges plus faces in the original mesh.
        Basically we just need a one-to-one mapping between original mesh elements
        and subdivided mesh vertices.

  Step III: Build a list of quads in the new (subdivided) mesh, as tuples of
        the element indices defined above. In other words, each new quad should be
        of the form (i,j,k,l), where i,j,k and l are four of the indices stored on
        our original mesh elements.  Note that it is essential to get the orientation
        right here: (i,j,k,l) is not the same as (l,k,j,i).  Indices of new faces
        should circulate in the same direction as old faces (think about the right-hand
        rule).

  Step IV: Pass the list of vertices and quads to a routine that clears
        the internal data for this halfedge mesh, and builds new halfedge data from
        scratch, using the two lists.
*/

/*
    Compute new vertex positions for a mesh that splits each polygon
    into quads (by inserting a vertex at the face midpoint and each
    of the edge midpoints).  The new vertex positions will be stored
    in the members Vertex::new_pos, Edge::new_pos, and
    Face::new_pos.  The values of the positions are based on
    simple linear interpolation, e.g., the edge midpoints and face
    centroids.
*/
void Halfedge_Mesh::linear_subdivide_positions() {

    // For each vertex, assign Vertex::new_pos to
    // its original position, Vertex::pos.
    for (VertexRef v = vertices_begin(); v != vertices_end(); v++) {
        v->new_pos = v->pos;
    }

    // For each edge, assign the midpoint of the two original
    // positions to Edge::new_pos.
    for (EdgeRef e = edges_begin(); e != edges_end(); e++) {
        HalfedgeRef h1 = e->halfedge();
        VertexRef v1 = h1->vertex();

        HalfedgeRef h2 = h1->twin();
        VertexRef v2 = h2->vertex();

        e->new_pos = (v1->pos + v2->pos) / 2;
    }

    // For each face, assign the centroid (i.e., arithmetic mean)
    // of the original vertex positions to Face::new_pos. Note
    // that in general, NOT all faces will be triangles!
    for (FaceRef f = faces_begin(); f != faces_end(); f++) {
        int count = 0;
        HalfedgeRef h = f->halfedge();

        f->new_pos = Vec3();
        do {
            f->new_pos += h->vertex()->pos;
            h = h->next();
            count++;
        } while (h != f->halfedge());

        f->new_pos /= count;
    }
}

/*
    Compute new vertex positions for a mesh that splits each polygon
    into quads (by inserting a vertex at the face midpoint and each
    of the edge midpoints).  The new vertex positions will be stored
    in the members Vertex::new_pos, Edge::new_pos, and
    Face::new_pos.  The values of the positions are based on
    the Catmull-Clark rules for subdivision.

    Note: this will only be called on meshes without boundary
*/
void Halfedge_Mesh::catmullclark_subdivide_positions() {

    // The implementation for this routine should be
    // a lot like Halfedge_Mesh:linear_subdivide_positions:(),
    // except that the calculation of the positions themsevles is
    // slightly more involved, using the Catmull-Clark subdivision
    // rules. (These rules are outlined in the Developer Manual.)

    // Faces
    for (FaceRef f = faces_begin(); f != faces_end(); f++) {
        int count = 0;
        HalfedgeRef h = f->halfedge();

        f->new_pos = Vec3();
        do {
            f->new_pos += h->vertex()->pos;
            h = h->next();
            count++;
        } while (h != f->halfedge());

        f->new_pos /= count;
    }

    // Edges
    for (EdgeRef e = edges_begin(); e != edges_end(); e++) {
        HalfedgeRef h1 = e->halfedge();
        VertexRef v1 = h1->vertex();
        FaceRef f1 = h1->face();

        HalfedgeRef h2 = h1->twin();
        VertexRef v2 = h2->vertex();
        FaceRef f2 = h2->face();

        e->new_pos = (v1->pos + v2->pos + f1->new_pos + f2->new_pos) / 4;
    }

    // Vertices
    for (VertexRef v = vertices_begin(); v != vertices_end(); v++) {
        Vec3 face_pos_sum(0, 0, 0);
        Vec3 edge_midpoint_sum(0, 0, 0);
        int n = 0;
        HalfedgeRef h = v->halfedge();
        do {
            face_pos_sum += h->face()->new_pos;
            edge_midpoint_sum += (h->twin()->vertex()->pos + v->pos) / 2;
            n++;
            h = h->twin()->next();
        } while (h != v->halfedge());

        Vec3 q = face_pos_sum / n;
        Vec3 r = edge_midpoint_sum / n;
        v->new_pos = (q + 2 * r + (n - 3) * v->pos) / n;
    }
}

/*
        This routine should increase the number of triangles in the mesh
        using Loop subdivision. Note: this is will only be called on triangle meshes.
*/
void Halfedge_Mesh::loop_subdivide() {

    // Compute new positions for all the vertices in the input mesh, using
    // the Loop subdivision rule, and store them in Vertex::new_pos.
    // -> At this point, we also want to mark each vertex as being a vertex of the
    //    original mesh. Use Vertex::is_new for this.
    // -> Next, compute the updated vertex positions associated with edges, and
    //    store it in Edge::new_pos.
    // -> Next, we're going to split every edge in the mesh, in any order.  For
    //    future reference, we're also going to store some information about which
    //    subdivided edges come from splitting an edge in the original mesh, and
    //    which edges are new, by setting the flat Edge::is_new. Note that in this
    //    loop, we only want to iterate over edges of the original mesh.
    //    Otherwise, we'll end up splitting edges that we just split (and the
    //    loop will never end!)
    // -> Now flip any new edge that connects an old and new vertex.
    // -> Finally, copy the new vertex positions into final Vertex::pos.

    for (VertexRef v = vertices_begin(); v != vertices_end(); v++) {
        v->is_new = false;

        int n = 0;
        Vec3 neighbor_sum(0, 0, 0);
        HalfedgeRef h = v->halfedge();
        do {
            VertexRef neighbor_v = h->twin()->vertex();
            neighbor_sum += neighbor_v->pos;
            n++;
            h = h->twin()->next();
        } while (h != v->halfedge());

        float u = (n == 3) ? (3 / 16) : (3 / (8 * n));
        v->new_pos = (1 - n * u) * v->pos + u * neighbor_sum;
    }

    for (EdgeRef e = edges_begin(); e != edges_end(); e++) {
        e->is_new = false;

        HalfedgeRef h = e->halfedge();
        HalfedgeRef twin_h = h->twin();
        VertexRef a = h->vertex();
        VertexRef b = twin_h->vertex();
        VertexRef c = h->next()->next()->vertex();
        VertexRef d = twin_h->next()->next()->vertex();

        e->new_pos = ((a->pos + b->pos) * 3 + c->pos + d->pos) / 8;
    }

    Size edge_count = n_edges();
    EdgeRef current_e = edges_begin();
    for (Size i = 0; i < edge_count; i++) {
        EdgeRef next_e = current_e;
        next_e++;

        if (!current_e->is_new) {
            Vec3 new_pos = current_e->new_pos;
            VertexRef new_v = *(split_edge(current_e));
            new_v->new_pos = new_pos;
            new_v->is_new = true;

            for (HalfedgeRef h = new_v->halfedge()->twin()->next(); h != new_v->halfedge(); h = h ->twin()->next()) {
                h->edge()->is_new = true;
            }
        }

        current_e = next_e;
    }

    for (EdgeRef e = edges_begin(); e != edges_end(); e++) {
        if (e->is_new && e->halfedge()->vertex()->is_new != e->halfedge()->twin()->vertex()->is_new) {
            flip_edge(e);
        }
    }

    for (VertexRef v = vertices_begin(); v != vertices_end(); v++) {
        v->pos = v->new_pos;
    }
}

/*
    Isotropic remeshing. Note that this function returns success in a similar
    manner to the local operations, except with only a boolean value.
    (e.g. you may want to return false if this is not a triangle mesh)
*/
bool Halfedge_Mesh::isotropic_remesh() {

    // Compute the mean edge length.
    // Repeat the four main steps for 5 or 6 iterations
    // -> Split edges much longer than the target length (being careful about
    //    how the loop is written!)
    // -> Collapse edges much shorter than the target length.  Here we need to
    //    be EXTRA careful about advancing the loop, because many edges may have
    //    been destroyed by a collapse (which ones?)
    // -> Now flip each edge if it improves vertex degree
    // -> Finally, apply some tangential smoothing to the vertex positions

    // Note: if you erase elements in a local operation, they will not be actually deleted
    // until do_erase or validate are called. This is to facilitate checking
    // for dangling references to elements that will be erased.
    // The rest of the codebase will automatically call validate() after each op,
    // but here simply calling collapse_edge() will not erase the elements.
    // You should use collapse_edge_erase() instead for the desired behavior.

    return false;
}

/* Helper type for quadric simplification */
struct Edge_Record {
    Edge_Record() {
    }
    Edge_Record(std::unordered_map<Halfedge_Mesh::VertexRef, Mat4>& vertex_quadrics,
                Halfedge_Mesh::EdgeRef e)
        : edge(e) {

        // Compute the combined quadric from the edge endpoints.
        // -> Build the 3x3 linear system whose solution minimizes the quadric error
        //    associated with these two endpoints.
        // -> Use this system to solve for the optimal position, and store it in
        //    Edge_Record::optimal.
        // -> Also store the cost associated with collapsing this edge in
        //    Edge_Record::cost.
    }
    Halfedge_Mesh::EdgeRef edge;
    Vec3 optimal;
    float cost;
};

/* Comparison operator for Edge_Records so std::set will properly order them */
bool operator<(const Edge_Record& r1, const Edge_Record& r2) {
    if(r1.cost != r2.cost) {
        return r1.cost < r2.cost;
    }
    Halfedge_Mesh::EdgeRef e1 = r1.edge;
    Halfedge_Mesh::EdgeRef e2 = r2.edge;
    return &*e1 < &*e2;
}

/** Helper type for quadric simplification
 *
 * A PQueue is a minimum-priority queue that
 * allows elements to be both inserted and removed from the
 * queue.  Together, one can easily change the priority of
 * an item by removing it, and re-inserting the same item
 * but with a different priority.  A priority queue, for
 * those who don't remember or haven't seen it before, is a
 * data structure that always keeps track of the item with
 * the smallest priority or "score," even as new elements
 * are inserted and removed.  Priority queues are often an
 * essential component of greedy algorithms, where one wants
 * to iteratively operate on the current "best" element.
 *
 * PQueue is templated on the type T of the object
 * being queued.  For this reason, T must define a comparison
 * operator of the form
 *
 *    bool operator<( const T& t1, const T& t2 )
 *
 * which returns true if and only if t1 is considered to have a
 * lower priority than t2.
 *
 * Basic use of a PQueue might look
 * something like this:
 *
 *    // initialize an empty queue
 *    PQueue<myItemType> queue;
 *
 *    // add some items (which we assume have been created
 *    // elsewhere, each of which has its priority stored as
 *    // some kind of internal member variable)
 *    queue.insert( item1 );
 *    queue.insert( item2 );
 *    queue.insert( item3 );
 *
 *    // get the highest priority item currently in the queue
 *    myItemType highestPriorityItem = queue.top();
 *
 *    // remove the highest priority item, automatically
 *    // promoting the next-highest priority item to the top
 *    queue.pop();
 *
 *    myItemType nextHighestPriorityItem = queue.top();
 *
 *    // Etc.
 *
 *    // We can also remove an item, making sure it is no
 *    // longer in the queue (note that this item may already
 *    // have been removed, if it was the 1st or 2nd-highest
 *    // priority item!)
 *    queue.remove( item2 );
 *
 */
template<class T> struct PQueue {
    void insert(const T& item) {
        queue.insert(item);
    }
    void remove(const T& item) {
        if(queue.find(item) != queue.end()) {
            queue.erase(item);
        }
    }
    const T& top(void) const {
        return *(queue.begin());
    }
    void pop(void) {
        queue.erase(queue.begin());
    }
    size_t size() {
        return queue.size();
    }

    std::set<T> queue;
};

/*
    Mesh simplification. Note that this function returns success in a similar
    manner to the local operations, except with only a boolean value.
    (e.g. you may want to return false if you can't simplify the mesh any
    further without destroying it.)
*/
bool Halfedge_Mesh::simplify() {

    std::unordered_map<VertexRef, Mat4> vertex_quadrics;
    std::unordered_map<FaceRef, Mat4> face_quadrics;
    std::unordered_map<EdgeRef, Edge_Record> edge_records;
    PQueue<Edge_Record> edge_queue;

    // Compute initial quadrics for each face by simply writing the plane equation
    // for the face in homogeneous coordinates. These quadrics should be stored
    // in face_quadrics
    // -> Compute an initial quadric for each vertex as the sum of the quadrics
    //    associated with the incident faces, storing it in vertex_quadrics
    // -> Build a priority queue of edges according to their quadric error cost,
    //    i.e., by building an Edge_Record for each edge and sticking it in the
    //    queue. You may want to use the above PQueue<Edge_Record> for this.
    // -> Until we reach the target edge budget, collapse the best edge. Remember
    //    to remove from the queue any edge that touches the collapsing edge
    //    BEFORE it gets collapsed, and add back into the queue any edge touching
    //    the collapsed vertex AFTER it's been collapsed. Also remember to assign
    //    a quadric to the collapsed vertex, and to pop the collapsed edge off the
    //    top of the queue.

    // Note: if you erase elements in a local operation, they will not be actually deleted
    // until do_erase or validate are called. This is to facilitate checking
    // for dangling references to elements that will be erased.
    // The rest of the codebase will automatically call validate() after each op,
    // but here simply calling collapse_edge() will not erase the elements.
    // You should use collapse_edge_erase() instead for the desired behavior.

    return false;
}
