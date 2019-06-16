// Copyright (C) 2008 Danil Kirsanov, MIT License
#pragma once

// here we define the building elements of the mesh:
// 3D-points, vertices, edges, faces, and surface points

#include <assert.h>
#include <cstddef>
#include <cmath>
#include <algorithm>

namespace geodesic {

class Vertex;
class Edge;
class Face;
class Mesh;
class MeshElementBase;

typedef Vertex* vertex_pointer;
typedef Edge* edge_pointer;
typedef Face* face_pointer;
typedef Mesh* mesh_pointer;
typedef MeshElementBase* base_pointer;

enum PointType
{
    VERTEX,
    EDGE,
    FACE,
    UNDEFINED_POINT
};

class MeshElementBase // prototype of vertices, edges and faces
{
  public:
    typedef std::vector<vertex_pointer> vertex_pointer_vector;
    typedef std::vector<edge_pointer> edge_pointer_vector;
    typedef std::vector<face_pointer> face_pointer_vector;

    MeshElementBase()
      : m_id(0)
      , m_type(UNDEFINED_POINT)
    {}

    vertex_pointer_vector& adjacent_vertices() { return m_adjacent_vertices; }
    edge_pointer_vector& adjacent_edges() { return m_adjacent_edges; }
    face_pointer_vector& adjacent_faces() { return m_adjacent_faces; }

    unsigned& id() { return m_id; }
    PointType type() const { return m_type; }

  protected:
    vertex_pointer_vector m_adjacent_vertices; // list of the adjacent vertices
    edge_pointer_vector m_adjacent_edges;      // list of the adjacent edges
    face_pointer_vector m_adjacent_faces;      // list of the adjacent faces

    unsigned m_id;    // unique id
    PointType m_type; // vertex, edge or face
};

class Point3D // point in 3D and corresponding operations
{
  public:
    Point3D() {}
    Point3D(const Point3D& p) { set(p.x(), p.y(), p.z()); }

    double x() const { return m_coordinates[0]; }
    double y() const { return m_coordinates[1]; }
    double z() const { return m_coordinates[2]; }

    void set(double new_x, double new_y, double new_z)
    {
        m_coordinates[0] = new_x;
        m_coordinates[1] = new_y;
        m_coordinates[2] = new_z;
    }

    double distance(const Point3D& v) const
    {
        const double dx = m_coordinates[0] - v.x();
        const double dy = m_coordinates[1] - v.y();
        const double dz = m_coordinates[2] - v.z();

        return std::sqrt(dx * dx + dy * dy + dz * dz);
    }

    void add(const Point3D& v)
    {
        m_coordinates[0] += v.x();
        m_coordinates[1] += v.y();
        m_coordinates[2] += v.z();
    }

    void multiply(double v)
    {
        m_coordinates[0] *= v;
        m_coordinates[1] *= v;
        m_coordinates[2] *= v;
    }

  protected:
    double m_coordinates[3]; // xyz
};

class Vertex
  : public MeshElementBase
  , public Point3D
{
  public:
    Vertex() { m_type = VERTEX; }

    ~Vertex() {}

    bool& saddle_or_boundary() { return m_saddle_or_boundary; }

  private:
    // this flag speeds up exact geodesic algorithm
    bool m_saddle_or_boundary; // it is true if total adjacent angle is larger than 2*PI or this
                               // vertex belongs to the mesh boundary
};

class Face : public MeshElementBase
{
  public:
    Face() { m_type = FACE; }

    ~Face() {}

    edge_pointer opposite_edge(vertex_pointer v);
    vertex_pointer opposite_vertex(edge_pointer e);
    edge_pointer next_edge(edge_pointer e, vertex_pointer v);

    double vertex_angle(vertex_pointer v)
    {
        for (unsigned i = 0; i < 3; ++i) {
            if (adjacent_vertices()[i]->id() == v->id()) {
                return m_corner_angles[i];
            }
        }
        assert(0);
        return 0;
    }

    double* corner_angles() { return m_corner_angles; }

  private:
    double m_corner_angles[3]; // triangle angles in radians; angles correspond to vertices in
                               // m_adjacent_vertices
};

class Edge : public MeshElementBase
{
  public:
    Edge() { m_type = EDGE; }

    ~Edge() {}

    double& length() { return m_length; }

    face_pointer opposite_face(face_pointer f)
    {
        if (adjacent_faces().size() == 1) {
            assert(adjacent_faces()[0]->id() == f->id());
            return nullptr;
        }

        assert(adjacent_faces()[0]->id() == f->id() || adjacent_faces()[1]->id() == f->id());

        return adjacent_faces()[0]->id() == f->id() ? adjacent_faces()[1] : adjacent_faces()[0];
    }

    vertex_pointer opposite_vertex(vertex_pointer v)
    {
        assert(belongs(v));

        return adjacent_vertices()[0]->id() == v->id() ? adjacent_vertices()[1]
                                                       : adjacent_vertices()[0];
    }

    bool belongs(vertex_pointer v) const
    {
        return m_adjacent_vertices[0]->id() == v->id() || m_adjacent_vertices[1]->id() == v->id();
    }

    bool is_boundary() const { return m_adjacent_faces.size() == 1; }

    vertex_pointer v0() { return adjacent_vertices()[0]; }
    vertex_pointer v1() { return adjacent_vertices()[1]; }

    void local_coordinates(const Point3D* point, double& x, double& y) const
    {
        double d0 = point->distance(*m_adjacent_vertices[0]);
        if (d0 < 1e-50) {
            x = 0.0;
            y = 0.0;
            return;
        }

        double d1 = point->distance(*m_adjacent_vertices[1]);
        if (d1 < 1e-50) {
            x = m_length;
            y = 0.0;
            return;
        }

        x = m_length / 2.0 + (d0 * d0 - d1 * d1) / (2.0 * m_length);
        y = std::sqrt(std::max(0.0, d0 * d0 - x * x));
        return;
    }

  private:
    double m_length; // length of the edge
};

class SurfacePoint : public Point3D // point on the surface of the mesh
{
  public:
    SurfacePoint()
      : m_p(nullptr)
    {}

    SurfacePoint(const vertex_pointer v)
      : // set the surface point in the vertex
      SurfacePoint::Point3D(*v)
      , m_p(v)
    {}

    SurfacePoint(const face_pointer f)
      : // set the surface point in the center of the face
      m_p(f)
    {
        set(0, 0, 0);
        add(*f->adjacent_vertices()[0]);
        add(*f->adjacent_vertices()[1]);
        add(*f->adjacent_vertices()[2]);
        multiply(1. / 3.);
    }

    SurfacePoint(const edge_pointer e, // set the surface point in the middle of the edge
                 double a = 0.5)
      : m_p(e)
    {
        double b = 1 - a;

        vertex_pointer v0 = e->adjacent_vertices()[0];
        vertex_pointer v1 = e->adjacent_vertices()[1];

        m_coordinates[0] = b * v0->x() + a * v1->x();
        m_coordinates[1] = b * v0->y() + a * v1->y();
        m_coordinates[2] = b * v0->z() + a * v1->z();
    }

    SurfacePoint(const base_pointer g, double x, double y, double z, PointType t = UNDEFINED_POINT)
      : m_p(g)
    {
        set(x, y, z);
    }

    void initialize(SurfacePoint const& p) { *this = p; }

    ~SurfacePoint() {}

    PointType type() const { return m_p ? m_p->type() : UNDEFINED_POINT; }
    base_pointer& base_element() { return m_p; }
    const base_pointer& base_element() const { return m_p; }

  protected:
    base_pointer m_p; // could be face, vertex or edge pointer
};

inline edge_pointer
Face::opposite_edge(vertex_pointer v)
{
    for (unsigned i = 0; i < 3; ++i) {
        edge_pointer e = adjacent_edges()[i];
        if (!e->belongs(v)) {
            return e;
        }
    }
    assert(0);
    return nullptr;
}

inline vertex_pointer
Face::opposite_vertex(edge_pointer e)
{
    for (unsigned i = 0; i < 3; ++i) {
        vertex_pointer v = adjacent_vertices()[i];
        if (!e->belongs(v)) {
            return v;
        }
    }
    assert(0);
    return nullptr;
}

inline edge_pointer
Face::next_edge(edge_pointer e, vertex_pointer v)
{
    assert(e->belongs(v));

    for (unsigned i = 0; i < 3; ++i) {
        edge_pointer next = adjacent_edges()[i];
        if (e->id() != next->id() && next->belongs(v)) {
            return next;
        }
    }
    assert(0);
    return nullptr;
}

struct HalfEdge // prototype of the edge; used for mesh construction
{
    unsigned face_id;
    unsigned vertex_0; // adjacent vertices sorted by id value
    unsigned vertex_1; // they are sorted, vertex_0 < vertex_1
};

inline bool
operator<(const HalfEdge& x, const HalfEdge& y)
{
    if (x.vertex_0 == y.vertex_0) {
        return x.vertex_1 < y.vertex_1;
    } else {
        return x.vertex_0 < y.vertex_0;
    }
}

inline bool
operator!=(const HalfEdge& x, const HalfEdge& y)
{
    return x.vertex_0 != y.vertex_0 || x.vertex_1 != y.vertex_1;
}

inline bool
operator==(const HalfEdge& x, const HalfEdge& y)
{
    return x.vertex_0 == y.vertex_0 && x.vertex_1 == y.vertex_1;
}

} // geodesic
