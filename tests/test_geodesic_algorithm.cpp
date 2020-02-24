#include <gtest/gtest.h>

#include <geodesic_algorithm_exact.h>
#include <geodesic_algorithm_dijkstra.h>

#include "../examples/common.h"

#include <vector>

// The fixture for testing geodesic.
class GeodesicTest : public ::testing::Test
{
  protected:
    GeodesicTest()
    {
        bool success = geodesic::read_mesh_from_file("flat_triangular_mesh.txt", points, faces);
        if (!success) {
            throw std::runtime_error("something is wrong with the input file");
        }
        // create internal mesh data structure including edges
        mesh.initialize_mesh_data(points, faces);
    }

    ~GeodesicTest() override
    {
        // You can do clean-up work that doesn't throw exceptions here.
    }

    void SetUp() override
    {
        // Code here will be called immediately after the constructor (right
        // before each test).
    }

    void TearDown() override
    {
        // Code here will be called immediately after each test (right
        // before the destructor).
    }

    std::vector<double> points;
    std::vector<unsigned> faces;
    geodesic::Mesh mesh;
};

TEST_F(GeodesicTest, geodesic_algorithm_exact)
{
    // simplest approximate algorithm: path only allowed on the edges of the mesh
    geodesic::GeodesicAlgorithmExact algorithm(&mesh);

    const unsigned int source_vertex_index = 0;
    const unsigned int target_vertex_index = 100;
    geodesic::SurfacePoint source(&mesh.vertices()[source_vertex_index]);
    geodesic::SurfacePoint target(&mesh.vertices()[target_vertex_index]);

    // in general, there could be multiple sources, but now we have only one
    std::vector<geodesic::SurfacePoint> all_sources(1, source);

    double const distance_limit = geodesic::GEODESIC_INF; // no limit for propagation

    // stop propagation when the target is covered
    std::vector<geodesic::SurfacePoint> stop_points(1, target);
    algorithm.propagate(all_sources,
                        distance_limit,
                        &stop_points); //"propagate(all_sources)" is also fine, but take
                                       // more time because covers the whole mesh

    // geodesic path is a sequence of SurfacePoints
    std::vector<geodesic::SurfacePoint> path;
    algorithm.trace_back(target, path); // trace back a single path

    EXPECT_EQ(19, path.size());
    EXPECT_FLOAT_EQ(1.811077f, geodesic::length(path));
}

TEST_F(GeodesicTest, geodesic_algorithm_dijkstra)
{
    // simplest approximate algorithm: path only allowed on the edges of the mesh
    geodesic::GeodesicAlgorithmDijkstra algorithm(&mesh);

    const unsigned int source_vertex_index = 0;
    const unsigned int target_vertex_index = 100;
    geodesic::SurfacePoint source(&mesh.vertices()[source_vertex_index]);
    geodesic::SurfacePoint target(&mesh.vertices()[target_vertex_index]);

    // in general, there could be multiple sources, but now we have only one
    std::vector<geodesic::SurfacePoint> all_sources(1, source);

    double const distance_limit = geodesic::GEODESIC_INF; // no limit for propagation

    // stop propagation when the target is covered
    std::vector<geodesic::SurfacePoint> stop_points(1, target);
    algorithm.propagate(all_sources,
                        distance_limit,
                        &stop_points); //"propagate(all_sources)" is also fine, but take
                                       // more time because covers the whole mesh

    // geodesic path is a sequence of SurfacePoints
    std::vector<geodesic::SurfacePoint> path;
    algorithm.trace_back(target, path); // trace back a single path

    EXPECT_EQ(11, path.size());
    EXPECT_FLOAT_EQ(2.f, geodesic::length(path));
}
