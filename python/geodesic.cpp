#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/numpy.h>

#include <geodesic/geodesic_mesh_elements.h>
#include <geodesic/geodesic_mesh.h>
#include <geodesic/geodesic_algorithm_graph_base.h>
#include <geodesic/geodesic_algorithm_dijkstra.h>
#include <geodesic/geodesic_algorithm_exact.h>
#include <geodesic/geodesic_algorithm_subdivision.h>

namespace py = pybind11;

PYBIND11_MODULE(geodesic, m)
{
    // clang-format off

    m.attr("__version__") = GEODESIC_VERSION;

    py::class_<geodesic::Vertex>(m, "Vertex")
        .def(py::init<>());

    py::class_<geodesic::SurfacePoint>(m, "SurfacePoint")
        .def(py::init<>())
        .def(py::init<geodesic::Vertex*>())
        .def("x", &geodesic::SurfacePoint::x)
        .def("y", &geodesic::SurfacePoint::y)
        .def("z", &geodesic::SurfacePoint::z);

    py::class_<geodesic::Mesh>(m, "Mesh")
        .def(py::init<>())
        .def("initialize_mesh_data",[](geodesic::Mesh &self, const py::array_t<double> &vertices, const py::array_t<unsigned> &faces)
        {
            std::vector<double> vtx(vertices.data(), vertices.data() + vertices.size());
            std::vector<unsigned> fcs(faces.data(), faces.data() + faces.size());
            self.initialize_mesh_data(vtx, fcs);
        });

    py::class_<geodesic::GeodesicAlgorithmBase>(m, "GeodesicAlgorithmBase")
        .def("propagate", &geodesic::GeodesicAlgorithmBase::propagate)
        .def("best_source", &geodesic::GeodesicAlgorithmBase::best_source)
        .def("geodesic",[](geodesic::GeodesicAlgorithmBase &self, int src, int dst)
        {
            geodesic::SurfacePoint source(&self.mesh()->vertices()[src]);
            geodesic::SurfacePoint target(&self.mesh()->vertices()[dst]);
            std::vector<geodesic::SurfacePoint> path;

            double const distance_limit = geodesic::GEODESIC_INF; // no limit for propagation

            std::vector<geodesic::SurfacePoint> all_sources(1, source); // in general, there could be multiple sources, but now we have only one
            std::vector<geodesic::SurfacePoint> stop_points(1, target); // stop propagation when the target is covered

            self.propagate(all_sources,
                            distance_limit,
                            &stop_points); //"propagate(all_sources)" is also fine, but take
                                           // more time because covers the whole mesh

            self.trace_back(target, path); // trace back a single path

            std::vector<std::vector<double>> spath;
            spath.resize(path.size());
            for (int k=0; k<path.size(); ++k) {
                spath[k].push_back(path[k].x());
                spath[k].push_back(path[k].y());
                spath[k].push_back(path[k].z());
            }
            return spath;
        })
        .def("mesh", &geodesic::GeodesicAlgorithmBase::mesh)
        .def("print_statistics", &geodesic::GeodesicAlgorithmBase::print_statistics);

    py::class_<geodesic::GeodesicAlgorithmDijkstra, geodesic::GeodesicAlgorithmBase>(m, "GeodesicAlgorithmDijkstra")
        .def(py::init<geodesic::Mesh*>());
    py::class_<geodesic::GeodesicAlgorithmExact, geodesic::GeodesicAlgorithmBase>(m, "GeodesicAlgorithmExact")
        .def(py::init<geodesic::Mesh*>());
    py::class_<geodesic::GeodesicAlgorithmSubdivision, geodesic::GeodesicAlgorithmBase>(m, "GeodesicAlgorithmSubdivision")
        .def(py::init<geodesic::Mesh*>());
}
