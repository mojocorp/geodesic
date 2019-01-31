#pragma once

#ifndef GEODESIC_DLL_IMPORT
#ifdef _WIN32
#define GEODESIC_DLL_IMPORT __declspec(dllimport)
#else
#define GEODESIC_DLL_IMPORT
#endif
#endif

#ifdef __cplusplus
extern "C"
{
#endif
    //! creates new mesh
    GEODESIC_DLL_IMPORT long new_mesh(long num_points,
                                      double* points,
                                      long num_triangles,
                                      long* triangles,
                                      long* num_edges,
                                      double** edges);

    //! creates a geodesic algorithm for a given mesh
    GEODESIC_DLL_IMPORT long new_algorithm(long mesh_id, long type, long subdivision);

    GEODESIC_DLL_IMPORT void delete_algorithm(long id);

    //! delete mesh and all associated algorithms
    GEODESIC_DLL_IMPORT void delete_mesh(long id);

    //! compute distance field for given source points
    GEODESIC_DLL_IMPORT void propagate(
      long algorithm_id,
      double* source_points,
      long num_sources,
      double* stop_points, //!< limitations on distance field propagation
      long num_stop_points,
      double max_propagation_distance);

    //! using procomputed distance field, compute a shortest path from destination to the closest
    //! source
    GEODESIC_DLL_IMPORT long trace_back(long algorithm_id, double* destination, double** path);

    //! quickly find what source this point belongs to and what is the distance
    //! to this source
    GEODESIC_DLL_IMPORT long distance_and_source(long algorithm_id,
                                                 double* destination,
                                                 double* best_source_distance);
    //! same idea as in the previous function
    GEODESIC_DLL_IMPORT long distance_and_source_for_all_vertices(
      long algorithm_id,
      double** distances, //!< list distance/source info for all vertices of the mesh
      long** sources);

#ifdef __cplusplus
}
#endif
