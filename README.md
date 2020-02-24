![C/C++ CI](https://github.com/mojocorp/geodesic/workflows/C/C++%20CI/badge.svg?branch=master)

# Introduction

This project is a fork from the original library http://code.google.com/p/geodesic/

This is an implementation of geodesic (shortest path) algorithm for triangular mesh (triangulated surface) first described by Mitchell, Mount and Papadimitriou in 1987[1] with some minor improvements, extensions and simplifications. The algorithm has O(n^2 log n) worst-case time complexity, but in practice can work with million-node meshes in reasonable time. For the quick overview, see [2].

# Details

The basic idea of the algorithm is very similar to Dijkstra's algorithm for finding shortest path on a weighted graph. It has two steps: 
- propagation of the distance field from sources over the surface of the mesh (slow)
- tracing back the shortest path from target point to the closest source (fast)

For debugging and comparison purposes I also implemented two approximate algorithms
- Dijkstra shortest path on the graph created by the vertices and edges of the mesh
- Subdivision (put N additional vertices on every edge of the mesh, directly connect all vertices belonging to the same face, run Dijkstra on the resulting graph)
The nice property of the subdivision algorithm is that it becomes Dijkstra when N=0 and computes exact distances when N->infinity.

The input mesh is represented as two arrays: vertices (each vertex has tree coordinates) and faces (each face is represented as indices of its vertices). Most of the communication with the algorithms is done through SurfacePoints (points on the surface of the mesh; they have three coordinates and a pointer to a mesh element they belong).

**C++ NOTES**
The base class of for all algorithms is GeodesicAlgorithmBase (defined in geodesic_algorithm_base.h). The most important functions defined in this class are
- propagate(...). It is possible to stop propagation after it reaches certain distance or covers certain points on the surface of the mesh; O(n^2 log n).
- trace_back(...). It traces the shortest path from target to the nearest source; O(n) .
- best_source(...). For a given point on the surface of the mesh, this function reports the closest source and the distance to this source; O(1).
Read example0.cpp and example1.cpp - they are self-explanatory.

**Limitations and known issues.**

The mesh should be edge-connected. I.e. for every two faces of the mesh there should exist a connecting path on the surface of the mesh that does not go through any vertex. This property implies the simple connectivity of the mesh. In particular, there should be no vertices that are not used by any of the triangles.

There is a list of possible features and improvements that could be done to the algorithm. 
- Memory is currently a bottleneck for large meshes. In theory, it is possible to overcome it if all destinations are known BEFORE the propagation step.
- For large flat parts of the mesh (vertices whose total adjacent angles sum up to exactly 2*pi) the current version of the algorithm uses more time and memory than necessary.

[1] J.S.B. Mitchell, D.~M. Mount, and C.~H. Papadimitriou. SIAM J. Comput., 16:647--668, 1987. 
[2] J. O'Rourke, Computational Geometry Column 35, SIGACT News, 30(2) Issue #111 (1999) 31-32, 1993 (could be found at citeseer.ist.psu.edu)

