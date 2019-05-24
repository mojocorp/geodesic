import numpy
from geodesic import Mesh
from geodesic import GeodesicAlgorithmExact

temp = numpy.loadtxt("flat_triangular_mesh.txt", skiprows=1)
vertices = temp[0:121].astype(numpy.float64)
triangles = temp[121:321].astype(numpy.int32)

mesh=Mesh()
mesh.initialize_mesh_data(vertices, triangles); # create internal mesh data structure including edges

algorithm=GeodesicAlgorithmExact(mesh)
path=algorithm.geodesic(0, 100)
algorithm.print_statistics()
print(path)