#pragma once

#include <assert.h>
#include <fstream>

namespace geodesic {

template<class Points, class Faces>
inline bool
read_mesh_from_file(const char* filename, Points& points, Faces& faces)
{
    std::ifstream file(filename);
    assert(file.is_open());
    if (!file.is_open())
        return false;

    unsigned num_points;
    file >> num_points;
    assert(num_points >= 3);

    unsigned num_faces;
    file >> num_faces;

    points.resize(num_points * 3);
    for (typename Points::iterator i = points.begin(); i != points.end(); ++i) {
        file >> *i;
    }

    faces.resize(num_faces * 3);
    for (typename Faces::iterator i = faces.begin(); i != faces.end(); ++i) {
        file >> *i;
    }
    file.close();

    return true;
}

} // geodesic
