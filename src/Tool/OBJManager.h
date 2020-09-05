#ifndef OBJ_READER_H
#define OBJ_READER_H
#include "ConsoleColor.h"
#include <vector>
#include "../Geometry/Geometry.h"
namespace fucking_cool
{
namespace tool
{
    bool ReadOBJ(const std::string &filename, geometry::Point3List &points, geometry::Point3List &normals, 
        geometry::Point3List &colors);
    bool ReadOBJ(const std::string &filename, geometry::Point3List &points, geometry::Point3List &normals, 
        geometry::Point3List &colors, std::vector<Eigen::Vector3i> &triangles);
    bool WriteOBJ(const std::string &filename, const geometry::Point3List &points, const geometry::Point3List &normals, 
        const geometry::Point3List &colors);
    bool WriteOBJ(const std::string &filename, const geometry::Point3List &points, const geometry::Point3List &normals, 
        const geometry::Point3List &colors, const std::vector<Eigen::Vector3i> &triangles);
}
}
#endif