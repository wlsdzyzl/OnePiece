#ifndef RPLY_READER_H
#define RPLY_READER_H
#include "ConsoleColor.h"
#include <vector>
#include "../Geometry/Geometry.h"
namespace fucking_cool
{
namespace tool
{
    bool ReadPLY(const std::string &filename, geometry::Point3List &points, geometry::Point3List &normals, 
        geometry::Point3List &colors);
    bool ReadPLY(const std::string &filename, geometry::Point3List &points, geometry::Point3List &normals, 
        geometry::Point3List &colors, std::vector<Eigen::Vector3i> &triangles);
    bool WritePLY(const std::string &filename, const geometry::Point3List&points, 
        const geometry::Point3List &normals, const geometry::Point3List &colors, bool use_ascii = true);
    bool WritePLY(const std::string &filename, const geometry::Point3List&points, 
        const geometry::Point3List &normals, const geometry::Point3List &colors,
        const std::vector<Eigen::Vector3i> &triangles, bool use_ascii = true);
}
}
#endif