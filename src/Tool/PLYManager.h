#ifndef PLY_MANAGER_H
#define PLY_MANAGER_H
#include "ConsoleColor.h"
#include <vector>
#include "Geometry/Geometry.h"
#include "tinyply.h"
namespace fucking_cool
{
namespace tool
{
    struct AdditionalElement
    {
        size_t count;
        size_t byte_size;
        std::string element_key;
        std::vector<std::string> element_property;
        tinyply::Type type;
        tinyply::Type list_type = tinyply::Type::INVALID;
        size_t list_count = 0;
        unsigned char *data; 
    };

    bool ReadPLY(const std::string &filename, geometry::Point3List &points, geometry::Point3List &normals,
        geometry::Point3List &colors, geometry::Point3uiList &triangles, 
        std::vector<AdditionalElement> & additional_labels);
    
    bool WritePLY(const std::string &filename, const geometry::Point3List&points, 
        const geometry::Point3List &normals, const geometry::Point3List &colors,
        const geometry::Point3uiList &triangles = geometry::Point3uiList(), 
        const std::vector<std::string> & comments = std::vector<std::string>(),
        const std::vector<AdditionalElement> & additional_labels = std::vector<AdditionalElement>(),
        bool use_ascii = false);
}
}
#endif