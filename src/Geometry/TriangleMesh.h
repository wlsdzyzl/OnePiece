#ifndef TRIANGLE_MESH_H
#define TRIANGLE_MESH_H
#include "Tool/ConsoleColor.h"
#include "Geometry/Geometry.h"
#include "Geometry/PointCloud.h"
#include <memory>
namespace fucking_cool
{
namespace geometry
{
    class TriangleMesh
    {
        public:
        bool LoadFromPLY(const std::string &filename);
        bool LoadFromOBJ(const std::string &filename);
        void ComputeNormals();
        void Transform(const geometry::TransformationMatrix & T);
        bool HasColors() const
        {
            return colors.size() == points.size() && colors.size() > 0;
        }
        void Reset()
        {
            triangles.clear();
            points.clear();
            normals.clear();
            colors.clear();
        }
        void LoadFromMeshes(const std::vector<TriangleMesh > &meshes);
        bool HasNormals() const
        {
            return normals.size() == points.size() && normals.size() > 0;
        }
        std::shared_ptr<geometry::TriangleMesh> QuadricSimplify(size_t target_num) const;
        std::shared_ptr<geometry::TriangleMesh> ClusteringSimplify(float grid_len) const;
        std::shared_ptr<geometry::TriangleMesh> Prune(size_t min_points) const;
        std::shared_ptr<geometry::PointCloud> GetPointCloud() const;
        size_t GetPointSize() const{return points.size();}
        size_t GetTriangleSize() const{return triangles.size();}
        bool WriteToPLY(const std::string &fileName) const;
        bool WriteToOBJ(const std::string &fileName) const;
        geometry::Point3uiList triangles;
        geometry::Point3List points;
        geometry::Point3List normals;
        geometry::Point3List colors; 
        
    };
}
}

#endif