#include "TriangleMesh.h"
#include "Tool/PLYManager.h"
#include "Tool/OBJManager.h"
#include <iostream>
#include <fstream>
#include "MeshSimplification.h"
#include <omp.h>
namespace fucking_cool
{
namespace geometry
{
    bool TriangleMesh::LoadFromPLY(const std::string &filename)
    {
        Reset();
        std::vector<tool::AdditionalElement> additional_labels;
        return tool::ReadPLY(filename,points,normals,colors,triangles, additional_labels);
    }
    bool TriangleMesh::LoadFromOBJ(const std::string& filename) 
    {
        Reset();
        return tool::ReadOBJ(filename,points,normals,colors,triangles);
    }
    void TriangleMesh::Transform(const geometry::TransformationMatrix & T)
    {
        geometry::TransformPoints(T,points);
        if(HasNormals())
        geometry::TransformNormals(T,normals);
    }
    std::shared_ptr<geometry::TriangleMesh> TriangleMesh::QuadricSimplify(size_t target_num) const
    {
        geometry::TriangleMesh s_mesh = *this;
        QuadricSimplification(s_mesh,target_num);
        return std::make_shared<geometry::TriangleMesh>(s_mesh);
    }
    std::shared_ptr<geometry::TriangleMesh> TriangleMesh::ClusteringSimplify(float grid_len) const
    {
        geometry::TriangleMesh s_mesh = *this;
        ClusteringSimplification(s_mesh,grid_len);
        return std::make_shared<geometry::TriangleMesh>(s_mesh);
    }
    std::shared_ptr<geometry::TriangleMesh> TriangleMesh::Prune(size_t min_points) const
    {
        geometry::TriangleMesh s_mesh = *this;
        MeshPruning(s_mesh,min_points);
        return std::make_shared<geometry::TriangleMesh>(s_mesh);
    }
    std::shared_ptr<geometry::PointCloud> TriangleMesh::GetPointCloud() const
    {
        geometry::PointCloud pcd;
        pcd.normals = normals;
        pcd.points = points;
        pcd.colors = colors;
        return std::make_shared<geometry::PointCloud>(pcd);
    }
    void TriangleMesh::LoadFromMeshes(const std::vector<TriangleMesh > &meshes)
    {
        Reset();
        int start_index = 0;
        size_t triangles_num;
        for(size_t i = 0; i < meshes.size(); ++i)
        {
            triangles.insert(triangles.end(),meshes[i].triangles.begin(),meshes[i].triangles.end());
            colors.insert(colors.end(),meshes[i].colors.begin(),meshes[i].colors.end());
            points.insert(points.end(),meshes[i].points.begin(),meshes[i].points.end());
            normals.insert(normals.end(),meshes[i].normals.begin(),meshes[i].normals.end());
            triangles_num = meshes[i].triangles.size();
            if(start_index != 0)
            for(size_t i = triangles.size()-1, j = 0;j!=triangles_num; ++j, --i )
            {
                triangles[i](0) = triangles[i](0) + start_index;
                triangles[i](1) = triangles[i](1) + start_index;
                triangles[i](2) = triangles[i](2) + start_index;
            }
            start_index += meshes[i].points.size();
        }
    }
    void TriangleMesh::ComputeNormals()
    {
        std::vector<Reference> references;
        references.resize(points.size());
        UpdateReferences(triangles,references);
        normals.resize(points.size());
        geometry::Point3List triangle_normals;
        triangle_normals.resize(triangles.size());
        geometry::Point3 n,p1,p2,p3;
      
        for(size_t i = 0;i!= triangles.size(); ++i)
        {
            p1 = points[triangles[i](0)];
            p2 = points[triangles[i](1)];
            p3 = points[triangles[i](2)];
            n = ((p2 - p1).cross(p3 - p1));
            n.normalize();
            triangle_normals[i] = n;
        }
		for(size_t i = 0;i!=points.size();++i)
		{

			geometry::Point3 vnormal; 
            vnormal.setZero();
            for(size_t j = 0;j!=references[i].size();++j)
			{
				vnormal += triangle_normals[references[i][j].first];
			}
			vnormal.normalize();

            normals[i] = vnormal;
		}
    }
    bool TriangleMesh::WriteToPLY(const std::string &filename) const
    {
        return tool::WritePLY(filename, points, normals, colors, triangles);
    }


    bool TriangleMesh::WriteToOBJ(const std::string& filename) const
    {
        return tool::WriteOBJ(filename,points,normals,colors,triangles);
    }
}
}