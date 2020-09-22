#ifndef MESH_SIMPLIFICATION_H
#define MESH_SIMPLIFICATION_H
#include "TriangleMesh.h"
#include <utility>
#include "Tool/ConsoleColor.h"
#include "Geometry/Geometry.h"
#include <unordered_map>
namespace fucking_cool 
{
namespace geometry
{
    //multi-resolution 3d approximations for rendering complex scenes, 1992.
    void ClusteringSimplification(TriangleMesh &wait_to_simplify, float grid_len);
    //Surface Simplification Using Quadric Error Metrics, 1997.
    void QuadricSimplification(TriangleMesh &wait_to_simplify, int target_triangle);
    void MeshPruning(TriangleMesh &mesh, int min_points);
    typedef std::pair<int, int> PositionInTriangle;//triangleID, and index in triangle
    typedef std::vector<std::pair<int, int>> Reference;//For each vertex, reference is used to find contained triangles
    struct QuadricHelper
    {
        std::vector<bool> t_dirty;
        std::vector<bool> t_deleted;
        geometry::Point3List *points_ptr;
        geometry::Point3List *normals_ptr;
        geometry::Point3List *colors_ptr;
        geometry::Point3uiList *triangles_ptr;
        std::vector<geometry::Matrix4> QMatrix;
        std::vector<std::vector<float>> error;
        std::vector<Reference> references;
        std::vector<bool> is_border;
        geometry::Point3List normals;//plane normal
        int deleted_triangle = 0;
    };

    struct ClusteringHelper
    {
        std::vector<bool> t_deleted;
        geometry::Point3List *points_ptr;
        geometry::Point3List *normals_ptr;
        geometry::Point3List *colors_ptr;
        geometry::Point3uiList *triangles_ptr;      
        std::vector<Reference> references;  
        float grid_len;
        std::unordered_map<Eigen::Vector3i, std::pair<int, int>, geometry::VoxelGridHasher > grid_map;
        std::unordered_map<Eigen::Vector3i, Point3, geometry::VoxelGridHasher > grid_to_point;
        
    };
    // delete the small patches of the mesh, which is not connected
    struct PruningHelper
    {
        std::vector<bool> t_deleted;
        geometry::Point3List *points_ptr;
        geometry::Point3List *normals_ptr;
        geometry::Point3List *colors_ptr;
        geometry::Point3uiList *triangles_ptr; 
        std::vector<Reference> references; 
        std::vector<bool > is_visited;
        std::set<int> unvisited;
        std::vector<std::set<int >> connected_patches;
    };
    void MeshPruning(TriangleMesh &mesh, int min_points);
    Eigen::Vector3i GetGridIndex(const geometry::Point3 &points, float grid_size);
    bool Flipped(QuadricHelper &helper, int p1, int p2, geometry::Point3 v, std::vector<bool> &deleted);
    void UpdateTriangles(QuadricHelper &helper,int p1, int p2, std::vector<bool> &deleted);
    void UpdateMesh(QuadricHelper &helper);
    void UpdateMesh(ClusteringHelper &helper);
    void UpdateMesh(PruningHelper &helper);
    void ComputeVertexNormals(QuadricHelper &helper);
    void ComputeVertexNormals(ClusteringHelper &helper);
    void CompactMesh(QuadricHelper &helper);
    void CompactMesh(ClusteringHelper &helper);
    void CompactMesh(PruningHelper &helper);
    void InitializeHelper(TriangleMesh &wait_to_simplify, QuadricHelper &helper);
    void InitializeHelper(TriangleMesh &wait_to_simplify, ClusteringHelper &helper);
    void InitializeHelper(TriangleMesh &wait_to_simplify, PruningHelper &helper);
    void ComputeError(QuadricHelper &helper);
    void CheckIsBorder(const geometry::Point3uiList &triangles, const std::vector<Reference> &references, std::vector<bool> &is_border);
    float ComputeError(QuadricHelper &helper, int i, int j, geometry::Point3 &new_p);
    void UpdateReferences(const geometry::Point3uiList &triangles, std::vector<Reference> &references);
    void ComputeNormalsAndQMatrix(const geometry::Point3uiList &triangles, const geometry::Point3List &points,
        std::vector<Reference> &references, geometry::Point3List &normals ,std::vector<geometry::Matrix4> &QMatrix);
    
}
}

#endif