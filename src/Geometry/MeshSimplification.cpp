#include "MeshSimplification.h"
#include <iostream>
#include <map>
namespace fucking_cool 
{
namespace geometry
{

    bool Flipped(QuadricHelper &helper, int p1, int p2, geometry::Point3 v, std::vector<bool> &deleted)
    {
        //a triangle flipped if the normal changed to much, or the three points are on a line 
        Reference &reference = helper.references[p1];
        std::vector<Eigen::Vector3i> &triangles = *helper.triangles_ptr;
        geometry::Point3List &points = *helper.points_ptr;
        geometry::Point3List &normals = helper.normals;
        for(int i = 0; i!= reference.size(); ++i)
        {
            int tid = reference[i].first;
            if(helper.t_deleted[tid]) continue;
            int vid = reference[i].second;
            // find the other two points
            int id1 = triangles[tid]((vid+1)%3), id2 = triangles[tid]((vid+2)%3);
            if(id1 == p2 || id2 == p2)
            {
                deleted[i] = true;
                continue;
            }
            
            geometry::Point3 tv1 = points[id1], tv2 = points[id2];
            geometry::Point3 d1 = tv1 - v, d2 = tv2 - v;
            d1.normalize();
            d2.normalize();
            if(d1.dot(d2) > 0.999) return true;
            geometry::Point3 n = (d1.cross(d2));
            n.normalize();

            if(n.dot(normals[tid]) < 0.2) return true;
            deleted[i] = false;
        }
        return false;
    }

    void UpdateTriangles(QuadricHelper &helper,int p1, int p2, std::vector<bool> &deleted)
    {
        //p1 original vertex, p2 new vertiex, replace p1 with p2
        Reference &reference = helper.references[p1];
        std::vector<Eigen::Vector3i> &triangles = *helper.triangles_ptr;
        geometry::Point3List &points = *helper.points_ptr;
        geometry::Point3List &normals = helper.normals;
        std::vector<std::vector<float>> &error = helper.error;
        for(int i = 0;i!= reference.size();++i)
        {
            int tid = reference[i].first;
            int vid = reference[i].second;
            // find the other two points
            if(helper.t_deleted[tid]) continue; 
            if(deleted[i]) 
            {
                helper.t_deleted[tid] = true;
                helper.deleted_triangle ++;
            }
            else
            {
                triangles[tid](vid) = p2;//update the vertex id
                geometry::Point3 v, v1, v2, v3, n;
                v1 = points[triangles[tid](0)];
                v2 = points[triangles[tid](1)];
                v3 = points[triangles[tid](2)];
                n = ((v2 - v1).cross(v3 - v1));
                n.normalize();

                error[tid][0] = ComputeError(helper,triangles[tid](0), triangles[tid](1), v );
                error[tid][1] = ComputeError(helper,triangles[tid](1), triangles[tid](2), v );
                error[tid][2] = ComputeError(helper,triangles[tid](2), triangles[tid](0), v );
                error[tid][3] = std::min(error[i][0], std::min(error[i][1], error[i][2]));
                //std::cout<<n.dot(helper.normals[tid])<<std::endl;
                helper.t_dirty[tid] = true;
                helper.normals[tid] = n;

                //no need to update references, because all the triangles related to p1 and p2 is dirty now.
            }
        }
    }
    void UpdateMesh(QuadricHelper &helper)
    {

        std::vector<Eigen::Vector3i> &triangles = *helper.triangles_ptr;
        geometry::Point3List &normals = helper.normals;
        int re_local = 0;
        for(int i = 0;i!=triangles.size();++i)
        {
            if(helper.t_deleted[i]) continue;
            triangles[re_local] = triangles[i];
            helper.error[re_local] = helper.error[i];
            helper.normals[re_local] = helper.normals[i];
            re_local ++;
            
        }
        //std::cout<<"triangles :"<<re_local<<std::endl;
        triangles.resize(re_local);
        helper.t_dirty.clear();
        helper.t_deleted.clear();
        helper.t_dirty.resize(re_local,0);
        helper.t_deleted.resize(re_local,0);
        //for(int i = 0;i!=helper.t_deleted.size(); ++i)
        //std::cout<<helper.t_deleted[i]<<" ";
        //std::cout<<std::endl;
        helper.error.resize(re_local);
        helper.normals.resize(re_local);
        UpdateReferences(*helper.triangles_ptr,  helper.references);
        
    }
    void UpdateMesh(ClusteringHelper &helper)
    {

        std::vector<Eigen::Vector3i> &triangles = *helper.triangles_ptr;
        geometry::Point3List &points = *helper.points_ptr;
        auto &grid_to_point = helper.grid_to_point;
        float grid_len = helper.grid_len;
        int re_local = 0;
        for(int i = 0;i!=triangles.size();++i)
        {
            if(helper.t_deleted[i]) continue;
            triangles[re_local] = triangles[i];
            re_local ++;
        }
        triangles.resize(re_local);
        //update the points position, we use the average value.
        for(auto iter = helper.grid_map.begin();iter!=helper.grid_map.end(); ++iter)
        {
            // points[iter->second.first] = Point3(iter->first(0) * helper.grid_len + helper.grid_len / 2.0, 
            //     iter->first(1) * helper.grid_len + helper.grid_len / 2.0, iter->first(2) * helper.grid_len + helper.grid_len / 2.0);
            
            points[iter->second.first] = grid_to_point[iter->first] / iter->second.second;
        }
        UpdateReferences(*helper.triangles_ptr, helper.references);
        
    }
    void UpdateMesh(PruningHelper &helper)
    {

        std::vector<Eigen::Vector3i> &triangles = *helper.triangles_ptr;
        int re_local = 0;
        for(int i = 0;i!=triangles.size();++i)
        {
            if(helper.t_deleted[i]) continue;
            triangles[re_local] = triangles[i];
            re_local ++;
            
        }
        //std::cout<<"triangles :"<<re_local<<std::endl;
        triangles.resize(re_local);
        UpdateReferences(*helper.triangles_ptr,  helper.references);
        
    }
    void QuadricSimplification(TriangleMesh &wait_to_simplify, int target_triangle)
    {
        QuadricHelper q_helper;
        InitializeHelper(wait_to_simplify,q_helper);
        int iteration = 1, max_iteration = 100;
        float error_threshold ;
        int origin_triangle = wait_to_simplify.triangles.size();

        if(origin_triangle <= target_triangle)
        {
        std::cout<<YELLOW << "[QuadricMeshSimplification]::[WARNING]::target number is larger than the original number."<<RESET<<std::endl;
        return;
        }
        std::vector<Eigen::Vector3i> &triangles = *q_helper.triangles_ptr;
        geometry::Point3List &points = *q_helper.points_ptr;
        for(;iteration<=max_iteration; ++iteration)
        {
            if( iteration % 5 == 0)
            UpdateMesh(q_helper);
            error_threshold = 0.000000001*std::pow(float(iteration+3),6);
            //std::cout<<"Iteration "<<iteration<<", deleted_triangle: "<<q_helper.deleted_triangle<<std::endl;
            for(int i=0;i!=q_helper.triangles_ptr->size(); ++i)
            {
                //std::cout<<q_helper.error[i][3]<<std::endl;
                if(q_helper.error[i][3] > error_threshold) continue;
                if(q_helper.t_deleted[i]) continue;
                if(q_helper.t_dirty[i]) continue;
                for( int j = 0;j!=3;++j)
                {
                    if(q_helper.error[i][j] <= error_threshold)
                    {
                        // try to delete the triangles_ptr
                        int p1 = triangles[i](j); 
                        geometry::Point3 &v1 = points[p1];
                        int p2 = triangles[i]((j+1)%3);
                        geometry::Point3 &v2 = points[p2];
                        if(q_helper.is_border[p1]!= q_helper.is_border[p2]) continue;

                        geometry::Point3 v3;
                        //compute the best position
                        ComputeError(q_helper, p1, p2, v3);

                        std::vector<bool> deleted_1, deleted_2;
                        deleted_1.resize(q_helper.references[p1].size());
                        deleted_2.resize(q_helper.references[p2].size());
                        if(Flipped(q_helper, p1, p2, v3, deleted_1)) continue;
                        if(Flipped(q_helper, p2, p1, v3, deleted_2)) continue;
                        v1 = v3;
                        q_helper.QMatrix[p1] =q_helper.QMatrix[p1] + q_helper.QMatrix[p2];
                        

                        // delete the triangle , update the vertex and 
                        UpdateTriangles(q_helper,p1,p1,deleted_1);
                        UpdateTriangles(q_helper,p2,p1,deleted_2);
                        break;       

                    }
                }
                if(origin_triangle - q_helper.deleted_triangle <= target_triangle) break;
            }
            if(origin_triangle - q_helper.deleted_triangle <= target_triangle) break;
        }
        //std::cout<<YELLOW << "[QuadricMeshSimplification]::[INFO]::Compacting..."<<RESET<<std::endl;
        CompactMesh(q_helper);
        std::cout<<GREEN<< "[QuadricMeshSimplification]::[INFO]::Simplify done. Contract "<<q_helper.deleted_triangle<<" pairs."<<RESET<<std::endl;
        return;
    }

    void ComputeVertexNormals(QuadricHelper &helper)
    {
        std::vector<Eigen::Vector3i> &triangles = *helper.triangles_ptr;
        geometry::Point3List &points = *helper.points_ptr;
        geometry::Point3List &normals = helper.normals;
        std::vector<Reference> &references = helper.references;
        //std::cout<<"Compacting.."<<std::endl;
		for(int i = 0;i!=points.size();++i)
		{

			geometry::Point3 vnormal; 
            vnormal.setZero();
            for(int j = 0;j!=references[i].size();++j)
			{
				vnormal += normals[references[i][j].first];
			}
			vnormal.normalize();

            (*helper.normals_ptr)[i] = vnormal;
		}
        //std::cout<<"Compute normals done..."<<std::endl;

    }
    void ComputeVertexNormals(ClusteringHelper &helper)
    {
        std::vector<Eigen::Vector3i> &triangles = *helper.triangles_ptr;
        geometry::Point3List &points = *helper.points_ptr;
        geometry::Point3List normals(triangles.size());
        std::vector<Reference> &references = helper.references;

        geometry::Point3 n,p1,p2,p3;
        
        for( int i = 0;i!= triangles.size(); ++i)
        {
            p1 = points[triangles[i](0)];
            p2 = points[triangles[i](1)];
            p3 = points[triangles[i](2)];
            n = ((p2 - p1).cross(p3 - p1));
            n.normalize();
            normals[i] = n;
        }
        //std::cout<<"Compacting.."<<std::endl;
		for(int i = 0;i!=points.size();++i)
		{

			geometry::Point3 vnormal; 
            vnormal.setZero();
            for(int j = 0;j!=references[i].size();++j)
			{
				vnormal += normals[references[i][j].first];
			}
			vnormal.normalize();

            (*helper.normals_ptr)[i] = vnormal;
		}
        //std::cout<<"Compute normals done..."<<std::endl;

    }
    void CompactMesh(QuadricHelper &helper)
    {
        UpdateMesh(helper);
        
        std::vector<Eigen::Vector3i> &triangles = *helper.triangles_ptr;
        geometry::Point3List &points = *helper.points_ptr;
        std::vector<Reference> &references = helper.references;
        int re_local = 0;
        for(int i = 0;i != points.size(); ++i)
        {
            if(references[i].size() == 0) continue;
            points[re_local] = points[i];
            for(int j = 0;j!=references[i].size();++j)
            {
                triangles[references[i][j].first](references[i][j].second) = re_local;
            }
            if(helper.colors_ptr!=nullptr)
            (*helper.colors_ptr)[re_local] = (*helper.colors_ptr)[i];
            re_local++;
        }
        UpdateReferences(*helper.triangles_ptr, helper.references);
        
        points.resize(re_local);
        if(helper.colors_ptr!=nullptr)
        helper.colors_ptr->resize(re_local);
        if(helper.normals_ptr!=nullptr)
        {
            ComputeVertexNormals(helper);
            helper.normals_ptr->resize(points.size());
        }
    }
    void CompactMesh(ClusteringHelper &helper)
    {
        UpdateMesh(helper);
        
        std::vector<Eigen::Vector3i> &triangles = *helper.triangles_ptr;
        geometry::Point3List &points = *helper.points_ptr;
        std::vector<Reference> &references = helper.references;
        int re_local = 0;
        for(int i = 0;i != points.size(); ++i)
        {
            if(references[i].size() == 0) continue;
            points[re_local] = points[i];
            for(int j = 0;j!=references[i].size();++j)
            {
                triangles[references[i][j].first](references[i][j].second) = re_local;
            }
            if(helper.colors_ptr!=nullptr)
            (*helper.colors_ptr)[re_local] = (*helper.colors_ptr)[i];
            re_local++;
        }
        UpdateReferences(*helper.triangles_ptr, helper.references);
        
        points.resize(re_local);
        if(helper.colors_ptr!=nullptr)
        helper.colors_ptr->resize(re_local);
        if(helper.normals_ptr!=nullptr)
        {
            ComputeVertexNormals(helper);
            helper.normals_ptr->resize(points.size());
        }
    }
    void CompactMesh(PruningHelper &helper)
    {
        UpdateMesh(helper);
        
        std::vector<Eigen::Vector3i> &triangles = *helper.triangles_ptr;
        geometry::Point3List &points = *helper.points_ptr;
        std::vector<Reference> &references = helper.references;
        int re_local = 0;
        for(int i = 0;i != points.size(); ++i)
        {
            if(references[i].size() == 0) continue;
            points[re_local] = points[i];
            for(int j = 0;j!=references[i].size();++j)
            {
                triangles[references[i][j].first](references[i][j].second) = re_local;
            }
            if(helper.colors_ptr!=nullptr)
            (*helper.colors_ptr)[re_local] = (*helper.colors_ptr)[i];
            if(helper.normals_ptr!=nullptr)
            (*helper.normals_ptr)[re_local] = (*helper.normals_ptr)[i];
            re_local++;
        }
        UpdateReferences(*helper.triangles_ptr, helper.references);
        
        points.resize(re_local);
        if(helper.colors_ptr!=nullptr)
        helper.colors_ptr->resize(re_local);
        if(helper.normals_ptr!=nullptr)
        helper.normals_ptr->resize(points.size());
    }
    void InitializeHelper(TriangleMesh &wait_to_simplify, QuadricHelper &helper)
    {
        helper.points_ptr = &wait_to_simplify.points;
        helper.triangles_ptr = &wait_to_simplify.triangles;

        if(wait_to_simplify.HasNormals())
            helper.normals_ptr = &wait_to_simplify.normals;
        else helper.normals_ptr = nullptr;
        
        if(wait_to_simplify.HasColors())
            helper.colors_ptr = &wait_to_simplify.colors;
        else helper.colors_ptr = nullptr;

        helper.t_dirty.resize(helper.triangles_ptr->size(),0);
        helper.t_deleted.resize(helper.triangles_ptr->size(),0);
        helper.QMatrix.resize(helper.points_ptr->size());
        helper.error.resize(helper.triangles_ptr->size(),std::vector<float>(4,0));
        helper.references.resize(helper.points_ptr->size());
        helper.is_border.resize(helper.points_ptr->size(),false);
        helper.normals.resize(helper.triangles_ptr->size());


        UpdateReferences(*helper.triangles_ptr, helper.references);
        //Compute Q Matrix
        ComputeNormalsAndQMatrix(*helper.triangles_ptr, *helper.points_ptr, helper.references, helper.normals, helper.QMatrix);
        //Check if the vertex is border
        CheckIsBorder(*helper.triangles_ptr, helper.references, helper.is_border);
        //compute error for each triangle
        ComputeError(helper);
        //ComputeVertexNormals(helper);

    }

    void InitializeHelper(TriangleMesh &wait_to_simplify, ClusteringHelper &helper)
    {
        helper.points_ptr = &wait_to_simplify.points;
        helper.triangles_ptr = &wait_to_simplify.triangles;

        if(wait_to_simplify.HasNormals())
            helper.normals_ptr = &wait_to_simplify.normals;
        else helper.normals_ptr = nullptr;
        
        if(wait_to_simplify.HasColors())
            helper.colors_ptr = &wait_to_simplify.colors;
        else helper.colors_ptr = nullptr;
        helper.references.resize(helper.points_ptr->size());
        helper.t_deleted.resize(helper.triangles_ptr->size(),0);
    }
    void InitializeHelper(TriangleMesh &wait_to_simplify, PruningHelper &helper)
    {
        helper.points_ptr = &wait_to_simplify.points;
        helper.triangles_ptr = &wait_to_simplify.triangles;

        if(wait_to_simplify.HasNormals())
            helper.normals_ptr = &wait_to_simplify.normals;
        else helper.normals_ptr = nullptr;
        
        if(wait_to_simplify.HasColors())
            helper.colors_ptr = &wait_to_simplify.colors;
        else helper.colors_ptr = nullptr;

        helper.references.resize(helper.points_ptr->size());
        helper.t_deleted.resize(helper.triangles_ptr->size(),0);
        helper.is_visited.resize(helper.points_ptr->size(), false);
        helper.unvisited.clear();
        for(int i = 0; i !=helper.points_ptr->size(); ++i )
        helper.unvisited.insert(i);
        helper.connected_patches.clear();
        UpdateReferences(*helper.triangles_ptr, helper.references);
    }
    //Check Border: if an edge is only possessed by one triangle, it is border.
    void CheckIsBorder(const std::vector<Eigen::Vector3i> &triangles,  const std::vector<Reference> &references, std::vector<bool> &is_border)
    {
        for(int i = 0;i!=references.size();++i)
        {
            std::map<int,int> id_count;
            for(int j = 0;j!=references[i].size();++j)
            {
                Eigen::Vector3i triangle = triangles[references[i][j].first];
                int vid = references[i][j].second;
                int id1 = triangle((vid+1)%3), id2 = triangle((vid+2)%3);
                if(id_count.find(id1) == id_count.end())
                id_count[id1]=1;
                else id_count[id1]+=1;

                if(id_count.find(id2) == id_count.end())
                id_count[id2]=1;
                else id_count[id2]+=1;    
            }
            for(auto iter = id_count.begin();iter!=id_count.end();++iter)
            {
                if(iter->second == 1)
                {
                    is_border[iter->first] = true;
                    is_border[i] = true;
                }
            }
        }
    }
    void ComputeError(QuadricHelper &helper)
    {
        std::vector<std::vector<float>> &error = helper.error;
        for(int i = 0;i!=helper.triangles_ptr->size();++i)
        {
            Eigen::Vector3i &triangle = (*helper.triangles_ptr)[i];
            geometry::Point3 p;
            error[i][0] = ComputeError(helper,triangle(0), triangle(1), p );
            error[i][1] = ComputeError(helper,triangle(1), triangle(2), p );
            error[i][2] = ComputeError(helper,triangle(2), triangle(0), p );
            error[i][3] = std::min(error[i][0], std::min(error[i][1], error[i][2]));
        }
    }

    float ComputeError(QuadricHelper &helper, int i, int j, geometry::Point3 &new_p)
    {
        geometry::Point3 pi = (*helper.points_ptr)[i];
        geometry::Point3 pj = (*helper.points_ptr)[j];

        geometry::Matrix4 Qi = helper.QMatrix[i];
        geometry::Matrix4 Qj = helper.QMatrix[j];
        geometry::Matrix4 new_Q = Qi+Qj;
        geometry::Matrix4 Sysmetrics = new_Q;
        geometry::Vector4 zero_one =  geometry::Vector4(0,0,0,1), homo_new_p;
        Sysmetrics.block<1,4>(3,0) =zero_one;
        

        bool is_border = helper.is_border[i] & helper.is_border[j];

        if(!is_border && Sysmetrics.fullPivLu().isInvertible())
        {
            homo_new_p = Sysmetrics.inverse()*zero_one;
            new_p = homo_new_p.head<3>();
            
            return (homo_new_p.transpose() * new_Q * homo_new_p)(0);

        }
        else
        {
            geometry::Vector4 homo_pi = geometry::Vector4(pi(0), pi(1), pi(2),1);
            geometry::Vector4 homo_pj = geometry::Vector4(pj(0), pj(1), pj(2),1);
            geometry::Vector4 homo_pij =( homo_pi + homo_pj )/2;

            float error_i = (homo_pi.transpose() * new_Q * homo_pi)(0);
            float error_j = (homo_pj.transpose() * new_Q * homo_pj)(0);
            float error_ij = (homo_pij.transpose() * new_Q * homo_pij)(0);
            float min_error = std::min(error_ij, std::min(error_i, error_j));
            if(min_error == error_i) new_p = pi;
            else if(min_error == error_j) new_p = pj;
            else new_p = homo_pij.head<3>();
            return min_error;
        }



    }

    void UpdateReferences(const std::vector<Eigen::Vector3i> &triangles, std::vector<Reference> &references)
    {
        for(int i=0; i<references.size();++i)
        references[i].clear();
        for(int i = 0;i!=triangles.size(); ++i)
        {
            references[triangles[i](0)].push_back(std::make_pair(i, 0)); 
            references[triangles[i](1)].push_back(std::make_pair(i, 1));
            references[triangles[i](2)].push_back(std::make_pair(i, 2));
        }
    }
    void ComputeNormalsAndQMatrix(const std::vector<Eigen::Vector3i> &triangles, const geometry::Point3List &points,
        std::vector<Reference> &references, geometry::Point3List &normals ,std::vector<geometry::Matrix4> &QMatrix)
    {
        std::vector<geometry::Matrix4> plane_matrix;
        geometry::Point3 n,p1,p2,p3;
        float d;
        geometry::Vector4 plane;

        plane_matrix.resize(triangles.size());
        
        for( int i = 0;i!= triangles.size(); ++i)
        {
            p1 = points[triangles[i](0)];
            p2 = points[triangles[i](1)];
            p3 = points[triangles[i](2)];
            n = ((p2 - p1).cross(p3 - p1));
            n.normalize();
            d = - n.dot(p1);
            normals[i] = n;
            plane = geometry::Vector4(n(0), n(1), n(2),d);
            plane_matrix[i] = plane * plane.transpose();
        }
        geometry::Matrix4 tmp; 
        //compute Q matrix by add all the plane matrix.       
        for(int i = 0;i!=points.size();++i)
        {
            tmp.setZero();
            
            for(int j = 0;j!=references[i].size();++j)
            tmp += plane_matrix[references[i][j].first];
            QMatrix[i] = tmp;
        }
    }
    Eigen::Vector3i GetGridIndex(const geometry::Point3 &points, float grid_size)
    {
        return Eigen::Vector3i(std::floor(points(0)/grid_size), std::floor(points(1)/grid_size),std::floor(points(2)/grid_size));
    }
    void ClusteringSimplification(TriangleMesh &wait_to_simplify, float grid_len)
    {
        ClusteringHelper c_helper;
        InitializeHelper(wait_to_simplify,c_helper);
        c_helper.grid_len = grid_len;
        if(grid_len <= 0)
        {
        std::cout<<RED<< "[ClusteringMeshSimplification]::[ERROR]::Grid length cannot be less than 0."<<RESET<<std::endl;
        return;
        }
        std::vector<Eigen::Vector3i> &triangles = *c_helper.triangles_ptr;
        geometry::Point3List &points = *c_helper.points_ptr;
        std::unordered_map<Eigen::Vector3i, std::pair<int, int>, geometry::VoxelGridHasher > &grid_map = c_helper.grid_map;
        auto &grid_to_point = c_helper.grid_to_point;
        for(int i = 0;i!=triangles.size();++i)
        {
            geometry::Point3 p1 = points[triangles[i](0)];
            geometry::Point3 p2 = points[triangles[i](1)];
            geometry::Point3 p3 = points[triangles[i](2)];
            Eigen::Vector3i grid_id1 = GetGridIndex(p1,grid_len);
            Eigen::Vector3i grid_id2 = GetGridIndex(p2,grid_len);
            Eigen::Vector3i grid_id3 = GetGridIndex(p3,grid_len);
            int v1, v2, v3;
            // if(grid_map.find(grid_id1) == grid_map.end())
            //     grid_map[grid_id1].first = triangles[i](0);
            // if(grid_map.find(grid_id2) == grid_map.end())
            //     grid_map[grid_id2].first = triangles[i](1);
            // if(grid_map.find(grid_id3) == grid_map.end())
            //     grid_map[grid_id3].first = triangles[i](2);
            if(grid_map.find(grid_id1) == grid_map.end())
            {
                grid_map[grid_id1].first = triangles[i](0);
                grid_map[grid_id1].second = 1;
                grid_to_point[grid_id1] = p1; 
            }
            else 
            {
                grid_to_point[grid_id1] += p1;
                grid_map[grid_id1].second += 1;
            }
            if(grid_map.find(grid_id2) == grid_map.end())
            {
                grid_map[grid_id2].first = triangles[i](1);
                grid_map[grid_id2].second = 1;
                grid_to_point[grid_id2] = p2;
            }
            else 
            {
                grid_to_point[grid_id2] += p2;
                grid_map[grid_id2].second += 1;
            }
            if(grid_map.find(grid_id3) == grid_map.end())
            {
                grid_map[grid_id3].first = triangles[i](2);
                grid_map[grid_id3].second = 1;
                grid_to_point[grid_id3] = p3;
            }
            else 
            {
                grid_to_point[grid_id3] += p3;
                grid_map[grid_id3].second += 1;
            }

            v1 = grid_map[grid_id1].first;
            v2 = grid_map[grid_id2].first;
            v3 = grid_map[grid_id3].first;

            if(v1 == v2 || v1 == v3 || v2 == v3)
                c_helper.t_deleted[i] = true;
            else
            {
                triangles[i](0) = v1;
                triangles[i](1) = v2;
                triangles[i](2) = v3;
            }
        }
        CompactMesh(c_helper);
        std::cout<<GREEN<< "[ClusteringMeshSimplification]::[INFO]::Simplify done."<<RESET<<std::endl;
    }
    void MeshPruning(TriangleMesh &mesh, int min_points)
    {
        PruningHelper pruning_helper;
        InitializeHelper(mesh, pruning_helper);
        //std::cout<<GREEN<< "[MeshPruning]::[INFO]::Prune mesh done. "<<RESET<<std::endl;
 
        std::vector<bool> &t_deleted = pruning_helper.t_deleted;
        geometry::Point3List &points = *pruning_helper.points_ptr;
        geometry::Point3List &normals = *pruning_helper.normals_ptr;
        geometry::Point3List &colors = *pruning_helper.colors_ptr;
        std::vector<Eigen::Vector3i> &triangles = *pruning_helper.triangles_ptr;
        std::vector<Reference> & references = pruning_helper.references;
        std::set<int> & unvisited = pruning_helper.unvisited;
        std::vector<bool> & is_visited = pruning_helper.is_visited;
        std::vector<std::set<int >> & connected_patches = pruning_helper.connected_patches;
        
        while(unvisited.size() != 0)
        {
            std::queue<int> wait_to_visited;
            int visit_id = *unvisited.begin();
            wait_to_visited.push(visit_id);
            unvisited.erase(visit_id);
            is_visited[visit_id] = true;
            connected_patches.push_back(std::set<int>());
            std::set<int> &current_patch = connected_patches.back();
            current_patch.insert(visit_id);
            
            while(!wait_to_visited.empty())
            {
                //std::cout<<wait_to_visited.size()
                int vid = wait_to_visited.front();
                wait_to_visited.pop();
                Reference & reference = references[vid];
                for(int i = 0; i != reference.size(); ++i)
                {
                    int tid = reference[i].first;
                    int t_vid = reference[i].second;
                    int id1 = triangles[tid]((t_vid+1)%3), id2 = triangles[tid]((t_vid+2)%3);
                    if(is_visited[id1] == false)
                    {
                        is_visited[id1] = true;
                        unvisited.erase(id1);
                        wait_to_visited.push(id1);
                        current_patch.insert(id1);
                    }

                    if(is_visited[id2] == false)
                    {
                        is_visited[id2] = true;
                        unvisited.erase(id2);
                        wait_to_visited.push(id2);
                        current_patch.insert(id2);
                    }
                }
            }
        }
        int prune_count = 0;
        for(int i = 0; i != connected_patches.size(); ++i)
        {
            if(connected_patches[i].size() <= min_points)
            {
                for(auto iter = connected_patches[i].begin(); 
                    iter != connected_patches[i].end(); ++iter)
                {
                    prune_count++;
                    Reference & reference = references[*iter];
                    for(int j = 0; j != reference.size(); ++j)
                    {
                        t_deleted[reference[j].first] = true;
                    }
                }
            }
        }

        CompactMesh(pruning_helper);
        std::cout<<GREEN<< "[MeshPruning]::[INFO]::Prune mesh done. "<<prune_count<<" points are pruned. "<<RESET<<std::endl;
    }


}
}