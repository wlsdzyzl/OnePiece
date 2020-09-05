#include "CubeHandler.h"
#include "Tool/MultiThreads.h"
#include "MarchingCube.h"
#define SINGLE_THREAD 0
namespace fucking_cool
{
namespace integration
{
    void CubeHandler::ExtractTriangleMesh(geometry::TriangleMesh &mesh)
    {
        /* 
        auto iter = cube_map.begin();
        for(int i = 0;i!=cube_map.size()-2; ++i)
        iter++;
        GenerateMeshByCube(iter->first,mesh);
         */
        //
#if SINGLE_THREAD   
        //single thread
        for(auto iter = cube_map.begin(); iter != cube_map.end(); ++iter)
        {
            GenerateMeshByCube(iter->first,mesh);
        }
        std::cout<<GREEN<<"[ExtractTriangleMesh]::[Info]::Finish mesh extraction( single thread)."<<RESET<<std::endl;
#else
        //Multi threads 
        std::vector<CubeID> cube_ids;
        std::vector<geometry::TriangleMesh> meshes;
        for(auto iter = cube_map.begin(); iter != cube_map.end(); ++iter)
        {
            cube_ids.push_back(iter->first);
            meshes.push_back(geometry::TriangleMesh());
        }

        std::function<void(const CubeID &, geometry::TriangleMesh &)> f = 
        [&](const CubeID & cube_id, geometry::TriangleMesh & _mesh)
        {
            GenerateMeshByCube(cube_id, _mesh);
        };
        tool::MultiThreads<CubeID, geometry::TriangleMesh>(cube_ids, meshes, f);
        mesh.LoadFromMeshes(meshes);
        std::cout<<GREEN<<"[ExtractTriangleMesh]::[Info]::Finish mesh extraction( multi-threads)."<<RESET<<std::endl;
#endif    
    }
    std::shared_ptr<geometry::PointCloud> CubeHandler::GetPointCloud() const
    {
        geometry::PointCloud pcd;
        for(auto iter = cube_map.begin(); iter != cube_map.end(); ++iter)
        {
            //std::cout<<iter->first<<std::endl;
            for(int x = 0 ; x != CUBE_SIZE; ++x)
            {
                for(int y = 0 ; y != CUBE_SIZE; ++y)
                {
                    for(int z = 0 ; z != CUBE_SIZE; ++z)
                    {       
                        int voxel_id = x + y * CUBE_SIZE + z * CUBE_SIZE * CUBE_SIZE;
                        if(iter->second.voxels[voxel_id].weight !=0 && std::fabs(iter->second.voxels[voxel_id].sdf) <integrator.truncation)
                        {
                            float fabs_sdf = std::fabs(iter->second.voxels[voxel_id].sdf) / integrator.truncation;
                            pcd.points.push_back(iter->second.GetOrigin(c_para) + c_para.VoxelCentroidOffSet[voxel_id]);
                            pcd.colors.push_back(geometry::Point3(fabs_sdf, fabs_sdf, fabs_sdf));
                        }
                    }
                }
            }
        }   
        return std::make_shared<geometry::PointCloud>(pcd);     
    }
    void CubeHandler::GenerateMeshByCube(const CubeID &cube_id, geometry::TriangleMesh &mesh)
    {
        std::vector<TSDFVoxel> corner_tsdf(8);
        geometry::Point3List corner_voxel(8);

        //std::cout<<cube_id<<std::endl;
        for(int x = 0 ; x != CUBE_SIZE; ++x)
        {
            for(int y = 0 ; y != CUBE_SIZE; ++y)
            {
                for(int z = 0 ; z != CUBE_SIZE; ++z)
                {        
                    int voxel_id = x + y * CUBE_SIZE + z * CUBE_SIZE * CUBE_SIZE;
                    int index = (x == (CUBE_SIZE-1) ) + ((y == (CUBE_SIZE-1))<<1) + ((z == (CUBE_SIZE -1)) << 2);
                    Eigen::Vector3i cube_offset =  c_para.NeighborCubeIDOffset[index];
                    //std::cout<<"CUBE OFFSET: "<<index<<std::endl;
                    bool all_neighbors_observed = true;
                    for( int i = 0;i!= 8; ++i)
                    {
                        Eigen::Vector3i xyz_offset = c_para.CornerXYZOffset.block<3,1>(0,i);
                        Eigen::Vector3i neighbor_cube_id = cube_id +Eigen::Vector3i( xyz_offset(0) & cube_offset(0), xyz_offset(1) & cube_offset(1), xyz_offset(2)&cube_offset(2));
                        Eigen::Vector3i vertex_offset = Eigen::Vector3i((x+xyz_offset(0))%CUBE_SIZE, (y+xyz_offset(1))%CUBE_SIZE, (z+xyz_offset(2))%CUBE_SIZE);
                        int neighbor_voxel_id = vertex_offset(0) + vertex_offset(1) * CUBE_SIZE + vertex_offset(2) * CUBE_SIZE * CUBE_SIZE;                        
                        if(!HasCube(neighbor_cube_id)) 
                        {
                            all_neighbors_observed = false;
                            break;
                        }
                        corner_voxel[i] = cube_map[neighbor_cube_id].GetOrigin(c_para) + c_para.VoxelCentroidOffSet[neighbor_voxel_id]; 
                        corner_tsdf[i] = cube_map[neighbor_cube_id].voxels[neighbor_voxel_id];
                        if(corner_tsdf[i].sdf >= 1) 
                        {
                            all_neighbors_observed = false;
                            break;
                        }
                        
                    }
                    if(all_neighbors_observed)  MarchingCube(corner_voxel, corner_tsdf, mesh);
                }
            }
        }

        return;
    }

    void CubeHandler::ComputeBounding(const cv::Mat &depth, const geometry::TransformationMatrix & pose, geometry::Point3 & max_pos, 
        geometry::Point3 &min_pos)
    {
        //compute points
        //compute boundary id
        
        geometry::PointCloud pcd;
        pcd.LoadFromDepth(depth,camera);
        pcd.Transform(pose);

        auto &points = pcd.points;
        Frustum frustum;
        frustum.ComputeFromCamera(camera, pose,far,near);
        max_pos = geometry::Point3(std::numeric_limits<float>::lowest () ,std::numeric_limits<float>::lowest () ,std::numeric_limits<float>::lowest () );
        min_pos = geometry::Point3(std::numeric_limits<float>::max () ,std::numeric_limits<float>::max () ,std::numeric_limits<float>::max () );
        for(int i = 0;i!=points.size(); ++i)
        {
            //std::cout <<points[i]<<std::endl;
            if(frustum.ContainPoint(points[i]))
            {    
                max_pos(0) = std::max(points[i](0), max_pos(0));
                max_pos(1) = std::max(points[i](1), max_pos(1));
                max_pos(2) = std::max(points[i](2), max_pos(2));                
            
                min_pos(0) = std::min(points[i](0), min_pos(0));
                min_pos(1) = std::min(points[i](1), min_pos(1));
                min_pos(2) = std::min(points[i](2), min_pos(2));
            }
        }
    }

    void CubeHandler::PrepareCubes(const cv::Mat &depth, const geometry::TransformationMatrix &pose, 
        std::vector<PreparedCubeID> &cube_id_list)
    {
        //std::cout<<c_para.VoxelResolution<<std::endl;
        geometry::Point3 max_point, min_point;
        ComputeBounding(depth, pose, max_point, min_point);
        //std::cout << max_point << " " << min_point << std::endl;
        CubeID max_cube_id = GetCubeID(max_point);
        CubeID min_cube_id = GetCubeID(min_point);

        cube_id_list.clear();
        std::vector<int> voxel_index
             {0, CUBE_SIZE -1, (CUBE_SIZE-1)*CUBE_SIZE, (CUBE_SIZE-1)*CUBE_SIZE + CUBE_SIZE-1,
            CUBE_SIZE * CUBE_SIZE * (CUBE_SIZE-1), CUBE_SIZE * CUBE_SIZE * (CUBE_SIZE-1) + CUBE_SIZE-1,
            CUBE_SIZE * CUBE_SIZE * (CUBE_SIZE-1) + CUBE_SIZE * (CUBE_SIZE-1),
            CUBE_SIZE * CUBE_SIZE * (CUBE_SIZE-1) + CUBE_SIZE * (CUBE_SIZE-1) + CUBE_SIZE-1};
        
        float cube_resolution = c_para.VoxelResolution * CUBE_SIZE;
        for(int i = min_cube_id(0)-1; i<= max_cube_id(0)+1; ++i)
        {
            for(int j = min_cube_id(1)-1; j<= max_cube_id(1)+1; ++j)
            {
                for(int k = min_cube_id(2)-1; k<= max_cube_id(2)+1; ++k)
                {
                    float min_sdf = std::numeric_limits<float>::max();
                    for(int c = 0; c!= 8; ++c)
                    {
                        geometry::Point3 voxel = geometry::Point3(i*cube_resolution, j*cube_resolution, 
                            k*cube_resolution) + c_para.VoxelCentroidOffSet[voxel_index[c]];
                        float sdf = integrator.GetSDF(voxel, camera, pose, depth);
                        if(min_sdf > std::fabs(sdf))
                            min_sdf =std::fabs(sdf);
                    }

                    if(min_sdf < integrator.truncation)
                    {
                        CubeID cube_id = CubeID(i,j,k);
                        bool is_new_cube=false;
                        if(cube_map.find(cube_id) == cube_map.end())
                        {
                            is_new_cube == true;
                            cube_map[cube_id] = VoxelCube(cube_id);
                        }
                        cube_id_list.push_back(std::make_tuple(cube_id, is_new_cube, false));
                    }
                    //std::cout<<k<<std::endl;
                }
            }
        }
    }
    void CubeHandler::IntegrateImage(const cv::Mat &depth, const cv::Mat &rgb, const geometry::TransformationMatrix & pose)
    {
        
        std::vector<PreparedCubeID> cube_id_list;
        PrepareCubes(depth, pose, cube_id_list);
#if DEBUG_MODE
        std::cout<<BLUE<<"[PrepareCubes]::[DEBUG]::Number of Candidate Cubes: "<<cube_id_list.size() <<RESET<<std::endl;
#endif
        for(int i = 0; i!= cube_id_list.size(); ++i)
        {
            integrator.IntegrateImage(depth, rgb,  pose, camera, cube_map[std::get<0>(cube_id_list[i])], c_para);
        }
        std::cout<<GREEN<<"[IntegrateImage]::[Info]::Finish image integration."<<RESET<<std::endl;
    }
    void CubeHandler::IntegrateImage(const geometry::RGBDFrame &rgbd, const geometry::TransformationMatrix & pose)
    {
        IntegrateImage(rgbd.depth, rgbd.rgb, pose);
    }
}
}