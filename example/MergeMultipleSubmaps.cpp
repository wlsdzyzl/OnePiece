#include "Integration/CubeHandler.h"
#include "Visualization/Visualizer.h"
#include "Tool/TickTock.h"
using namespace fucking_cool;

int main(int argc, char **argv)
{
    if(argc < 3) 
    {
        std::cout<<"usage: [map_path] [pose_path]"<<std::endl;
        return 0;
    }
    std::string map_path = argv[1];
    std::string pose_path = argv[2];
    integration::CubeHandler cube_handler;
    size_t submap_count;
    std::vector<geometry::TransformationMatrix> relative_poses;
    std::ifstream ifs(pose_path);
    ifs>>submap_count;
    relative_poses.resize(submap_count);
    geometry::Matrix4 tmp_pos = geometry::Matrix4::Zero();
    for(size_t i = 0; i != relative_poses.size(); ++i)
    {
        int submap_id;
        ifs>>submap_id;
        assert(submap_id == i);
        ifs>>tmp_pos(0,0)>>tmp_pos(0,1)>>tmp_pos(0,2)>>
            tmp_pos(1,0)>>tmp_pos(1,1)>>tmp_pos(1,2)>>
            tmp_pos(2,0)>>tmp_pos(2,1)>>tmp_pos(2,2)>>
            tmp_pos(0,3)>>tmp_pos(1,3)>>tmp_pos(2,3);
        tmp_pos(3,3) = 1.0;
        relative_poses[i] = tmp_pos;
    }    
    for(size_t i = 0; i < submap_count; ++i)
    {
        std::cout<<"merge "<<i<<"th submap..."<<std::endl;
        integration::CubeHandler tmp_cube_handler;
        std::string tmp_path = map_path + "/m"+std::to_string(i)+".map";
        tmp_cube_handler.ReadFromFileFloat(tmp_path);
        auto transformed_handler = tmp_cube_handler.Transform(relative_poses[i]);
        cube_handler.Merge(*transformed_handler);
    }
    geometry::TriangleMesh mesh;
    cube_handler.ExtractTriangleMesh(mesh);
    auto mesh_ptr = mesh.ClusteringSimplify(0.01);
    if(!mesh_ptr->HasNormals()) mesh_ptr->ComputeNormals();
    mesh_ptr->WriteToPLY("./merged_mesh.ply");
    return 0;
}