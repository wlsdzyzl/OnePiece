#include "Geometry/Geometry.h"
#include "Integration/CubeHandler.h"
#include "Visualization/Visualizer.h"
#include "Tool/TickTock.h"
#include "Tool/ImageProcessing.h"
#include "Tool/IO.h"
#include "Tool/CppExtension.h"
#include <random>

using namespace fucking_cool;

int main(int argc, char ** argv)
{
    // this code is to generate an incomplete model by setting the local rate, and global rate
    // usage: generate incomplete model
    // scannet dataset: guoqing@svr23:/hdd10T/lhanaf/scannet_data/train
    std::default_random_engine e;
    e.seed(time(0));
    std::uniform_real_distribution<float> u(0.0, 1.0);
    float global_rate = 0.7;
    float local_rate = 0.7;
    if(argc < 3)
    {
        std::cout<<"Usage: GenerateIncompleteModel [file_path] [voxel_resolution] [global_rate = 0.7] [local_rate = 0.7]"<<std::endl;
        return 0;
    }
    float voxel_resolution = atof(argv[2]);
    if(argc > 3)
    {
        global_rate = atof(argv[3]);
        local_rate = atof(argv[4]);
    }
    //generated 3D mesh
    geometry::TriangleMesh mesh;
    //pinhole camera
    camera::PinholeCamera depth_camera;
    camera::PinholeCamera color_camera;
    std::vector<std::string> rgb_files, depth_files;
    std::vector<geometry::TransformationMatrix> poses;
    std::string file_path = argv[1];
    tool::ReadImageSequenceFromScannetWithPose(argv[1], rgb_files, depth_files, poses, 
        color_camera, depth_camera);

    std::cout<<"Read "<<rgb_files.size() <<" frames..."<<std::endl;
    std::cout<<"Depth Intrinsics:\n"<<depth_camera.ToCameraMatrix()<<std::endl;
    std::cout<<"Color Intrinsics:\n"<<color_camera.ToCameraMatrix()<<std::endl;
    integration::CubeHandler cube_handler(depth_camera);
    cube_handler.SetVoxelResolution(voxel_resolution); 

    visualization::Visualizer visualizer;
    cube_handler.SetTruncation(0.15);
    // according to global rate to choose some frames.
    int frame_used_for_reconstruction = rgb_files.size() * global_rate;

    int start =  u(e) * (rgb_files.size() - frame_used_for_reconstruction);

    for(int i = 0; i != frame_used_for_reconstruction; ++i )
    {
        int frame_id = i + start;
        cv::Mat refined_depth;
        cv::Mat filtered_depth;
        std::cout <<"Processing on "<<frame_id <<"th image"<<std::endl;
        std::cout<<rgb_files[i]<<"\n"<<depth_files[i]<<std::endl;
        cv::Mat rgb = cv::imread(rgb_files[frame_id]);
        cv::Mat depth = cv::imread(depth_files[frame_id],-1);
        //std::cout<<depth.depth()<<" "<<CV_16UC1<<std::endl; 
        tool::ConvertDepthTo32F(depth, refined_depth, depth_camera.GetDepthScale());
        cv::Mat aligned_rgb = tool::AlignColorToDepth(rgb, refined_depth, color_camera, depth_camera);
        // cv::imshow("aligned_color", aligned_rgb);
        // cv::waitKey(0);

        
        
        tool::BilateralFilter(refined_depth, filtered_depth);
        cube_handler.IntegrateImage(filtered_depth,aligned_rgb,poses[frame_id]);        
    }
    // according to local rate to delete some cubes
    auto hashing_map = cube_handler.GetCubeMap();
    int deleted_cube_size = hashing_map.size() - hashing_map.size() * local_rate;

    std::vector<int> cube_id_1d(hashing_map.size());

    std::vector<integration::CubeID> cube_id_3d(hashing_map.size());

    {
        auto iter = hashing_map.begin();
        for(int i = 0; i != cube_id_1d.size(); ++i)
        {
            cube_id_1d[i] = i;
            cube_id_3d[i] = iter->first;
            iter ++;
        }
    }
    // disorder the map
    for(int i = 0; i != cube_id_1d.size(); ++i)
    {
        int random_index = u(e) * cube_id_1d.size();
        if(random_index >= cube_id_1d.size()) random_index = cube_id_1d.size() - 1;
        std::swap<int>(cube_id_1d[i], cube_id_1d[random_index]);
    }

    for(int i = 0; i != deleted_cube_size; ++i)
    {
        hashing_map.erase(cube_id_3d[cube_id_1d[i]]);
    }

    cube_handler.SetCubeMap(hashing_map);
    cube_handler.ExtractTriangleMesh(mesh);
    auto c_mesh_ptr = mesh.ClusteringSimplify(voxel_resolution);
    auto strs = tool::RSplit(file_path, "/", 1);
    if(strs.size() == 2)
    { 
        c_mesh_ptr->WriteToPLY(strs[0] +"/incomplete_models/"+strs[1]+".ply");
    }
    else
    {
        std::cout<<"Something wrong when parsing the path"<<std::endl;
    }
    return 0;
}