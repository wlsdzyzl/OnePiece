#include "Geometry/Geometry.h"
#include "Integration/CubeHandler.h"
#include "Tool/TickTock.h"
#include "Tool/ImageProcessing.h"
#include "Tool/IO.h"
#include "Tool/CppExtension.h"
#include <random>

using namespace fucking_cool;

int main(int argc, char ** argv)
{
    // this code is to generate a model from scannet

    if(argc < 3)
    {
        std::cout<<"Usage: GenerateModelFromScannet [file_path] [voxel_resolution] "<<std::endl;
        return 0;
    }
    float voxel_resolution = atof(argv[2]);

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
    cube_handler.SetFarPlane(3);
    cube_handler.SetTruncation(0.15);
    // according to global rate to choose some frames.
// #if LOCAL_SAMPLE == 2
//     if(local_rate < 1)
//     {
//         tool::ShuffleVector<>(filtered_index);
//         filtered_index.resize(local_rate * filtered_index.size());
//     }
// #endif
    for(int i = 0; i != rgb_files.size(); ++i) 
    { 
        if(i % 10 != 0) continue;
        int frame_id = i;
        cv::Mat refined_depth;
        cv::Mat filtered_depth;
        std::cout <<"Processing on "<<i<<"th image"<<std::endl;
        std::cout<<rgb_files[frame_id]<<"\n"<<depth_files[frame_id]<<std::endl;
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