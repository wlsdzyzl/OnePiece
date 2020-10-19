#include "Integration/CubeHandler.h"
#include "Visualization/Visualizer.h"
#include "DenseSlam.h"
#include <string>
#include "Tool/TickTock.h"
#include "Tool/IO.h"
using namespace one_piece;

int main(int argc, char **argv)
{
    if(argc < 3)
    {
        std::cout<<"Usage: [basepath] [voxel resolution]"<<std::endl;
        return 0;
    }
    std::string base_path = std::string(argv[1]);
    float voxel_resolution = atof(argv[2]);
    geometry::TriangleMesh mesh;
    tool::Timer timer;
    //integration::InitializeVoxelCube();
    camera::PinholeCamera camera;
    camera.SetCameraType(camera::CameraType::OPEN3D_DATASET);
    //camera.SetCameraType(camera::CameraType::TUM_DATASET);


    std::vector<std::string> rgb_files, depth_files;

    std::vector<optimization::Correspondence> global_correspondences;

    tool::ReadImageSequence(base_path,rgb_files, depth_files);
    // rgb_files.resize(150);
    // depth_files.resize(150);
    visualization::Visualizer visualizer;
    visualizer.Initialize();

    DenseSlam dense_slam(camera);
    for(size_t frame_id = 0; frame_id < rgb_files.size(); ++frame_id)
    {
        cv::Mat rgb = cv::imread(rgb_files[frame_id]);
        cv::Mat depth = cv::imread(depth_files[frame_id],-1);
        //std::cout<<rgb_files[frame_id]<<std::endl;
        geometry::RGBDFrame frame(rgb, depth);
        
        dense_slam.UpdateFrame(frame);
        //Add Point and camera to visualizer.
        if(dense_slam.visible_pcd_updated)
        {
            auto pcd_ptr = dense_slam.GetPosedPCD();
            visualizer.Reset();
            visualizer.AddPointCloud(*pcd_ptr);
            dense_slam.visible_pcd_updated = false;
        }
        geometry::SE3List camera_poses;
        camera_poses.push_back(dense_slam.global_poses.back());

        geometry::Point3List camera_colors;
        camera_colors.push_back( geometry::Point3(0, 1, 0));

        visualizer.AddCameraSet(camera_poses, camera_colors);
   
        visualizer.ShowOnce();
    
    }
    if(!dense_slam.submaps.back().is_registered)
    {
        if(dense_slam.submaps.back().containted_frames.size() > 10)
        {
            dense_slam.submaps.back().GenerateSubmapModel(dense_slam.global_frames, 
                dense_slam.relative_poses, dense_slam.camera);
            dense_slam.RegisterSubmap(dense_slam.submaps.back().submap_id);
        }
    }
    dense_slam.Optimize();
    //update all the poses
    dense_slam.UpdateAllPoses();

    integration::CubeHandler cube_handler(camera);
    cube_handler.SetVoxelResolution(voxel_resolution);
    
    std::ofstream ofs(base_path +"/trajectory.txt");
    for(size_t i = 0; i < dense_slam.global_frames.size() ; ++i)
    {
        // int keyframe_id = dense_slam.keyframe_ids[i];
        if(!dense_slam.global_frames[i].tracking_success) continue;
        if(i % 8 == 0)
        {
            cv::Mat rgb = cv::imread(rgb_files[i]);
            cv::Mat depth = cv::imread(depth_files[i],-1);
            cv::Mat refined_depth;
            cv::Mat filtered_depth;
            std::cout <<"Processing on "<<i <<"th image"<<std::endl;
            tool::ConvertDepthTo32F(depth, refined_depth, camera.GetDepthScale());
            tool::BilateralFilter(refined_depth, filtered_depth);
            cube_handler.IntegrateImage(filtered_depth,
                rgb, dense_slam.global_poses[i]);
            //break;
        }   
        ofs<<dense_slam.global_poses[i](0,0)<<" "<<dense_slam.global_poses[i](0,1)<<" "<<dense_slam.global_poses[i](0,2)<<" "<<dense_slam.global_poses[i](0,3)<<" "
           <<dense_slam.global_poses[i](1,0)<<" "<<dense_slam.global_poses[i](1,1)<<" "<<dense_slam.global_poses[i](1,2)<<" "<<dense_slam.global_poses[i](1,3)<<" "
           <<dense_slam.global_poses[i](2,0)<<" "<<dense_slam.global_poses[i](2,1)<<" "<<dense_slam.global_poses[i](2,2)<<" "<<dense_slam.global_poses[i](2,3)<<" "
           <<dense_slam.global_poses[i](3,0)<<" "<<dense_slam.global_poses[i](3,1)<<" "<<dense_slam.global_poses[i](3,2)<<" "<<dense_slam.global_poses[i](3,3)<<std::endl;
    }  
    ofs.close(); 
    cube_handler.ExtractTriangleMesh(mesh);
    auto c_mesh = mesh.ClusteringSimplify(voxel_resolution);
    c_mesh->WriteToPLY("./densefusion_generated_mesh.ply");
    //visualizer.DrawPhongRendering();
    visualizer.AddTriangleMesh(*c_mesh);
    //visualizer.AddPointCloud(*tsdf_pcd);
    visualizer.Show();  
    return 1;    
    
}