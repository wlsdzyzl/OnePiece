
#include "Integration/CubeHandler.h"
#include "Visualization/Visualizer.h"
#include "FBASlam.h"
#include <string>
#include "Tool/TickTock.h"
#include "Tool/IO.h"
using namespace fucking_cool;

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
    // rgb_files.resize(500);
    // depth_files.resize(500);
    visualization::Visualizer visualizer;
    visualizer.Initialize();

    FBASlam fba_slam(camera);
    for(size_t frame_id = 0; frame_id < rgb_files.size(); ++frame_id)
    {
        cv::Mat rgb = cv::imread(rgb_files[frame_id]);
        cv::Mat depth = cv::imread(depth_files[frame_id],-1);
        //std::cout<<rgb_files[frame_id]<<std::endl;
        geometry::RGBDFrame frame(rgb, depth);
        
        fba_slam.UpdateFrame(frame);

        //Add Point and camera to visualizer.
        if(fba_slam.global_frames.back().keyframe_kid != -1)
        {
            auto pcd_ptr = fba_slam.GetPosedPCD();
            visualizer.Reset();
            visualizer.AddPointCloud(*pcd_ptr);
        }
        // geometry::SE3List camera_poses = fba_slam.global_keyframe_poses;
        // camera_poses.push_back(fba_slam.global_poses.back());

        // geometry::Point3List camera_colors(camera_poses.size(), geometry::Point3(1, 0, 0));
        // camera_colors.back() = geometry::Point3(0, 1, 0);
        // visualizer.AddCameraSet(camera_poses, camera_colors);
        visualizer.SetModelViewMatrix(fba_slam.global_poses.back());
        visualizer.ShowOnce();
    }
    
    //update all the poses
    fba_slam.UpdateAllPoses();

    integration::CubeHandler cube_handler(camera);
    cube_handler.SetVoxelResolution(voxel_resolution);
    
    std::ofstream ofs(base_path +"/trajectory.txt");
    for(size_t i = 0; i < fba_slam.global_frames.size() ; ++i)
    {
        // int keyframe_id = fba_slam.keyframe_ids[i];
        if(!fba_slam.global_frames[i].tracking_success) continue;
        if(i % 10 == 0)
        {

            cv::Mat refined_depth;
            cv::Mat filtered_depth;
            std::cout <<"Processing on "<<i <<"th image"<<std::endl;
            tool::ConvertDepthTo32F(fba_slam.global_frames[i].depth, refined_depth, camera.GetDepthScale());
            tool::BilateralFilter(refined_depth, filtered_depth);
            cube_handler.IntegrateImage(filtered_depth,
                fba_slam.global_frames[i].rgb, fba_slam.global_poses[i]);
            //break;
        }   
        ofs<<fba_slam.global_poses[i](0,0)<<" "<<fba_slam.global_poses[i](0,1)<<" "<<fba_slam.global_poses[i](0,2)<<" "<<fba_slam.global_poses[i](0,3)<<" "
           <<fba_slam.global_poses[i](1,0)<<" "<<fba_slam.global_poses[i](1,1)<<" "<<fba_slam.global_poses[i](1,2)<<" "<<fba_slam.global_poses[i](1,3)<<" "
           <<fba_slam.global_poses[i](2,0)<<" "<<fba_slam.global_poses[i](2,1)<<" "<<fba_slam.global_poses[i](2,2)<<" "<<fba_slam.global_poses[i](2,3)<<" "
           <<fba_slam.global_poses[i](3,0)<<" "<<fba_slam.global_poses[i](3,1)<<" "<<fba_slam.global_poses[i](3,2)<<" "<<fba_slam.global_poses[i](3,3)<<std::endl;
    }  
    ofs.close(); 
    cube_handler.ExtractTriangleMesh(mesh);
    auto c_mesh = mesh.ClusteringSimplify(voxel_resolution);
    c_mesh->WriteToPLY("./fbafusion_generated_mesh.ply");
    visualizer.DrawPhongRendering();
    visualizer.AddTriangleMesh(*c_mesh);
    //visualizer.AddPointCloud(*tsdf_pcd);
    visualizer.Show();  
    return 1;    
    
}
