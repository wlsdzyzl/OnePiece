#include "Geometry/Geometry.h"
#include "Integration/CubeHandler.h"
#include "Visualization/Visualizer.h"
#include "Tool/TickTock.h"
#include "Tool/ImageProcessing.h"
#include "Tool/IO.h"
using namespace fucking_cool;
int main(int argc, char *argv[])
{
    if (argc < 2)
    {
        std::cout << "usage::MCGenerateMesh [dataset_path]" << std::endl;
        return 0;
    }
    geometry::TriangleMesh mesh;
    tool::Timer timer;
    //integration::InitializeVoxelCube();
    
    camera::PinholeCamera camera;
    integration::CubeHandler cube_handler(camera);
    cube_handler.SetVoxelResolution(0.00625);
    std::vector<std::string> rgb_files, depth_files;
    std::vector<geometry::TransformationMatrix> poses;
    tool::ReadImageSequenceWithPose(argv[1],rgb_files, depth_files,poses);
    visualization::Visualizer visualizer;
    //cube_handler.SetTruncation(0.15);
    for(int i = 0; i < poses.size() ; ++i)
    {
        if(i%10 == 0)
        {
            cv::Mat rgb = cv::imread(rgb_files[i]);
            cv::Mat depth = cv::imread(depth_files[i],-1);
            cv::Mat refined_depth;
            cv::Mat filtered_depth;
            std::cout <<"Processing on "<<i <<"th image"<<std::endl;
            tool::ConvertDepthTo32F(depth, refined_depth, camera.GetDepthScale());
            tool::BilateralFilter(refined_depth, filtered_depth);
            cube_handler.IntegrateImage(filtered_depth,rgb,poses[i]);

            //break;
        }   
    }
    //auto tsdf_p = cube_handler.GetPointCloud();

    std::cout<<BLUE<<"Transform the voxels ..."<<RESET<<std::endl;
    

    auto transformed_cube_handler = cube_handler.TransformNearest(poses[poses.size() / 2]);
    std::cout<<BLUE<<"Transform down."<<RESET<<std::endl;
    //auto tsdf_pcd = transformed_cube_handler->GetPointCloud();
    timer.TICK("Extract Triangle Mesh");
    //cube_handler.ExtractTriangleMesh(mesh);
    transformed_cube_handler->ExtractTriangleMesh(mesh);
    timer.TOCK("Extract Triangle Mesh");
    //if(!mesh.HasNormals()) mesh.ComputeNormals();
    auto clustered_mesh = mesh.ClusteringSimplify(0.00625);

    clustered_mesh->WriteToPLY("./image_integration.ply");
    //q_pcd->WriteToPLY("./image_integration.ply");

    timer.LogAll();
    timer.Reset();

    //visualizer.DrawPhongRendering();
    visualizer.AddTriangleMesh(*clustered_mesh);
    //visualizer.AddPointCloud(*tsdf_pcd);
    visualizer.Show();  
    
  
    
    return 0;
}
