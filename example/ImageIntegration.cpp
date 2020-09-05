#include "Geometry/Geometry.h"
#include "Integration/CubeHandler.h"
#include "Visualization/Visualizer.h"
#include "Tool/TickTock.h"
#include "Tool/ImageProcessing.h"
using namespace fucking_cool;
int main(int argc, char *argv[])
{
    if (argc < 3)
    {
        std::cout << "usage::MCGenerateMesh [rgb_path] [depth_path]" << std::endl;
        return 0;
    }
    geometry::TriangleMesh mesh;
    tool::Timer timer;
    //integration::InitializeVoxelCube();
    integration::CubeHandler cube_handler;
    cv::Mat rgb = cv::imread(argv[1]);
    cv::Mat depth = cv::imread(argv[2],-1);
    cv::Mat refined_depth;
    cv::Mat filtered_depth;
    camera::PinholeCamera camera;

    tool::ConvertDepthTo32F(depth, refined_depth, camera.GetDepthScale());
    //filtered_depth = refined_depth;
    tool::BilateralFilter(refined_depth, filtered_depth);

    cube_handler.IntegrateImage(filtered_depth,rgb,geometry::Matrix4::Identity());
    
    integration::Frustum frustum;
    geometry::PointCloud pcd;


    frustum.ComputeFromCamera(camera, geometry::Matrix4::Identity(), 4, 0.5);
    auto frustum_p = frustum.GetPointCloud();
    auto tsdf_p = cube_handler.GetPointCloud();
    pcd.LoadFromRGBD(rgb,depth,camera);
    timer.TICK("Extract Triangle Mesh");
    cube_handler.ExtractTriangleMesh(mesh);
    timer.TOCK("Extract Triangle Mesh");
    timer.TICK("Compute Normals");
    if(!mesh.HasNormals())
    {
        std::cout << "Mesh don't have normals, estimate normals..."<<std::endl;
        mesh.ComputeNormals();
    }
    mesh.WriteToPLY("./image_integration.ply");
    timer.TOCK("Compute Normals");
    //auto mesh_pcd = mesh.GetPointCloud();
    //cube_handler.WriteToFile("./607.cubes");
    timer.LogAll();
    timer.Reset();
    visualization::Visualizer visualizer;
    visualizer.DrawPhongRendering();
    //mesh_pcd->normals.clear();  
    visualizer.AddTriangleMesh(mesh);
    //visualizer.AddPointCloud(pcd);
    //visualizer.AddPointCloud(*frustum_p);
    //visualizer.AddPointCloud(*tsdf_p);
    visualizer.Show();  

    
  
    
    return 0;
}