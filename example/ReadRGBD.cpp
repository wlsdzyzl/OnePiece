#include "Geometry/PointCloud.h"
#include "Visualization/Visualizer.h"
using namespace fucking_cool;
int main(int argc, char* argv[]) 
{
    if(argc != 3)
    {
        std::cout << "Usage: ReadRGBD [color] [depth]"<<std::endl;
        return 0;
    }
    camera::PinholeCamera camera;
    camera.SetCameraType(camera::CameraType::OPEN3D_DATASET); 
    cv::Mat rgb = cv::imread(argv[1]);
    cv::Mat depth = cv::imread(argv[2],-1);
    geometry::PointCloud pcd;
    pcd.LoadFromRGBD(rgb, depth,camera);

    pcd.WriteToPLY("0.ply");
    visualization::Visualizer visualizer;
    visualizer.AddPointCloud(pcd);
    visualizer.Show();
    return 0;
}