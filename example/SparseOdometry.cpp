#include "Odometry/Odometry.h"
#include "Geometry/Geometry.h"
#include "Geometry/PointCloud.h"
#include "Visualization/Visualizer.h"
#include <string>
#include "Tool/TickTock.h"
using namespace one_piece;
int main(int argc, char** argv)
{
    if(argc < 5)
    {
        std::cout<<"Usage: SparseOdometry [source_rgb] [source_depth] [target_rgb] [targe_depth]"<<std::endl;
        return 0;
    }
    camera::PinholeCamera camera;
    camera.SetCameraType(camera::CameraType::OPEN3D_DATASET);
    odometry::Odometry rgbd_odometry(camera);
    
    tool::Timer timer;
    cv::Mat source_rgb = cv::imread(argv[1]);
    cv::Mat source_depth = cv::imread(argv[2],-1);
    cv::Mat target_rgb = cv::imread(argv[3]);
    cv::Mat target_depth = cv::imread(argv[4],-1);
    geometry::Matrix4 T = geometry::Matrix4::Identity();
    geometry::PointCorrespondenceSet PCS;
    bool is_success;
    timer.TICK("Spase Estimation");
    auto tracking_result = rgbd_odometry.SparseTracking(source_rgb, target_rgb,source_depth, target_depth);
    is_success = tracking_result->tracking_success;
    T = tracking_result->T;
    float rmse = tracking_result->rmse;
    timer.TOCK("Spase Estimation");
    timer.LogAll();
    geometry::PointCloud pcd_source, pcd_target;
    pcd_source.LoadFromRGBD(source_rgb,source_depth,camera);
    pcd_target.LoadFromRGBD(target_rgb,target_depth,camera);

    pcd_source.Transform(T);
    visualization::Visualizer visualizer;
    visualizer.AddPointCloud(pcd_source);
    visualizer.AddPointCloud(pcd_target);
    visualizer.Show();
    if(is_success)
    std::cout << "Successful Matching!"<<std::endl;
    else 
    std::cout << "Bad Matching!"<<std::endl;
    std::cout<<"RMSE: "<<rmse<<std::endl;
    std::cout<<T<<std::endl;

}