#include "Odometry/Odometry.h"
#include "Geometry/Geometry.h"
#include "Geometry/PointCloud.h"
#include "Visualization/Visualizer.h"
#include <string>
using namespace one_piece;
int main(int argc, char** argv)
{
    if(argc < 5)
    {
        std::cout<<"Usage: DenseOdometry [source_rgb] [source_depth] [target_rgb] [targe_depth]"<<std::endl;
        return 0;
    }
    camera::PinholeCamera camera;
    camera.SetCameraType(camera::CameraType::OPEN3D_DATASET);
    odometry::Odometry rgbd_odometry(camera);
    
    
    cv::Mat source_rgb = cv::imread(argv[1]);
    cv::Mat source_depth = cv::imread(argv[2],-1);
    cv::Mat target_rgb = cv::imread(argv[3]);
    cv::Mat target_depth = cv::imread(argv[4],-1);
    geometry::RGBDFrame source_frame(source_rgb, source_depth);
    geometry::RGBDFrame target_frame(target_rgb, target_depth);
    geometry::Matrix4 T = geometry::Matrix4::Identity();
    geometry::PointCorrespondenceSet PCS;
    bool is_success;
    //rgbd_odometry.SparseTracking(source_rgb, target_rgb,source_depth, target_depth,T,PCS);
    auto tracking_result = rgbd_odometry.DenseTracking(source_frame, target_frame, T, 0);
    is_success = tracking_result->tracking_success;
    T = tracking_result->T;
    geometry::PointCloud pcd_source, pcd_target;
    pcd_source.LoadFromRGBD(source_frame,camera);
    pcd_target.LoadFromRGBD(target_frame,camera);

    pcd_source.Transform(T);
    visualization::Visualizer visualizer;
    visualizer.AddPointCloud(pcd_source);
    visualizer.AddPointCloud(pcd_target);
    visualizer.Show();
    if(is_success)
    std::cout << "Successful Matching!"<<std::endl;
    else 
    std::cout << "Bad Matching!"<<std::endl;
    std::cout<<T<<std::endl;

}