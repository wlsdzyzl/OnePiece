#include "Tool/OpenNIReader.h"
#include "Odometry/Odometry.h"
#include "Geometry/Geometry.h"
#include "Geometry/PointCloud.h"
#include "Visualization/Visualizer.h"
#include <fstream>
#include <iostream>
using namespace fucking_cool;

int main()
{
    tool::OpenNIReader reader;
    reader.Init();
    cv::Mat rgb, depth, last_rgb, last_depth;
    int i = 0;
    std::ofstream ofs_trajectory("data/trajectory.txt");
    std::ofstream ofs_associate("data/associate.txt");
    camera::PinholeCamera camera;
    camera.SetCameraType(camera::CameraType::OPEN3D_DATASET);
    odometry::Odometry rgbd_odometry(camera);
    geometry::Matrix4 T = geometry::Matrix4::Identity(), relative_T = geometry::Matrix4::Identity();
    geometry::PointCorrespondenceSet PCS;
    bool is_success;
    visualization::Visualizer visualizer;
    while(i < 20)
    {
        reader.GetNextRGBD(rgb, depth);
        cv::imwrite("data/rgb/"+std::to_string(i)+".png", rgb);
        cv::imwrite("data/depth/"+std::to_string(i)+".png", depth);
        std::cout<<i<<std::endl;
        ofs_associate<<i<<" rgb/"<<i<<".png "<<i<<" depth/"<<i<<".png"<<std::endl;
        if(i > 0)
        {
            //is_success = rgbd_odometry.DenseTracking(last_rgb, rgb, 
              //  last_depth, depth, relative_T,PCS,1);
            auto tracking_result = rgbd_odometry.SparseTrackingMILD(last_rgb, rgb, 
                last_depth, depth);
            bool is_success = tracking_result->tracking_success;
            relative_T = tracking_result->T;
            if(is_success)
                T = T * relative_T.inverse();
            else
            {
                std::cout<<"Fail to estimate pose."<<std::endl;
                break;
            }/*
            geometry::PointCloud pcd_source, pcd_target;
            pcd_source.LoadFromRGBD(last_rgb,last_depth,camera);
            pcd_target.LoadFromRGBD(rgb,depth,camera);

            pcd_source.Transform(T);

            visualizer.AddPointCloud(pcd_source);
            visualizer.AddPointCloud(pcd_target);
            visualizer.Show();
            visualizer.Reset();
            */
        }
        ofs_trajectory<<T(0,0)<<" "<<T(0,1)<<" "<<T(0,2)<<" "<<T(0,3)
            <<" "<<T(1,0)<<" "<<T(1,1)<<" "<<T(1,2)<<" "<<T(1,3)
            <<" "<<T(2,0)<<" "<<T(2,1)<<" "<<T(2,2)<<" "<<T(2,3)
            <<" "<<T(3,0)<<" "<<T(3,1)<<" "<<T(3,2)<<" "<<T(3,3)<<std::endl;

        last_rgb = rgb;
        last_depth = depth;

        i++;
    }
    ofs_associate.close();
    ofs_trajectory.close();

    reader.Close();
    return 0;
}

