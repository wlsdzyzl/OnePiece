#ifndef KEYFRAME_BASED_SLAM_H
#define KEYFRAME_BASED_SLAM_H

#include "Camera/Camera.h"
#include "Odometry/Odometry.h"
#include "Geometry/Geometry.h"
#include "Geometry/PointCloud.h"
#include "LCDetection/MildLCDetector.h"
#include "Optimization/Optimizer.h"

namespace fucking_cool
{
namespace tool
{
    class KeyframeBasedSlam
    {
        public:
        KeyframeBasedSlam() = default;
        KeyframeBasedSlam(const camera::PinholeCamera &_camera):camera(_camera)
        {
            rgbd_odometry.SetCamera(camera);
        }
        virtual void UpdateFrame(const geometry::RGBDFrame &frame) = 0;

        geometry::PointCloudPtr GetPosedPCD()
        {
            geometry::PointCloud global_pcd;
            for(size_t i = 0; i != keyframe_ids.size(); ++i)
            {
                auto pcd = global_frames[keyframe_ids[i]].down_sampled_pcd;
                pcd.Transform(global_poses[keyframe_ids[i]]);
                global_pcd.MergePCD(pcd);
            }
            return std::make_shared<geometry::PointCloud>(global_pcd);
        }
        void UpdateAllPoses()
        {
            int anchor_keyframe_id = 0;
            for(size_t i = 0; i < global_poses.size(); ++i)
            {
                if(global_frames[i].keyframe_kid != -1)
                anchor_keyframe_id = i;
                global_poses[i] = global_poses[anchor_keyframe_id] * relative_poses[i].inverse();
            }
        }

        virtual void Optimize() = 0;
        camera::PinholeCamera camera;
        odometry::Odometry rgbd_odometry;
        lcdetection::MildLCDetector mild_lcd;
        geometry::SE3List global_poses;
        geometry::SE3List relative_poses;//relative_pose to the anchor keyframe
        geometry::SE3List global_keyframe_poses;//just used for optimization
        optimization::Optimizer optimizer;
        std::vector<geometry::RGBDFrame> global_frames;
        std::vector<int> keyframe_ids;        
        //some parameters
        float max_average_disparity = 30.0;
        float max_reprojection_error_3d = 0.05;
        float voxel_len = 0.1;
    };
}
}
#endif