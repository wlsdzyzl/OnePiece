#ifndef DENSE_SLAM_H
#define DENSE_SLAM_H

#include "Geometry/Geometry.h"
#include "Camera/Camera.h"
#include "Odometry/Odometry.h"
#include "Geometry/RGBDFrame.h"
#include "Optimization/Optimizer.h"
#include "Registration/GlobalRegistration.h"
namespace one_piece
{
    class Submap
    {
        public:

        Submap() = default;
        Submap(int _submap_id):submap_id(_submap_id){}

        geometry::PointCloudPtr GenerateSubmapModel(const std::vector<geometry::RGBDFrame> global_frames, const geometry::SE3List &relative_poses, const camera::PinholeCamera &camera)
        {
            geometry::PointCloud pcd;
            for(size_t i = 0; i < containted_frames.size(); i+=3)
            {
                geometry::PointCloud tmp_pcd;
                int containted_frame_id = containted_frames[i];
                tmp_pcd.LoadFromRGBD(global_frames[containted_frame_id], camera);
                tmp_pcd.Transform(relative_poses[containted_frame_id]);
                auto tmp_pcd_ptr = tmp_pcd.DownSample(0.025);

                pcd.MergePCD(*tmp_pcd_ptr);
            }
            return std::make_shared<geometry::PointCloud>(pcd);
        }

        std::vector<int > containted_frames;
        geometry::PointCloud downsampled_pcd;
        registration::FeatureSet features;
        int submap_id;
        bool is_registered = false;
    };
    class DenseSlam
    {
        //use dense tracking
        //submap strategy
        //use global registration to detect loop closure
        public:
        DenseSlam()
        {
            r_para.search_radius = 0.25;
            r_para.max_nn = 100;
            r_para.voxel_len = 0.05;
            r_para.threshold = 0.5;
            r_para.search_radius_normal = 0.1;
            r_para.max_nn_normal = 30;
            r_para.max_iteration = 40000;
            r_para.threshold = 0.1;   
        }
        DenseSlam(const camera::PinholeCamera &_camera):camera(_camera)
        {
            rgbd_odometry.SetCamera(camera);
            r_para.search_radius = 0.25;
            r_para.max_nn = 100;
            r_para.voxel_len = 0.05;
            r_para.threshold = 0.5;
            r_para.search_radius_normal = 0.1;
            r_para.max_nn_normal = 30;
            r_para.max_iteration = 40000;
            r_para.threshold = 0.1;   
        }
        void UpdateFrame(const geometry::RGBDFrame &frame);
        void RegisterSubmap(int submap_id);

        geometry::PointCloudPtr GetPosedPCD()
        {
            geometry::PointCloud v_pcd;
            for(size_t i = 0; i != submaps.size(); ++i)
            {
                auto tmp_pcd = submaps[i].downsampled_pcd;
                tmp_pcd.Transform(submap_poses[i]);
                v_pcd.MergePCD(tmp_pcd);                
            }
            return std::make_shared<geometry::PointCloud>(v_pcd);
        }
        void ReleaseSubmapFrame(int submap_id)
        {
            auto &containted_frames = submaps[submap_id].containted_frames;
            for(size_t i = 0; i != containted_frames.size(); ++i)
            {
                global_frames[containted_frames[i]].Release();
            }
        }
        void UpdateAllPoses()
        {
        
            for(size_t i = 0; i != submaps.size(); ++i)
            {
                auto &containted_frames = submaps[i].containted_frames;
                for(size_t j = 0; j != containted_frames.size(); ++j)
                    global_poses[containted_frames[j]] = submap_poses[i] * relative_poses[containted_frames[j]];
            }
        }

        void Optimize();
        camera::PinholeCamera camera;
        odometry::Odometry rgbd_odometry;
        geometry::SE3List global_poses;
        geometry::SE3List relative_poses;//relative_pose in corresponding submap
        std::vector<Submap> submaps;
        geometry::SE3List submap_poses;
        optimization::Optimizer optimizer;
        
        std::vector<geometry::RGBDFrame> global_frames;
        std::vector<optimization::Correspondence> global_submap_correspondences;
        // //to optimize the poses of frames in a submap
        // std::vector<optimization::Correspondence> local_correspondences;
        int last_tracking_frame_id = -1;
        bool visible_pcd_updated = false;

        bool new_submap_flag = true;
        //some parameters
        float max_reprojection_error_3d = 1.5;
        float voxel_len = 0.1;        
        int step = 50;//each submap contains 100 frames
        int current_submap_id = -1;
        float max_reprojection_error_3d_ransac = 0.1;
        float min_inliers_ransac = 100;
        registration::RANSACParameter r_para;
        

    };
}
#endif