#ifndef RGBD_FRAME_H
#define RGBD_FRAME_H

#include "Geometry.h"
#include "PointCloud.h"
namespace fucking_cool 
{
namespace geometry
{
    
    class RGBDFrame
    {
        public:
        RGBDFrame() = default;
        RGBDFrame(const cv::Mat &_rgb, const cv::Mat &_depth, int id = -1)
        {
            rgb = _rgb;
            depth = _depth;
            frame_id = id;
        }
        cv::Mat rgb;
        cv::Mat depth;
        //std::shared_ptr<PointCloud> pcd;
        
        PointCloud down_sampled_pcd;//for visualization
        int frame_id = -1;
        /*for sparse odometry*/
        geometry::Descriptor descriptor;
        geometry::KeyPointSet keypoints;
        PointCloud feature_pcd;//for optimization


        /*for dense odometry*/
        cv::Mat gray;//gray image of rgb
        cv::Mat depth32f;//refined depth
        std::vector<cv::Mat> color_pyramid;
        std::vector<cv::Mat> depth_pyramid;
        std::vector<cv::Mat> color_dx_pyramid;
        std::vector<cv::Mat> color_dy_pyramid;
        std::vector<cv::Mat> depth_dx_pyramid;
        std::vector<cv::Mat> depth_dy_pyramid;
        std::vector<geometry::ImageXYZ> image_xyz;
        int keyframe_kid = -1;
        bool tracking_success = false;
        
        bool IsPreprocessedSparse() const
        {
            return feature_pcd.points.size() != 0;
        }
        void Release()
        {
            rgb.release();
            descriptor.release();
            depth.release();
            keypoints.clear();
            feature_pcd.points.clear();

        }
        bool IsPreprocessedDense() const
        {
            return image_xyz.size() != 0;
        }

        void PrepareDownSamplePointCloud(const camera::PinholeCamera &camera, 
            float voxel_len = 0.01);

    };
};
}
#endif