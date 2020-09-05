#ifndef DENSE_ODOMETRY_FUNCTION_H
#define DENSE_ODOMETRY_FUNCTION_H
//define some function for dense tracking
#include "Camera/Camera.h"
#include "Geometry/Geometry.h"
#include "Tool/ImageProcessing.h"
#include <opencv2/opencv.hpp>
#include "OdometryPredefined.h"
namespace fucking_cool
{
namespace odometry
{

    void AddElementToCorrespondenceMap(cv::Mat &wraping_map,cv::Mat &wraping_depth,
        int u_s, int v_s, int u_t, int v_t, float transformed_d_s);
    
    void ComputeCorrespondencePixelWise(const cv::Mat &source, const cv::Mat &target, const camera::PinholeCamera &camera,
        const geometry::TransformationMatrix & relative_pose, geometry::PixelCorrespondenceSet &correspondences);

    void TransformToMatXYZ(const cv::Mat &image, const camera::PinholeCamera &camera, geometry::ImageXYZ &imageXYZ);

    void ComputeJacobianHybridTerm(
        int row,std::vector<geometry::Se3> &J, std::vector<float> &residual, 
        const cv::Mat &source_color, const cv::Mat & source_depth, 
        const cv::Mat & target_color, const cv::Mat & target_depth, 
        const cv::Mat &target_color_dx, const cv::Mat & target_depth_dx, 
        const cv::Mat & target_color_dy, const cv::Mat & target_depth_dy, 
        const geometry::ImageXYZ & source_XYZ, const camera::PinholeCamera &camera, const geometry::TransformationMatrix & relative_pose, 
        const geometry::PixelCorrespondenceSet &correspondences);
    void ComputeJacobianPhotoTerm(
        int row,std::vector<geometry::Se3> &J, std::vector<float> &residual, 
        const cv::Mat &source_color, const cv::Mat & source_depth, 
        const cv::Mat & target_color, const cv::Mat & target_depth, 
        const cv::Mat &target_color_dx, const cv::Mat & target_color_dy, 
        const geometry::ImageXYZ & source_XYZ, const camera::PinholeCamera &camera, 
        const geometry::TransformationMatrix & relative_pose, 
        const geometry::PixelCorrespondenceSet &correspondences);
    void ComputeJacobianDepthTerm(
        int row,std::vector<geometry::Se3> &J, std::vector<float> &residual, 
        const cv::Mat &source_color, const cv::Mat & source_depth, 
        const cv::Mat & target_color, const cv::Mat & target_depth, 
        const cv::Mat & target_depth_dx, const cv::Mat & target_depth_dy, 
        const geometry::ImageXYZ & source_XYZ, const camera::PinholeCamera &camera, const geometry::TransformationMatrix & relative_pose, 
        const geometry::PixelCorrespondenceSet &correspondences);
    std::tuple<geometry::Matrix6,geometry::Se3,float> ComputeJTJandJTrHybridTerm(
        const cv::Mat &source_color, const cv::Mat & source_depth, 
        const cv::Mat & target_color, const cv::Mat & target_depth, 
        const cv::Mat &target_color_dx, const cv::Mat & target_depth_dx, 
        const cv::Mat & target_color_dy, const cv::Mat & target_depth_dy, 
        const geometry::ImageXYZ & source_XYZ, const camera::PinholeCamera &camera, const geometry::TransformationMatrix & relative_pose, 
        const geometry::PixelCorrespondenceSet &correspondences);

    std::tuple<geometry::Matrix6,geometry::Se3,float> ComputeJTJandJTrPhotoTerm(
        const cv::Mat &source_color, const cv::Mat & source_depth, 
        const cv::Mat & target_color, const cv::Mat & target_depth, 
        const cv::Mat &target_color_dx, const cv::Mat & target_color_dy, 
        const geometry::ImageXYZ & source_XYZ, const camera::PinholeCamera &camera, const geometry::TransformationMatrix & relative_pose, 
        const geometry::PixelCorrespondenceSet &correspondences);
    void ConvertDepthTo32FNaN(const cv::Mat &depth, cv::Mat &refined_depth, float depth_scale);

    void ConvertColorToIntensity32F(const cv::Mat &color, cv::Mat &intensity, float scale);
/*
    void NormalizeIntensity(const cv::Mat &intensity,cv::Mat &normalized_intensity, float scale)
    {
    normalized_intensity.create(intensity.rows, intensity.cols, CV_32FC1, cv::Scalar(0));
    for(int i = 0; i < intensity.rows * intensity.cols; ++i)
        normalized_intensity.at<float>(i) = intensity.at<int>(i) /scale;        
    }
*/
    void NormalizeIntensity(cv::Mat &source_gray, cv::Mat & target_gray, const geometry::PixelCorrespondenceSet &correspondence);

    void DoSingleIteration(
        const cv::Mat &source_color, const cv::Mat & source_depth, 
        const cv::Mat & target_color, const cv::Mat & target_depth, 
        const cv::Mat &target_color_dx, const cv::Mat & target_depth_dx, 
        const cv::Mat & target_color_dy, const cv::Mat & target_depth_dy, 
        const geometry::ImageXYZ & source_XYZ, const camera::PinholeCamera &camera, geometry::TransformationMatrix & relative_pose, 
        geometry::PixelCorrespondenceSet &correspondences);
    void DoSingleIterationPhotoTerm(
        const cv::Mat &source_color, const cv::Mat & source_depth, 
        const cv::Mat & target_color, const cv::Mat & target_depth, 
        const cv::Mat &target_color_dx, const cv::Mat & target_color_dy,
        const geometry::ImageXYZ & source_XYZ, const camera::PinholeCamera &camera, geometry::TransformationMatrix & relative_pose, 
        geometry::PixelCorrespondenceSet &correspondences);
    void DoSingleIterationDepthTerm(
        const cv::Mat &source_color, const cv::Mat & source_depth, 
        const cv::Mat & target_color, const cv::Mat & target_depth, 
        const cv::Mat &target_depth_dx, const cv::Mat & target_depth_dy,
        const geometry::ImageXYZ & source_XYZ, const camera::PinholeCamera &camera, geometry::TransformationMatrix & relative_pose, 
        geometry::PixelCorrespondenceSet &correspondences);
    std::tuple<geometry::Matrix6,geometry::Se3,float> ComputeJTJandJTr(
        const cv::Mat &source_color, const cv::Mat & source_depth, 
        const cv::Mat & target_color, const cv::Mat & target_depth, 
        const cv::Mat &target_color_dx, const cv::Mat & target_depth_dx, 
        const cv::Mat & target_color_dy, const cv::Mat & target_depth_dy, 
        const geometry::ImageXYZ & source_XYZ, const camera::PinholeCamera &camera, const geometry::TransformationMatrix & relative_pose, 
        const geometry::PixelCorrespondenceSet &correspondences);

}
}
#endif