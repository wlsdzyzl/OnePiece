#ifndef ODOMETRY_H
#define ODOMETRY_H


#include "Camera/Camera.h"
#include "Geometry/Geometry.h"
#include <iostream>
#include "DenseOdometryFunction.h"
#include "SparseOdometryFunction.h"
#include "Tool/ImageProcessing.h"
#include "SparseMatcher.h"
#include "Geometry/RGBDFrame.h"
#include <ctime>

namespace fucking_cool
{
namespace odometry
{

    class SparseTrackingResult
    {
        public:
        geometry::TransformationMatrix T;
        geometry::FMatchSet correspondence_set_index;
        geometry::PointCorrespondenceSet correspondence_set;
        double rmse = 1e6;
        bool tracking_success;
    };
    class DenseTrackingResult
    {
        public:
        geometry::TransformationMatrix T;
        geometry::PixelCorrespondenceSet pixel_correspondence_set;
        geometry::PointCorrespondenceSet correspondence_set;
        double rmse = 1e6;
        bool tracking_success;
    };
    class Odometry
    {

        public:
        
        Odometry()
        {
            feature_extractor = cv::ORB::create();
            std::random_device rd;
            engine.seed(rd());
            bf_matcher = cv::BFMatcher(cv::NORM_HAMMING,true);
        }

        Odometry(const camera::PinholeCamera &_camera)
        {
            feature_extractor = cv::ORB::create();
            camera = _camera;
            std::random_device rd;
            engine.seed(rd());
        }


        void Find2DMathes(const cv::Mat &source, const cv::Mat &target, 
        geometry::Descriptor &source_descriptors,geometry::Descriptor &target_descriptors,
        geometry::KeyPointSet &source_keypoints, geometry::KeyPointSet &target_keypoints, geometry::DMatchSet & matches);

        std::shared_ptr<SparseTrackingResult> SparseTracking(const cv::Mat &source_color, const cv::Mat &target_color, 
        const cv::Mat &source_depth, const cv::Mat &target_depth);


        std::shared_ptr<SparseTrackingResult> SparseTracking(geometry::RGBDFrame &source_frame, 
        geometry::RGBDFrame &target_frame);


        std::shared_ptr<SparseTrackingResult> SparseTrackingMILD(const cv::Mat &source_color, const cv::Mat &target_color, 
        const cv::Mat &source_depth, const cv::Mat &target_depth);

        std::shared_ptr<SparseTrackingResult> SparseTrackingMILD(geometry::RGBDFrame &source_frame, 
        geometry::RGBDFrame &target_frame);

        std::shared_ptr<DenseTrackingResult> DenseTracking(const cv::Mat &source_color, const cv::Mat &target_color, 
        const cv::Mat &source_depth, const cv::Mat &target_depth, 
        const geometry::TransformationMatrix &initial_T, int term_type = 0);
        
        std::shared_ptr<DenseTrackingResult> DenseTracking(geometry::RGBDFrame &source_frame, 
        geometry::RGBDFrame &target_frame, 
        const geometry::TransformationMatrix &initial_T, int term_type = 0);

        void SetCamera(const camera::PinholeCamera &_camera )
        {
            camera = _camera;
        }
        void SetCameraPara(float _fx, float _fy, float _cx, float _cy, int _width, int _height, float depthScale, float *_distortion = nullptr)
        {
            camera.SetPara(_fx, _fy, _cx, _cy, _width,_height,depthScale,_distortion);
        }

        void SetFeatureNumber(int _feature_number)
        {
            feature_number = _feature_number;
        }

        
        void SetMultiScale(int layer_count)
        {
            multi_scale_level = layer_count;
            iter_count_per_level.resize(layer_count,4);       
        }
        int GetFeatureNumber()
        {
            return feature_number;
        }
        std::vector<camera::PinholeCamera > CreatePyramidCameras()
        {
            std::vector<camera::PinholeCamera > result;
            for(int i=0;i!=multi_scale_level;++i)
            {
                if(i == 0) result.push_back(camera);
                else 
                result.push_back(result[i-1].GenerateNextPyramid());
            }
            return result;
        }
        bool ComputeTransformation(geometry::PointCorrespondenceSet & correspondence_set, 
            geometry::FMatchSet & matches,
            geometry::TransformationMatrix &T);        
        void CreateImagePyramid(const cv::Mat& color,  const cv::Mat& depth, 
            std::vector<cv::Mat> &color_pyramid, std::vector<cv::Mat> &depth_pyramid, 
            std::vector<cv::Mat> &color_dx_pyramid, std::vector<cv::Mat> &color_dy_pyramid,
            std::vector<cv::Mat> &depth_dx_pyramid, std::vector<cv::Mat> &depth_dy_pyramid);
        
        void CreateImageXYZPyramid(const std::vector<cv::Mat> &depth_pyramid, 
            const std::vector<camera::PinholeCamera > &pyramid_cameras, 
            std::vector<geometry::ImageXYZ> &source_XYZ);        
        
        // bool MultiScaleComputing(const cv::Mat& source_color, const cv::Mat& target_color, 
        //     const cv::Mat& source_depth, const cv::Mat& target_depth, geometry::TransformationMatrix &T, 
        //     geometry::PointCorrespondenceSet &correspondence_set, int use_hybrid);
        
        bool MultiScaleComputing(const std::vector<cv::Mat> &source_color_pyramid,
            const std::vector<cv::Mat> &target_color_pyramid, 
            const std::vector<cv::Mat> &source_depth_pyramid, const std::vector<cv::Mat> &target_depth_pyramid, 
            const std::vector<cv::Mat> &target_depth_dx_pyramid, const std::vector<cv::Mat> &target_depth_dy_pyramid, 
            const std::vector<cv::Mat> &target_color_dx_pyramid, const std::vector<cv::Mat> &target_color_dy_pyramid, 
            const std::vector<geometry::ImageXYZ> &source_image_xyz_pyramid, 
            const std::vector<geometry::ImageXYZ> &target_image_xyz_pyramid,
            const std::vector<camera::PinholeCamera> &camera_pyramid, geometry::TransformationMatrix &T, 
            geometry::PointCorrespondenceSet &correspondence_set, 
            geometry::PixelCorrespondenceSet &pixel_correspondence_set, int use_hybrid);        
        void InitializeRGBDDenseTracking(const cv::Mat &color, const cv::Mat &depth, 
            cv::Mat &gray, cv::Mat &refined_depth);
        void GetLocalPointsFromKeyPoints(const cv::Mat &depth, const geometry::KeyPointSet &keypoints, 
            geometry::Point3List &local_points);     
        void GetCorrespondencesFromMatches(const geometry::Point3List & source_local_points, 
            const geometry::Point3List & target_local_points,
            const geometry::DMatchSet & matches, geometry::FMatchSet &match_set, 
            geometry::PointCorrespondenceSet &correspondence_set);
        // void GetCorrespondencesFromMatches(const cv::Mat & source_depth, const cv::Mat & target_depth,
        //     const geometry::KeyPointSet &source_keypoints,const geometry::KeyPointSet &target_keypoints,
        //     const geometry::DMatchSet & matches, geometry::PointCorrespondenceSet &correspondence_set);

        protected:
        camera::PinholeCamera camera;
        cv::Ptr<cv::ORB> feature_extractor;
        cv::BFMatcher bf_matcher;
        SparseMatcher sparse_matcher;
        OutlierFilter outlier_filter ;
        std::default_random_engine engine;        
        int feature_number = 1000;
        //for dense tracking
        int multi_scale_level = 3;

        std::vector<int> iter_count_per_level={4,8,16};


    };
}
}
#endif