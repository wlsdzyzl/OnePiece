#include "Odometry.h"
#include <random>
#include <Eigen/Dense>
#include <iostream>
#include <algorithm>
#include "Tool/TickTock.h"
namespace fucking_cool
{
namespace odometry
{



    bool Odometry::ComputeTransformation(geometry::PointCorrespondenceSet & correspondence_set, 
        geometry::FMatchSet & matches,
        geometry::TransformationMatrix &T)
    {
        if(correspondence_set.size() < MIN_INLIER_SPARSE)
        return false;
        return RANSAC3d(correspondence_set, matches, engine, T);
    }
    // void Odometry::GetCorrespondencesFromMatches(const cv::Mat & source_depth, const cv::Mat & target_depth,
    // const geometry::KeyPointSet &source_keypoints,const geometry::KeyPointSet &target_keypoints,
    // const geometry::DMatchSet & matches, geometry::PointCorrespondenceSet &correspondence_set)
    // {
    //     correspondence_set.clear();
    //     float x1,y1,z1,x2,y2,z2;
    //     for(int i = 0; i < matches.size(); ++i)
    //     {

    //         const cv::KeyPoint &source = source_keypoints[matches[i].queryIdx];
    //         const cv::KeyPoint &target = target_keypoints[matches[i].trainIdx];
    //         if(source_depth.depth() == CV_32FC1)
    //             z1 = source_depth.at<float>(source.pt.y,source.pt.x);
    //         else 
    //             z1 = source_depth.at<unsigned short>(source.pt.y,source.pt.x)/camera.GetDepthScale();

    //         if(target_depth.depth() == CV_32FC1)
    //             z2 = target_depth.at<float>(target.pt.y,target.pt.x);
    //         else 
    //             z2 = target_depth.at<unsigned short>(target.pt.y,target.pt.x)/camera.GetDepthScale();            
            
    //         x1 = (source.pt.x - camera.GetCx())/camera.GetFx()*z1;
    //         y1 = (source.pt.y - camera.GetCy())/camera.GetFy()*z1;
            
    //         x2 = (target.pt.x - camera.GetCx())/camera.GetFx()*z2;
    //         y2 = (target.pt.y - camera.GetCy())/camera.GetFy()*z2;

    //         correspondence_set.push_back(std::make_pair(geometry::Point3(x1,y1,z1),geometry::Point3(x2,y2,z2)));
    //     }
    //     return ;
    // }
    void Odometry::GetCorrespondencesFromMatches(const geometry::Point3List & source_local_points, 
        const geometry::Point3List & target_local_points,
        const geometry::DMatchSet & matches, geometry::FMatchSet &match_set, geometry::PointCorrespondenceSet &correspondence_set)
    {
        correspondence_set.clear();
        match_set.clear();
        for(int i = 0; i < matches.size(); ++i)
        {
            correspondence_set.push_back(std::make_pair(source_local_points[matches[i].queryIdx],target_local_points[matches[i].trainIdx]));
            match_set.push_back(std::make_pair(matches[i].queryIdx, matches[i].trainIdx));
        }
        return ;
    }
    void Odometry::GetLocalPointsFromKeyPoints(const cv::Mat &depth, const geometry::KeyPointSet &keypoints, 
        geometry::Point3List &local_points)
    {
        float x,y,z;
        local_points.resize(keypoints.size());
        for(int i = 0; i < keypoints.size(); ++i)
        {

            const cv::KeyPoint &keypoint = keypoints[i];
            if(depth.depth() == CV_32FC1)
                z = depth.at<float>(keypoint.pt.y,keypoint.pt.x); 
            else
                z = depth.at<unsigned short>(keypoint.pt.y,keypoint.pt.x)/camera.GetDepthScale();
            
            x = (keypoint.pt.x - camera.GetCx())/camera.GetFx()*z;
            y = (keypoint.pt.y - camera.GetCy())/camera.GetFy()*z;
            local_points[i] = geometry::Point3(x,y,z);
        }
        return;
    }

    


    
    std::shared_ptr<SparseTrackingResult> Odometry::SparseTracking(const cv::Mat &source_color, const cv::Mat &target_color, 
        const cv::Mat &source_depth, const cv::Mat &target_depth)
    {
        //Sparse Matching: use feature matching to find correspondence, and use SVD to get the Matrix.
        //need to use ransac to filter outlier and further improve the accuracy.
        geometry::Descriptor source_descriptors;
        geometry::Descriptor target_descriptors;
        geometry::KeyPointSet source_keypoints;
        geometry::KeyPointSet target_keypoints;
        geometry::Point3List source_local_points;
        geometry::Point3List target_local_points;
        geometry::DMatchSet matches;
        geometry::FMatchSet fmatch_set;
        geometry::PointCorrespondenceSet correspondence_set;
        geometry::TransformationMatrix transformation;
        SparseTrackingResult tracking_result;
        //Find 2D matches
        if(source_keypoints.size() == 0)
        {
            feature_extractor->setMaxFeatures(feature_number);
            feature_extractor->detectAndCompute(source_color, cv::noArray(), source_keypoints, source_descriptors);
        }
        if(target_keypoints.size() == 0)
        {

            feature_extractor->setMaxFeatures(feature_number);
            feature_extractor->detectAndCompute(target_color, cv::noArray(), target_keypoints, target_descriptors);
        }

        GetLocalPointsFromKeyPoints(source_depth, source_keypoints,source_local_points);
        GetLocalPointsFromKeyPoints(target_depth, target_keypoints,target_local_points);
#if DEBUG_MODE
        std::cout << "extract features.."<<std::endl;
        cv::Mat outimg1,outimg2;
        cv::drawKeypoints(source_color,source_keypoints,outimg1,cv::Scalar::all(-1),cv::DrawMatchesFlags::DEFAULT);
        cv::drawKeypoints(target_color,target_keypoints,outimg2,cv::Scalar::all(-1),cv::DrawMatchesFlags::DEFAULT);
        cv::imshow("source",outimg1);
        cv::imshow("target",outimg2);
        cv::waitKey(0);
        cv::destroyWindow("source");
        cv::destroyWindow("target");
#endif

        // feature matching
        tool::Timer timer;
        matches.clear();
        timer.TICK("bf matcher");
        //outlier_filter.KnnMatch(source_descriptors,target_descriptors,bf_matcher,matches);
        bf_matcher.match(source_descriptors,target_descriptors,matches);
        timer.TOCK("bf matcher");
        timer.LogAll();

        outlier_filter.RemoveInvalidMatches(source_local_points, target_local_points, matches);
        //outlier_filter.KnnMatch(source_descriptors,target_descriptors,bf_matcher,matches);
        //use RANSAC or other algorithms for outlier filter.
        //outlier_filter.MinDistance(source_keypoints,target_keypoints,matches);
        outlier_filter.Ransac(source_keypoints,target_keypoints,matches);
        //FilterOutliers(source_keypoints,target_keypoints,matches);

#if DEBUG_MODE
        //show matching results
        std::cout << "feature matching.."<<std::endl;
        cv::Mat img_good_matches;
        cv::drawMatches(source_color,source_keypoints,target_color,target_keypoints,matches,img_good_matches);        
        cv::imshow("matching_result",img_good_matches);
        cv::waitKey(0);
        cv::destroyWindow("matching_result");
#endif 
        GetCorrespondencesFromMatches(source_local_points, target_local_points, matches, fmatch_set, correspondence_set);

        auto is_success = ComputeTransformation(correspondence_set, fmatch_set, transformation);
        // std::cout<<(correspondence_set.size()+0.0) / matches.size()<<std::endl;
        tracking_result.T = transformation;
        tracking_result.correspondence_set = correspondence_set;
        tracking_result.tracking_success = is_success;
        tracking_result.rmse = geometry::ComputeReprojectionError3D(correspondence_set, transformation);
        return std::make_shared<SparseTrackingResult>(tracking_result);
    }

    std::shared_ptr<SparseTrackingResult> Odometry::SparseTrackingMILD(const cv::Mat &source_color, const cv::Mat &target_color, 
        const cv::Mat &source_depth, const cv::Mat &target_depth)
    {
        //Sparse Matching: use feature matching to find correspondence, and use SVD to get the Matrix.
        //need to use ransac to filter outlier and further improve the accuracy.
        geometry::Descriptor source_descriptors;
        geometry::Descriptor target_descriptors;
        geometry::KeyPointSet source_keypoints;
        geometry::KeyPointSet target_keypoints;
        geometry::Point3List source_local_points;
        geometry::Point3List target_local_points;
        geometry::DMatchSet matches;
        geometry::FMatchSet fmatch_set;
        geometry::PointCorrespondenceSet correspondence_set;
        geometry::TransformationMatrix transformation;
        SparseTrackingResult tracking_result;

        if(source_keypoints.size() == 0)
        {
            feature_extractor->setMaxFeatures(feature_number);
            feature_extractor->detectAndCompute(source_color, cv::noArray(), source_keypoints, source_descriptors);
        }
        if(target_keypoints.size() == 0)
        {

            feature_extractor->setMaxFeatures(feature_number);
            feature_extractor->detectAndCompute(target_color, cv::noArray(), target_keypoints, target_descriptors);
        }


#if 0
        std::cout << "extract features.."<<std::endl;
        cv::Mat outimg1,outimg2;
        cv::drawKeypoints(source_color,source_keypoints,outimg1,cv::Scalar::all(-1),cv::DrawMatchesFlags::DEFAULT);
        cv::drawKeypoints(target_color,target_keypoints,outimg2,cv::Scalar::all(-1),cv::DrawMatchesFlags::DEFAULT);
        cv::imshow("source",outimg1);
        cv::imshow("target",outimg2);
        cv::waitKey(0);
        cv::destroyWindow("source");
        cv::destroyWindow("target");
#endif

        GetLocalPointsFromKeyPoints(source_depth, source_keypoints,source_local_points);
        GetLocalPointsFromKeyPoints(target_depth, target_keypoints,target_local_points);
        sparse_matcher.Match(source_descriptors, target_descriptors, matches);

        outlier_filter.RemoveInvalidMatches(source_local_points, target_local_points, matches);

        outlier_filter.RanSaPC(source_local_points,target_local_points,engine,matches);
        outlier_filter.RanSaPC(source_local_points,target_local_points,engine,matches);
        outlier_filter.RanSaPC(source_local_points,target_local_points,engine,matches);
        outlier_filter.RanSaPC(source_local_points,target_local_points,engine,matches);
        outlier_filter.RanSaPC(source_local_points,target_local_points,engine,matches);

        GetCorrespondencesFromMatches(source_local_points, target_local_points, matches, fmatch_set, correspondence_set);
        ComputeTransformation(correspondence_set, fmatch_set, transformation);
        matches.clear();
        sparse_matcher.RefineMatches(source_descriptors,source_local_points,source_keypoints,target_keypoints,transformation,matches);

        outlier_filter.RanSaPC(source_local_points,target_local_points,engine,matches);
        outlier_filter.RanSaPC(source_local_points,target_local_points,engine,matches);
        outlier_filter.RanSaPC(source_local_points,target_local_points,engine,matches);
        outlier_filter.RanSaPC(source_local_points,target_local_points,engine,matches);
        outlier_filter.RanSaPC(source_local_points,target_local_points,engine,matches);
        GetCorrespondencesFromMatches(source_local_points, target_local_points, matches, fmatch_set, correspondence_set);
         
#if 0
        //show matching results
        std::cout << "feature matching.."<<std::endl;
        cv::Mat img_good_matches;
        cv::drawMatches(source_color,source_keypoints,target_color,target_keypoints,matches,img_good_matches);        
        cv::imshow("matching_result",img_good_matches);
        cv::waitKey(0);
        cv::destroyWindow("matching_result");
#endif
        auto is_success = ComputeTransformation(correspondence_set, fmatch_set, transformation);
        // std::cout<<(correspondence_set.size()+0.0) / matches.size()<<std::endl;
        tracking_result.T = transformation;
        tracking_result.correspondence_set = correspondence_set;        
        tracking_result.tracking_success = is_success;
        tracking_result.rmse = geometry::ComputeReprojectionError3D(correspondence_set, transformation);
        return std::make_shared<SparseTrackingResult>(tracking_result);
    }

    std::shared_ptr<SparseTrackingResult> Odometry::SparseTracking(geometry::RGBDFrame &source_frame, 
        geometry::RGBDFrame &target_frame)
    {
        //Sparse Matching: use feature matching to find correspondence, and use SVD to get the Matrix.
        //need to use ransac to filter outlier and further improve the accuracy.
        geometry::Descriptor &source_descriptors = source_frame.descriptor;
        geometry::Descriptor &target_descriptors = target_frame.descriptor;
        geometry::KeyPointSet &source_keypoints = source_frame.keypoints;
        geometry::KeyPointSet &target_keypoints = target_frame.keypoints;
        geometry::Point3List &source_local_points = source_frame.feature_pcd.points;
        geometry::Point3List &target_local_points = target_frame.feature_pcd.points;
        geometry::DMatchSet matches;
        geometry::FMatchSet fmatch_set;
        geometry::PointCorrespondenceSet correspondence_set;
        geometry::TransformationMatrix transformation;
        SparseTrackingResult tracking_result;

        if(!source_frame.IsPreprocessedSparse())
        {
            feature_extractor->setMaxFeatures(feature_number);
            feature_extractor->detectAndCompute(source_frame.rgb, cv::noArray(), source_keypoints, source_descriptors);
            
            GetLocalPointsFromKeyPoints(source_frame.depth, source_keypoints,source_local_points);    
        }
        if(!target_frame.IsPreprocessedSparse())
        {
            feature_extractor->setMaxFeatures(feature_number);
            feature_extractor->detectAndCompute(target_frame.rgb, cv::noArray(), target_keypoints, target_descriptors);
            GetLocalPointsFromKeyPoints(target_frame.depth, target_keypoints,target_local_points);
        }


#if DEBUG_MODE
        std::cout << "extract features.."<<std::endl;
        cv::Mat outimg1,outimg2;
        cv::drawKeypoints(source_frame.rgb,source_keypoints,outimg1,cv::Scalar::all(-1),cv::DrawMatchesFlags::DEFAULT);
        cv::drawKeypoints(target_frame.rgb,target_keypoints,outimg2,cv::Scalar::all(-1),cv::DrawMatchesFlags::DEFAULT);
        cv::imshow("source",outimg1);
        cv::imshow("target",outimg2);
        cv::waitKey(0);
        cv::destroyWindow("source");
        cv::destroyWindow("target");
#endif

        // feature matching
        tool::Timer timer;
        matches.clear();
        timer.TICK("bf matcher");
        outlier_filter.KnnMatch(source_descriptors,target_descriptors,bf_matcher,matches);
        //bf_matcher.match(source_descriptors,target_descriptors,matches);
        timer.TOCK("bf matcher");
        timer.LogAll();
        outlier_filter.RemoveInvalidMatches(source_local_points, target_local_points, matches);
        
        //outlier_filter.KnnMatch(source_descriptors,target_descriptors,bf_matcher,matches);
        //use RANSAC or other algorithms for outlier filter.
        //outlier_filter.MinDistance(source_keypoints,target_keypoints,matches);
        //outlier_filter.Ransac(source_keypoints,target_keypoints,matches);
        //FilterOutliers(source_keypoints,target_keypoints,matches);

#if 1
        //show matching results
        std::cout << "feature matching.."<<std::endl;
        cv::Mat img_good_matches;
        cv::drawMatches(source_frame.rgb,source_keypoints,target_frame.rgb,target_keypoints,matches,img_good_matches);        
        cv::imshow("matching_result",img_good_matches);
        cv::waitKey(0);
        cv::destroyWindow("matching_result");
#endif 
        GetCorrespondencesFromMatches(source_local_points, target_local_points, matches, fmatch_set, correspondence_set);        
        auto is_success = ComputeTransformation(correspondence_set, fmatch_set, transformation);
        // std::cout<<(correspondence_set.size()+0.0) / matches.size()<<std::endl;
        tracking_result.T = transformation;
        tracking_result.correspondence_set = correspondence_set;
        tracking_result.correspondence_set_index = fmatch_set;
        tracking_result.tracking_success = is_success;
        tracking_result.rmse = geometry::ComputeReprojectionError3D(correspondence_set, transformation);
#if DEBUG_MODE
        //show matching results
        matches.clear();
        for(int i = 0; i != fmatch_set.size() && i != 3; ++i)
        {
            auto match = cv::DMatch();
            match.queryIdx = fmatch_set[i].first;
            match.trainIdx = fmatch_set[i].second;
            matches.push_back(match);
        }
        std::cout << "feature matching.."<<std::endl;
        cv::Mat final_matches;
        cv::drawMatches(source_frame.rgb,source_keypoints,target_frame.rgb,target_keypoints,matches,final_matches);        
        cv::imshow("matching_result",final_matches);
        cv::waitKey(0);
        cv::destroyWindow("matching_result");
#endif 
        return std::make_shared<SparseTrackingResult>(tracking_result);
    }

    std::shared_ptr<SparseTrackingResult> Odometry::SparseTrackingMILD(geometry::RGBDFrame &source_frame, 
        geometry::RGBDFrame &target_frame)
    {
        //Sparse Matching: use feature matching to find correspondence, and use SVD to get the Matrix.
        //need to use ransac to filter outlier and further improve the accuracy.
        geometry::Descriptor &source_descriptors = source_frame.descriptor;
        geometry::Descriptor &target_descriptors = target_frame.descriptor;
        geometry::KeyPointSet &source_keypoints = source_frame.keypoints;
        geometry::KeyPointSet &target_keypoints = target_frame.keypoints;
        geometry::Point3List &source_local_points = source_frame.feature_pcd.points;
        geometry::Point3List &target_local_points = target_frame.feature_pcd.points;
        geometry::DMatchSet matches;
        geometry::FMatchSet fmatch_set;
        geometry::PointCorrespondenceSet correspondence_set;
        geometry::TransformationMatrix transformation;
        SparseTrackingResult tracking_result;

        if(!source_frame.IsPreprocessedSparse())
        {
            feature_extractor->setMaxFeatures(feature_number);
            feature_extractor->detectAndCompute(source_frame.rgb, cv::noArray(), source_keypoints, source_descriptors);
            GetLocalPointsFromKeyPoints(source_frame.depth, source_keypoints,source_local_points);    
        }
        if(!target_frame.IsPreprocessedSparse())
        {
            feature_extractor->setMaxFeatures(feature_number);
            feature_extractor->detectAndCompute(target_frame.rgb, cv::noArray(), target_keypoints, target_descriptors);
            GetLocalPointsFromKeyPoints(target_frame.depth, target_keypoints,target_local_points);
        }


#if 0
        std::cout << "extract features.."<<std::endl;
        cv::Mat outimg1,outimg2;
        cv::drawKeypoints(source_color,source_keypoints,outimg1,cv::Scalar::all(-1),cv::DrawMatchesFlags::DEFAULT);
        cv::drawKeypoints(target_color,target_keypoints,outimg2,cv::Scalar::all(-1),cv::DrawMatchesFlags::DEFAULT);
        cv::imshow("source",outimg1);
        cv::imshow("target",outimg2);
        cv::waitKey(0);
        cv::destroyWindow("source");
        cv::destroyWindow("target");
#endif



        sparse_matcher.Match(source_descriptors, target_descriptors, matches);

        outlier_filter.RemoveInvalidMatches(source_local_points, target_local_points, matches);

        outlier_filter.RanSaPC(source_local_points,target_local_points,engine,matches);
        outlier_filter.RanSaPC(source_local_points,target_local_points,engine,matches);
        outlier_filter.RanSaPC(source_local_points,target_local_points,engine,matches);
        outlier_filter.RanSaPC(source_local_points,target_local_points,engine,matches);
        outlier_filter.RanSaPC(source_local_points,target_local_points,engine,matches);

        GetCorrespondencesFromMatches(source_local_points, target_local_points, matches, fmatch_set, correspondence_set);
        ComputeTransformation(correspondence_set, fmatch_set, transformation);
        matches.clear();
        sparse_matcher.RefineMatches(source_descriptors,source_local_points,source_keypoints,target_keypoints,transformation,matches);

        outlier_filter.RanSaPC(source_local_points,target_local_points,engine,matches);
        outlier_filter.RanSaPC(source_local_points,target_local_points,engine,matches);
        outlier_filter.RanSaPC(source_local_points,target_local_points,engine,matches);
        outlier_filter.RanSaPC(source_local_points,target_local_points,engine,matches);
        outlier_filter.RanSaPC(source_local_points,target_local_points,engine,matches);
        GetCorrespondencesFromMatches(source_local_points, target_local_points, matches, fmatch_set, correspondence_set);
         
#if 0
        std::cout << "feature matching.."<<std::endl;
        cv::Mat img_good_matches;
        cv::drawMatches(source_frame.rgb,source_keypoints,target_frame.rgb,target_keypoints,matches,img_good_matches);        
        cv::imshow("matching_result",img_good_matches);
        cv::waitKey(0);
        cv::destroyWindow("matching_result");
#endif
        
        auto is_success = ComputeTransformation(correspondence_set, fmatch_set, transformation);
        // std::cout<<(correspondence_set.size()+0.0) / matches.size()<<std::endl;
        tracking_result.T = transformation;
        tracking_result.correspondence_set = correspondence_set; 
        tracking_result.correspondence_set_index = fmatch_set;       
        tracking_result.tracking_success = is_success;
        tracking_result.rmse = geometry::ComputeReprojectionError3D(correspondence_set, transformation);
        return std::make_shared<SparseTrackingResult>(tracking_result);
    }
    void Odometry::CreateImagePyramid(const cv::Mat& color,  const cv::Mat& depth, 
        std::vector<cv::Mat> &color_pyramid, std::vector<cv::Mat> &depth_pyramid, 
        std::vector<cv::Mat> &color_dx_pyramid, std::vector<cv::Mat> &color_dy_pyramid,
        std::vector<cv::Mat> &depth_dx_pyramid, std::vector<cv::Mat> &depth_dy_pyramid)
    {
        tool::CreatePyramid(color, color_pyramid, multi_scale_level);
        tool::CreatePyramid(depth, depth_pyramid, multi_scale_level);

        tool::SobelFiltering(depth_pyramid, depth_dx_pyramid,'x');
        tool::SobelFiltering(depth_pyramid, depth_dy_pyramid,'y');

        tool::SobelFiltering(color_pyramid, color_dx_pyramid,'x');
        tool::SobelFiltering(color_pyramid, color_dy_pyramid,'y');
    }

    void Odometry::CreateImageXYZPyramid(const std::vector<cv::Mat> &depth_pyramid, 
        const std::vector<camera::PinholeCamera > &pyramid_cameras, 
        std::vector<geometry::ImageXYZ> &source_XYZ)
    {
        source_XYZ.clear();
        source_XYZ.resize(pyramid_cameras.size());
        for(int i = 0; i != pyramid_cameras.size(); ++i)
        {
            geometry::TransformToMatXYZ(depth_pyramid[i], pyramid_cameras[i], source_XYZ[i]);
        }
    }
    
    std::shared_ptr<DenseTrackingResult> Odometry::DenseTracking(const cv::Mat &source_color, const cv::Mat &target_color, 
        const cv::Mat &source_depth, const cv::Mat &target_depth, 
        const geometry::TransformationMatrix &initial_T, int term_type)
    {
        //Real-time visual odometry from dense RGB-D images.
        cv::Mat source_gray;
        cv::Mat source_refined_depth;
        cv::Mat target_gray;
        cv::Mat target_refined_depth;
        
        geometry::PixelCorrespondenceSet pixel_correspondence_set;
        geometry::PointCorrespondenceSet correspondence_set;
        geometry::TransformationMatrix transformation = initial_T;
        DenseTrackingResult tracking_result;
        //std::cout<<"Initialize Dense Tracking.."<<std::endl;
        InitializeRGBDDenseTracking(source_color,source_depth,source_gray,source_refined_depth);
        InitializeRGBDDenseTracking(target_color,target_depth,target_gray,target_refined_depth);
        geometry::PixelCorrespondenceSet correspondences;
        
        ComputeCorrespondencePixelWise(source_refined_depth,target_refined_depth,camera,geometry::TransformationMatrix::Identity(),correspondences);
        NormalizeIntensity(source_gray,target_gray,correspondences);

#if DEBUG_MODE
        if(term_type == 0 )
        std::cout<<BLUE<<"[DEBUG]::Using hybrid term"<<RESET<<std::endl;
        else if(term_type == 1)
        std::cout<<BLUE<<"[DEBUG]::Using photo term"<<RESET<<std::endl;
        else if(term_type == 2)
        std::cout<<BLUE<<"[DEBUG]::Using geometry term"<<RESET<<std::endl;
#endif
        auto camera_pyramid = CreatePyramidCameras();
        
        std::vector<cv::Mat> source_color_pyramid, source_depth_pyramid, 
            source_color_dx_pyramid, source_color_dy_pyramid, source_depth_dx_pyramid, source_depth_dy_pyramid;

        std::vector<cv::Mat> target_color_pyramid, target_depth_pyramid, 
            target_color_dx_pyramid, target_color_dy_pyramid, target_depth_dx_pyramid, target_depth_dy_pyramid;    
        
        std::vector<geometry::ImageXYZ> source_xyz_pyramid, target_xyz_pyramid;
        CreateImagePyramid(source_gray, source_refined_depth, source_color_pyramid, source_depth_pyramid,
            source_color_dx_pyramid, source_color_dy_pyramid, source_depth_dx_pyramid, source_depth_dy_pyramid);

        CreateImagePyramid(target_gray, target_refined_depth, target_color_pyramid, target_depth_pyramid,
            target_color_dx_pyramid, target_color_dy_pyramid, target_depth_dx_pyramid, target_depth_dy_pyramid);

        CreateImageXYZPyramid(source_depth_pyramid, camera_pyramid, source_xyz_pyramid);
        CreateImageXYZPyramid(target_depth_pyramid, camera_pyramid, target_xyz_pyramid);
         
        bool is_success = MultiScaleComputing(source_color_pyramid, target_color_pyramid, source_depth_pyramid, target_depth_pyramid,
            target_depth_dx_pyramid, target_depth_dy_pyramid, target_color_dx_pyramid, target_color_dy_pyramid,
            source_xyz_pyramid, target_xyz_pyramid, camera_pyramid, transformation, correspondence_set, 
            pixel_correspondence_set, term_type);
        

        tracking_result.T = transformation;
        tracking_result.correspondence_set = correspondence_set; 
        tracking_result.pixel_correspondence_set = pixel_correspondence_set;       
        tracking_result.tracking_success = is_success;
        tracking_result.rmse = geometry::ComputeReprojectionError3D(correspondence_set, transformation);
        return std::make_shared<DenseTrackingResult>(tracking_result);
    }


    std::shared_ptr<DenseTrackingResult> Odometry::DenseTracking(geometry::RGBDFrame &source_frame, geometry::RGBDFrame &target_frame,
        const geometry::TransformationMatrix &initial_T, int term_type)
    {
        //Real-time visual odometry from dense RGB-D images.

        cv::Mat &source_color = source_frame.rgb;
        cv::Mat &target_color = target_frame.rgb;
        cv::Mat &source_depth = source_frame.depth;
        cv::Mat &target_depth = target_frame.depth;


        geometry::PointCorrespondenceSet correspondence_set;
        geometry::TransformationMatrix transformation = initial_T;
        DenseTrackingResult tracking_result;
        geometry::PixelCorrespondenceSet pixel_correspondence_set;
        //std::cout<<"Initialize Dense Tracking.."<<std::endl;
        // InitializeRGBDDenseTracking(source_color,source_depth,target_color,target_depth,
        // source_gray,source_refined_depth, target_gray,target_refined_depth);
#if DEBUG_MODE
        if(term_type == 0 )
        std::cout<<BLUE<<"[DEBUG]::Using hybrid term"<<RESET<<std::endl;
        else if(term_type == 1)
        std::cout<<BLUE<<"[DEBUG]::Using photo term"<<RESET<<std::endl;
        else if(term_type == 2)
        std::cout<<BLUE<<"[DEBUG]::Using geometry term"<<RESET<<std::endl;
#endif
        auto camera_pyramid = CreatePyramidCameras();
        
        std::vector<cv::Mat> & source_color_pyramid = source_frame.color_pyramid;
        std::vector<cv::Mat> & source_depth_pyramid = source_frame.depth_pyramid;
        std::vector<cv::Mat> & source_color_dx_pyramid = source_frame.color_dx_pyramid;
        std::vector<cv::Mat> & source_color_dy_pyramid = source_frame.color_dy_pyramid;
        std::vector<cv::Mat> & source_depth_dx_pyramid = source_frame.depth_dx_pyramid;
        std::vector<cv::Mat> & source_depth_dy_pyramid = source_frame.depth_dy_pyramid;
        std::vector<geometry::ImageXYZ> &source_xyz_pyramid = source_frame.image_xyz;

 
        std::vector<cv::Mat> & target_color_pyramid = target_frame.color_pyramid;
        std::vector<cv::Mat> & target_depth_pyramid = target_frame.depth_pyramid;
        std::vector<cv::Mat> & target_color_dx_pyramid = target_frame.color_dx_pyramid;
        std::vector<cv::Mat> & target_color_dy_pyramid = target_frame.color_dy_pyramid;
        std::vector<cv::Mat> & target_depth_dx_pyramid = target_frame.depth_dx_pyramid;
        std::vector<cv::Mat> & target_depth_dy_pyramid = target_frame.depth_dy_pyramid;
        std::vector<geometry::ImageXYZ> &target_xyz_pyramid = target_frame.image_xyz;

        if(!source_frame.IsPreprocessedDense())
        {
            InitializeRGBDDenseTracking(source_color,source_depth,source_frame.gray,source_frame.depth32f);
        
            CreateImagePyramid(source_frame.gray,source_frame.depth32f, source_color_pyramid, source_depth_pyramid,
                source_color_dx_pyramid, source_color_dy_pyramid, source_depth_dx_pyramid, source_depth_dy_pyramid);
            CreateImageXYZPyramid(source_depth_pyramid, camera_pyramid, source_xyz_pyramid);
        }
        
        if(!target_frame.IsPreprocessedDense())
        {
            InitializeRGBDDenseTracking(target_color,target_depth,target_frame.gray,target_frame.depth32f);

            CreateImagePyramid(target_frame.gray,target_frame.depth32f, target_color_pyramid, target_depth_pyramid,
                target_color_dx_pyramid, target_color_dy_pyramid, target_depth_dx_pyramid, target_depth_dy_pyramid);
            CreateImageXYZPyramid(target_depth_pyramid, camera_pyramid, target_xyz_pyramid);
        }
        cv::Mat source_gray = source_frame.gray;
        cv::Mat source_refined_depth = source_frame.depth32f;
        cv::Mat target_gray = target_frame.gray;
        cv::Mat target_refined_depth = target_frame.depth32f;
        geometry::PixelCorrespondenceSet correspondences;
        
        ComputeCorrespondencePixelWise(source_refined_depth,target_refined_depth,camera,geometry::TransformationMatrix::Identity(),correspondences);
        NormalizeIntensity(source_gray,target_gray,correspondences);
        bool is_success = MultiScaleComputing(source_color_pyramid, target_color_pyramid, source_depth_pyramid, target_depth_pyramid,
            target_depth_dx_pyramid, target_depth_dy_pyramid, target_color_dx_pyramid, target_color_dy_pyramid,
            source_xyz_pyramid, target_xyz_pyramid, camera_pyramid, transformation, correspondence_set, 
            pixel_correspondence_set, term_type);
        
       // std::cout<<(correspondence_set.size()+0.0) / matches.size()<<std::endl;
        tracking_result.T = transformation;
        tracking_result.correspondence_set = correspondence_set;        
        tracking_result.tracking_success = is_success;
        tracking_result.pixel_correspondence_set = pixel_correspondence_set;    
        tracking_result.rmse = geometry::ComputeReprojectionError3D(correspondence_set, transformation);
        return std::make_shared<DenseTrackingResult>(tracking_result);
    }
    void Odometry::InitializeRGBDDenseTracking(const cv::Mat &color, const cv::Mat &depth, 
        cv::Mat &gray, cv::Mat &refined_depth)
    {
    //frame precessing, including transfer to intensity, and filtering
        cv::Mat refined_depth_tmp , target_refined_depth_tmp, intensity_tmp, target_intensity_tmp;
        ConvertColorToIntensity32F(color, intensity_tmp,255.0);
        ConvertDepthTo32FNaN(depth, refined_depth_tmp,camera.GetDepthScale());
        tool::GaussianFiltering(intensity_tmp,gray);
        //refined_depth = refined_depth_tmp;
        //target_refined_depth = target_refined_depth_tmp;
        tool::GaussianFiltering(refined_depth_tmp,refined_depth);
    }
    bool Odometry::MultiScaleComputing(const std::vector<cv::Mat> &source_color_pyramid,
        const std::vector<cv::Mat> &target_color_pyramid, 
        const std::vector<cv::Mat> &source_depth_pyramid, const std::vector<cv::Mat> &target_depth_pyramid, 
        const std::vector<cv::Mat> &target_depth_dx_pyramid, const std::vector<cv::Mat> &target_depth_dy_pyramid, 
        const std::vector<cv::Mat> &target_color_dx_pyramid, const std::vector<cv::Mat> &target_color_dy_pyramid, 
        const std::vector<geometry::ImageXYZ> &source_image_xyz_pyramid, 
        const std::vector<geometry::ImageXYZ> &target_image_xyz_pyramid,
        const std::vector<camera::PinholeCamera> &camera_pyramid, geometry::TransformationMatrix &T, 
        geometry::PointCorrespondenceSet &correspondence_set, 
        geometry::PixelCorrespondenceSet &pixel_correspondence_set, int use_hybrid)
    {

        //compute pyramid
        float fx = camera.GetFx(), fy = camera.GetFy(), cx = camera.GetCx(), cy = camera.GetCy();   
        int width = camera.GetWidth(), height = camera.GetHeight();

        geometry::PixelCorrespondenceSet correspondences;
        
        
        for(int i = multi_scale_level-1;i>=0;--i)
        {
            //std::cout<<"Scale "<<i<<"..."<<std::endl;
            //std::cout<<"Getting XYZ:"<<std::endl;
            
            //tool::PrintMat(source_depth_pyramid[i]);
            for(int j=0;j!= iter_count_per_level[i]; ++j)
            {
                correspondences.clear();
                if(use_hybrid == 0)
                {
                    DoSingleIteration(source_color_pyramid[i], source_depth_pyramid[i], target_color_pyramid[i], target_depth_pyramid[i],
                        target_color_dx_pyramid[i], target_depth_dx_pyramid[i],
                        target_color_dy_pyramid[i], target_depth_dy_pyramid[i], 
                        source_image_xyz_pyramid[i], camera_pyramid[i], T, correspondences);
                }
                else if(use_hybrid == 1)
                {
                    DoSingleIterationPhotoTerm(source_color_pyramid[i], source_depth_pyramid[i], target_color_pyramid[i], target_depth_pyramid[i],
                        target_color_dx_pyramid[i], target_color_dy_pyramid[i], 
                        source_image_xyz_pyramid[i], camera_pyramid[i], T, correspondences);
                }
                else if(use_hybrid == 2)
                {
                    DoSingleIterationDepthTerm(source_color_pyramid[i], source_depth_pyramid[i], target_color_pyramid[i], target_depth_pyramid[i],
                        target_depth_dx_pyramid[i], target_depth_dy_pyramid[i], 
                        source_image_xyz_pyramid[i], camera_pyramid[i], T, correspondences);                    
                }
                if((float)correspondences.size()/(height * width) > MAX_INLIER_RATIO_DENSE)
                break;
            }
        }
        // int step = correspondences.size()/feature_number;

        for(int i = 0; i<correspondences.size(); ++i)
        {
            int v_s = correspondences[i].first(0);
            int u_s = correspondences[i].first(1);
            int v_t = correspondences[i].second(0);
            int u_t = correspondences[i].second(1);
            correspondence_set.push_back(std::make_pair(source_image_xyz_pyramid[0][v_s][u_s], 
                target_image_xyz_pyramid[0][v_s][u_s]));
        }
        pixel_correspondence_set = correspondences;
        return (float)correspondences.size()/(height * width) >= MIN_INLIER_RATIO_DENSE;
    }
};
}

