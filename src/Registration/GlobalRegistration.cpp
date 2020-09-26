#include "GlobalRegistration.h"
#include <opencv2/core/eigen.hpp>
namespace fucking_cool
{
namespace registration
{
    float ComputeRMSE(const geometry::PointCorrespondenceSet &inliers, 
        const geometry::TransformationMatrix &T)
    {
        float sum_error = 0.0;
        for(size_t i = 0; i != inliers.size(); ++i)
            sum_error += (T.block<3,3>(0,0) * inliers[i].first + T.block<3,1>(0,3) - inliers[i].second).squaredNorm();
            //std::cout<<sum_error<<std::endl;
        return sqrt(sum_error / inliers.size());
    }
    void Eigen2OpenCV(const FeatureSet &features, std::vector<cv::Vec<float, 33>> &cv_features)
    {
        cv_features.clear();
        for(size_t i = 0; i != features.size(); ++i)
        {
            cv::Mat feature_cv;
            //std::cout<<features[i].transpose()<<std::endl;
            cv::eigen2cv(features[i], feature_cv);
            //feature_cv.reshape(1);
            cv_features.push_back(feature_cv);
            //std::cout<<" "<<feature_cv<<std::endl;
        }
    }
    void FeatureMatching3D(const FeatureSet &source_feature, const FeatureSet &target_feature, 
        geometry::FMatchSet &matching_index)
    {
        std::vector<std::vector<int>> similar_features;
        
        similar_features.resize(source_feature.size());
        // std::vector<cv::Vec<float, 33>>  target_feature_cv;
        
        // Eigen2OpenCV(target_feature, target_feature_cv);
        
        //target_feature_cv.resize(100);
#if DEBUG_MODE
        std::cout<<BLUE<<"[DEBUG]::[FPFHFeature]::Build KDTree."<<RESET<<std::endl;
#endif
        // cv::flann::Index kdtree(cv::Mat(target_feature_cv).reshape(1), indexParams);    
        geometry::KDTree<33> kdtree;
        kdtree.BuildTree(target_feature);
        int k = 1;

        for(size_t i = 0; i != source_feature.size(); ++i)
        {
            // cv::Mat query;
            // cv::eigen2cv(source_feature[i], query);
            // cv::Vec<float, 33> query_v = query;
            //std::cout<<query<<std::endl;
            //std::cout<<source_feature[i].transpose()<<std::endl;
            //
            // std::vector<float> query(source_feature[i].data(), source_feature[i].data() + 
            //     source_feature[i].rows() * source_feature[i].cols());

            std::vector<int> indices; 
            std::vector<float> dists;       

            kdtree.KnnSearch(source_feature[i], indices, dists, k, geometry::SearchParameter(1024));
            similar_features[i] = indices;      
            //std::cout<<i<<" "<<indices.size()<<std::endl;
        }
        matching_index.clear();
        for(size_t i = 0; i != similar_features.size(); ++i)
        {
            if(similar_features[i].size())
            {
                matching_index.push_back(std::make_pair(i, similar_features[i][0]));
            }
        }
    }
    //same algorithm in rejection of 2d matches.
    void RejectMatchesRanSaPC(const geometry::Point3List &source_points, 
        const geometry::Point3List &target_points, std::default_random_engine &engine, 
        geometry::FMatchSet &init_matches, int candidate_num, float difference)
    {
        size_t N = init_matches.size();
        geometry::FMatchSet filtered_matches;
        filtered_matches.reserve(N);
        std::uniform_int_distribution<int> uniform(0,N-1);
        for (size_t i = 0; i < N; i++)
        {
            geometry::Point3 ref_point = source_points[init_matches[i].first];
            geometry::Point3 new_point = target_points[init_matches[i].second];

            int distance_preserve_flag = 0;
            for (size_t j = 0; j < static_cast<size_t>(candidate_num); j++)
            {
                int rand_choice = uniform(engine);
                geometry::Point3 ref_point_p = source_points[init_matches[rand_choice].first];
                geometry::Point3 new_point_p = target_points[init_matches[rand_choice].second];
                float d1 = (ref_point_p - ref_point).norm();
                float d2 = (new_point_p - new_point).norm();
                if (fabs(d1 - d2) <= difference * d1)
                {
                    distance_preserve_flag = 1;
                    break;
                }
            }
            if (distance_preserve_flag)
            {
                filtered_matches.push_back(init_matches[i]);
            }
        }
        init_matches = filtered_matches;
    }
    //For debug
    void SaveEigenMatrix(const std::string&filename, const FeatureSet &mat)
    {
        std::ofstream ofs(filename);
        ofs<<mat.size()<<" "<<mat[0].rows()<<std::endl;
        for(size_t i = 0; i != mat.size(); ++ i)
        {
            ofs<<mat[i].transpose()<<std::endl;
        }
        ofs.close();
    }
    std::shared_ptr<RegistrationResult> RansacRegistration(const geometry::PointCloud &source_pcd, const geometry::PointCloud &target_pcd, 
        const RANSACParameter &r_para)
    {
#if DEBUG_MODE
        std::cout<<BLUE<<"[DEBUG]::[GlobalRegistrationRANSAC]::Downsampling, voxel_len: "<<r_para.voxel_len<<RESET<<std::endl;
#endif
        auto source_ptr = source_pcd.DownSample(r_para.voxel_len);
        auto target_ptr = target_pcd.DownSample(r_para.voxel_len);
#if DEBUG_MODE
        std::cout<<BLUE<<"[DEBUG]::[GlobalRegistrationRANSAC]::Source PCD: "<<source_ptr->points.size()<<RESET<<std::endl;
        std::cout<<BLUE<<"[DEBUG]::[GlobalRegistrationRANSAC]::Target PCD: "<<target_ptr->points.size()<<RESET<<std::endl;
#endif
#if DEBUG_MODE
        std::cout<<BLUE<<"[DEBUG]::[GlobalRegistrationRANSAC]::Estimating normals..."<<RESET<<std::endl;
#endif
        if(!source_ptr->HasNormals()) source_ptr->EstimateNormals(r_para.search_radius_normal, r_para.max_nn_normal);
        if(!target_ptr->HasNormals()) target_ptr->EstimateNormals(r_para.search_radius_normal, r_para.max_nn_normal);
        FeatureSet source_features;
        FeatureSet target_features;
#if DEBUG_MODE
        std::cout<<BLUE<<"[DEBUG]::[GlobalRegistrationRANSAC]::Compute Features..."<<RESET<<std::endl;
#endif
        ComputeFPFHFeature(*source_ptr, source_features, r_para.max_nn, r_para.search_radius);
        ComputeFPFHFeature(*target_ptr, target_features, r_para.max_nn, r_para.search_radius);
        // SaveEigenMatrix("source_fpfh.em", source_features);
        // SaveEigenMatrix("target_fpfh.em", target_features);
        geometry::FMatchSet matches;
#if DEBUG_MODE
        std::cout<<BLUE<<"[DEBUG]::[GlobalRegistrationRANSAC]::Feature Matching..."<<RESET<<std::endl;
#endif
        FeatureMatching3D(source_features, target_features, matches);
        std::default_random_engine engine;
#if DEBUG_MODE
        std::cout<<BLUE<<"[DEBUG]::[GlobalRegistrationRANSAC]::Reject False Matches..."<<RESET<<std::endl;
        int before = matches.size();
#endif
        RejectMatchesRanSaPC(source_ptr->points, target_ptr->points, engine, matches);
        RejectMatchesRanSaPC(source_ptr->points, target_ptr->points, engine, matches);
        RejectMatchesRanSaPC(source_ptr->points, target_ptr->points, engine, matches);
#if DEBUG_MODE
        int after = matches.size();
        std::cout<<BLUE<<"[DEBUG]::[GlobalRegistrationRANSAC]::Before/After: "<<before<<"/"<<after<<RESET<<std::endl;
#endif        
        geometry::PointCorrespondenceSet correspondence_set;
        for(size_t i = 0; i != matches.size(); ++i)
        {
            correspondence_set.push_back(std::make_pair(source_ptr->points[matches[i].first], 
                target_ptr->points[matches[i].second]));
        };
        geometry::PointCorrespondenceSet inliers;
#if DEBUG_MODE
        std::cout<<BLUE<<"[DEBUG]::[GlobalRegistrationRANSAC]::Ransac Estimation..."<<RESET<<std::endl;
#endif
        std::vector<int> inlier_ids;
        auto T = geometry::EstimateRigidTransformationRANSAC(correspondence_set, inliers, inlier_ids,
            r_para.max_iteration, r_para.threshold);
        RegistrationResult result;
        result.T = T;
        result.correspondence_set = inliers;
        result.rmse = ComputeRMSE(inliers, T);
        for(size_t i = 0; i < inlier_ids.size(); ++i)
        result.correspondence_set_index.push_back(matches[inlier_ids[i]]);
#if DEBUG_MODE
        std::cout<<BLUE<<"[DEBUG]::[GlobalRegistrationRANSAC]::Inliers: "<<inliers.size()<<RESET<<std::endl;
        std::cout<<BLUE<<"[DEBUG]::[GlobalRegistrationRANSAC]::Inlier RMSE: "<<result.rmse<<RESET<<std::endl;
#endif
        //result.matches = matches;
        return std::make_shared<RegistrationResult>(result);
    }
    std::tuple<geometry::PointCloud, FeatureSet> DownSampleAndExtractFeature(const geometry::PointCloud &pcd, 
        const RANSACParameter &r_para)
    {
#if DEBUG_MODE
        std::cout<<BLUE<<"[DEBUG]::[DownSampleAndExtractFeature]::Downsampling, voxel_len: "<<r_para.voxel_len<<RESET<<std::endl;
#endif
        auto ptr = pcd.DownSample(r_para.voxel_len);
        if(!ptr->HasNormals()) ptr->EstimateNormals(r_para.search_radius_normal, r_para.max_nn_normal);
        FeatureSet features;
        ComputeFPFHFeature(*ptr, features, r_para.max_nn, r_para.search_radius);
        return std::make_tuple(*ptr, features);
    }
    
    std::shared_ptr<RegistrationResult> RansacRegistration(const geometry::PointCloud &source_feature_pcd, 
        const geometry::PointCloud &target_feature_pcd, 
        const FeatureSet & source_features,
        const FeatureSet & target_features,
        const RANSACParameter &r_para)
    {
        geometry::FMatchSet matches;
        FeatureMatching3D(source_features, target_features, matches);
        std::default_random_engine engine;
#if DEBUG_MODE
        std::cout<<BLUE<<"[DEBUG]::[GlobalRegistrationRANSAC]::Reject False Matches..."<<RESET<<std::endl;
        int before = matches.size();
#endif
        RejectMatchesRanSaPC(source_feature_pcd.points, target_feature_pcd.points, engine, matches);
        RejectMatchesRanSaPC(source_feature_pcd.points, target_feature_pcd.points, engine, matches);
        RejectMatchesRanSaPC(source_feature_pcd.points, target_feature_pcd.points, engine, matches);
#if DEBUG_MODE
        int after = matches.size();
        std::cout<<BLUE<<"[DEBUG]::[GlobalRegistrationRANSAC]::Before/After: "<<before<<"/"<<after<<RESET<<std::endl;
#endif        
        geometry::PointCorrespondenceSet correspondence_set;
        for(size_t i = 0; i != matches.size(); ++i)
        {
            correspondence_set.push_back(std::make_pair(source_feature_pcd.points[matches[i].first], 
                target_feature_pcd.points[matches[i].second]));
        };
        geometry::PointCorrespondenceSet inliers;
#if DEBUG_MODE
        std::cout<<BLUE<<"[DEBUG]::[GlobalRegistrationRANSAC]::Ransac Estimation..."<<RESET<<std::endl;
#endif
        std::vector<int> inlier_ids;
        auto T = geometry::EstimateRigidTransformationRANSAC(correspondence_set, inliers, inlier_ids,
            r_para.max_iteration, r_para.threshold);
        RegistrationResult result;
        result.T = T;
        result.correspondence_set = inliers;
        result.rmse = ComputeRMSE(inliers, T);
        for(size_t i = 0; i < inlier_ids.size(); ++i)
        result.correspondence_set_index.push_back(matches[inlier_ids[i]]);
#if DEBUG_MODE
        std::cout<<BLUE<<"[DEBUG]::[GlobalRegistrationRANSAC]::Inliers: "<<inliers.size()<<RESET<<std::endl;
        std::cout<<BLUE<<"[DEBUG]::[GlobalRegistrationRANSAC]::Inlier RMSE: "<<result.rmse<<RESET<<std::endl;
#endif
        //result.matches = matches;
        return std::make_shared<RegistrationResult>(result);
    }
}
}
