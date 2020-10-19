#ifndef SPARSE_ODOMETRY_FUNCTION_H
#define SPARSE_ODOMETRY_FUNCTION_H
#include "Geometry/Geometry.h"
#include "OdometryPredefined.h"
#include <random>
namespace one_piece
{
namespace odometry
{
    class OutlierFilter
    {
        public:
        // remove the matches who has one point out the frustum
        void RemoveInvalidMatches(const geometry::Point3List &source_local_points, 
            const geometry::Point3List &target_local_points, std::vector< cv::DMatch > &init_matches)
        {
            int ptr = 0;
            for(size_t i = 0; i < init_matches.size(); ++i)
            {
                auto source = source_local_points[init_matches[i].queryIdx];
                auto target = target_local_points[init_matches[i].trainIdx];
                if(source(2) < MIN_DEPTH || target(2) < MIN_DEPTH || source(2) > MAX_DEPTH || target(2) > MAX_DEPTH)
                continue;
                init_matches[ptr] = init_matches[i];
                ptr ++;
            }
            init_matches.resize(ptr);
        }
        void MinDistance(const geometry::KeyPointSet &source_keypoints, const geometry::KeyPointSet &target_keypoints,geometry::DMatchSet & matches)
        {
            /*the most naive way*/
            float mindist = 1000000;
            for(size_t i = 0;i!=matches.size();i++)
            {
                float dist = matches[i].distance;
                
                if(dist < mindist) mindist = dist;
            }         
            if(mindist < 20) mindist = 20;
            std::cout<<"MinDistance: "<<mindist<<std::endl;
            geometry::DMatchSet good_matches;
            for(size_t i = 0;i!=matches.size();++i)
            {
                if(matches[i].distance <= 2*mindist)
                    good_matches.push_back(matches[i]);
            }
            matches = good_matches;
        }
        //Randomly Sample a pair of points, and test if the pair is consistent, similar to RANSAC
        void RanSaPC(const geometry::Point3List &source_local_points, 
            const geometry::Point3List &target_local_points, std::default_random_engine &engine, std::vector< cv::DMatch > &init_matches)
        {
            size_t candidate_num = 8;
            float distance_threshold = 0.015;
            size_t N = init_matches.size();
            std::vector< cv::DMatch > filtered_matches;
            filtered_matches.reserve(N);
            std::uniform_int_distribution<int> uniform(0,N-1);
            for (size_t i = 0; i < N; i++)
            {
                geometry::Point3 ref_point = source_local_points[init_matches[i].queryIdx];
                geometry::Point3 new_point = target_local_points[init_matches[i].trainIdx];

                int distance_preserve_flag = 0;
                for (size_t j = 0; j < candidate_num; j++)
                {
                    int rand_choice = uniform(engine);
                    geometry::Point3 ref_point_p = source_local_points[init_matches[rand_choice].queryIdx];
                    geometry::Point3 new_point_p = target_local_points[init_matches[rand_choice].trainIdx];
                    float d1 = (ref_point_p - ref_point).norm();
                    float d2 = (new_point_p - new_point).norm();
                    if (fabs(d1 - d2) / ref_point(2) < distance_threshold)
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
        //use 2nn. Only use for BFMatcher
        void KnnMatch(const geometry::Descriptor &source, geometry::Descriptor & target, const cv::BFMatcher &matcher, 
            std::vector< cv::DMatch > &matches, float minRatio = 1.5)
        {
            matches.clear();
            std::vector<std::vector<cv::DMatch>> knnMatches;
            matcher.knnMatch(source, target, knnMatches, 2);
            for(size_t i = 0; i < knnMatches.size(); ++i)
            {
                const cv::DMatch & bestMatch = knnMatches[i][0];
                const cv::DMatch & betterMatch = knnMatches[i][1];
                float  distanceRatio = betterMatch.distance / bestMatch.distance;
                if (distanceRatio >= minRatio)
                    matches.push_back(bestMatch);
            }
        }
        
        //Ransac
        void Ransac(const geometry::KeyPointSet &source_keypoints, const geometry::KeyPointSet &target_keypoints,geometry::DMatchSet & matches)
        {

            //Prepare data for findHomography
            std::vector<cv::Point2f> source_points(matches.size());
            std::vector<cv::Point2f> target_points(matches.size());

            for (size_t i = 0; i < matches.size(); i++) 
            {
                source_points[i] = source_keypoints[matches[i].queryIdx].pt;
                target_points[i] = target_keypoints[matches[i].trainIdx].pt;
            }

            //find homography matrix and get inliers mask
            std::vector<uchar> inlier_mask(source_points.size());
            cv::findHomography(source_points, target_points, CV_FM_RANSAC, REPROJECTION_ERROR_2D_THRESHOLD, inlier_mask);

            std::vector<cv::DMatch> inliers;
            for (size_t i = 0; i < inlier_mask.size(); i++){
                if (inlier_mask[i])
                    inliers.push_back(matches[i]);
            }
            matches.swap(inliers);
        }
    };

    bool RANSAC3d(geometry:: PointCorrespondenceSet & correspondence_set, 
        geometry::FMatchSet &matches, 
        std::default_random_engine &engine, 
        geometry::TransformationMatrix &T);
    float ReprojectionError3D(const geometry::TransformationMatrix &T, const geometry::Point3 &source, 
        const geometry::Point3 &target);
    std::vector<int> CountInlier(const geometry::TransformationMatrix &T, const geometry::PointCorrespondenceSet & correspondence_set);
}
}
#endif