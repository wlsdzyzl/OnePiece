#include "SparseOdometryFunction.h"
#include <Eigen/Dense>
#include <iostream>
#include <algorithm>

namespace one_piece
{
namespace odometry
{
    bool RANSAC3d(geometry:: PointCorrespondenceSet & correspondence_set, 
        geometry::FMatchSet &matches, 
        std::default_random_engine &engine, 
        geometry::TransformationMatrix &T)
    {
        //RANSAC.
        int random_match_number = 8;
        int max_iteration = 400;
        int N = correspondence_set.size();
        std::uniform_int_distribution<int> uniform(0,N-1);       
        std::vector<int> max_inliers; 
        std::vector<int> inlier_indices;
        for(int iter = 0;iter<max_iteration;++iter)
        {
            //randomly pick some candidates
            geometry::PointCorrespondenceSet subset;
            geometry::TransformationMatrix temp_T;
            for(int i = 0;i!=random_match_number; ++i)
            {
                subset.push_back(correspondence_set[uniform(engine)]);
            }
            temp_T = geometry::EstimateRigidTransformation(subset);
            inlier_indices = CountInlier(temp_T,correspondence_set); 
            if(inlier_indices.size() > max_inliers.size())
            {
                T = temp_T;
                max_inliers = inlier_indices;
                if((float)max_inliers.size()/ correspondence_set.size() > MAX_INLIER_RATIO_SPARSE)
                break;
            }
        }
        //std::cout<<"best inlier ratio: "<<(float)max_inliers.size()/correspondence_set.size()<<std::endl;
        if((float)max_inliers.size()/correspondence_set.size() < MIN_INLIER_RATIO_SPARSE)
        {
            
            return false;
        }
        for(size_t i = 0; i < max_inliers.size(); ++i)
        {
            correspondence_set[i] = correspondence_set[max_inliers[i]];
            matches[i] = matches[max_inliers[i]];
        }
        correspondence_set.resize(max_inliers.size());
        matches.resize(max_inliers.size());
        return true;
    }
    float ReprojectionError3D(const geometry::TransformationMatrix &T, const geometry::Point3 &source, 
        const geometry::Point3 &target)
    {
        const geometry::Point3 predict_target = (T * geometry::Vector4(source(0),source(1),source(2),1)).head<3>();
        return (predict_target - target).norm()/source(2);
    }
    std::vector<int> CountInlier(const geometry::TransformationMatrix &T, const geometry::PointCorrespondenceSet & correspondence_set)
    {
        std::vector<int> inliers;        
        for(size_t i = 0;i!=correspondence_set.size();++i)
        {
            float error;
            error = ReprojectionError3D(T,correspondence_set[i].first,correspondence_set[i].second);
            
            if(error <= REPROJECTION_ERROR_3D_THRESHOLD) inliers.push_back(i);;
        }
        return inliers;
    }

}
}