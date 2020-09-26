#include "ICP.h"
#include <omp.h>
#include "Geometry/KDTree.h"
namespace fucking_cool
{
namespace registration
{
    //count inliers based on the distance, and compute the inlier rmse
    double CountInliers(const geometry::Point3List &source, const geometry::Point3List target, 
        const std::vector<int> &correspondence_index, const geometry::TransformationMatrix & T,  
        double threshold, geometry::FMatchSet &inliers)
    {
        double sum_error = 0;
        inliers.clear();
        double squared_threshold = threshold * threshold;
        for(size_t i = 0; i != correspondence_index.size(); ++i)
        {
            if(correspondence_index[i] != -1)
            {
                double tmp_error = (T.block<3, 3>(0, 0) * source[i] + T.block<3, 1>(0, 3) - 
                    target[correspondence_index[i]]).squaredNorm();
                if(tmp_error < squared_threshold)
                {
                    inliers.push_back(std::make_pair(i, correspondence_index[i]));
                    sum_error += tmp_error;
                }
            }
        }
        return sqrt(sum_error / inliers.size());
    }
    std::shared_ptr<RegistrationResult> PointToPoint(const geometry::PointCloud &source, const geometry::PointCloud &target, 
        const geometry::TransformationMatrix &init_T, const ICPParameter &icp_para)
    {
        // cost function: Tpi - pj
        //find closest point use kd-tree
        auto start_T = init_T;
        RegistrationResult result;

        geometry::FMatchSet inliers;
        // cv::flann::KDTreeIndexParams indexParams; 
        // cv::flann::Index kdtree(cv::Mat(cv_pcd).reshape(1), indexParams);
        geometry::KDTree<> kdtree;
        kdtree.BuildTree(target.points);
        int k = 1;
        int iteration = 0;
        std::vector<int> corresponding_index(source.points.size(), -1);
        while(iteration < icp_para.max_iteration)
        {   
            iteration ++ ;

            for(size_t i = 0; i != corresponding_index.size(); ++i)
            corresponding_index[i] = -1;
            auto transformed_points = source.points;
            geometry::TransformPoints(start_T, transformed_points);
#pragma omp parallel for
            for(size_t i = 0; i < source.points.size(); ++i)
            {
                std::vector<int> indices; 
                std::vector<float> dists; 
                kdtree.KnnSearch(transformed_points[i], indices, dists,k,geometry::SearchParameter(128));
                
                if(indices.size()>0)
                    corresponding_index[i] = indices[0];
            }
            //std::cout<<"fuck"<<std::endl;
            geometry::PointCorrespondenceSet correspondence_set;

            CountInliers(source.points, target.points, corresponding_index, start_T, icp_para.threshold, inliers);
            for(size_t i = 0; i != inliers.size(); ++i)
            {
                correspondence_set.push_back(std::make_pair(transformed_points[inliers[i].first], 
                    target.points[inliers[i].second]));
            }
            auto tmp_T = geometry::EstimateRigidTransformation(correspondence_set);
            start_T = tmp_T * start_T;
            

#if DEBUG_MODE
            std::cout<<BLUE<<"[DEBUG]::[ICPPointToPoint]::iteration "<<iteration<<", inliers "<<inliers.size()<<RESET<<std::endl;
#endif
        }
        
        result.rmse = CountInliers(source.points, target.points, corresponding_index, 
            start_T, icp_para.threshold, inliers);
        result.correspondence_set_index = inliers;
        result.T = start_T;

        for(size_t i = 0; i != inliers.size(); ++i)
        {
            result.correspondence_set.push_back(std::make_pair(source.points[inliers[i].first], 
                target.points[inliers[i].second]));
        }
        return std::make_shared<RegistrationResult>(result);
        //unify, not good.        
    }
    geometry::TransformationMatrix EstimateRigidTransformationPointToPlane(const geometry::Point3List &source, 
        const geometry::Point3List &target, const geometry::Point3List &target_normal, 
        const geometry::FMatchSet &inliers)
    {
        //approximate the rotation matrix to three angle
        //based on paper: Linear least squares-optimization for point to plane ICP surface registration
        //However, if we directly solve the Ax = b, the dimension is related to the number of points, which can be very large.
        //We use a optimization way to solve it, because we need to compute the jacobian matrix, and these jacobian matrix can be added together directly.

        geometry::Matrix6 JTJ = geometry::Matrix6::Zero();
        geometry::Se3 JTr = geometry::Se3::Zero();
        geometry::Se3 x;

        for(size_t i = 0; i != inliers.size(); ++i)
        {
            int source_id = inliers[i].first;
            int target_id = inliers[i].second;

            geometry::Se3 tmp_row;
            double tmp_r = (target_normal[target_id].transpose() * source[source_id] - 
                target_normal[target_id].transpose() * target[target_id])(0);
            //std::cout<<tmp_r<<std::endl;
            tmp_row.block<3, 1>(0,0) = target_normal[target_id]; 
            tmp_row.block<3, 1>(3,0) = source[source_id].cross(target_normal[target_id]);
            
            JTJ.noalias() += tmp_row * tmp_row.transpose();
            JTr.noalias() += tmp_r * tmp_row;
            
        }
        Eigen::JacobiSVD<geometry::MatrixX> svd(JTJ, Eigen::ComputeThinU | Eigen::ComputeThinV);
        x = svd.solve(-JTr);
        //std::cout<<x.transpose()<<std::endl;
        //geometry::TransformationMatrix tmp_T = TransformVector6fToMatrix4f(x);
        //std::cout<<tmp_T<<std::endl;
        //return geometry::TransformVector6fToMatrix4f(x);
        return geometry::Se3ToSE3(x);
    }
    
    std::shared_ptr<RegistrationResult> PointToPlane(const geometry::PointCloud &source, const geometry::PointCloud &target,
        const geometry::TransformationMatrix &init_T, const ICPParameter &icp_para)
    {
        //cost function n(Tpi - pj)
        
        if(!target.HasNormals())
        {
            std::cout<<RED<<"[ERROR]::[ICPPointToPlane]::target point cloud need to have normals."<<RESET<<std::endl;
            return std::make_shared<RegistrationResult>(RegistrationResult());
        }
        
        geometry::FMatchSet inliers;
        
        auto start_T = init_T;
        RegistrationResult result;

        // cv::flann::KDTreeIndexParams indexParams; 
        // cv::flann::Index kdtree(cv::Mat(cv_pcd).reshape(1), indexParams);
        geometry::KDTree<> kdtree;
        kdtree.BuildTree(target.points);
        int k = 1;
        int iteration = 0;
        std::vector<int> corresponding_index(source.points.size(), -1);
        while(iteration < icp_para.max_iteration)
        {   
            iteration ++ ;
            for(size_t i = 0; i != corresponding_index.size(); ++i)
            corresponding_index[i] = -1;
            auto transformed_points = source.points;
            geometry::TransformPoints(start_T, transformed_points);
#pragma omp parallel for
            for(size_t i = 0; i < source.points.size(); ++i)
            {
                std::vector<int> indices(k); 
                std::vector<float> dists(k); 
                kdtree.KnnSearch(transformed_points[i], indices, dists,k,geometry::SearchParameter(128));
                if(indices.size()>0)
                    corresponding_index[i] = indices[0];
            }
            CountInliers(source.points, target.points, corresponding_index, start_T, icp_para.threshold, inliers);

            geometry::TransformationMatrix tmp_T = EstimateRigidTransformationPointToPlane(transformed_points, target.points, 
                target.normals, inliers);
            //std::cout<<tmp_T<<std::endl;
            start_T = tmp_T * start_T;
            
            //std::cout<<start_T<<std::endl;
            
#if DEBUG_MODE
            std::cout<<BLUE<<"[DEBUG]::[ICPPointToPlane]::iteration "<<iteration<<", inliers "<<inliers.size()<<RESET<<std::endl;
#endif
        }
        result.rmse = CountInliers(source.points, target.points, corresponding_index, start_T, icp_para.threshold, inliers);
        result.correspondence_set_index = inliers;
        result.T = start_T;
        for(size_t i = 0; i != inliers.size(); ++i)
        {
            result.correspondence_set.push_back(std::make_pair(source.points[inliers[i].first], 
                target.points[inliers[i].second]));
        }
        return std::make_shared<RegistrationResult>(result);
        
    }    
}
}