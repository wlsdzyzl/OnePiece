#ifndef KDTREE_H
#define KDTREE_H
#include "Geometry.h"
#include <opencv2/core/eigen.hpp>
namespace fucking_cool
{
namespace geometry
{
    //kdtree index
    class SearchParameter
    {
        public:
        //kdtree parameter
        //is sorted
        SearchParameter(int _checks = 256, 
            float _eps = 1e-8, bool _sorted = true)
        {
            checks = _checks;
            eps = _eps;
            sorted = _sorted;
        }
        bool sorted = true;
        //eps
        float eps = 1e-8;
        //recursively check
        int checks = 256;
        

    };
    template<int T = 3>
    class KDTree
    {
        public:
        KDTree(int _trees = 4)
        {
            trees = 4;
        }
        void BuildTree(const geometry::PointXList &points)
        {
            std::vector<cv::Vec<float,T>> cv_pcd;
            for(int i = 0; i != points.size(); ++i)
            {
                if(points[i].rows() != T)
                {
                    std::cout<<RED<<"[ERROR]::[BuildKDTree]::The dimension of point is not equal to the dimension of kdtree."<<RESET<<std::endl;
                    exit(0);
                }
                cv::Mat point_cv;
                const Eigen::VectorXf point_f = points[i].cast<float>();
                cv::eigen2cv(point_f, point_cv);
                cv_pcd.push_back(point_cv);
            }
            input_array = cv::Mat(cv_pcd).clone();
            kdtree.build(input_array.reshape(1), cv::flann::KDTreeIndexParams(trees));
        }

        void BuildTree(const geometry::PointList<T> &points)
        {
            std::vector<cv::Vec<float,T>> cv_pcd;
            for(int i = 0; i < points.size(); ++i)
            {
                cv::Mat point_cv;
                const Eigen::VectorXf point_f = points[i].template cast<float>();
                cv::eigen2cv(point_f, point_cv);
                cv_pcd.push_back(point_cv);
            } 

            input_array = cv::Mat(cv_pcd).clone();
            kdtree.build(input_array.reshape(1), cv::flann::KDTreeIndexParams(trees));
            // for(int i = 0; i != indices.size(); ++i)
            // std::cout<<indices[i]<<std::endl;
        }
        //strange rules in opencv flann: 
        //when you do knnsearch, you need to resize(k) at beginning.
        //when you do radius search, you need to get the returned points_count and do a resize for the results.
        void RadiusSearch(const geometry::VectorX &point, std::vector<int> &indices, 
            std::vector<float > &dists, double radius, int max_result, 
            const SearchParameter &sp = SearchParameter())
        {
            
            const Eigen::VectorXf point_f = point.cast<float>();
            std::vector<float> point_cv(point_f.data(), point_f.data() + point_f.rows() * point_f.cols());
            double squared_radius = radius * radius;
            int points_count = kdtree.radiusSearch(point_cv, indices, dists, squared_radius, 
                max_result, cv::flann::SearchParams(sp.checks, sp.eps, sp.sorted));
            indices.resize(points_count);
            dists.resize(points_count);
        }
        void RadiusSearch(const geometry::Vector<T> &point, std::vector<int> &indices, 
            std::vector<float > &dists, double radius, int max_result, 
            const SearchParameter sp = SearchParameter())
        {
            
            //magic code
            Eigen::Matrix<float, T, 1> point_f = point.template cast<float>();
            std::vector<float> point_cv(point_f.data(), point_f.data() + point_f.rows() * point_f.cols());
            double squared_radius = radius * radius;
            
            int points_count = kdtree.radiusSearch(point_cv, indices, dists, squared_radius, 
                max_result, cv::flann::SearchParams(sp.checks, sp.eps, sp.sorted));
            indices.resize(points_count);
            dists.resize(points_count);
        }
        void KnnSearch(const geometry::VectorX &point, std::vector<int> &indices, 
            std::vector<float > &dists, int k, 
            const SearchParameter &sp = SearchParameter())
        {

            indices.resize(k);
            dists.resize(k);
            
            const Eigen::VectorXf point_f = point.cast<float>();

            std::vector<float> point_cv(point_f.data(), point_f.data() + point_f.rows() * point_f.cols());
            kdtree.knnSearch(point_cv, indices, dists, 
                k, cv::flann::SearchParams(sp.checks, sp.eps, sp.sorted));            
        }
        void KnnSearch(const geometry::Vector<T> &point, std::vector<int> &indices, 
            std::vector<float > &dists, int k, 
            const SearchParameter &sp = SearchParameter())
        {
            indices.resize(k);
            dists.resize(k);
            
            Eigen::Matrix<float, T, 1> point_f = point.template cast<float>();
            std::vector<float> point_cv(point_f.data(), point_f.data() + point_f.rows() * point_f.cols());
            kdtree.knnSearch(point_cv, indices, dists, 
                k, cv::flann::SearchParams(sp.checks, sp.eps, sp.sorted));            
        }
        cv::flann::Index kdtree;
        int trees;
        cv::Mat input_array;
    };
}
}
#endif