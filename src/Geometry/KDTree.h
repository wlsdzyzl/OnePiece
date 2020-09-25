#ifndef KDTREE_H
#define KDTREE_H
#include "Geometry.h"
#include <opencv2/core/eigen.hpp>
#include "nanoflann.hpp"
#include <memory>
//use nanoflann library
#define NANO_IMPLAMENTATION 1
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
        int checks = 32;
        

    };
#if NANO_IMPLAMENTATION

    template <int _D>
    struct NanoPointList
    {
        
        geometry::PointList<_D> point_list;

        // Must return the number of data points
        inline size_t kdtree_get_point_count() const { return point_list.size(); }

        // Returns the dim'th component of the idx'th point in the class:
        // Since this is inlined and the "dim" argument is typically an immediate value, the
        //  "if/else's" are actually solved at compile time.
        inline geometry::scalar kdtree_get_pt(const size_t idx, const size_t dim) const
        {
            return point_list[idx][dim];
        }

        // Optional bounding-box computation: return false to default to a standard bbox computation loop.
        //   Return true if the BBOX was already computed by the class and returned in "bb" so it can be avoided to redo it again.
        //   Look at bb.size() to find out the expected dimensionality (e.g. 2 or 3 for point clouds)
        template <class BBOX>
        bool kdtree_get_bbox(BBOX& /* bb */) const { return false; }
    };
#endif
// nanoflann::KDTreeSingleIndexAdaptor<nanoflann::L2_Simple_Adaptor<float, fucking_cool::geometry::NanoPointList<3>, float>, 
//     fucking_cool::geometry::NanoPointList<3>, 3, long unsigned int>::KDTreeSingleIndexAdaptor()
    template<int T = 3>
    class KDTree
    {
        public:
#if NANO_IMPLAMENTATION
        typedef nanoflann::KDTreeSingleIndexAdaptor<
            nanoflann::L2_Simple_Adaptor<geometry::scalar, NanoPointList<T> > ,
            NanoPointList<T>,
            T
            > NanoKDTree;
        typedef std::shared_ptr<NanoKDTree> NanoKDTreePtr; 
        KDTree(int _max_leaf = 10):max_leaf(_max_leaf)
        {
            //kdtree = NanoKDTree();
        }
        void BuildTree(const geometry::PointXList &points)
        {
            nano_point_list.point_list.clear();
            for(int i = 0; i != points.size(); ++i)
            nano_point_list.point_list.push_back(points[i]);
            kdtree_ptr = NanoKDTreePtr( new NanoKDTree(T /*dim*/, nano_point_list, 
                nanoflann::KDTreeSingleIndexAdaptorParams(max_leaf /* max leaf */) ));  
            kdtree_ptr->buildIndex();
        }
        void BuildTree(const geometry::PointList<T> &points)
        {
            nano_point_list.point_list = points;
            kdtree_ptr = NanoKDTreePtr( new NanoKDTree(T /*dim*/, nano_point_list, 
                nanoflann::KDTreeSingleIndexAdaptorParams(max_leaf /* max leaf */) ));
            kdtree_ptr->buildIndex();
        }
        void RadiusSearch(const geometry::VectorX &point, std::vector<int> &indices, 
            std::vector<float > &dists, double radius, size_t max_result, 
            const SearchParameter &sp = SearchParameter())
        {
            std::vector<size_t> _indices;
            RadiusSearch(point, _indices, dists, radius, max_result, sp);
            indices.resize(_indices.size());
            for(int i = 0; i != _indices.size(); ++i)
            indices[i] = static_cast<int> (_indices[i]);
        }
        void RadiusSearch(const geometry::VectorX &point, std::vector<size_t> &indices, 
            std::vector<float > &dists, double radius, size_t max_result, 
            const SearchParameter &sp = SearchParameter())
        {
            if(point.rows() != T)
            {
                std::cout<<RED<<"[ERROR]::[RadiusSearch]::Wrong dimension!"<<RESET<<std::endl;
                return;
            }
            geometry::Vector<T> _point = point;
            RadiusSearch(_point, indices, dists, radius, max_result, sp);
        }
        void RadiusSearch(const geometry::Vector<T> &point, std::vector<int> &indices, 
            std::vector<float > &dists, double radius, size_t max_result, 
            const SearchParameter sp = SearchParameter())
        {
            std::vector<size_t> _indices;
            RadiusSearch(point, _indices, dists, radius, max_result, sp);
            indices.resize(_indices.size());
            for(int i = 0; i != _indices.size(); ++i)
            indices[i] = static_cast<int> (_indices[i]);
        }
        void RadiusSearch(const geometry::Vector<T> &point, std::vector<size_t> &indices, 
            std::vector<float > &dists, double radius, size_t max_result, 
            const SearchParameter sp = SearchParameter())
        {
            std::vector<std::pair<size_t, geometry::scalar> >   ret_matches;
            //std::cout<<"max result: "<<max_result<<std::endl;
            size_t search_num = kdtree_ptr->radiusSearch(point.data(), radius, ret_matches, max_result * 2.5,
                nanoflann::SearchParams(sp.checks, sp.eps, sp.sorted));
            if(search_num > max_result)
            search_num = max_result;
            indices.resize(search_num);
            dists.resize(search_num);

            for (size_t i = 0; i < search_num; ++i)
            {   
                indices[i] = static_cast<float>(ret_matches[i].first);
                dists[i] = ret_matches[i].second;
            }
        }
        void KnnSearch(const geometry::VectorX &point, std::vector<int> &indices, 
            std::vector<float > &dists, int k, 
            const SearchParameter &sp = SearchParameter())
        {
            std::vector<size_t> _indices;
            KnnSearch(point, _indices, dists, k, sp);
            indices.resize(_indices.size());
            for(int i = 0; i != _indices.size(); ++i)
            indices[i] = static_cast<int> (_indices[i]);
        }
        void KnnSearch(const geometry::VectorX &point, std::vector<size_t> &indices, 
            std::vector<float > &dists, int k, 
            const SearchParameter &sp = SearchParameter())
        {
            if(point.rows() != T)
            {
                std::cout<<RED<<"[ERROR]::[KnnSearch]::Wrong dimension!"<<RESET<<std::endl;
                return;
            }
            geometry::Vector<T> _point = point;
            KnnSearch(_point, indices, dists, k, sp);

        }
        void KnnSearch(const geometry::Vector<T> &point, std::vector<int> &indices, 
            std::vector<float > &dists, int k, 
            const SearchParameter &sp = SearchParameter())
        {
            std::vector<size_t> _indices;
            KnnSearch(point, _indices, dists, k, sp);
            indices.resize(_indices.size());
            for(int i = 0; i != _indices.size(); ++i)
            indices[i] = static_cast<int> (_indices[i]);
        }
        void KnnSearch(const geometry::Vector<T> &point, std::vector<size_t> &indices, 
            std::vector<float > &dists, int k, 
            const SearchParameter &sp = SearchParameter())
        {
            indices.resize(k);
            std::vector<geometry::scalar> out_dist_sqr(k);

            const size_t search_num = kdtree_ptr->knnSearch(point.data(), k, &indices[0], &out_dist_sqr[0]);
            
            // In case of less points in the tree than requested: if that happen, we just use searched result.
            // ret_index.resize(num_results);
            // out_dist_sqr.resize(num_results);
            indices.resize(search_num);
            dists.resize(search_num);

            for (size_t i = 0; i < search_num; ++i)
            {
                dists[i] = out_dist_sqr[i];
            }           
        }
        protected:
        NanoKDTreePtr kdtree_ptr;
        int max_leaf = 10;
        NanoPointList<T> nano_point_list;
#else

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
        protected:
        cv::flann::Index kdtree;
        int trees;
        cv::Mat input_array;
#endif
    };
}
}
#endif