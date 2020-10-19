#include "PatchDetection.h"
#include "Geometry/KDTree.h"
#include <algorithm>
#include <queue>
namespace one_piece
{
namespace algorithm
{
    bool CompareResidual(const std::pair<int, double> &a, const std::pair<int, double> &b)
    {
        return a.second < b.second;
    }
    bool IsInlier(const geometry::Point3 &point, const geometry::Vector4 &tangnet, 
        const geometry::Vector4 &plane, float radius)
    {
        float dist = std::fabs( point.transpose() * plane.head<3>() + plane(3));
        float normal_prod = std::fabs(tangnet.head<3>().transpose() * plane.head<3>());
        //A another judgement: is visible.
        return dist <= radius/2 && normal_prod >= 0.8; 
    }
    bool IsInlier(const geometry::Vector2 &point, const geometry::Point3 &tangnet, 
        const geometry::Point3 &plane, float radius)
    {
        float dist = std::fabs( point.transpose() * plane.head<2>() + plane(2));
        float normal_prod = std::fabs(tangnet.head<2>().transpose() * plane.head<2>());
        //A another judgement: is visible.
        return dist <= radius/2 && normal_prod >= 0.8; 
    }
    int ChooseSeed(const std::set<int> &un_visited, const std::vector<double> &residuals)
    {
        std::vector<std::pair<int, double>> seed_r; 
        for(auto iter = un_visited.begin(); iter != un_visited.end(); ++iter)
        {
            seed_r.push_back(std::make_pair(*iter, residuals[*iter]));
        }
        std::sort(seed_r.begin(), seed_r.end(), CompareResidual);
        return seed_r[0].first;
    }
    void LineDetection(const geometry::Point2List &points, 
        std::vector<LinePatch> &results)
    {
        //ComputeNormalResidual
        //Choose Seed
        //Use Seed itelatively grow patch
        results.clear();
        geometry::Point3List k_lines(points.size());
        std::vector<double> residuals(points.size());      
        std::set<int> un_visited;
        std::vector<std::vector<int>> adj_points(points.size());
        for(size_t i = 0; i < points.size(); ++i)
        {
            un_visited.insert(i);
        } 
        // cv::flann::KDTreeIndexParams indexParams; 
        // cv::flann::Index kdtree(cv::Mat(cv_pcd).reshape(1), indexParams);
        geometry::KDTree<2> kdtree;
        kdtree.BuildTree(points);
#if 0 
        int k = 100;
        std::cout<<BLUE<<"[EstimateNormals]::[INFO]::Search "<<k<<" nearest points."<<RESET<<std::endl;
        for(size_t i = 0; i < points.size(); ++i)
        {
            
            std::vector<int> indices; 
            std::vector<float> dists; 
            kdtree.KnnSearch(points[i], indices, dists, k, geometry::SearchParameter(64));

            geometry::Point2List nearest_points;
            for(size_t j = 0; j!= indices.size();++j)
            {
                nearest_points.push_back(points[indices[j]]);
                //std::cout <<indices[j]<<" "<<std::endl;
            }
            //std::cout <<std::endl;
            auto result = geometry::FitLine(nearest_points);
            k_lines[i].block<2,1>(0,0) = std::get<0>(result);
            k_lines[i](2) = std::get<1>(result);
            
            residuals[i] = std::get<2>(result);
            //std::cout<<residuals[i] <<" "<<std::get<2>(result)<<std::endl;
        }
#else 
        float radius = 0.1;
        
        std::cout<<BLUE<<"[LineDetection]::[INFO]::radius: "<<radius<<RESET<<std::endl;
        for(size_t i = 0; i < points.size(); ++i)
        {
            
            std::vector<int> indices; 
            std::vector<float> dists; 
            kdtree.RadiusSearch(points[i], indices, dists, radius ,1024 ,geometry::SearchParameter(1024));

            geometry::Point2List radius_points;
            for(size_t j = 0; j!= indices.size();++j)
            {
                radius_points.push_back(points[indices[j]]);
            }
            adj_points[i] = indices;

            auto result = geometry::FitLine(radius_points);

            
            residuals[i] = std::get<2>(result);
            int remain_iter_times = 2;
            while(remain_iter_times > 0)
            {
                geometry::Point2List inliers;
                geometry::Vector2 n = std::get<0>(result);
                float d  =std::get<1>(result);
                
                for(size_t i = 0; i!= radius_points.size(); ++i)
                {
                    if(std::fabs(n.transpose() * radius_points[i] + d ) <= radius / 2)
                    {
                        inliers.push_back(radius_points[i]);
                    }
                }
                result = geometry::FitLine(inliers);
                radius_points = inliers;
                remain_iter_times -- ;
            }
            k_lines[i].block<2,1>(0,0) = std::get<0>(result);
            k_lines[i](2) = std::get<1>(result);
            //std::cout<<residuals[i] <<" "<<std::get<2>(result)<<std::endl;
        }
#endif
        int iter_th = 0;
        while(un_visited.size() > 0)
        {
            int seed_id = ChooseSeed(un_visited, residuals);
            geometry::Point3 line = k_lines[seed_id];
            geometry::Point2List tmp_points;
            std::vector<int > tmp_indexs;


            float indicator = 1e6;
#if DEBUG_MODE
            std::cout<<BLUE<<"[LineDetection]::[Debug]::"<<iter_th<<"th iteration."<<RESET<<std::endl;
#endif
            iter_th += 1;
#if 1
            //using region growing...       
            std::queue<int> wait_to_search;
            wait_to_search.push(seed_id);
            un_visited.erase(seed_id);
            tmp_points.push_back(points[seed_id]);
            tmp_indexs.push_back(seed_id);  
            while(wait_to_search.size())
            {
                int search_id = wait_to_search.front();
                wait_to_search.pop();
                //int out_lier = 0;
                for(size_t i = 0; i!= adj_points[search_id].size(); ++i)
                {
                    int index = adj_points[search_id][i];
                    if(un_visited.find(index) == un_visited.end())
                    continue;
                    un_visited.erase(index);
                    if(IsInlier(points[index], k_lines[index], line, radius))
                    {
                        wait_to_search.push(index);
                        tmp_points.push_back(points[index]);
                        tmp_indexs.push_back(index);                    
                    }
                }
                if(tmp_points.size() >= 3)
                {
                    auto result = geometry::FitLine(tmp_points);
                    line.block<2,1>(0,0) = std::get<0>(result);
                    line(2) = std::get<1>(result);
                    indicator = std::get<2>(result);
                }
            }
#else
            int add_number = 1000;
            while(add_number >= 100)
            {
            
                std::vector<int> deleted_id;
                
                for(auto iter = un_visited.begin(); iter != un_visited.end(); ++iter)
                {
                    int i = *iter;
                    //std::cout<<residual<<std::endl;
                    if(IsInlier(points[i],k_lines[i], line, radius))
                    {
                        tmp_points.push_back(points[i]);
                        tmp_indexs.push_back(i);
                        deleted_id.push_back(i);
                    }
                }
                for(size_t i = 0; i != deleted_id.size(); ++i)
                {
                    un_visited.erase(deleted_id[i]);
                }
                un_visited.erase(seed_id);
                add_number = deleted_id.size();
                std::cout<<add_number <<" points are deleted from un_visited_set."<<std::endl;
                // update the line
                if(tmp_points.size() >= 2)
                {
                    auto result = geometry::FitLine(tmp_points);
                    line.block<2,1>(0,0) = std::get<0>(result);
                    line(2) = std::get<1>(result);
                    indicator = std::get<2>(result);

                }
            }
        
            std::cout<<BLUE<<"[Debug]::[LineDetection]::The indicator of noise of this line: "
                <<indicator<<RESET<<std::endl;
#endif
#if 1
            if(tmp_points.size() > 500 && indicator < 0.1)
            {
                LinePatch result(line);
                result.indexs = tmp_indexs;
                result.items = tmp_points;
                results.push_back(result);
            } 
#else
            LinePatch result(line);
            result.indexs = tmp_indexs;
            result.items = tmp_points;
            results.push_back(result);
#endif
        }

    }


    void PlaneDetection(const geometry::Point3List &points, 
        std::vector<PlanePatch> &results)
    {
        //ComputeNormalResidual
        //Choose Seed
        //Use Seed itelatively grow patch
        results.clear();
        std::vector<geometry::Vector4> each_plane(points.size());
        std::vector<double> residuals(points.size());      
        std::vector<std::vector<int>> adj_points(points.size());
        std::set<int> un_visited;
        for(size_t i = 0; i < points.size(); ++i)
        {

            un_visited.insert(i);
        } 
        // cv::flann::KDTreeIndexParams indexParams; 
        // cv::flann::Index kdtree(cv::Mat(cv_pcd).reshape(1), indexParams);
        geometry::KDTree<> kdtree;
        kdtree.BuildTree(points);
        //int k = 100;
        float radius = 0.3;
        std::cout<<BLUE<<"[PlaneDetection]::[INFO]::Search "<<radius<<RESET<<std::endl;
        for(size_t i = 0; i < points.size(); ++i)
        {
            std::vector<int> indices; 
            std::vector<float> dists; 
            kdtree.RadiusSearch(points[i], indices, dists, radius ,1024 ,geometry::SearchParameter(1024));

            geometry::Point3List radius_points;
            for(size_t j = 0; j!= indices.size();++j)
            {
                radius_points.push_back(points[indices[j]]);
                //std::cout <<indices[j]<<" "<<std::endl;
            }
            
            adj_points[i] = indices;

            auto result = geometry::FitPlane(radius_points);

            
            residuals[i] = std::get<2>(result);
            int remain_iter_times = 2;
            while(remain_iter_times > 0)
            {
                geometry::Point3List inliers;
                geometry::Point3 n = std::get<0>(result);
                float d  =std::get<1>(result);
                
                for(size_t i = 0; i!= radius_points.size(); ++i)
                {
                    if(std::fabs(n.transpose() * radius_points[i] + d ) <= radius / 2)
                    {
                        inliers.push_back(radius_points[i]);
                    }
                }
                result = geometry::FitPlane(inliers);
                radius_points = inliers;
                remain_iter_times -- ;
            }
            //std::cout<<<<std::endl;
            each_plane[i].block<3,1>(0,0) = std::get<0>(result);
            each_plane[i](3) = std::get<1>(result);
            //std::cout<<residuals[i] <<" "<<std::get<2>(result)<<std::endl;
        }

        int iter_th = 0;
        while(un_visited.size() > 0)
        {
            int seed_id = ChooseSeed(un_visited, residuals);
            geometry::Vector4 plane = each_plane[seed_id];
            geometry::Point3List tmp_points;
            std::vector<int > tmp_indexs;



            float indicator = 1e6;
#if DEBUG_MODE
            std::cout<<BLUE<<"[LineDetection]::[Debug]::"<<iter_th<<"th iteration."<<RESET<<std::endl;
#endif
            iter_th += 1;

#if 1
            //using region growing...       
            std::queue<int> wait_to_search;
            wait_to_search.push(seed_id);
            un_visited.erase(seed_id);
            tmp_points.push_back(points[seed_id]);
            tmp_indexs.push_back(seed_id);  
            while(wait_to_search.size())
            {
                int search_id = wait_to_search.front();
                wait_to_search.pop();
                //int out_lier = 0;
                //std::set<int> inlier_seed;
                for(size_t i = 0; i!= adj_points[search_id].size(); ++i)
                {
                    int index = adj_points[search_id][i];
                    if(un_visited.find(index) == un_visited.end())
                    continue;
                    un_visited.erase(index);
                    if(IsInlier(points[index], each_plane[index], plane, radius))
                    {
                        wait_to_search.push(index);
                        tmp_points.push_back(points[index]);
                        tmp_indexs.push_back(index);                    
                    }
                }
                if(tmp_points.size() >= 3)
                {
                    auto result = geometry::FitPlane(tmp_points);
                    plane.block<3,1>(0,0) = std::get<0>(result);
                    plane(3) = std::get<1>(result);
                    indicator = std::get<2>(result);
                }
            }
#else
            int add_number = 1000;
            while(add_number >= 100)
            {
            
                std::vector<int> deleted_id;
                
                for(auto iter = un_visited.begin(); iter != un_visited.end(); ++iter)
                {
                    int i = *iter;
                    //std::cout<<residual<<std::endl;
                    if(IsInlier(points[i], each_plane[i], plane, radius))
                    {
                        tmp_points.push_back(points[i]);
                        tmp_indexs.push_back(i);
                        deleted_id.push_back(i);
                    }
                }
                for(size_t i = 0; i != deleted_id.size(); ++i)
                {
                    un_visited.erase(deleted_id[i]);
                }
                un_visited.erase(seed_id);
                add_number = deleted_id.size();
                std::cout<<add_number <<" points are deleted from un_visited_set."<<std::endl;
                // update the line
                if(tmp_points.size() >= 3)
                {
                    auto result = geometry::FitPlane(tmp_points);
                    plane.block<3,1>(0,0) = std::get<0>(result);
                    plane(3) = std::get<1>(result);
                    indicator = std::get<2>(result);
                }
            }
#endif
#if 1
            if(tmp_indexs.size() > 500 && indicator < radius)
            {
                PlanePatch result(plane);
                result.indexs = tmp_indexs;
                result.items = tmp_points;
                results.push_back(result);
            } 
#else
            PlanePatch result(plane);
            result.indexs = tmp_indexs;
            result.items = tmp_points;
            results.push_back(result);
#endif
        }

    }
}
}