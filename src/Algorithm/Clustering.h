#ifndef CLUSTERING_H
#define CLUSTERING_H
#include "Geometry/Geometry.h"
#include "Geometry/KDTree.h"
#include "Tool/ConsoleColor.h"
#include <opencv2/core/core.hpp>
namespace fucking_cool
{
namespace algorithm
{
    //clustering algorithm, we will implement k-means, means-shift, k-medoids clustering
    template <int T > class Cluster
    {

        public:
        Cluster()=default;
        Cluster(const Eigen::Matrix<geometry::scalar,T,1> & c)
        {
            center = c;
        }
        Eigen::Matrix<geometry::scalar,T,1> center;
        std::vector<Eigen::Matrix<geometry::scalar,T,1>, Eigen::aligned_allocator<Eigen::Matrix<geometry::scalar,T,1>> > items;
        std::vector<int > indexs;
    };
    class ClusterDynamic
    {
        public:
        ClusterDynamic()=default;
        ClusterDynamic(const geometry::VectorX & c)
        {
            center = c;
        }
        geometry::VectorX center;
        geometry::PointXList items;
        std::vector<int > indexs;        
    };
    template <int T >
    void KMeansClustering(const std::vector<Eigen::Matrix<geometry::scalar,T,1>, Eigen::aligned_allocator<Eigen::Matrix<geometry::scalar,T,1>> > & wait_to_cluster, std::vector<Cluster<T>> &clustering_result, int K)
    {
        std::vector<cv::Vec<float,T>> cv_pcd;
        for(int i = 0; i < wait_to_cluster.size(); ++i)
        {
            cv::Vec<float,T> item;
            for(int j = 0;j!=T ; ++j)
            {
                item(j) = wait_to_cluster[i](j);
            }
            //std::cout<<std::endl;
            cv_pcd.push_back(item);
        }
        if(cv_pcd.size() <=0) return;
        clustering_result.clear();
        std::vector<int> labels(wait_to_cluster.size());
        std::vector<cv::Vec<float, T>> centers(K);
        cv::kmeans(cv_pcd,K,labels,cv::TermCriteria( CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 10, 1.0),
            3, cv::KMEANS_PP_CENTERS,centers);     
            
        for(int i = 0;i!=K; ++i)
        {
            Eigen::Matrix<geometry::scalar, T, 1> item;
            for(int j = 0;j!= T ; ++j)
            {
                item(j) = centers[i](j);
            }
            clustering_result.push_back(Cluster<T>(item));
        }
        for(int i = 0; i!=labels.size(); ++i)
        {
            int label = labels[i];
            clustering_result[label].items.push_back(wait_to_cluster[i]);   
            clustering_result[label].indexs.push_back(i);
        }
    }


    template <int T >
        void MeansShiftClustering(const std::vector<Eigen::Matrix<geometry::scalar,T,1>, Eigen::aligned_allocator<Eigen::Matrix<geometry::scalar,T,1>> > & wait_to_cluster, 
        std::vector<Cluster<T>> &clustering_result, float radius)
    {
        //initialize
        std::set<int > un_visited;
        
        for(int i = 0;i!= wait_to_cluster.size(); ++i)
        {
            un_visited.insert(i);
        }

        std::vector<std::map<int, int>> visited_times(wait_to_cluster.size());
        clustering_result.clear();
        

        //build kdtree
        // cv::flann::KDTreeIndexParams indexParams; 
        // cv::flann::Index kdtree(cv::Mat(cv_pcd).reshape(1), indexParams);
        geometry::KDTree<T> kdtree;
        kdtree.BuildTree(wait_to_cluster);
        while(un_visited.size() != 0)
        {
            int index = *un_visited.begin();

            un_visited.erase(index);
            Eigen::Matrix<geometry::scalar, T, 1> center = wait_to_cluster[index];

            int c_id = clustering_result.size();
            int merged = -1;
            std::set<int> c_index_temp;
            while(true)
            {
                // radius search
                Eigen::Matrix<geometry::scalar, T, 1> shift;
                shift.setZero();
                // std::vector<float> query={center(0), center(1), center(2)};
                std::vector<int> indices; 
                std::vector<float> dists; 
                //1024 max_result, SearchParams(1024), The number of times the tree(s) in the index should be recursively traversed. 
                //SearchParams should be larger than max_result
                //Attention!! The radius is squared radius.
                kdtree.RadiusSearch(center, indices,dists,radius,1024, geometry::SearchParameter(1024));
                //std::cout<<"search points number: "<<find_num<<" "<<cv_pcd.size()<<std::endl;
                //std::cout<<"clustered points number: "<<find_num<<std::endl;
                for(int i = 0; i!= indices.size(); ++i)
                {
                    //std::cout<<indices[i]<<std::endl;
                    shift += (wait_to_cluster[indices[i]] - center);
                    if(visited_times[indices[i]].find(c_id) == visited_times[indices[i]].end())
                    visited_times[indices[i]][c_id] = 1;
                    else
                    visited_times[indices[i]][c_id] +=1;
                    un_visited.erase(indices[i]);
                    c_index_temp.insert(indices[i]);
                }
                //compute the mean shift
                
                shift /= indices.size();
                if(shift.norm() < 0.001)
                break;
                // move the center
                center = center + shift;
            }

            //A merger for the clusters whose centers are close. 
            for(int i = 0; i!= clustering_result.size(); ++i)
            {
                if((clustering_result[i].center - center).norm() < 0.01)
                {
                    clustering_result[i].center = (clustering_result[i].center + center)/2;
                    merged = i;
                    break;
                }
            }
            if(merged >= 0)
            {
 
                for(auto iter = c_index_temp.begin(); iter != c_index_temp.end(); ++iter)
                {
                    visited_times[*iter][merged] += visited_times[*iter][c_id];
                    visited_times[*iter][c_id] = 0;
                }
            }
            else
            clustering_result.push_back(Cluster<T>(center));

        }
        // distribute the points to the cluster who has highest frequency
        for(int i = 0; i!= wait_to_cluster.size(); ++i)
        {
            int max_times=0;
            int max_id = -1;

            for(auto iter = visited_times[i].begin(); iter != visited_times[i].end(); ++iter)
            {
                if(max_times < iter->second)
                {
                    max_times = iter->second;
                    max_id = iter->first;
                }
            }
            if(max_id >= 0)
            {
                clustering_result[max_id].items.push_back(wait_to_cluster[i]);
                clustering_result[max_id].indexs.push_back(i);
            }
        }
        //return clustering_result;
    }

    template <int T >
    void KMedoidsClustering(const std::vector<Eigen::Matrix<geometry::scalar,T,1>, Eigen::aligned_allocator<Eigen::Matrix<geometry::scalar,T,1>> > & wait_to_cluster, 
        std::vector<Cluster<T>> &clustering_result, int target_number, bool initialized = false, 
        const std::vector<int> & initialized_index = std::vector<int>())
    {
        //randomly pick $target_number points as the start medoids
        //compute the items
        std::vector<int> indexs;
        if(initialized )
        {
            indexs = initialized_index;
            if(indexs.size() != target_number)
            std::cout<<RED<<"[KMedoidsClustering]::[Error]::The number of Initialized medoids is not equal to the target number."<<RESET<<std::endl;
        }
        else
        {
            for(int i = 0; i != wait_to_cluster.size(); ++i)
            indexs.push_back(i);
            std::random_shuffle(indexs.begin(), indexs.end());
            indexs.resize(target_number);
        }
        int max_iteration = 100000;
        int iteration = 0;
        std::vector<std::vector<int>> cluster_index(target_number);
        while(iteration < max_iteration)
        {
            //std::cout<<"iteration "<<iteration<<std::endl;
            std::vector<bool> no_change(target_number, false);
            iteration += 1;
            for(int i = 0; i != target_number; ++i)
            {
                cluster_index[i].clear();
                //cluster_index[i].push_back(indexs[i]);
            }
            //std::cout<<"choose new medoid "<<std::endl;
            for(int i = 0; i != wait_to_cluster.size(); ++i)
            {
                int min_index = -1;
                float min_distance = std::numeric_limits<float>::max ();
                for(int j = 0; j != target_number; ++j)
                {
                    float distance = (wait_to_cluster[i] - wait_to_cluster[indexs[j]]).norm();
                    if( distance < min_distance)
                    {
                        min_distance = distance;
                        min_index = j;
                    }
                }

                cluster_index[min_index].push_back(i);
            }
            //choose new medoid

            for(int i = 0; i != target_number; ++i)
            {
                float min_distance_sum = std::numeric_limits<float>::max ();
                int new_medoid_index = -1;
                auto &current_cluster_index = cluster_index[i];
                for(int test_id = 0; test_id != current_cluster_index.size(); ++test_id)
                {
                    float distance_sum = 0;
                    for(int j = 0; j != current_cluster_index.size(); ++j)
                    {
                        distance_sum +=
                            (wait_to_cluster[current_cluster_index[j]] -wait_to_cluster[current_cluster_index[test_id]]).norm();
                    }
                    if(distance_sum < min_distance_sum)
                    {
                        new_medoid_index = current_cluster_index[test_id];
                        min_distance_sum = distance_sum;
                    }
                }
                if(indexs[i] == new_medoid_index)
                    no_change[i] = true;
                else indexs[i] = new_medoid_index;
            }
            bool all_no_change = true;
            for(int i = 0; i != target_number; ++i)
            {
                all_no_change = all_no_change & no_change[i];
            }

            if(all_no_change == true)
            break;
        }
        for(int i = 0; i != target_number; ++i)
        {
             
            clustering_result.push_back(Cluster<T>(wait_to_cluster[indexs[i]]));
            clustering_result.back().indexs = cluster_index[i];
            for(int j = 0; j != cluster_index[i].size(); ++j)
                clustering_result.back().items.push_back(wait_to_cluster[cluster_index[i][j]]);
        }
    }

    void KMedoidsClusteringDynamic(const geometry::PointXList & wait_to_cluster, 
        std::vector<ClusterDynamic> &clustering_result, int target_number, bool initialized = false, 
        const std::vector<int> & initialized_index = std::vector<int>());
    
}
}
#endif