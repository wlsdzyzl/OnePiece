#include "Clustering.h"
namespace fucking_cool
{
namespace algorithm
{
    //For Dynamic Vector
    void KMedoidsClusteringDynamic(const geometry::PointXList & wait_to_cluster, 
        std::vector<ClusterDynamic> &clustering_result, size_t target_number, bool initialized, 
        const std::vector<int> & initialized_index)
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
            for(size_t i = 0; i != wait_to_cluster.size(); ++i)
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
            for(size_t i = 0; i != target_number; ++i)
            {
                cluster_index[i].clear();
                //cluster_index[i].push_back(indexs[i]);
            }
            //std::cout<<"choose new medoid "<<std::endl;
            for(size_t i = 0; i != wait_to_cluster.size(); ++i)
            {
                int min_index = -1;
                float min_distance = std::numeric_limits<float>::max ();
                for(size_t j = 0; j != target_number; ++j)
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

            for(size_t i = 0; i != target_number; ++i)
            {
                float min_distance_sum = std::numeric_limits<float>::max ();
                int new_medoid_index = -1;
                auto &current_cluster_index = cluster_index[i];
                for(size_t test_id = 0; test_id != current_cluster_index.size(); ++test_id)
                {
                    float distance_sum = 0;
                    for(size_t j = 0; j != current_cluster_index.size(); ++j)
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
            for(size_t i = 0; i != target_number; ++i)
            {
                all_no_change = all_no_change & no_change[i];
            }

            if(all_no_change == true)
            break;
        }
        for(size_t i = 0; i != target_number; ++i)
        {
             
            clustering_result.push_back(ClusterDynamic(wait_to_cluster[indexs[i]]));
            clustering_result.back().indexs = cluster_index[i];
            for(size_t j = 0; j != cluster_index[i].size(); ++j)
                clustering_result.back().items.push_back(wait_to_cluster[cluster_index[i][j]]);
        }
    }
}
}