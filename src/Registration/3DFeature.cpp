#include "3DFeature.h"
#include <omp.h>
namespace one_piece
{
namespace registration
{
    //compute three angles and distance.
    PairDescriptor ComputePairDescriptor(const geometry::Point3 &ps, const geometry::Point3 &ns,
        const geometry::Point3 &pt, const geometry::Point3 &nt)
    {
        float distance = (ps - pt).norm();
        //coordinate based on ps
        geometry::Point3 u,v,w;
        u = ns;
        v = u.cross((pt - ps) / distance);
        if(v.norm() == 0) return PairDescriptor::Zero();
        w = u.cross(v);
        PairDescriptor result;
        result(3) = distance;
        result(1) = (v.transpose() * nt)(0);
        result(2) = (u.transpose() * (pt - ps) / distance)(0);
        result(0) = atan2((w.transpose()*nt)(0), (u.transpose() *nt)(0));
        return result;
    }


    void ComputeSPFH(const geometry::PointCloud &pcd, geometry::KDTree<> &kdtree, 
        FeatureSet &spfh_features, std::vector<std::vector<int>> &neighbors, int knn, float radius)
    {
        Feature init_f;
        init_f.resize(33);
        spfh_features.resize(pcd.points.size(), init_f);
        neighbors.resize(pcd.points.size());
        //std::cout<<squared_radius<<std::endl;

        for(size_t i = 0; i != pcd.points.size(); ++i)
        {
            
            std::vector<int> indices; 
            std::vector<float> dists; 
            
            kdtree.RadiusSearch(pcd.points[i], indices, dists, 
                radius, knn, geometry::SearchParameter(1024));
            
            int points_num = indices.size();
            neighbors[i].clear();
            spfh_features[i].setZero();
            if(points_num - 1 > 0)
            {
                if(points_num > knn) points_num = knn;
                double each_addtion = 100 / (points_num - 1);
                neighbors[i].resize(points_num - 1 );
                
                for(int j = 1; j != points_num; ++j)
                {
                    
                    neighbors[i][j-1] = indices[j];
                    //std::cout <<indices[j]<<" "<<std::endl;
                    auto descriptor = ComputePairDescriptor(pcd.points[i], pcd.normals[i], 
                        pcd.points[indices[j]], pcd.normals[indices[j]]);
                    //std::cout<<j<<" "<<neighbors[i].size()<<" "<<descriptor.transpose()<<std::endl;
                    //construct the histogram
                    int h_index_0 = (int)(floor(11 * (descriptor(0) + M_PI) / (2.0 * M_PI)));
                    int h_index_1 = (int)(floor(11 * (descriptor(1) + 1) / 2.0 ));
                    int h_index_2 = (int)(floor(11 * (descriptor(2) + 1) / 2.0 ));
                    if(h_index_0 > 10 ) h_index_0 = 10;
                    if(h_index_1 > 10 ) h_index_1 = 10;
                    if(h_index_2 > 10 ) h_index_2 = 10;

                    if(h_index_0 < 0 ) h_index_0 = 0;
                    if(h_index_1 < 0 ) h_index_1 = 0;
                    if(h_index_2 < 0 ) h_index_2 = 0;

                    spfh_features[i](h_index_0) += each_addtion;
                    spfh_features[i][h_index_1 + 11] += each_addtion;
                    spfh_features[i](h_index_2 + 22) += each_addtion;
                }
            }
       
        }
    }

    void ComputeFPFHFeature(const geometry::PointCloud &pcd, FeatureSet &fpfh_features, int knn, float radius)
    {
        // cv::flann::KDTreeIndexParams indexParams; 
        // cv::flann::Index kdtree(cv::Mat(cv_pcd).reshape(1), indexParams);       
        geometry::KDTree<> kdtree;
        kdtree.BuildTree(pcd.points);
        FeatureSet spfh_features;
        std::vector<std::vector<int>> neighbor_indexs;
#if DEBUG_MODE
        std::cout<<BLUE<<"[DEBUG]::[FPFHFeature]::Compute SPFH, knn="<<knn<<", radius="<<radius<<RESET<<std::endl;
#endif
        ComputeSPFH(pcd, kdtree, spfh_features, neighbor_indexs, knn, radius); 
        fpfh_features.resize(spfh_features.size());
        Feature f_zero;
        f_zero.resize(33);
        f_zero.setZero();
#if DEBUG_MODE
        std::cout<<BLUE<<"[DEBUG]::[FPFHFeature]::Compute FPFH"<<RESET<<std::endl;
#endif

        for(size_t i = 0; i != spfh_features.size(); ++i)
        {
            
            std::vector<double> sum = {0,0,0};
            fpfh_features[i] = f_zero;
            for(size_t j = 0; j != neighbor_indexs[i].size(); ++j)
            {
                float dist = (pcd.points[i] - pcd.points[neighbor_indexs[i][j]]).norm();
                if(dist != 0.0)
                {
                    float w_d =1 / dist;
                    fpfh_features[i] +=  w_d * spfh_features[neighbor_indexs[i][j]];
                    sum[0] += spfh_features[neighbor_indexs[i][j]].block<11, 1>(0,0).sum();
                    sum[1] += spfh_features[neighbor_indexs[i][j]].block<11, 1>(11,0).sum();
                    sum[2] += spfh_features[neighbor_indexs[i][j]].block<11, 1>(22,0).sum();
                }
                
            }
            sum[0] = 100.0/ sum[0];
            sum[1] = 100.0/ sum[1];
            sum[2] = 100.0/ sum[2];
            //std::cout<<" "<<sum[0]<<" "<<sum[1]<<" "<<sum[2]<<std::endl;
            fpfh_features[i].block<11, 1>(0,0) *= sum[0];
            fpfh_features[i].block<11, 1>(11,0) *= sum[1];
            fpfh_features[i].block<11, 1>(22,0) *= sum[2];
            
            fpfh_features[i] += spfh_features[i] ;
        }
    }
}
}