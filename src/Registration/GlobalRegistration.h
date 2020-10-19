#ifndef GLOBAL_REGISTRATION_H
#define GLOBAL_REGISTRATION_H
#include "Geometry/Geometry.h"
#include "Geometry/Ransac.h"
#include "RegistrationResult.h"
#include "3DFeature.h"
namespace one_piece
{
namespace registration
{
    //Feature matching based registration
    class RANSACParameter
    {
        public:
        int max_iteration = 30;//max_iteration
        double threshold = 0.2;//max_distance of the correspondence
        double scaling = 1.0;
        int max_nn = 100;//max number of neighbors in fpfh.
        int max_nn_normal = 30;
        float search_radius_normal = 0.1;
        float voxel_len = 0.1;//downsample voxel size;
        float search_radius = 0.25;
        //can add some kdtree parameters here.
    };    
    std::shared_ptr<RegistrationResult> RansacRegistration(const geometry::PointCloud &source_pcd, const geometry::PointCloud &target_pcd, 
        const RANSACParameter &r_para = RANSACParameter());
    void FeatureMatching3D(const FeatureSet &source_feature, const FeatureSet &target_feature, 
        geometry::FMatchSet &matching_index);
    void RejectMatchesRanSaPC(const geometry::Point3List &source_points, 
        const geometry::Point3List &target_points, std::default_random_engine &engine, 
        geometry::FMatchSet &init_matches, int candidate_num = 4, float difference = 0.1);
    std::tuple<geometry::PointCloud, FeatureSet> DownSampleAndExtractFeature(const geometry::PointCloud &pcd, 
        const RANSACParameter &r_para); 
    std::shared_ptr<RegistrationResult> RansacRegistration(const geometry::PointCloud &source_feature_pcd, 
        const geometry::PointCloud &target_feature_pcd, 
        const FeatureSet & source_features,
        const FeatureSet & target_features,
        const RANSACParameter &r_para);

}
}
#endif