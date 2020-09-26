#include "SparseMatcher.h"
namespace fucking_cool
{
namespace odometry
{

    void SparseMatcher::Match( const geometry::Descriptor &source_descriptors,  const geometry::Descriptor &target_descriptors, 
        geometry::DMatchSet & matches)
    {
        Clear();
        //sparse_matcher = MILD::SparseMatcher(FEATURE_TYPE_ORB, 32, 0, 50);
        sparse_matcher.train(target_descriptors);
        sparse_matcher.search_8(source_descriptors, matches, hamming_distance_threshold);
    }
    //this function need to be called after the match function.
    void SparseMatcher::Clear()
    {
        sparse_matcher.features_buffer = std::vector<MILD::sparse_match_entry>(sparse_matcher.features_buffer.size());
		for(size_t i = 0; i < sparse_matcher.features_buffer.size(); i++)
		{
			sparse_matcher.features_buffer[i].clear();
		}
		sparse_matcher.statistics_num_distance_calculation = 0;
    }
    void SparseMatcher::RefineMatches(const geometry::Descriptor &source_descriptors,const geometry::Point3List &source_local_points,
        const geometry::KeyPointSet &source_keypoints,  const geometry::KeyPointSet &target_keypoints, 
        const geometry::TransformationMatrix & H,geometry::DMatchSet & matches)
    {
        std::vector< cv::DMatch > predict_matches;
        std::vector<cv::KeyPoint> predict_ref_points;
        float cx = camera.GetCx();
        float cy = camera.GetCy();
        float fx = camera.GetFx();
        float fy = camera.GetFy();
        predict_ref_points.resize(source_local_points.size());
        for(size_t i = 0; i < predict_ref_points.size(); i++)
        {
            geometry::Vector4 homo_points;
            homo_points << source_local_points[i](0), source_local_points[i](1), source_local_points[i](2), 1;
            geometry::Point3 predict_points = (H*homo_points).head<3>();
            predict_points = predict_points / predict_points(2);
            cv::KeyPoint predict_ref = source_keypoints[i];
            predict_ref.pt.x = predict_points(0) * fx + cx;
            predict_ref.pt.y = predict_points(1) * fy + cy;
            predict_ref_points[i] = predict_ref;
        }

        sparse_matcher.search_8_with_range(source_descriptors, matches, target_keypoints, 
            predict_ref_points, range, hamming_distance_threshold * 1.5);
    }

}
}