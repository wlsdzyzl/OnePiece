#ifndef SPARSE_MATCHER_H
#define SPARSE_MATCHER_H
#include "sparse_match.h"
#include "Geometry/Geometry.h"
#include "Camera/Camera.h"
#include "Tool/ConsoleColor.h"
namespace one_piece
{
namespace odometry
{
    class SparseMatcher
    {
        public:
        SparseMatcher():sparse_matcher( MILD::SparseMatcher(FEATURE_TYPE_ORB, 32, 0, 50))
        {
            hamming_distance_threshold = 50;
        }
        void Match(const geometry::Descriptor &source_descriptors, const geometry::Descriptor &target_descriptors, 
            geometry::DMatchSet & matches);
        //this function need to be called after the match function.
        void RefineMatches(const geometry::Descriptor &source_descriptors, const geometry::Point3List &source_local_points,
            const geometry::KeyPointSet &source_keypoints,  const geometry::KeyPointSet &target_keypoints, 
            const geometry::TransformationMatrix & H,geometry::DMatchSet & matches);
        void Clear();
        int hamming_distance_threshold;
        int range = 30;
        camera::PinholeCamera camera;
        MILD::SparseMatcher sparse_matcher;
    };
}
}

#endif