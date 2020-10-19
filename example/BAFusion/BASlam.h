#ifndef BASLAM_H
#define BASLAM_H
#include "Tool/KeyframeBasedSlam.h"

namespace one_piece
{
    class BASlam: public tool::KeyframeBasedSlam
    {
        
        public:
        BASlam() = default;
        BASlam(const camera::PinholeCamera &_camera): tool::KeyframeBasedSlam(_camera){}
        void UpdateFrame(const geometry::RGBDFrame &frame);
        void Optimize();
        //Bundle Adjustment
        geometry::Point3List world_points;
        //for each keyframe, we have a projected points
        std::vector<optimization::ProjectedPointsOnFrame> projected_points;
        //for each keyframe, we preserve the correponding world_point_index of local points;
        //notice that we do not have to put all the local points into the world point,
        //we only transform the matched local_points to world points.
        std::vector<std::vector<int>> points_local_to_global;
        
    };
}
#endif