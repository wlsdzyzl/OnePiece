#ifndef OPTIMIZER_H
#define OPTIMIZER_H
#include "BundleAdjustment.h"
#include "SimpleBA.h"

namespace fucking_cool
{
namespace optimization
{
    class Optimizer
    {
        public:
        //optimize world points and camera poses jointly
        void BA(geometry::Point3List &world_points, 
        std::vector<ProjectedPointsOnFrame> &projected_points, 
        geometry::SE3List &poses,
        const camera::PinholeCamera &camera, int max_iteration = 20)
        {
            BundleAdjustment(world_points, projected_points, poses, camera, max_iteration);
        }
        //only optimize the camera poses.
        void FastBA(const std::vector<Correspondence> &correspondences, geometry::SE3List &poses, int max_iteration =5)
        {
            SimpleBA(correspondences, poses, max_iteration);
        }
    };
}
}
#endif