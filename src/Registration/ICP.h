#ifndef ICP_H
#define ICP_H
#include "Geometry/Geometry.h"
#include "Geometry/PointCloud.h"
#include "RegistrationResult.h"
#include <memory>
namespace fucking_cool
{
namespace registration
{
    //ICP algorithm: Point to Point, and Point to Plane

    class ICPParameter
    {
        public:
        int max_iteration = 30;//max_iteration
        double threshold = 0.2;//max_distance of the correspondence
        double scaling = 1.0;
    };
    geometry::TransformationMatrix EstimateRigidTransformationPointToPlane(const geometry::Point3List &source, 
        const geometry::Point3List &target, const geometry::Point3List &target_normal, 
        const geometry::FMatchSet &inliers);
    std::shared_ptr<RegistrationResult> PointToPoint(const geometry::PointCloud &source, const geometry::PointCloud &target, 
        const geometry::TransformationMatrix &init_T = geometry::TransformationMatrix::Identity(), const ICPParameter &icp_para = ICPParameter() );
    std::shared_ptr<RegistrationResult> PointToPlane(const geometry::PointCloud &source, const geometry::PointCloud &target,
        const geometry::TransformationMatrix &init_T = geometry::TransformationMatrix::Identity(), const ICPParameter &icp_para = ICPParameter() );
}
}
#endif