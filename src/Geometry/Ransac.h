#ifndef RANSAC_SUMMARY_H
#define RANSAC_SUMMARY_H
#include "Geometry/TransformationModel.hpp"
#include "Geometry/PlaneFittingModel.hpp"
#include "GRANSAC.hpp"
namespace one_piece
{
namespace geometry
{
    //A summary of using ransac to estimate rigid transformation and plane fitting and so on.
    TransformationMatrix EstimateRigidTransformationRANSAC(const PointCorrespondenceSet &correspondence_set,
        PointCorrespondenceSet & inliers, std::vector<int> &inlier_ids, int max_iteration = 2000, float threshold = 0.1);

    std::tuple<geometry::Point3, double, double> FitPlaneRANSAC(const Point3List &points, Point3List & inliers, 
        std::vector<int> &inlier_ids, int max_iteration = 2000, float threshold = 0.05);
     
};
}

#endif
