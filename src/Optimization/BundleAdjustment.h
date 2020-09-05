/*
Unlike SimpleBA, traditional BA requires to optimize the points in world space and 
camera poses jointly. This is so different with just optimize the camera poses, because
space points is much more than camera poses generally.

We also need to use Schur completeness and other tricks to solve this problem.
*/
#ifndef BUNDLE_ADJUSTMENT_H
#define BUNDLE_ADJUSTMENT_H

#include "Geometry/Geometry.h"
#include "Camera/Camera.h"
#include "Geometry/RGBDFrame.h"
namespace fucking_cool
{
namespace optimization
{
    //first index of world point, second is the coordinate on frame
    typedef std::unordered_map<int, geometry::Point2> ProjectedPointsOnFrame; 
    //3D camera coordinate and 2d frame coordinate
    typedef std::unordered_map<int, std::pair<geometry::Point3, geometry::Point2> > ReprojectedPoints;
    void BundleAdjustment(geometry::Point3List &world_points, 
        std::vector<ProjectedPointsOnFrame> &projected_points, 
        geometry::SE3List &poses,
        const camera::PinholeCamera &camera, int max_iteration = 20);
    std::tuple<geometry::MatrixX, geometry::MatrixX> 
        ComputeJacobian(const geometry::Point3 &world_point, const geometry::Point3 &transformed_p, 
        const geometry::SE3 &pose, const camera::PinholeCamera &camera);
    //some function we need:
    //function we calculated the points which fall in the camera's frustum, and not be occluded
    // void CountValidPoints(const geometry::PointCloud &world_points, const geometry::SE3 &pose, 
    //     const camera::PinholeCamera &camera, PixelMapIndex & pixel_to_id, PixelMapPoint &pixel_to_point);
    
}
}
#endif