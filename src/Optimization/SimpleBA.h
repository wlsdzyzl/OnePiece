/*
A simple implementation of bundle adjustment, only camera poses are optimized.
Use basic libraries such as opencv and eigen3 for data processing and mathematics.
No acceleration skills.
No other libraries such as g2o.
*/

#ifndef SIMPLE_BA_H
#define SIMPLE_BA_H
#include "Geometry/Geometry.h"
#include "Optimization/Correspondence.h"
namespace fucking_cool
{
namespace optimization
{
    std::tuple<geometry::Matrix6, geometry::Matrix6, geometry::Matrix6, geometry::Matrix6, geometry::Se3, geometry::Se3 >
        ComputeJTJAndJTr(const Correspondence &correspondence, geometry::SE3List &camera_poses);

    void SimpleBA(const std::vector<Correspondence> &correspondences, geometry::SE3List &poses, int max_iteration = 5);
}
}

#endif