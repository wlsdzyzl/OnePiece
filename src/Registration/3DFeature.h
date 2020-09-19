/*
Compute 3D feature.
Now we only support fpfh feature.
*/

#ifndef _3D_FEATURE_H
#define _3D_FEATURE_H

#include "Geometry/Geometry.h"
#include "Geometry/PointCloud.h"
#include "Geometry/KDTree.h"
namespace fucking_cool
{
namespace registration
{
    typedef geometry::Vector4 PairDescriptor;
    //here fpfh is a 33 dimensional vector.
    typedef geometry::VectorX Feature;
    typedef geometry::PointXList FeatureSet;
    PairDescriptor ComputePairDescriptor(const geometry::Point3 &ps, const geometry::Point3 &ns,
        const geometry::Point3 &pt, const geometry::Point3 &nt);
    void ComputeSPFH(const geometry::PointCloud &pcd, geometry::KDTree<> &kdtree, 
        FeatureSet &spfh_features, std::vector<std::vector<int>> &neighbors, int knn, float radius);
    void ComputeFPFHFeature(const geometry::PointCloud &pcd, FeatureSet &fpfh_features, int knn = 100, float radius = 0.1);
}
}
#endif