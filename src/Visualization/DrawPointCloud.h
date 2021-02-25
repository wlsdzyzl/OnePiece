#ifndef DRAW_POINTCLOUD_H
#define DRAW_POINTCLOUD_H

#include "Algorithm/PatchDetection.h"
#include "Algorithm/Clustering.h"
#include "ColorTab.h"
#include "Geometry/PointCloud.h"


namespace one_piece
{
namespace visualization
{
    std::shared_ptr<geometry::PointCloud> PatchPointCloud(const std::vector<algorithm::PlanePatch> &p_patches);
}
}

#endif