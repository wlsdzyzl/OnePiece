#ifndef DRAW_POINTCLOUD_H
#define DRAW_POINTCLOUD_H

#include "Algorithm/PatchDetection.h"
#include "Algorithm/Clustering.h"
#include "ColorTab.h"
#include "Geometry/PointCloud.h"


namespace fucking_cool
{
namespace visualization
{
    std::shared_ptr<geometry::PointCloud> PatchPointCloud(const std::vector<algorithm::PlanePatch> &p_patches)
    {
        geometry::PointCloud pcd_3d;    
        for(int i = 0;i!=p_patches.size(); ++i)
        {
            for(int j = 0; j !=p_patches[i].items.size(); ++j)
            {
                pcd_3d.points.push_back(geometry::Point3(p_patches[i].items[j](0), p_patches[i].items[j](1), p_patches[i].items[j](2)));
                pcd_3d.colors.push_back(geometry::Point3(visualization::color_tab[i](0), 
                    visualization::color_tab[i](1), visualization::color_tab[i](2))/255.0);
            }
        }
        return std::make_shared<geometry::PointCloud>(pcd_3d);
    }
}
}

#endif