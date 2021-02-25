#include "DrawPointCloud.h"

namespace one_piece
{
namespace visualization
{
    std::shared_ptr<geometry::PointCloud> PatchPointCloud(const std::vector<algorithm::PlanePatch> &p_patches)
    {
        geometry::PointCloud pcd_3d;    
        for(size_t i = 0;i!=p_patches.size(); ++i)
        {
            for(size_t j = 0; j !=p_patches[i].items.size(); ++j)
            {
                pcd_3d.points.push_back(geometry::Point3(p_patches[i].items[j](0), p_patches[i].items[j](1), p_patches[i].items[j](2)));
                pcd_3d.colors.push_back(geometry::Point3(visualization::color_tab[i% color_tab.size()](0), 
                    visualization::color_tab[i](1% color_tab.size()), visualization::color_tab[i % color_tab.size()](2))/255.0);
            }
        }
        return std::make_shared<geometry::PointCloud>(pcd_3d);
    }
}
}
