#include "RGBDFrame.h"

namespace one_piece
{
namespace geometry
{

    void RGBDFrame::PrepareDownSamplePointCloud(const camera::PinholeCamera &camera, float voxel_len)
    {
        
            down_sampled_pcd.LoadFromRGBD(rgb,depth,camera);
            auto tmp_pcd = down_sampled_pcd.DownSample(voxel_len);
            down_sampled_pcd = *tmp_pcd;
    }
}
}