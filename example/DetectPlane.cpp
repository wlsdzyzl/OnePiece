#include "Algorithm/PatchDetection.h"
#include "Geometry/PointCloud.h"
#include "Visualization/DrawPointCloud.h"
using namespace fucking_cool;
int main(int argc, char **argv)
{
    if (argc < 2)
    {
        std::cout << "usage::DetectPlane [pointcloud.ply]" << std::endl;
        return 0;
    }

    geometry::PointCloud pcd;
    pcd.LoadFromPLY(argv[1]);
    std::vector<algorithm::PlanePatch> result;
    algorithm::PlaneDetection(pcd.points, result);
    for(int i = 0; i != result.size(); ++i)
    {
        std::cout<<"Plane "<<i<<": "<<result[i].items.size()<<" "
        <<result[i].rep(0)<<" "<<result[i].rep(1)<<" "<<result[i].rep(2)<<" "<<result[i].rep(3)<<std::endl;
    }
    auto pcd_draw = visualization::PatchPointCloud(result);
    pcd_draw->WriteToPLY("./Planes.ply");
    return 0;
}