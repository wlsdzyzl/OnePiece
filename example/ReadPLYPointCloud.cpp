#include "Geometry/PointCloud.h"
#include "Visualization/Visualizer.h"
using namespace fucking_cool;
int main(int argc, char* argv[]) 
{
    if(argc < 2)
    {
        std::cout << "Usage: ReadPLYPCD [filename.ply] [voxel_len = 0.01]"<<std::endl;
        return 0;
    }
    geometry::PointCloud pcd;
    float voxel_len = 0.01;
    if(argc == 3)
        voxel_len = atof(argv[2]);
    pcd.LoadFromPLY(argv[1]);
    auto pcd_ptr = pcd.DownSample(voxel_len);
    std::cout<<"down sample: from "<< pcd.points.size() <<" to "<<pcd_ptr->points.size()<<std::endl;
    visualization::Visualizer visualizer;
    visualizer.AddPointCloud( * pcd_ptr);
    visualizer.Show();
    
    return 0;
}