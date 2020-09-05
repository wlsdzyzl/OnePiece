#include "Geometry/PointCloud.h"
#include "Visualization/Visualizer.h"
using namespace fucking_cool;
int main(int argc, char* argv[]) 
{
    if(argc != 2)
    {
        std::cout << "Usage: ReadPLYPCD [filename.ply]"<<std::endl;
        return 0;
    }
    geometry::PointCloud pcd;
    pcd.LoadFromPLY(argv[1]);
    /*
    if(!pcd.HasNormals())
    {
        pcd.EstimateNormals();
    }
    */
    //pcd.WriteToPLY("Duplicate.ply");
    auto d_pcd = pcd.DownSample(0.05);
    d_pcd->WriteToPLY("down_sample_pcd.ply");
    visualization::Visualizer visualizer;
    visualizer.AddPointCloud(*d_pcd);
    visualizer.Show();
    
    return 0;
}