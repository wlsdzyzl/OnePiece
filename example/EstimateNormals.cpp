#include "Geometry/PointCloud.h"
#include "Visualization/Visualizer.h"
using namespace one_piece;
void GeneratePlane(double a, double b, double c, double d, geometry::Point3List &points)
{
    for(int i = 0;i != 100; ++i)
     for(int j = 0;j != 100; ++j)
     {
         double x = i * 0.1 + 0.5;
         double y = j * 0.1 + 0.5;
         double z = -(a*x + b*y + d) / c;
         points.push_back(geometry::Point3(x, y, z));
     }
}
int main(int argc, char* argv[]) 
{

    geometry::PointCloud pcd;
    GeneratePlane(1,2,3,4,pcd.points);
    if(!pcd.HasNormals())
    {
        pcd.EstimateNormals();
    }
    pcd.WriteToPLY("./bunny_n.ply");
    visualization::Visualizer visualizer;
    visualizer.AddPointCloud(pcd);

    //visualizer.AddPointCloud(*dspcd);
    visualizer.Show();
    return 0;
}