#include "Geometry/PointCloud.h"
#include "Visualization/Visualizer.h"
using namespace one_piece;
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
//     geometry::TransformationMatrix mat;
//     mat<<   0.8573, -0.145003,  0.493974, -0.364654,
// 0.0116798,  0.964746,  0.262924, -0.531801,
// -0.514684, -0.219636,  0.828771,   0.242593,
//         0,         0,         0,         1;
//     pcd.Transform(mat);
    pcd.WriteToPLY("./transformed_pcd.ply");
    
    auto pcd_ptr = pcd.DownSample(voxel_len);
    std::cout<<"down sample: from "<< pcd.points.size() <<" to "<<pcd_ptr->points.size()<<std::endl;
    visualization::Visualizer visualizer;
    visualizer.AddPointCloud( * pcd_ptr);
    visualizer.Show();
    
    return 0;
}