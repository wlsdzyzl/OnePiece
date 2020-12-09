#include "Geometry/PointCloud.h"
#include "Visualization/Visualizer.h"
using namespace one_piece;
int main(int argc, char* argv[]) 
{
    
    if(argc != 2)
    {
        std::cout << "Usage: ReadPLYMesh [filename.ply]"<<std::endl;
        return 0;
    }
    geometry::TriangleMesh mesh;
    mesh.LoadFromPLY(argv[1]);

    if(!mesh.HasNormals())
        mesh.ComputeNormals();
    visualization::Visualizer visualizer;
    //visualizer.SetDrawColor(true);
    visualizer.AddTriangleMesh(mesh);
    visualizer.Show();
    mesh.WriteToPLY("./open3d.ply");
    return 0;
}