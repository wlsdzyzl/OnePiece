#include "Geometry/PointCloud.h"
#include "Visualization/Visualizer.h"
using namespace fucking_cool;
int main(int argc, char* argv[]) 
{
    
    if(argc != 2)
    {
        std::cout << "Usage: ReadPLYMesh [filename.ply]"<<std::endl;
        return 0;
    }
    geometry::TriangleMesh mesh;

    mesh.LoadFromPLY(argv[1]);
    /*
    if(!mesh.HasNormals())
    {
        std::cout << "Mesh don't have normals, estimate normals..."<<std::endl;
        mesh.ComputeNormals();
    }*/
    for(int i = 0; i != mesh.colors.size(); ++i)
    {
        float tmp = mesh.colors[i](0);
        mesh.colors[i](0) = mesh.colors[i](2);
        mesh.colors[i](2) = tmp;
    }
    visualization::Visualizer visualizer;
    visualizer.AddTriangleMesh(mesh);
    visualizer.Show();
    mesh.WriteToPLY("./open3d.ply");
    return 0;
}