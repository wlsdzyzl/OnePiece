#include "Geometry/PointCloud.h"
#include "Visualization/Visualizer.h"
using namespace one_piece;
int main(int argc, char* argv[]) 
{
    if(argc != 3)
    {
        std::cout << "Usage: SimplifyMeshClustering [filename.ply] [grid_length]"<<std::endl;
        return 0;
    }
    geometry::TriangleMesh mesh;
    mesh.LoadFromPLY(argv[1]);
    float grid_length = atof(argv[2]);
    std::shared_ptr<geometry::TriangleMesh> s_mesh = mesh.ClusteringSimplify(grid_length);
    s_mesh->WriteToPLY("clustered_mesh.ply");
    visualization::Visualizer visualizer;
    visualizer.AddTriangleMesh(*s_mesh);
    visualizer.Show();
    return 0;
}