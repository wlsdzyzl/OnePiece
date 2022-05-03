#include "Geometry/PointCloud.h"
#include "Visualization/Visualizer.h"
using namespace one_piece;
int main(int argc, char* argv[]) 
{
    if(argc < 3)
    {
        std::cout << "Usage: SimplifyMeshClustering [filename.ply] [grid_length] [output=filename_clustered.ply]"<<std::endl;
        return 0;
    }
    geometry::TriangleMesh mesh;
    std::string input = argv[1];
    mesh.LoadFromPLY(input);
    float grid_length = atof(argv[2]);
    std::string output = input.substr(0, input.size() - 3 )+"_clustered"+std::string(".ply");
    if(argc > 3)
        output = argv[3];
    std::shared_ptr<geometry::TriangleMesh> s_mesh = mesh.ClusteringSimplify(grid_length);
    s_mesh->WriteToPLY(output);
    visualization::Visualizer visualizer;
    visualizer.AddTriangleMesh(*s_mesh);
    visualizer.Show();
    return 0;
}