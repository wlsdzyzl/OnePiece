#include "Geometry/TriangleMesh.h"
#include "Visualization/Visualizer.h"
using namespace fucking_cool;
int main(int argc, char* argv[]) 
{
    if(argc != 3)
    {
        std::cout << "Usage: PruneMesh [filename.ply] [min_points]"<<std::endl;
        return 0;
    }
    geometry::TriangleMesh mesh;
    mesh.LoadFromPLY(argv[1]);
    //mesh.LoadFromOBJ(argv[1]);
    float min_points = atof(argv[2]);
    std::shared_ptr<geometry::TriangleMesh> s_mesh = mesh.Prune(min_points);
    s_mesh->WriteToPLY( argv[1] + std::string("_pruned.ply"));
    return 0;
}