#include "Geometry/PointCloud.h"
#include "Visualization/Visualizer.h"
using namespace one_piece;
int main(int argc, char* argv[]) 
{
    if(argc < 3)
    {
        std::cout << "Usage: SimplifyMeshQuadric [filename.ply] [simplify_ratio] [output=filename_simplified.ply]"<<std::endl;
        return 0;
    }
    geometry::TriangleMesh mesh;
    std::string input = argv[1];
    mesh.LoadFromPLY(input);
    float simplify_ratio = atof(argv[2]);
    std::string output = input.substr(0, input.size() - 3 )+"_simplified"+std::string(".ply");
    if(argc > 3)
        output = argv[3];
    int target_num = mesh.triangles.size() * simplify_ratio;
    std::shared_ptr<geometry::TriangleMesh> s_mesh = mesh.QuadricSimplify(target_num);
    s_mesh->WriteToPLY(output);
    visualization::Visualizer visualizer;
    visualizer.AddTriangleMesh(*s_mesh);
    visualizer.Show();
    return 0;
}