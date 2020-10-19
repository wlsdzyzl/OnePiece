#include "Geometry/PointCloud.h"
#include "Visualization/Visualizer.h"
using namespace one_piece;
int main(int argc, char* argv[]) 
{
    if(argc != 3)
    {
        std::cout << "Usage: SimplifyMeshQuadric [filename.ply] [simplify_ratio]"<<std::endl;
        return 0;
    }
    geometry::TriangleMesh mesh;
    mesh.LoadFromPLY(argv[1]);
    float simplify_ratio = atof(argv[2]);
    int target_num = mesh.triangles.size() * simplify_ratio;
    std::shared_ptr<geometry::TriangleMesh> s_mesh = mesh.QuadricSimplify(target_num);

    visualization::Visualizer visualizer;
    visualizer.AddTriangleMesh(*s_mesh);
    visualizer.Show();
    return 0;
}