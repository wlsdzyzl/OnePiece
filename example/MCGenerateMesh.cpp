#include "Integration/CubeHandler.h"
#include "Visualization/Visualizer.h"
#include "Tool/TickTock.h"
using namespace fucking_cool;
int main(int argc, char *argv[])
{
    if (argc < 2)
    {
        std::cout << "usage::MCGenerateMesh [file_path]" << std::endl;
        return 0;
    }
    geometry::TriangleMesh mesh;
    tool::Timer timer;
    //integration::InitializeVoxelCube();
    integration::CubeHandler cube_handler;
    cube_handler.ReadFromFileFloat(argv[1]);
    timer.TICK("Extract Triangle Mesh");
    cube_handler.ExtractTriangleMesh(mesh);
    timer.TOCK("Extract Triangle Mesh");
    timer.TICK("Compute Normals");
    if(!mesh.HasNormals())
    {
        std::cout << "Mesh don't have normals, estimate normals..."<<std::endl;
        mesh.ComputeNormals();
    }
    timer.TOCK("Compute Normals");
    mesh.WriteToPLY("mc_mesh.ply");
    //auto tsdf_pcd = cube_handler.GetPointCloud();
    timer.LogAll();
    timer.Reset();
    //std::shared_ptr<geometry::TriangleMesh> mesh_clustered = mesh.ClusteringSimplify(0.01);
    //mesh_clustered->WriteToPLY("./clustered_mesh.ply");
    visualization::Visualizer visualizer;
    visualizer.SetDrawColor(true);
    visualizer.AddTriangleMesh(mesh);
    //visualizer.AddPointCloud(*tsdf_pcd);
    visualizer.Show();  
    
    
    return 0;
}