#include "Geometry/Geometry.h"
#include "Registration/ICP.h"
#include "Visualization/Visualizer.h"
using namespace fucking_cool;

int main(int argc, char **argv)
{
    if(argc < 3)
    {
        std::cout<<"Usage: ICPTest [source_pcd] [target_pcd]"<<std::endl;
        return 1;
    }

    geometry::PointCloud s_pcd, t_pcd;
    s_pcd.LoadFromPLY(argv[1]);
    t_pcd.LoadFromPLY(argv[2]);
    /*
    if(!pcd.HasNormals())
    {
        pcd.EstimateNormals();
    }
    */
    //pcd.WriteToPLY("Duplicate.ply");
    geometry::TransformationMatrix init_T = geometry::TransformationMatrix::Identity();

    if(!s_pcd.HasNormals())
    s_pcd.EstimateNormals();
    if(!t_pcd.HasNormals())
    t_pcd.EstimateNormals();
    registration::ICPParameter icp_para;
    icp_para.threshold = 0.01;
    //auto result1 = registration::PointToPoint(s_pcd, t_pcd, init_T, icp_para);
    auto result1 = registration::PointToPlane(s_pcd, t_pcd, init_T, icp_para);
    std::cout<<result1->T<<std::endl;
    s_pcd.colors.resize(s_pcd.points.size());
    t_pcd.colors.resize(t_pcd.points.size());

    for(size_t i = 0; i != s_pcd.points.size(); ++i)
    {
        s_pcd.colors[i] = geometry::Point3(1, 0, 0);
    }
    s_pcd.Transform(result1->T);
    for(size_t i = 0; i != t_pcd.points.size(); ++i)
        t_pcd.colors[i] = geometry::Point3(0, 1, 0);

    visualization::Visualizer visualizer;
    visualizer.SetDrawColor(1);
    visualizer.AddPointCloud(s_pcd);
    visualizer.AddPointCloud(t_pcd);
    visualizer.Show();    
    return 0;
}