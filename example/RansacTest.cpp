#include "Registration/GlobalRegistration.h"
#include "Visualization/Visualizer.h"
using namespace one_piece;
std::shared_ptr<geometry::PointCloud> DrawMatches(const geometry::PointCloud &source, 
    const geometry::PointCloud &target, const geometry::PointCorrespondenceSet &inliers)
{
    geometry::PointCloud result = source;
    geometry::TransformationMatrix T = geometry::TransformationMatrix::Identity();
    T.block<3,1>(0,3) = geometry::Point3(0,2,2);
    result.Transform(T);

    result.points.insert(result.points.end(), target.points.begin(), target.points.end());
    result.colors.insert(result.colors.end(), target.colors.begin(), target.colors.end());
    for(size_t i = 0; i != inliers.size(); ++i)
    {
        
        geometry::Point3 first = T.block<3,3>(0,0) * inliers[i].first + T.block<3,1>(0,3);
        geometry::Point3 second = inliers[i].second;

        float step_x = (second(0) - first(0))/1000;
        float step_y = (second(1) - first(1))/1000;
        float step_z = (second(2) - first(2))/1000;
        for(int c = 0; c != 1000; ++c)
        {
            result.points.push_back(first + geometry::Point3(step_x, step_y, step_z) * c);
            result.colors.push_back(geometry::Point3(1,0,0));
        } 
    }    
    return std::make_shared<geometry::PointCloud>(result);
}

// using namespace registration;
// std::shared_ptr<RANSACRegistrationResult> halfGlobalRegistration(const geometry::PointCloud &source_pcd, 
//     const geometry::PointCloud &target_pcd, const geometry::MatrixX &source_fpfh, const geometry::MatrixX &target_fpfh, 
//     const RANSACParameter &r_para)
// {
//     FeatureSet source_features;
//     FeatureSet target_features;
//     for(int i = 0; i != source_fpfh.cols(); ++i)
//     {
//         source_features.push_back(source_fpfh.col(i));
//     }
//     for(int i = 0; i != target_fpfh.cols(); ++i)
//     {
//         target_features.push_back(target_fpfh.col(i));
//     }
//     geometry::FMatchSet matches;
// #if DEBUG_MODE
//     std::cout<<BLUE<<"[DEBUG]::[GlobalRegistrationRANSAC]::Feature Matching..."<<RESET<<std::endl;
// #endif
//     FeatureMatching3D(source_features, target_features, matches);
//     std::default_random_engine engine;
// #if DEBUG_MODE
//     std::cout<<BLUE<<"[DEBUG]::[GlobalRegistrationRANSAC]::Reject False Matches..."<<RESET<<std::endl;
//     int before = matches.size();
// #endif
//     RejectMatchesRanSaPC(source_pcd.points, target_pcd.points, engine, matches);
//     RejectMatchesRanSaPC(source_pcd.points, target_pcd.points, engine, matches);
//     RejectMatchesRanSaPC(source_pcd.points, target_pcd.points, engine, matches);
// #if DEBUG_MODE
//     int after = matches.size();
//     std::cout<<BLUE<<"[DEBUG]::[GlobalRegistrationRANSAC]::Before/After: "<<before<<"/"<<after<<RESET<<std::endl;
// #endif        
//     geometry::PointCorrespondenceSet correspondence_set;
//     for(size_t i = 0; i != matches.size(); ++i)
//     {
//         correspondence_set.push_back(std::make_pair(source_pcd.points[matches[i].first], 
//             target_pcd.points[matches[i].second]));
//     };
//     geometry::PointCorrespondenceSet inliers;
// #if DEBUG_MODE
//     std::cout<<BLUE<<"[DEBUG]::[GlobalRegistrationRANSAC]::Ransac Estimation..."<<RESET<<std::endl;
// #endif
//     auto T = geometry::EstimateRigidTransformationRANSAC(correspondence_set, inliers, 
//         r_para.max_iteration, r_para.threshold);
//     RANSACRegistrationResult result;
//     result.T = T;
//     result.correspondence_set = inliers;
//     result.down_sample_source = source_pcd;
//     result.down_sample_target = target_pcd;
//     result.matches = matches;
//     return std::make_shared<RANSACRegistrationResult>(result);
// }
// void LoadMatrixFromFile(const std::string &filename, geometry::MatrixX &e)
// {
//     std::ifstream ifs(filename);
//     int rows, cols;
//     ifs>>rows>>cols;
//     e.resize(rows, cols);
//     for(int i = 0; i != rows; ++i)
//     for(int j = 0; j != cols; ++j)
//     ifs>>e(i,j);
//     ifs.close();
// }
int main(int argc, char **argv)
{
#if 1
    if(argc < 3)
    {
        std::cout<<"Usage: RansacTest [source_pcd] [target_pcd]"<<std::endl;
        return 1;
    }

    geometry::PointCloud s_pcd, t_pcd;
    s_pcd.LoadFromPLY(argv[1]);
    t_pcd.LoadFromPLY(argv[2]);
    auto rand_T = geometry::RandomTransformation();
    rand_T.block<3,1>(0,3) = geometry::Point3(3,0,0);
    s_pcd.Transform(rand_T);

    //auto result1 = registration::PointToPoint(s_pcd, t_pcd, init_T, icp_para);
    registration::RANSACParameter r_para;
    r_para.search_radius = 0.25;
    r_para.max_nn = 100;
    r_para.voxel_len = 0.05;
    r_para.search_radius_normal = 0.1;
    r_para.max_nn_normal = 30;
    r_para.max_iteration = 400;
    r_para.threshold = 0.1;
    // r_para.scaling = 1.0;
    auto result1 = registration::RansacRegistration(s_pcd, t_pcd, r_para);
    std::cout<<result1->T<<std::endl;
    // auto draw_matches = DrawMatches(result1->down_sample_source, result1->down_sample_target, result1->correspondence_set);
    // std::cout<<"draw_matches: "<<draw_matches->points.size()<<std::endl;
    // result1->down_sample_source.WriteToPLY("source_pre.ply");
    // result1->down_sample_target.WriteToPLY("target_pre.ply");
    // draw_matches->WriteToOBJ("matches.obj");
    s_pcd.colors.resize(s_pcd.points.size());
    t_pcd.colors.resize(t_pcd.points.size());

    for(size_t i = 0; i != s_pcd.points.size(); ++i)
    {
        s_pcd.colors[i] = geometry::Point3(1, 0, 0);
    }

    //s_pcd.Transform(result1->T);

    for(size_t i = 0; i != t_pcd.points.size(); ++i)
        t_pcd.colors[i] = geometry::Point3(0, 0, 1);
#else
    geometry::PointCloud s_pcd, t_pcd;
    s_pcd.LoadFromPLY(argv[1]);
    t_pcd.LoadFromPLY(argv[2]);
    registration::RANSACParameter r_para;
    r_para.search_radius = 0.25;
    r_para.max_nn = 30;
    r_para.voxel_len = 0.05;
    r_para.threshold = 0.05;
    geometry::MatrixX source_fpfh;
    geometry::MatrixX target_fpfh;
    LoadMatrixFromFile(argv[3], source_fpfh);
    LoadMatrixFromFile(argv[4], target_fpfh);
    std::cout<<source_fpfh<<std::endl;

    auto result1 = halfGlobalRegistration(s_pcd, t_pcd, source_fpfh, target_fpfh, r_para);
    std::cout<<result1->T<<std::endl;
    auto draw_matches = DrawMatches(result1->down_sample_source, result1->down_sample_target, result1->correspondence_set, result1->matches);
    std::cout<<"draw_matches: "<<draw_matches->points.size()<<std::endl;
    draw_matches->WriteToOBJ("matches.obj");
    s_pcd.colors.resize(s_pcd.points.size());
    t_pcd.colors.resize(t_pcd.points.size());

    for(size_t i = 0; i != s_pcd.points.size(); ++i)
    {
        s_pcd.colors[i] = geometry::Point3(1, 0, 0);
    }

    s_pcd.Transform(result1->T);
    for(size_t i = 0; i != t_pcd.points.size(); ++i)
        t_pcd.colors[i] = geometry::Point3(0, 1, 0);
#endif
    //Draw correspondence
    geometry::Point3List line_points;
    geometry::Point3List line_colors;
    std::vector<std::pair<int, int>> line_index;
    for(size_t i = 0; i != result1->correspondence_set.size() ; ++i)
    {
        line_points.push_back(result1->correspondence_set[i].first);
        line_points.push_back(result1->correspondence_set[i].second);

        line_colors.push_back(geometry::Point3(0, 1, 0));
        line_index.push_back(std::make_pair(line_points.size()-2,line_points.size()-1));
    }
    

    geometry::SE3List camera_poses;
    geometry::Point3List camera_colors;
    camera_poses.push_back(rand_T);
    camera_poses.push_back(geometry::TransformationMatrix::Identity());
    camera_colors.resize(2, geometry::Point3::Zero());

    visualization::Visualizer visualizer;
    visualizer.SetDrawColor(1);
    //visualizer.AddPointCloud(result1->down_sample_source);
    //visualizer.AddPointCloud(result1->down_sample_target);
    visualizer.AddPointCloud(s_pcd);
    visualizer.AddPointCloud(t_pcd);

    for(size_t i = 0; i != s_pcd.points.size(); ++i)
    {
        s_pcd.colors[i] = geometry::Point3(1, 0, 1);
    }
    s_pcd.Transform(result1->T);
    visualizer.AddPointCloud(s_pcd);
    visualizer.AddLineSet(line_points, line_index, line_colors);
    visualizer.AddCameraSet(camera_poses, camera_colors);
    visualizer.Show();    
    return 0;
}