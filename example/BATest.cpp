#include "Optimization/BundleAdjustment.h"
#include "Odometry/Odometry.h"
#include "Visualization/Visualizer.h"
using namespace one_piece;

//0: randomly generate data
//1: feature matching
//2: input data
#define TYPE 1
void GenerateData(geometry::Point3List &world_points, 
    std::vector<optimization::ProjectedPointsOnFrame> &projected_points, 
    geometry::SE3List &poses,
    const camera::PinholeCamera &camera, int world_points_size = 100, int frame_size = 20)
{
    int radius = 8;
    world_points.resize(world_points_size);
    poses.resize(frame_size); 
    projected_points.resize(frame_size);

    for(int i = 0; i < frame_size; ++i)
    {
        float theta =  i / (frame_size + 0.0)*2*M_PI/4;

        geometry::Matrix3 R=Eigen::AngleAxis<geometry::scalar>(theta, geometry::Point3::UnitZ()).toRotationMatrix();
        geometry::Point3 t = geometry::Point3(radius * cos(theta) - radius, radius * sin(theta), 1 * sin(2 * theta));
        geometry::SE3 pose;
        pose.setZero();
        pose.block<3, 3>(0, 0) = R;
        pose.block<3, 1>(0, 3) = t;
        pose(3,3) = 1.0;
        //std::cout<<pose<<std::endl;
        poses[i] = pose;
    }

    // generate world points
    std::default_random_engine generator;
    std::normal_distribution<double> noise_pdf(0.0, 1.0);  
    auto K = camera.ToCameraMatrix();
    for (int i = 0; i < world_points_size; ++i) 
    {
        std::uniform_real_distribution<double> xy_rand(-8, 8.0);
        std::uniform_real_distribution<double> z_rand(2., 16.);

        geometry::Point3 Pw(xy_rand(generator), xy_rand(generator), z_rand(generator));
        world_points[i] = Pw;

        // projected points
        for (int j = 0; j < frame_size; ++j) 
        {

            geometry::Point3 Pc = K * geometry::TransformPoint(poses[j].inverse(), Pw);       
            geometry::Vector2 uv = (Pc / Pc(2)).head<2>();
            uv(0)+= noise_pdf(generator);
            uv(1)+= noise_pdf(generator);
            projected_points[j][i] = uv;

        }
    }
}
void GenerateData(geometry::Point3List &world_points, 
    std::vector<optimization::ProjectedPointsOnFrame> &projected_points, 
    geometry::SE3List &poses)
{
    //input data
    int frame_size = 0;
    int points_world_size = 0;
    std::cin>>points_world_size;
    std::cout<<std::endl;
    world_points.resize(points_world_size, geometry::Point3::Zero());
    
    for(int i = 0; i < points_world_size;i++)
    {
        std::cin>>world_points[i](0)>>world_points[i](1)>>world_points[i](2);
        std::cout<<world_points[i].transpose()<<std::endl;
    }

    std::cin>>frame_size;
    projected_points.resize(frame_size);
    poses.resize(frame_size);
    for(int i = 0; i < frame_size; ++i)
    {
        geometry::Matrix4 pose;
        for(int x = 0; x < 4; ++x)
        for(int y = 0; y < 4; ++y)
        std::cin>>pose(x,y);

        poses[i] = pose;
    }
    for(int i = 0; i != frame_size; ++i)
    {
        for(int j = 0; j != points_world_size; ++j)
        {
            int world_id;
            std::cin>>world_id;
            geometry::Vector2 uv;
            std::cin>>uv(0)>>uv(1);
            projected_points[i][world_id] = uv;
        }
    }
}
void GenerateData(geometry::Point3List &world_points, 
    std::vector<optimization::ProjectedPointsOnFrame> &projected_points, 
    geometry::SE3List &poses,
    const camera::PinholeCamera &camera, 
    const odometry::SparseTrackingResult &tracking_result, 
    const geometry::RGBDFrame &source_frame, 
    const geometry::RGBDFrame &target_frame)
{
    auto &matches = tracking_result.correspondence_set_index;
    auto &T = tracking_result.T;
    poses.resize(2, geometry::SE3::Identity());
    projected_points.resize(2);

    poses[1] = T.inverse();
    std::vector<float> residuals;
    std::set<int> source_local_id;
    std::set<int> target_local_id;
    std::cout<<matches.size()<<std::endl;
    for(size_t i = 0; i != matches.size(); ++i)
    {
        // std::cout<<matches[i].first<<" "<<matches[i].second<<std::endl;
        residuals.push_back((source_frame.feature_pcd.points[matches[i].first] - geometry::TransformPoint(T.inverse() ,
            target_frame.feature_pcd.points[matches[i].second])).norm());
        // std::cout<<<<std::endl;
        if(source_local_id.find(matches[i].first) != source_local_id.end())
        {
            std::cout<<"source id duplicated"<<std::endl;
            continue;

        }
        if(target_local_id.find(matches[i].second) != target_local_id.end())
        {
            continue;
            std::cout<<"target id duplicated"<<std::endl;
        }
        source_local_id.insert(matches[i].first);
        target_local_id.insert(matches[i].second);
        world_points.push_back(source_frame.feature_pcd.points[matches[i].first]);
        geometry::Point2 uv_source, uv_target;
        uv_source(0) = source_frame.keypoints[matches[i].first].pt.x;
        uv_source(1) = source_frame.keypoints[matches[i].first].pt.y;
        uv_target(0) = target_frame.keypoints[matches[i].second].pt.x;
        uv_target(1) = target_frame.keypoints[matches[i].second].pt.y;
        projected_points[0][world_points.size() - 1] = uv_source;
        projected_points[1][world_points.size() - 1] = uv_target;
    }
}
optimization::ProjectedPointsOnFrame GetReprojectedPoints(const geometry::Point3List &world_points, 
    const geometry::SE3 &pose, const camera::PinholeCamera &camera)
{
    optimization::ProjectedPointsOnFrame result;
    auto K = camera.ToCameraMatrix();
    for(size_t i = 0; i < world_points.size(); ++i)
    {
        auto reprojected_point_3d = K * geometry::TransformPoint(pose.inverse(), world_points[i]);
        auto uv = (reprojected_point_3d/ reprojected_point_3d(2)).head<2>();
        result[i] = uv;
    }
    return result;
}
cv::Mat DrawCircles(const cv::Mat &img, const optimization::ProjectedPointsOnFrame &projected_points, 
    const geometry::Point3 &color, int radius = 3)
{
    cv::Mat result = img.clone();
    for(auto iter = projected_points.begin(); iter != projected_points.end(); ++iter)
    {
        cv::circle(result, cv::Point(iter->second(0), iter->second(1)), radius, cv::Scalar(color(0) * 255, color(1) * 255, color(2) * 255), -1);
    }
    return result;
}
int main(int argc, char** argv)
{
    geometry::Point3List world_points;
    std::vector<optimization::ProjectedPointsOnFrame> projected_points; 
    geometry::SE3List poses;
    camera::PinholeCamera camera;
    camera.SetCameraType(camera::CameraType::OPEN3D_DATASET);
    
#if TYPE==0
    GenerateData(world_points, projected_points, poses, camera);
    optimization::BundleAdjustment(world_points, projected_points, poses, camera);
#elif TYPE==1
    if(argc != 5)
    {
        std::cout<<"Usage: BATest [source_rgb] [source_depth] [target_rgb] [targe_depth]"<<std::endl;
        return 1;
    }
    odometry::Odometry rgbd_odometry(camera);
    
    cv::Mat source_rgb = cv::imread(argv[1]);
    cv::Mat source_depth = cv::imread(argv[2],-1);
    cv::Mat target_rgb = cv::imread(argv[3]);
    cv::Mat target_depth = cv::imread(argv[4],-1);

    geometry::RGBDFrame source_frame(source_rgb, source_depth);
    geometry::RGBDFrame target_frame(target_rgb, target_depth);

    auto tracking_result = rgbd_odometry.SparseTrackingMILD(source_frame, target_frame);
    // tracking_result->correspondence_set_index.resize(3);
    // tracking_result->correspondence_set.resize(3);
    tracking_result->T = geometry::SE3::Identity();
    if(tracking_result->tracking_success)
    {
        GenerateData(world_points, projected_points, poses, camera, *tracking_result, source_frame, target_frame);
    }
    //poses[1] = geometry::Matrix4::Identity();
    auto reprojected_points_source = GetReprojectedPoints(world_points, poses[0], camera);
    auto reprojected_points_target = GetReprojectedPoints(world_points, poses[1], camera);

    cv::Mat gt_source = DrawCircles(source_rgb, projected_points[0], geometry::Point3(1,0,0), 4);
    cv::Mat before_BA_source = DrawCircles(gt_source, reprojected_points_source, geometry::Point3(0,1,0));


    cv::Mat gt_target = DrawCircles(target_rgb, projected_points[1], geometry::Point3(1,0,0), 4);
    cv::Mat before_BA_target = DrawCircles(gt_target, reprojected_points_target, geometry::Point3(0,1,0));

   
    optimization::BundleAdjustment(world_points, projected_points, poses, camera);

    reprojected_points_source = GetReprojectedPoints(world_points, poses[0], camera);
    reprojected_points_target = GetReprojectedPoints(world_points, poses[1], camera);
    cv::Mat after_BA_source = DrawCircles(gt_source, reprojected_points_source, geometry::Point3(0,1,0));
    cv::Mat after_BA_target = DrawCircles(gt_target, reprojected_points_target, geometry::Point3(0,1,0));
    
    cv::imwrite("before_BA_source.png", before_BA_source);
    cv::imwrite("after_BA_source.png", after_BA_source);

    cv::imwrite("before_BA_target.png", before_BA_target);
    cv::imwrite("after_BA_target.png", after_BA_target);
    
    geometry::PointCloud pcd_source, pcd_target;
    pcd_source.LoadFromRGBD(source_rgb,source_depth,camera);
    pcd_target.LoadFromRGBD(target_rgb,target_depth,camera);
    pcd_target.Transform(poses[1]);
    for(size_t i = 0; i != pcd_source.colors.size(); ++i)
    pcd_source.colors[i] = geometry::Point3(1,0,0);

    for(size_t i = 0; i != pcd_target.colors.size(); ++i)
    pcd_target.colors[i] = geometry::Point3(0,1,0);
    
    visualization::Visualizer visualizer;
    visualizer.AddPointCloud(pcd_source);
    visualizer.AddPointCloud(pcd_target);
    visualizer.Show();
#else 

    GenerateData(world_points, projected_points, poses);
    std::cout << poses[1]<<std::endl;
    std::cout << poses[1].inverse()<<std::endl;
    optimization::BundleAdjustment(world_points, projected_points, poses, camera, 100);
#endif 
    return 1;
}