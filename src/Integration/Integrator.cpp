
#include "Integrator.h"

namespace one_piece
{
namespace integration
{
    float Integrator::GetSDF(const geometry::Point3 &point, const camera::PinholeCamera &camera, 
        const geometry::TransformationMatrix &pose, const cv::Mat &depth)
    {
        float fx = camera.GetFx();
        float fy = camera.GetFy();
        float cx = camera.GetCx();
        float cy = camera.GetCy();
        float depth_scale = camera.GetDepthScale();
        int width = camera.GetWidth();
        int height = camera.GetHeight();
        geometry::TransformationMatrix pose_inv = pose.inverse();
        geometry::Point3 camera_point = (pose_inv * geometry::Vector4(point(0), point(1), point(2), 1) ).head<3>();
        int u = fx * camera_point(0)/ camera_point(2)+0.5 +cx;
        int v = fy * camera_point(1)/ camera_point(2)+0.5 +cy;        
        //std::cout <<u << " " << v << std::endl;
        if(v < 0 || v >= height || u < 0|| u >= width)
        return 999;
        float d;
        if(depth.depth() == CV_32FC1)
            d = depth.at<float>(v,u);
        else 
            d = depth.at<unsigned short>(v,u)/ depth_scale;
        if(d <= 0) return 999;

        float new_sdf = d -  camera_point(2);   
        //std::cout << new_sdf << std::endl;
        return new_sdf;     
    }
    void Integrator::IntegrateImage(const cv::Mat &depth, const cv::Mat &rgb, 
        const geometry::TransformationMatrix & pose, const camera::PinholeCamera &camera, VoxelCube &voxel_cube, const CubePara &c_para)
    {

        float fx = camera.GetFx();
        float fy = camera.GetFy();
        float cx = camera.GetCx();
        float cy = camera.GetCy();
        int width = camera.GetWidth();
        int height = camera.GetHeight();
        float depth_scale = camera.GetDepthScale();

        geometry::TransformationMatrix pose_inv = pose.inverse();

        // bool updated = false;

        for(int x = 0; x!= CUBE_SIZE; ++x)
        {
            for(int y = 0; y!= CUBE_SIZE; ++y)
            {
                for(int z = 0; z!= CUBE_SIZE; ++z)
                {
                    int voxel_id = x + y * CUBE_SIZE + z * CUBE_SIZE * CUBE_SIZE;
                    geometry::Point3 current_point = c_para.GetGlobalPoint(voxel_cube.cube_id, voxel_id);
                    geometry::Point3 camera_point = (pose_inv * geometry::Vector4(current_point(0), current_point(1), current_point(2), 1) ).head<3>();
                    int u = fx * camera_point(0)/ camera_point(2)+0.5 +cx;
                    int v = fy * camera_point(1)/ camera_point(2)+0.5 +cy;
                    if(v < 0 || v >= height || u < 0|| u >= width) continue;

                    float d;
                    if(depth.depth() == CV_32FC1)
                        d = depth.at<float>(v,u);
                    else 
                        d = depth.at<unsigned short>(v,u)/ depth_scale;
                    if(d <=0) continue;

                    float new_sdf = d -  camera_point(2);
                    
                    if(std::fabs(new_sdf) < truncation)
                    {

                        TSDFVoxel new_voxel(new_sdf,1.0,
                            geometry::Point3(rgb.at<cv::Vec3b>(v, u)[0], rgb.at<cv::Vec3b>(v, u)[1], rgb.at<cv::Vec3b>(v, u)[2])/255.0);
                        if(voxel_cube.voxels[voxel_id].IsValid())
                        {
                            //float old_sdf = voxel_cube.voxels[voxel_id].weight;
                            voxel_cube.voxels[voxel_id] = voxel_cube.voxels[voxel_id] + new_voxel;
                            //std::cout << voxel_cube.voxels[voxel_id].weight<<" "<<old_sdf<<" "<< new_voxel.weight << std::endl;
                        }
                        else
                            voxel_cube.voxels[voxel_id] = new_voxel;
                        // updated = true;
                    }

                    
                }
            }
        }
    }
    
}
}