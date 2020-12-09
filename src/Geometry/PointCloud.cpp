#include "PointCloud.h"
#include "Geometry/KDTree.h"
#include "Tool/ConsoleColor.h"
#include "Tool/PLYManager.h"
#include "Tool/OBJManager.h"
#include "Tool/CppExtension.h"
#include <fstream>
#include <iostream>
#include <unordered_set>
#include <omp.h>
#include "RGBDFrame.h"
namespace one_piece
{
namespace geometry 
{

    void PointCloud::LoadFromRGBD(const cv::Mat &rgb, const cv::Mat & depth, const camera::PinholeCamera &camera )
    {
        Reset();
        float fx = camera.GetFx(), fy = camera.GetFy(), cx = camera.GetCx(), cy = camera.GetCy();
        float depth_scale = camera.GetDepthScale();
        float scale = 255.0;
        points.resize(depth.rows*depth.cols);
        colors.resize(rgb.rows*rgb.cols);
        int cnt = 0;
        for (int i = 0; i < depth.rows; i++) 
        {
            unsigned char *pc = rgb.data;
            for (int j = 0; j < depth.cols; j++, pc += 3) {
                float  z;
                if(depth.depth() == CV_32FC1)
                    z = depth.at<float>(i,j);
                else 
                    z = depth.at<unsigned short>(i,j)/ depth_scale;
                if (z > 0) {
                    float x = (j - cx) * z / fx;
                    float y =
                            (i - cy) * z / fy;
                    points[cnt] = geometry::Point3(x, y, z);
                    colors[cnt++] =
                            geometry::Point3(rgb.at<cv::Vec3b>(i, j)[0], rgb.at<cv::Vec3b>(i, j)[1], rgb.at<cv::Vec3b>(i, j)[2]) /
                            scale;
                }
            }
        }
        points.resize(cnt);
        colors.resize(cnt);
    }
    void PointCloud::MergePCD(const PointCloud & another_pcd)
    {
        int merged_point_size = points.size() + another_pcd.points.size();
        int merged_colors_size = colors.size() + another_pcd.colors.size();
        int merged_normals_size = normals.size() + another_pcd.normals.size();
        if(merged_point_size != merged_colors_size && merged_colors_size > 0)
        {
            std::cout<<RED<<"[Error]::[MergePCD]::The color are not matching."<<RESET<<std::endl;
            return;
        }
        if(merged_point_size != merged_normals_size && merged_normals_size > 0)
        {
            std::cout<<RED<<"[Error]::[MergePCD]::The normal are not matching."<<RESET<<std::endl;
            return;
        }
        points.insert(points.end(), another_pcd.points.begin(), another_pcd.points.end());
        colors.insert(colors.end(), another_pcd.colors.begin(), another_pcd.colors.end());
        normals.insert(normals.end(), another_pcd.normals.begin(), another_pcd.normals.end());
    }
    void PointCloud::LoadFromRGBD(const RGBDFrame &rgbd, const camera::PinholeCamera &camera )
    {
        LoadFromRGBD(rgbd.rgb, rgbd.depth, camera);
    }
    void PointCloud::LoadFromDepth(const cv::Mat &depth, const camera::PinholeCamera &camera )
    {
        Reset();   
        float fx = camera.GetFx(), fy = camera.GetFy(), cx = camera.GetCx(), cy = camera.GetCy();
        float depth_scale = camera.GetDepthScale();
        // float scale = 255.0;
        points.resize(depth.rows*depth.cols);
        int cnt = 0;
        for (int i = 0; i < depth.rows; i++) 
        {
            for (int j = 0; j < depth.cols; j++) {
                float  z;
                
                if(depth.depth() == CV_32FC1)
                    z = depth.at<float>(i,j);
                else 
                    z = depth.at<unsigned short>(i,j)/ depth_scale;
                
                if (z > 0) {
                    float x = (j - cx) * z / fx;
                    float y =
                            (i - cy) * z / fy;
                    points[cnt++] = geometry::Point3(x, y, z);
                }

            }
        }
        points.resize(cnt);
    }

    void PointCloud::EstimateNormals(float radius, int knn)
    {
#if 0
        normals.clear();

        geometry::Point3 mean_point(0,0,0);

        auto dspcd = DownSample(0.05);
        for(size_t i = 0; i != dspcd->points.size(); ++i)
            mean_point+= dspcd->points[i];
        mean_point /= dspcd->points.size();
#endif
        
        normals.resize(points.size());        
        // fill the cv_pcd with the cartesian coordinates (x, y z) 


        // std::vector<cv::Point3f> cv_pcd;
        // for(size_t i = 0; i < points.size(); ++i)
        // {
        //     cv_pcd.push_back(cv::Point3f(points[i](0), points[i](1), points[i](2)));
        // } 
        // cv::Mat input = cv::Mat(cv_pcd);
        // std::cout<<input.rows<<" "<<input.cols<<std::endl;
        // cv::flann::KDTreeIndexParams indexParams; 
        // cv::flann::Index kdtree(cv::Mat(cv_pcd).reshape(1), indexParams);
    
        //std::vector<std::vector<int>>
        geometry::KDTree<> kdtree;
        kdtree.BuildTree(points);
        std::cout<<BLUE<<"[EstimateNormals]::[INFO]::RadiusSearch "<<knn<<" nearest points, radius: "<<radius<<RESET<<std::endl;

        for(size_t i = 0; i < points.size(); ++i)
        {
            
            std::vector<int> indices; 
            std::vector<float> dists; 
            
            // kdtree.RadiusSearch(points[i], indices, dists, 
            //     radius, knn, geometry::SearchParameter(128));
            kdtree.RadiusSearch(points[i], indices, dists, radius, 
                30, geometry::SearchParameter(30));
            
            geometry::Point3List nearest_points;


            for(size_t j = 0; j!= indices.size() && j != static_cast<size_t>(knn);++j)
            {
                nearest_points.push_back(points[indices[j]]);
                //std::cout <<indices[j]<<" "<<std::endl;
            }
            
            //std::cout <<std::endl;
            auto result = FitPlane(nearest_points);
            normals[i] = std::get<0>(result);
        }

#if 0
        for(size_t i = 0; i < cv_pcd.size(); ++i)
        {
            if(normals[i].dot(points[i] - mean_point) < 0)
            normals[i] = -normals[i];
        }
#endif 
    }
    std::shared_ptr<PointCloud> PointCloud::DownSample(float grid_len) const
    {
        PointCloud pcd = *this;
        std::unordered_map<Eigen::Vector3i, std::pair<int, int>, geometry::VoxelGridHasher > grids;     
         
        int ptr = 0;  
        bool has_color = HasColors();
        bool has_normal = HasNormals();
        //float half_len = grid_len / 2;
        for(size_t i = 0;i!= pcd.points.size(); ++i)
        {
            Eigen::Vector3i index = Eigen::Vector3i(std::floor(pcd.points[i](0) / grid_len), 
                std::floor(pcd.points[i](1) / grid_len), std::floor(pcd.points[i](2) / grid_len));
            if(grids.find(index) == grids.end())
            {
                pcd.points[ptr] = pcd.points[i];
                if(has_color)
                pcd.colors[ptr] = pcd.colors[i];
                if(has_normal)
                pcd.normals[ptr] = pcd.normals[i];
                grids[index] = std::make_pair(ptr, 1);
                ptr++;
            }
            else
            {
                pcd.points[grids[index].first] += pcd.points[i];
                if(has_color)
                pcd.colors[grids[index].first] += pcd.colors[i];
                if(has_normal)
                pcd.normals[grids[index].first] += pcd.normals[i];
                grids[index].second += 1;
            }
        }
        for(auto iter = grids.begin(); iter != grids.end(); ++iter)
        {
            pcd.points[iter->second.first] /= iter->second.second;
            if(has_color) pcd.colors[iter->second.first] /= iter->second.second;
            if(has_normal) pcd.normals[iter->second.first] /= iter->second.second;            
        }
        pcd.points.resize(ptr);
        if(has_color) pcd.colors.resize(ptr);
        if(has_normal) pcd.normals.resize(ptr);
        //if(HasNormals()) pcd.EstimateNormals();
        return std::make_shared<PointCloud>(pcd);
    }
    void PointCloud::LoadFromXYZ(const ImageXYZ &xyz)
    {
        Reset();
        int cnt = 0;
        if(xyz.size() == 0) return;
        points.resize(xyz.size()* xyz[0].size()); 

        for(size_t i = 0;i!=xyz.size();++i)
        {
            for(size_t j = 0;j!=xyz[i].size();++j)
            if(xyz[i][j](2)>0)
                points[cnt++] = xyz[i][j];
        }
        points.resize(cnt);
    }
    bool PointCloud::LoadFromPLY(const std::string &filename)
    {
        Reset();
        std::vector<tool::AdditionalElement> additional_labels;
        geometry::Point3uiList triangles;
        return tool::ReadPLY(filename,points,normals,colors, triangles, additional_labels);
    }
    bool PointCloud::LoadFromOBJ(const std::string &filename)
    {
        Reset();
        return tool::ReadOBJ(filename,points,normals,colors);
    }
    bool PointCloud::LoadFromFile(const std::string & filename)
    {
        std::vector<std::string> result = tool::RSplit(filename, ".", 1);
        if(result.size() == 2)
        {
            if(result[1] == "obj")//obj
            {
                return LoadFromOBJ(filename);
            }
            else if(result[1] == "ply")
            {
                return LoadFromPLY(filename);
            }
        }
        std::cout<<YELLOW<<"[WARNING]::[LoadFromFile]::Dragon only supports obj and ply file."<<RESET<<std::endl;
        return false;
    }
    bool PointCloud::WriteToOBJ(const std::string &filename)
    {
        return tool::WriteOBJ(filename,points,normals,colors);
    }
    void PointCloud::Transform(const TransformationMatrix &T)
    {
        geometry::TransformPoints(T,points);
        if(HasNormals())
        geometry::TransformNormals(T,normals);
    }

    bool PointCloud::WriteToPLY(const std::string &filename) const
    {
        return tool::WritePLY(filename,points,normals,colors);
    }
}
}