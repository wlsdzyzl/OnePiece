#ifndef IO_H 
#define IO_H   
#include "Geometry/Geometry.h"
#include "Geometry/PointCloud.h"
#include "ConsoleColor.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
namespace fucking_cool
{
namespace tool
{
    //get an aligned rgb image
    cv::Mat AlignColorToDepth(const cv::Mat &color, const cv::Mat &depth, 
        const camera::PinholeCamera &rgb_camera, const camera::PinholeCamera &depth_camera,
        const geometry::TransformationMatrix &color_to_depth = geometry::TransformationMatrix::Identity());
    //read image sequence from tum
    void ReadImageSequence(const std::string & path, std::vector<std::string> &rgb_files, std::vector<std::string> &depth_files);
    void ReadImageSequenceWithPose(const std::string & path, std::vector<std::string> &rgb_files, std::vector<std::string> &depth_files, 
        std::vector<geometry::TransformationMatrix > & poses);

    //read image sequence from scannet
    void ReadImageSequenceFromScannet(const std::string & path, std::vector<std::string> &rgb_files, 
        std::vector<std::string> &depth_files, camera::PinholeCamera &rgb_camera, 
        camera::PinholeCamera &depth_camera);

    void ReadImageSequenceFromScannetWithPose(const std::string & path, std::vector<std::string> &rgb_files, 
        std::vector<std::string> &depth_files, std::vector<geometry::TransformationMatrix > & poses, 
        camera::PinholeCamera &rgb_camera, camera::PinholeCamera &depth_camera);

} 
}
#endif