#ifndef IO_H 
#define IO_H   
#include "Geometry/Geometry.h"
#include "ConsoleColor.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
namespace fucking_cool
{
namespace tool
{
    void ReadImageSequence(const std::string & path, std::vector<std::string> &rgb_files, std::vector<std::string> &depth_files);
    void ReadImageSequenceWithPose(const std::string & path, std::vector<std::string> &rgb_files, std::vector<std::string> &depth_files, 
        std::vector<geometry::TransformationMatrix > & poses);
} 
}
#endif