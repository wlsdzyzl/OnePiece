#include "IO.h"
namespace fucking_cool
{
namespace tool
{
    void ReadImageSequence(const std::string & path,std::vector<std::string> &rgb_files, std::vector<std::string> &depth_files)
    {
        std::ifstream ifs((path+"/associate.txt").c_str());
        char line[1000];
        std::string t_rgb, t_depth;
        std::string rgb, depth;
        //ifs>>t_rgb;
        //ifs.getline(line, 1000,'\n');
        //std::cout << std::string(line)<<std::endl;
        while(ifs.getline(line, 1000,'\n'))
        {
            std::istringstream iss(line);
            iss >> t_rgb;
            iss >> rgb;
            iss >> t_depth;
            iss >> depth;
            rgb_files.push_back(path + "/" +rgb);
            depth_files.push_back(path + "/" +depth);

        }
        std::cout<<GREEN<<"[ReadImageSequence]::[INFO]::Read "<<rgb_files.size()<<" images successfully."<<RESET<<std::endl;
    }
    void ReadImageSequenceWithPose(const std::string & path, std::vector<std::string> &rgb_files, std::vector<std::string> &depth_files, 
        std::vector<geometry::TransformationMatrix > & poses)
    {
        std::ifstream ifs((path+"/trajectory.txt").c_str());
        if(!ifs)
        {
            std::cout<<RED << "[ReadImageSequenceWithPose]::[ERROR]::No file named trajectory.txt."<<RESET<<std::endl;
            return;
        }
        char line[1000];
        ReadImageSequence(path, rgb_files, depth_files);
        geometry::TransformationMatrix pose;
        while(ifs.getline(line,1000,'\n'))
        {
            std::istringstream iss(line);
            iss >> pose(0,0) >> pose(0,1) >> pose(0,2) >> pose(0,3) 
                >> pose(1,0) >> pose(1,1) >> pose(1,2) >> pose(1,3)
                >> pose(2,0) >> pose(2,1) >> pose(2,2) >> pose(2,3)
                >> pose(3,0) >> pose(3,1) >> pose(3,2) >> pose(3,3);
            
            poses.push_back(pose);
        }
        if(poses.size() != rgb_files.size())
        {
            std::cout<<YELLOW << "[ReadImageSequenceWithPose]::[WARNING]:: The number of images and poses do not match."<<RESET<<std::endl;
            return;
        }
    }
}
}