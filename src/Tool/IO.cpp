#include "IO.h"
#include "CppExtension.h"
namespace fucking_cool
{
namespace tool
{
    cv::Mat AlignColorToDepth(const cv::Mat &color, const cv::Mat &depth, 
        const camera::PinholeCamera &rgb_camera, const camera::PinholeCamera &depth_camera,
        const geometry::TransformationMatrix &color_to_depth)
    {
        //color_to_depth means relative pose from rgb_camera to depth_camera
        geometry::ImageXYZ xyz;
        geometry::TransformToMatXYZ(depth, depth_camera, xyz);
        auto color_K = rgb_camera.ToCameraMatrix();
        // auto depth_K = depth_camera.ToCameraMatrix();
        int depth_width = depth_camera.GetWidth();
        int depth_height = depth_camera.GetHeight();
        int color_width = rgb_camera.GetWidth();
        int color_height = depth_camera.GetHeight();
        cv::Mat aligned_color(depth_height, depth_width, CV_8UC3,cv::Scalar(0, 0, 0));
        //transform the point cloud into color camera coordinate
        if(xyz.size() <= 0)
        {
            std::cout<<RED<<"[ERROR]::[AlignColorToDepth]::XYZImage has zero rows."<<RESET<<std::endl;
            return aligned_color;
        }
        for(size_t v = 0; v != xyz.size(); ++v)
        {            
            for(size_t u = 0; u != xyz[0].size(); ++u)
            {
                auto tmp_point = xyz[v][u];
                if(tmp_point[2] > 0)
                {
                    auto transformed_point = geometry::TransformPoint(color_to_depth, tmp_point);
                    geometry::Point2 uv_color = (color_K * (transformed_point / transformed_point[2])).head<2>();
                    int color_u = (int)(uv_color(0) + 0.5);
                    int color_v = (int)(uv_color(1) + 0.5);
                    if(color_u >= 0 && color_u < color_width && color_v >= 0 && color_v < color_height)
                    aligned_color.at<cv::Vec3b>(v, u) = color.at<cv::Vec3b>(color_v, color_u);
                }
            }
        }
        return aligned_color;
    }
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
    void ReadImageSequenceFromScannet(const std::string & path, std::vector<std::string> &rgb_files, 
        std::vector<std::string> &depth_files, camera::PinholeCamera &rgb_camera, 
        camera::PinholeCamera &depth_camera)
    {
        std::ifstream ifs((path+"/_info.txt").c_str());
        char line[1000];
        int color_width = -1, color_height = -1;
        int depth_width = -1, depth_height = -1;
        int depth_scale = -1;
        size_t frames_size = 0;
        //color intrinsics and depth intrinsics
        float fx_c = 0, cx_c = 0, fy_c = 0, cy_c = 0;
        float fx_d = 0, cx_d = 0, fy_d = 0, cy_d = 0;
        while(ifs.getline(line, 1000,'\n'))
        {
            std::vector<std::string> parse_results = Split(line, " = ");
            if(parse_results.size() != 2)
            {
                std::cout<<YELLOW<<"[Warning]::[ReadImageSequenceFromScannet]::Wrong format of _info.txt"<<RESET<<std::endl;
                break;
            }
            if(parse_results[0] == "m_versionNumber") continue;
            else if(parse_results[0] == "m_sensorName") continue;
            else if(parse_results[0] == "m_colorWidth") color_width = atoi(parse_results[1].c_str());
            else if(parse_results[0] == "m_colorHeight") color_height = atoi(parse_results[1].c_str());
            else if(parse_results[0] == "m_depthWidth") depth_width = atoi(parse_results[1].c_str());
            else if(parse_results[0] == "m_depthHeight") depth_height = atoi(parse_results[1].c_str());
            else if(parse_results[0] == "m_depthShift") depth_scale = atoi(parse_results[1].c_str());
            else if(parse_results[0] == "m_calibrationColorIntrinsic")
            {
                std::vector<std::string> intrinsic = Split(parse_results[1], " ");
                fx_c = atof(intrinsic[0].c_str());
                cx_c = atof(intrinsic[2].c_str());
                fy_c = atof(intrinsic[5].c_str());
                cy_c = atof(intrinsic[6].c_str());
            }
            else if(parse_results[0] == "m_calibrationColorExtrinsic") continue;
            else if(parse_results[0] == "m_calibrationDepthIntrinsic")
            {
                std::vector<std::string> intrinsic = Split(parse_results[1], " ");
                fx_d = atof(intrinsic[0].c_str());
                cx_d = atof(intrinsic[2].c_str());
                fy_d = atof(intrinsic[5].c_str());
                cy_d = atof(intrinsic[6].c_str());
            }
            else if(parse_results[0] == "m_calibrationDepthExtrinsic") continue;
            else if(parse_results[0] == "m_frames.size") frames_size = atoi(parse_results[1].c_str());
            else
            {
                std::cout<<YELLOW<<"[Warning]::[ReadImageSequenceFromScannet]::Wrong format of _info.txt"<<RESET<<std::endl;
                break;
            }
        }

        rgb_camera.SetPara(fx_c, fy_c, cx_c, cy_c, color_width, color_height);
        depth_camera.SetPara(fx_d, fy_d, cx_d, cy_d, depth_width, depth_height, depth_scale);
        rgb_files.clear();
        depth_files.clear();
        for(size_t i = 0; i != frames_size; ++i)
        {
            std::ostringstream oss;
            oss<<std::setw(6)<<std::setfill('0')<<i; 
            std::string index = oss.str();
            rgb_files.push_back(path + "/frame-" +index +".color.jpg");
            depth_files.push_back(path + "/frame-" +index +".depth.pgm");
        }
    }

    void ReadImageSequenceFromScannetWithPose(const std::string & path, std::vector<std::string> &rgb_files, 
        std::vector<std::string> &depth_files, std::vector<geometry::TransformationMatrix > & poses, 
        camera::PinholeCamera &rgb_camera, camera::PinholeCamera &depth_camera)
    {
        ReadImageSequenceFromScannet(path, rgb_files, depth_files, rgb_camera, depth_camera);
        poses.clear();
        for(size_t i = 0; i < rgb_files.size(); ++i)
        {
            
            std::ostringstream oss;
            oss<<std::setw(6)<<std::setfill('0')<<i; 
            std::string pose_file = path + "/frame-" + oss.str() +".pose.txt";
            std::ifstream pose_in(pose_file);
            geometry::TransformationMatrix T;
            pose_in >> T(0, 0) >> T(0, 1) >> T(0, 2) >> T(0, 3)
                >>T(1, 0) >> T(1, 1) >> T(1, 2) >> T(1, 3)
                >>T(2, 0) >> T(2, 1) >> T(2, 2) >> T(2, 3)
                >>T(3, 0) >> T(3, 1) >> T(3, 2) >> T(3, 3);                
            poses.push_back(T);
        }
    }
}
}