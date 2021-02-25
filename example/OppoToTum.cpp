#include "Geometry/Geometry.h"
#include "Tool/TickTock.h"
#include "Tool/ImageProcessing.h"
#include "Tool/IO.h"
#include "Tool/CppExtension.h"
#include <random>

using namespace one_piece;
void ReadImageSequenceFromOppo(const std::string & path, std::vector<std::string> &rgb_files, 
    std::vector<std::string> &depth_files)
{
    size_t frames_size = 0;
    std::ifstream ifs((path+"/_info.txt").c_str());
    ifs>>frames_size;
    rgb_files.clear();
    depth_files.clear();
    for(size_t i = 0; i != frames_size; ++i)
    {
        std::ostringstream oss;
        oss<<std::setw(4)<<std::setfill('0')<<i; 
        std::string index = oss.str();
        rgb_files.push_back(path + "/rgb" +index +".png");
        depth_files.push_back(path + "/d" +index +".png");
    }
}
int main(int argc, char ** argv)
{
    // this code is to generate a model from scannet

    if(argc < 3)
    {
        std::cout<<"Usage: OppoToTum [input_folder] [output_folder] "<<std::endl;
        return 0;
    }
    
    //pinhole camera
    // camera::PinholeCamera camera(914.494141, 914.377991, 958.065430, 548.986206, 1920, 1080, 1000);
    camera::PinholeCamera camera(914.494141, 914.377991, 958.065430 / 2, 548.986206 / 2, 960, 540, 1000);
    std::vector<std::string> rgb_files, depth_files;
    std::string input_folder = argv[1];
    std::string output_folder = argv[2];
    tool::MakeDir(output_folder);
    tool::MakeDir(output_folder+"/rgb");
    tool::MakeDir(output_folder+"/depth");


    std::ofstream ofs_associate(output_folder+"/associate.txt");

    ReadImageSequenceFromOppo(argv[1], rgb_files, depth_files);
    std::cout<<"Read "<<rgb_files.size() <<" frames..."<<std::endl;
    std::cout<<"Depth Intrinsics:\n"<<camera.ToCameraMatrix()<<std::endl;

    
    // geometry::TransformationMatrix depth_to_color = geometry::TransformationMatrix::Identity();
    // depth_to_color << 0.999981,0.005941, -0.001712, -0.032088730,
    //                 -0.005734, 0.994731, 0.102359, -0.001874862,
    //                 0.002311, -0.102347, 0.994746, 0.003994728,
    //                 0, 0, 0, 1;
    // geometry::TransformationMatrix color_to_depth = depth_to_color.inverse();
    cv::Rect roi(480, 270, 960, 540);
    for(size_t i = 0; i != rgb_files.size(); ++i) 
    { 
        int frame_id = i;

        std::cout <<"Processing on "<<i<<"th image"<<std::endl;
        std::cout<<depth_files[frame_id]<<std::endl;
        cv::Mat rgb = cv::imread(rgb_files[frame_id]);
        cv::Mat depth = cv::imread(depth_files[frame_id],-1);
        cv::Mat truncated_rgb = rgb(roi);
        cv::Mat truncated_depth = depth(roi);
        //std::cout<<depth.depth()<<" "<<CV_16UC1<<std::endl; 
        // cv::Mat aligned_rgb = tool::AlignColorToDepth(rgb, depth, camera, camera, color_to_depth);

        cv::imwrite(output_folder+"/rgb/"+std::to_string(i)+".png", truncated_rgb);
        cv::imwrite(output_folder+"/depth/"+std::to_string(i)+".png", truncated_depth);
        ofs_associate<<i <<" rgb/"<<i<<".png "<<i<<" depth/"<<i<<".png"<<std::endl;
    }

    ofs_associate.close();

    std::ofstream ofs_calib(output_folder+"/calib.txt");
    ofs_calib<<camera.GetWidth()<<" "<<camera.GetHeight()<<" "<<camera.GetFx()
        <<" "<<camera.GetFy()<<" "<<camera.GetCx()<<" "<<camera.GetCy()<<" 0.0 0.0 0.0 0.0 0.0 "
        <<camera.GetDepthScale()<<" 10\n";
    ofs_calib.close();
    return 0;
}