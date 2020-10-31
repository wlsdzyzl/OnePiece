// this file is to convert scannet-style sequence into tum-style sequence.
#include "Geometry/Geometry.h"
#include "Tool/TickTock.h"
#include "Tool/ImageProcessing.h"
#include "Tool/IO.h"
#include "Tool/CppExtension.h"
#include <random>

using namespace one_piece;

int main(int argc, char ** argv)
{
    // this code is to generate a model from scannet

    if(argc < 3)
    {
        std::cout<<"Usage: ScannetToTum [input_folder] [output_folder] "<<std::endl;
        return 0;
    }
    
    //pinhole camera
    camera::PinholeCamera depth_camera;
    camera::PinholeCamera color_camera;
    std::vector<std::string> rgb_files, depth_files;
    std::vector<geometry::TransformationMatrix> poses;
    std::string input_folder = argv[1];
    std::string output_folder = argv[2];
    tool::MakeDir(output_folder);
    tool::MakeDir(output_folder+"/rgb");
    tool::MakeDir(output_folder+"/depth");


    std::ofstream ofs_associate(output_folder+"/associations.txt");

    tool::ReadImageSequenceFromScannet(argv[1], rgb_files, depth_files, 
        color_camera, depth_camera);
    
    std::cout<<"Read "<<rgb_files.size() <<" frames..."<<std::endl;
    std::cout<<"Depth Intrinsics:\n"<<depth_camera.ToCameraMatrix()<<std::endl;
    std::cout<<"Color Intrinsics:\n"<<color_camera.ToCameraMatrix()<<std::endl;

    

    for(size_t i = 0; i != rgb_files.size(); ++i) 
    { 
        int frame_id = i;

        std::cout <<"Processing on "<<i<<"th image"<<std::endl;
        std::cout<<depth_files[frame_id]<<std::endl;
        cv::Mat rgb = cv::imread(rgb_files[frame_id]);
        cv::Mat depth = cv::imread(depth_files[frame_id],-1);
        //std::cout<<depth.depth()<<" "<<CV_16UC1<<std::endl; 
        cv::Mat aligned_rgb = tool::AlignColorToDepth(rgb, depth, color_camera, depth_camera);

        cv::imwrite(output_folder+"/rgb/"+std::to_string(i)+".png", aligned_rgb);
        cv::imwrite(output_folder+"/depth/"+std::to_string(i)+".png", depth);
        ofs_associate<<i <<" depth/"<<i<<".png "<<i<<" rgb/"<<i<<".png"<<std::endl;
    }

    ofs_associate.close();

    std::ofstream ofs_calib(output_folder+"/calib.txt");
    ofs_calib<<depth_camera.GetWidth()<<" "<<depth_camera.GetHeight()<<" "<<depth_camera.GetFx()
        <<" "<<depth_camera.GetFy()<<" "<<depth_camera.GetCx()<<" "<<depth_camera.GetCy()<<" 0.0 0.0 0.0 0.0 0.0 "
        <<depth_camera.GetDepthScale()<<" 10\n";
    ofs_calib.close();
    return 0;
}

