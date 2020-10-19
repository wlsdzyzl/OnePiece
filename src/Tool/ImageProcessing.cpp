#include "ImageProcessing.h"
namespace one_piece
{
namespace tool 
{
    void CreatePyramid(const cv::Mat &image, std::vector<cv::Mat> &results, int layer_number)
    {
        results.clear();
        for(int i = 0; i < layer_number;++i)
        {
            if(i == 0)
            results.push_back(image);
            else
            {
                cv::Mat tmp;
                cv::pyrDown( results[i-1], tmp, cv::Size( results[i-1].cols/2, results[i-1].rows/2 ));
                results.push_back(tmp);
            }
        }
    }
    void Convert2Gray(const cv::Mat &source, cv::Mat &target)
    {
        cv::cvtColor(source, target, CV_RGB2GRAY);
    }
    void SobelFiltering(const cv::Mat &source, cv::Mat &target, char type)
    {
        int x = 0,y = 0;
        if(type == 'x') x = 1;
        else if(type == 'y') y=1;
        cv::Sobel( source, target, CV_32F, x ,y);
        //cv::Scharr(source, target, CV_32F, x ,y);
        //PrintMat(target);
        return ;
    }
    void SobelFiltering(const std::vector<cv::Mat> &sources,std::vector<cv::Mat> &targets, char type )
    {
        targets.resize(sources.size());
        for(size_t i = 0;i!=sources.size(); ++i)
        {
            SobelFiltering(sources[i], targets[i], type);
        }
    }
    void GaussianFiltering(const cv::Mat &image, cv::Mat &result, int k_size)
    {
        cv::GaussianBlur(image, result, cv::Size(k_size,k_size), 0);
    }
    void PrintMat(const cv::Mat & mat)
    {
        for(int i = 0;i!=mat.rows;++i)
        {
            for(int j = 0;j!=mat.cols;++j)
            std::cout<<mat.at<float>(i,j)<<" ";
            std::cout<<std::endl;
        }
    }
    void LinearTransform( cv::Mat &source,float scale, float offset)
    {
        for(int i = 0;i!=source.rows;++i)
        {
            for(int j = 0;j!=source.cols;++j)
            source.at<float>(i,j) = source.at<float>(i,j) * scale + offset;
        }
    }
    void BilateralFilter(const cv::Mat &source, cv::Mat &target, int range)
    {
        cv::bilateralFilter(source, target, range, 0.03,4.5);
    }
    void ConvertDepthTo32F(const cv::Mat &depth, cv::Mat &refined_depth, float depth_scale)
    {
        refined_depth.create(depth.rows, depth.cols, CV_32FC1);
        //std::cout <<depth.depth()<<std::endl;
        if(depth.depth() == CV_32FC1)
        {
            for(int i = 0; i < depth.rows * depth.cols; ++i)
            refined_depth.at<float>(i) = depth.at<float>(i);
        }
        else if(depth.depth() == CV_16UC1)
        {
            for(int i = 0; i < depth.rows * depth.cols; ++i)
            {
                refined_depth.at<float>(i) = depth.at<unsigned short>(i) /depth_scale;
                if(refined_depth.at<float>(i) < 0)
                refined_depth.at<float>(i) = 0;
            }
        }
        else
        {
            std::cout <<RED<< "[ImageProcessing]::[ERROR]::Unknown depth image type: "<<depth.depth()<<RESET<<std::endl;
            exit(1);
        }
    }
}
}