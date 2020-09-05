#ifndef IMAGE_PROCESSING_H
#define IMAGE_PROCESSING_H
#include "ConsoleColor.h"
#include <opencv2/opencv.hpp>


#define SOBLE_SCALE 0.125
namespace fucking_cool
{
namespace tool
{
    void CreatePyramid(const cv::Mat &image, std::vector<cv::Mat> &result, int layer_number);
    void Convert2Gray(const cv::Mat &source, cv::Mat &target);
    void SobelFiltering(const cv::Mat &source, cv::Mat &target, char type = 'x');
    void SobelFiltering(const std::vector<cv::Mat> &sources,std::vector<cv::Mat> &targets, char type = 'x');
    void GaussianFiltering(const cv::Mat &image, cv::Mat &result, int k_size = 3);
    void PrintMat(const cv::Mat & mat);
    void LinearTransform( cv::Mat &source, float scale, float offset);
    void BilateralFilter(const cv::Mat &source, cv::Mat &target, int range = 7);
    void ConvertDepthTo32F(const cv::Mat &depth, cv::Mat &refined_depth, float depth_scale);
};
}
#endif 