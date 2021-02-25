#ifndef MATRIX_IMAGE_H
#define MATRIX_IMAGE_H
#include "Algorithm/DCEL.h"
#include "Algorithm/PatchDetection.h"
#include "Algorithm/Clustering.h"
#include "ColorTab.h"
namespace one_piece
{
namespace visualization
{

    cv::Mat MatrixImage(const geometry::MatrixX &m);
    cv::Mat DepthRainbow(const cv::Mat &depth);
    cv::Mat PatchImage(const std::vector<algorithm::LinePatch> &patches);
    cv::Mat ClusterImage(const std::vector<algorithm::Cluster<2>> &clusters);
    cv::Mat DCELImage(const algorithm::DCEL & dcel);
}
}
#endif