#include "Algorithm/Clustering.h"
#include "Visualization/DrawImage.h"
using namespace fucking_cool;
int main()
{
    int point_number = 200;
    const int MAX_CLUSTERS = 7;
    cv::Scalar color_tab[] =   
    {
        cv::Scalar(0, 0, 255),
        cv::Scalar(0,255,0),
        cv::Scalar(255,255,0),
        cv::Scalar(255,0,255),
        cv::Scalar(0,255,255),
        cv::Scalar(255,255,255),
        cv::Scalar(255,0,0)
    };
    geometry::Point2List points;
    for(int i = 0;i != point_number; ++i)
    {
        points.push_back(geometry::Point2::Random() + geometry::Point2(-5, 5));
        //std::cout<<points[i]<<std::endl;
    }

    for(int i = 0;i != point_number; ++i)
    {
        points.push_back(geometry::Point2::Random() + geometry::Point2(5, 5));
        //std::cout<<points[i]<<std::endl;
    }
    for(int i = 0;i != point_number; ++i)
    {
        points.push_back(geometry::Point2::Random()  + geometry::Point2(-5, -5));
        //std::cout<<points[i]<<std::endl;
    }
    for(int i = 0;i != point_number; ++i)
    {
        points.push_back(geometry::Point2::Random()  + geometry::Point2(5, -5));
        //std::cout<<points[i]<<std::endl;
    }
    for(int i = 0;i != point_number; ++i)
    {
        points.push_back(geometry::Point2::Random()   + geometry::Point2(0, 0));
        //std::cout<<points[i]<<std::endl;
    }

    std::vector<algorithm::Cluster<2>> clusters;
    //algorithm::KMeansClustering<2>(points,clusters,5);
    //algorithm::MeansShiftClustering<2>(points,clusters,0.8);
    algorithm::KMedoidsClustering<2>(points,clusters,5);
    cv::Mat img;
    img = visualization::ClusterImage(clusters);
    cv::imshow("clusters", img);
    cv::waitKey(0);
    return 0;
}