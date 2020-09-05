#ifndef PATCH_DETECTION_H
#define PATCH_DETECTION_H
#include "Geometry/Geometry.h"
namespace fucking_cool
{
namespace algorithm
{
    
    template <int T > class Patch
    {

        public:
        Patch()=default;
        Patch(const Eigen::Matrix<geometry::scalar,T,1> & c)
        {
            //representation
            rep = c;
        }
        Eigen::Matrix<geometry::scalar,T,1> rep;
        std::vector<Eigen::Matrix<geometry::scalar,T - 1,1>, Eigen::aligned_allocator<Eigen::Matrix<geometry::scalar,T - 1,1>> > items;
        std::vector<int > indexs;
    };  

    typedef Patch<3> LinePatch;
    typedef Patch<4> PlanePatch;


    int ChooseSeed(const std::set<int> &un_visited, const std::vector<double> &residuals);
    void LineDetection(const geometry::Point2List &points, 
        std::vector<LinePatch> &results);
    void PlaneDetection(const geometry::Point3List &points, 
        std::vector<PlanePatch> &Patches);
    bool IsInlier(const geometry::Point3 &point, const geometry::Vector4 &tangnet, 
        const geometry::Vector4 &plane, float radius);
    bool IsInlier(const geometry::Vector2 &point, const geometry::Point3 &tangnet, 
        const geometry::Point3 &plane, float radius);
}
}
#endif