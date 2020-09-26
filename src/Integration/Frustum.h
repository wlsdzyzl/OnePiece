#ifndef FRUSTUM_H
#define FRUSTUM_H
#include "Geometry/Geometry.h"
#include "Camera/Camera.h"
#include "Geometry/PointCloud.h"
namespace fucking_cool
{
namespace integration
{
    class Frustum
    {
        
        public:

        void ComputeFromCamera(const camera::PinholeCamera &camera, const geometry::TransformationMatrix &T,
            float far_dist, float near_dist);
        void ComputeFromVectors(const geometry::Point3 &forward, const geometry::Point3 &position, const geometry::Point3 &right, 
            const geometry::Point3 &up, float far_dist, float near_dist, float fov, float aspect);
        geometry::Plane GetFarPlane() const
        {
            return far_plane;
        }
        geometry::Plane GetNearPlane() const
        {
            return near_plane;
        }
        geometry::Plane GetTopPlane() const
        {
            return top_plane;
        }
        geometry::Plane GetBottomPlane() const
        {
            return bottom_plane;
        }
        geometry::Plane GetLeftPlane() const
        {
            return left_plane;
        }
        geometry::Plane GetRightPlane() const
        {
            return right_plane;
        }
        std::shared_ptr<geometry::PointCloud> GetPointCloud() const
        {
            geometry::PointCloud pcd;
            int point_num = 1000;
            for(int i = 0; i!=12; ++i)
            {
                geometry::Point3 diff = lines[i].first - lines[i].second;
                diff.normalize();

                geometry::Point3 point;
                float t = 0;
                float step = 0;
                for(int j = 0;j!= 3; ++j)
                if(std::fabs(diff(j)) >= 0.000001)
                {
                    step = (lines[i].second(j) - lines[i].first(j))/diff(j);
                    break;
                }
                step /= point_num;
                for(int j = 0; j!= point_num; ++j)
                {
                    point(0) = lines[i].first(0) + diff(0)*t;
                    point(1) = lines[i].first(1) + diff(1)*t;
                    point(2) = lines[i].first(2) + diff(2)*t;
                    t+= step;
                    pcd.points.push_back(point);
                    pcd.colors.push_back(diff);
                }
            }
            return std::make_shared<geometry::PointCloud>(pcd);
        }
        bool ContainPoint(const geometry::Point3 &p)
        {
            float distance; 
            distance = top_plane.head<3>().dot(p) + top_plane(3);
            //std::cout << distance << std::endl;
            if(distance < 0) return false;
            if(distance == 0) return true;

            distance = left_plane.head<3>().dot(p) + left_plane(3);
            if(distance < 0) return false;
            if(distance == 0) return true;

            distance = right_plane.head<3>().dot(p) + right_plane(3);
            if(distance < 0) return false;
            if(distance == 0) return true;

            distance = bottom_plane.head<3>().dot(p) + bottom_plane(3);
            if(distance < 0) return false;
            if(distance == 0) return true;

            distance = near_plane.head<3>().dot(p) + near_plane(3);
            if(distance < 0) return false;
            if(distance == 0) return true;

            distance = far_plane.head<3>().dot(p) + far_plane(3);
            if(distance < 0) return false;
            if(distance == 0) return true;

            return true;
        }
        geometry::Point3 corners[8];
        std::pair<geometry::Point3, geometry::Point3> lines[12];
        geometry::Plane top_plane;
        geometry::Plane left_plane;
        geometry::Plane right_plane;
        geometry::Plane bottom_plane;
        geometry::Plane near_plane;
        geometry::Plane far_plane;
    };
    
}
}
#endif