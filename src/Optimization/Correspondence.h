#ifndef CORRESPONDENCE_H
#define CORRESPONDENCE_H
#include "Geometry/Geometry.h"
#include "Camera/Camera.h"
namespace fucking_cool
{
namespace optimization
{
    //A basic structure of optimization, you need to have: poses of each object, correspondences between objects.
    class Correspondence
    {
        public:
        Correspondence() = default;
        Correspondence(int sid, int tid, const geometry::PointCorrespondenceSet &cs 
            = geometry::PointCorrespondenceSet())
        {
            source_id = sid;
            target_id = tid;
            correspondence_set = cs;
        }

        void CalculateAverageDisparity(const camera::PinholeCamera &camera)
        {
            if(correspondence_set.size() <= 0) return;

            auto K = camera.ToCameraMatrix();
            float sum_disparity = 0;
            
            //disparity means the distance of matching points on 2D
            for(size_t i = 0; i < correspondence_set.size(); ++i)
            {
                geometry::Point3 ref_p = K * correspondence_set[i].first;
                geometry::Point3 new_p = K * correspondence_set[i].second;
                geometry::Point2 ref_p_2d = (ref_p/ ref_p(2)).head<2>();
                geometry::Point2 new_p_2d = (new_p/ new_p(2)).head<2>();
                sum_disparity += (ref_p_2d - new_p_2d).norm(); 
            }
            average_disparity = sum_disparity / correspondence_set.size();
            //std::cout <<average_disparity<<std::endl;
        }
        float ComputeReprojectionError3D(const geometry::SE3List &camera_poses) const
        {
            float sum_error = 0.0;
            for(size_t j = 0; j != correspondence_set.size(); ++j)
            {
                sum_error += 
                    (geometry::TransformPoint(camera_poses[source_id],
                        correspondence_set[j].first) - 
                    geometry::TransformPoint(camera_poses[target_id],
                        correspondence_set[j].second)).squaredNorm();
            }        
            return sqrt(sum_error / correspondence_set.size());
        }
        float ComputeReprojectionError3D(const geometry::SE3 &camera_pose) const
        {
            //camera pose from source to target.
            float sum_error = 0.0;
            for(size_t j = 0; j != correspondence_set.size(); ++j)
            {
                sum_error += 
                    (geometry::TransformPoint(camera_pose,
                        correspondence_set[j].first) - 
                        correspondence_set[j].second).squaredNorm();
            }        
            return sqrt(sum_error / correspondence_set.size());
        }
        int source_id = -1;
        int target_id = -1;
        float average_disparity = 1e6;
        geometry::PointCorrespondenceSet correspondence_set;
    };


}
}
#endif