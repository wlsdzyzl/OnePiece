#include "Frustum.h"

namespace one_piece
{
namespace integration
{
        void Frustum::ComputeFromCamera(const camera::PinholeCamera &camera, const geometry::TransformationMatrix &T,
            float far_dist, float near_dist)
        {
            float fx = camera.GetFx();
            float fy = camera.GetFy();
            // float cx = camera.GetCx();
            float cy = camera.GetCy();
            float height = camera.GetHeight();
            float width = camera.GetWidth();
            geometry::Matrix3 R = T.block<3,3>(0,0);
            geometry::Point3 right = R.col(0);
            geometry::Point3 up = -R.col(1);
            geometry::Point3 d = R.col(2);
            geometry::Point3 t = T.block<3,1>(0,3);
            
            float aspect = (fy * width) / (fx * height);
            float fov = atan2(cy, fy) + atan2(height - cy, fy);
            ComputeFromVectors(d, t, right, up, far_dist, near_dist, fov, aspect);     
        }
        void Frustum::ComputeFromVectors(const geometry::Point3 &forward, const geometry::Point3 &pos, const geometry::Point3 & right, 
            const geometry::Point3 &up, float far_dist, float near_dist, float fov, float aspect )
        {
            float angle_tangent = tan(fov / 2);
            float height_far = angle_tangent * far_dist;
            float width_far = height_far * aspect;
            float height_near = angle_tangent * near_dist;
            float width_near = height_near * aspect;
            geometry::Point3 far_center = pos + forward * far_dist;
            geometry::Point3 far_top_left = far_center + (up * height_far) - (right * width_far);
            geometry::Point3 far_top_right = far_center + (up * height_far) + (right * width_far);
            geometry::Point3 far_bottom_left = far_center - (up * height_far) - (right * width_far);
            geometry::Point3 far_bottom_right = far_center - (up * height_far) + (right * width_far);

            geometry::Point3 near_center = pos + forward * near_dist;
            geometry::Point3 near_top_left = near_center + (up * height_near) - (right * width_near);
            geometry::Point3 near_top_right = near_center + (up * height_near) + (right * width_near);
            geometry::Point3 near_bottom_left = near_center - (up * height_near) - (right * width_near);
            geometry::Point3 near_bottom_right = near_center - (up * height_near) + (right * width_near);

            near_plane = geometry::GetPlane(near_bottom_left, near_top_left, near_bottom_right);
            far_plane = geometry::GetPlane(far_top_right, far_top_left, far_bottom_right);
            left_plane = geometry::GetPlane(far_top_left, near_top_left, far_bottom_left);
            right_plane = geometry::GetPlane(near_top_right, far_top_right, near_bottom_right);
            top_plane = geometry::GetPlane(near_top_left, far_top_left, near_top_right);
            bottom_plane = geometry::GetPlane(near_bottom_right, far_bottom_left, near_bottom_left);

            corners[0] = far_top_left;
            corners[1] = far_top_right;
            corners[2] = far_bottom_left;
            corners[3] = far_bottom_right;
            corners[4] = near_bottom_right;
            corners[5] = near_top_left;
            corners[6] = near_top_right;
            corners[7] = near_bottom_left;

            // Far face lines.
            lines[0].first = corners[0];
            lines[0].second = corners[1];
            lines[1].first = corners[3];
            lines[1].second = corners[2];
            lines[2].first = corners[1];
            lines[2].second = corners[3];
            lines[3].first = corners[2];
            lines[3].second = corners[0];

            // Near face lines.
            lines[4].first = corners[4];
            lines[4].second = corners[7];
            lines[5].first = corners[6];
            lines[5].second = corners[5];
            lines[6].first = corners[5];
            lines[6].second = corners[7];
            lines[7].first = corners[6];
            lines[7].second = corners[4];

            // Connecting lines.
            lines[8].first = corners[0];
            lines[8].second = corners[5];
            lines[9].first = corners[1];
            lines[9].second = corners[6];
            lines[10].first = corners[2];
            lines[10].second = corners[7];
            lines[11].first = corners[3];
            lines[11].second = corners[4];            
        }
}
}