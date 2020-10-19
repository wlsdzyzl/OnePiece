#ifndef INTEGRATOR_H
#define INTEGRATOR_H
#include "Geometry/Geometry.h"
#include "Camera/Camera.h"
#include "VoxelCube.h"
namespace one_piece
{
namespace integration
{
    class Integrator
    {
        public:
        Integrator()=default;

        void IntegrateImage(const cv::Mat &depth, const cv::Mat &rgb, 
            const geometry::TransformationMatrix & pose, const camera::PinholeCamera &camera, VoxelCube &voxel_cube, const CubePara &c_para);
        float GetSDF(const geometry::Point3 &point, const camera::PinholeCamera &camera, 
            const geometry::TransformationMatrix &pose, const cv::Mat &depth);       
        void SetTruncation(float _trunc)
        {
            truncation = _trunc;
        } 
        float truncation = 0.1;   
        //For simplifity, we use constant weight here.
        float Weight = 1.0 / (2 * truncation);
    };
    
}
}
#endif