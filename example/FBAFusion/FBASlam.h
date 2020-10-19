#ifndef FBASLAM_H
#define FBASLAM_H

#include "Tool/KeyframeBasedSlam.h"

namespace one_piece
{
    class FBASlam: public tool::KeyframeBasedSlam
    {
        
        public:
        FBASlam() = default;
        FBASlam(const camera::PinholeCamera &_camera): tool::KeyframeBasedSlam(_camera){}
        void UpdateFrame(const geometry::RGBDFrame &frame);
        void Optimize();
        std::vector<optimization::Correspondence> global_correspondences;
    };
}
#endif