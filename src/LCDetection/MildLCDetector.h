#ifndef MILD_LCDETECTOR_H
#define MILD_LCDETECTOR_H

#include "loop_closure_detector.h"
#include "BayesianFilter.hpp"
#include "sparse_match.h"
#include "Geometry/RGBDFrame.h"

namespace fucking_cool
{
namespace lcdetection
{
    //a warped class of mild lcd detection

    class MildLCDetector
    {   
        public:
        MildLCDetector() = default;
        void SelectCandidates(const geometry::RGBDFrame &frame, std::vector<int> &candidates);

        int GetSize(){return lcdetector.features_descriptor.size();}
        void Insert(const geometry::RGBDFrame &frame);
        MILD::LoopClosureDetector lcdetector;
        
        float salient_score_threshold = 1.5;
        int max_candidate_num = 7;
    };
}
}
#endif