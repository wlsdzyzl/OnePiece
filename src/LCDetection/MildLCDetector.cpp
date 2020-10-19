#include "MildLCDetector.h"
#include <functional>
namespace one_piece
{
namespace lcdetection
{
    void MildLCDetector::SelectCandidates(const geometry::RGBDFrame &frame, std::vector<int> &candidates)
    {
        candidates.clear();
        if(!frame.IsPreprocessedSparse())
        {
            std::cout<<RED<<"[ERROR]::[MildLCD]::ORB feature is necessary. Make sure the frame has been preprocessed."<<RESET<<std::endl;
            return;
        }
        MILD::BayesianFilter spatial_filter;
        std::vector<float > similarity_score;
        lcdetector.query_database(frame.descriptor, similarity_score);

        // select candidates
        std::vector<float> salient_score;
        std::vector<MILD::LCDCandidate> tmp_candidates;
        spatial_filter.calculateSalientScore(similarity_score, salient_score);
        // for(int i = 0; i < salient_score.size(); ++i)
        // std::cout<<salient_score[i]<<std::endl;
        int database_size = GetSize();
        for (int k = 0; k != database_size; ++k)
        {
            if (salient_score[k] > salient_score_threshold )
            {
                MILD::LCDCandidate candidate(salient_score[k], k);
                tmp_candidates.push_back(candidate);
            }
        }

        std::sort(tmp_candidates.begin(), tmp_candidates.end(),std::greater<MILD::LCDCandidate>());
        for (size_t k = 0; k < tmp_candidates.size() && k < max_candidate_num; ++k)
        {
            candidates.push_back(tmp_candidates[k].index);
        }
    }
    void MildLCDetector::Insert(const geometry::RGBDFrame &frame)
    {
        lcdetector.construct_database(frame.descriptor);
    }
}
}