/**
* This file is part of MILD.
*
* Copyright (C) Lei Han(lhanaf@connect.ust.hk) and Lu Fang(fanglu@sz.tsinghua.edu.cn)
* For more information see <https://github.com/lhanaf/MILD>
*
**/


#ifndef BAYSIAN_FILTER_H
#define BAYSIAN_FILTER_H

#include <vector>
#include <eigen3/Eigen/Eigen>
#include <algorithm>
#include "mild.hpp"

namespace MILD
{

    class BayesianFilter
    {
    public:

        BayesianFilter(
                float input_probability_threshold = 0.6,
                int input_non_loop_closure_threshold = 4,
                float input_min_shared_score_threshold = 4,
                int input_min_distance = 1)
        {
            probability_threshold = input_probability_threshold;
            non_loop_closure_threshold = input_non_loop_closure_threshold;
            min_shared_score_threshold = input_min_shared_score_threshold;
            min_distance = input_min_distance;
        }

        // calcuate salient score from similarity score
        // input	:	similarity_score
        // output	:	salient_score, (similarity - std)/average, which reveals the probability of loop closure
        inline void calculateSalientScore(const std::vector<float> &similarity_score, std::vector<float> &salient_score)
        {

            int dataset_size = similarity_score.size();

            if (dataset_size == 0)
            {
                return;
            }
            salient_score.clear();
            salient_score = std::vector<float>(dataset_size);
            float average_similarity_score = 0;
            for (int i = 0; i < dataset_size; i++)
            {
                average_similarity_score += similarity_score[i];
            }

            average_similarity_score /= dataset_size;

            int history_loop = 0;
            for (history_loop = dataset_size - 1; history_loop >= 0; history_loop--)
            {
                if (similarity_score[history_loop] < average_similarity_score)
                {
                    break;
                }
            }
            if (history_loop <= 0)					// indicating all the frames are significiant
            {
                for (int i = 0; i < dataset_size; i++)
                {
                    salient_score[i] = 3;
                }
                return;
            }
            Eigen::VectorXf sim_score(history_loop);
            for (int i = 0; i < history_loop; i++)
            {
                sim_score[i] = similarity_score[i];
            }

            float mean_score = sim_score.mean();

            if (mean_score < 1e-8 || history_loop < 3)
            {
                for (int i = 0; i < dataset_size; i++)
                {
                    salient_score[i] = 1;
                }
                return;
            }

            float delta = (sim_score.rowwise() - sim_score.colwise().mean()).norm() / fmax(sqrt(sim_score.size() - 1), 1);

            for (int i = 0; i < dataset_size; i++)
            {
                float saliency = (similarity_score[i] - delta) / mean_score;
                salient_score[i] = saliency;
            }
            return;
        }

        // filter the input similarity score sequentially
        inline void filter(std::vector<float> similarity_score,
            Eigen::VectorXf &previous_visit_probability,
            std::vector<Eigen::VectorXf> &privious_visit_flag
            )
        {

            float trans_model[2][2] = { 0.95, 0.05, 0.05, 0.95 };
            // bayesian filter
            int dataset_size = similarity_score.size();
            if (dataset_size > min_distance)
            {
                Eigen::VectorXf sim_score(dataset_size - min_distance);
                for (int i = 0; i < dataset_size - min_distance; i++)
                {
                    sim_score[i] = similarity_score[i];
                }
                float mean_score = sim_score.mean();
                float delta = (sim_score.rowwise() - sim_score.colwise().mean()).norm() / fmax(sqrt(sim_score.size() - 1), 1);
                Eigen::VectorXf current_visit_probability(sim_score.size());
                Eigen::VectorXf current_visit_flag(sim_score.size());
                for (int i = 0; i < sim_score.size(); i++)
                {
                    float salient_score = (sim_score[i] - delta) / mean_score;
                    if (sim_score[i] < min_shared_score_threshold)
                    {
                        salient_score = 1;
                    }
                    float likelihood = salient_score < 1 ? 1 : salient_score;
                    int neighbor_left = fmax(i - 2, 0);
                    int neighbor_right = fmin((int)sim_score.size() - 1, i + 3);
                    float alpha = previous_visit_probability.segment(neighbor_left, neighbor_right - neighbor_left).maxCoeff();
                    float prob1 = likelihood * (trans_model[1][0]) * (1 - alpha) + likelihood * trans_model[1][1] * alpha;
                    float prob2 = non_loop_closure_threshold * (trans_model[0][0]) * (1 - alpha) + non_loop_closure_threshold * trans_model[0][1] * alpha;
                    current_visit_probability[i] = prob1 / (prob1 + prob2);
                    current_visit_flag[i] = current_visit_probability[i] > probability_threshold;
                }
                int previous_visit_idx = privious_visit_flag.size() - 1;
                if (privious_visit_flag.size() >= 4)
                {
                    int search_range = privious_visit_flag[previous_visit_idx].size();
                    for (int i = 0; i < search_range; i++)
                    {
                        if (privious_visit_flag[previous_visit_idx][i] > 0)
                        {
                            int start_loop_closure = fmax(i - 4, 0);
                            while (privious_visit_flag[previous_visit_idx][i] > 0 && i < search_range)
                            {
                                i++;
                            }
                            int end_loop_closure = fmin(i + 4, search_range - 3);
                            if (current_visit_flag.segment(start_loop_closure, end_loop_closure - start_loop_closure).maxCoeff() == 0)
                            {
                                int p2flag = privious_visit_flag[previous_visit_idx - 2].segment(start_loop_closure, end_loop_closure - start_loop_closure).maxCoeff();
                                int p1flag = privious_visit_flag[previous_visit_idx - 1].segment(start_loop_closure, end_loop_closure - start_loop_closure).maxCoeff();
                                if (p2flag + p1flag < 2)
                                {
                                    privious_visit_flag[previous_visit_idx - 2].segment(start_loop_closure, end_loop_closure - start_loop_closure).setZero();
                                    privious_visit_flag[previous_visit_idx - 1].segment(start_loop_closure, end_loop_closure - start_loop_closure).setZero();
                                    privious_visit_flag[previous_visit_idx].segment(start_loop_closure, end_loop_closure - start_loop_closure).setZero();
                                }
                            }
                        }

                    }
                }
                previous_visit_probability = current_visit_probability;
                privious_visit_flag.push_back(current_visit_flag);

            }
        }
    private:
        float probability_threshold;
        int non_loop_closure_threshold;
        float min_shared_score_threshold;
        int min_distance;

    };

}

#endif
