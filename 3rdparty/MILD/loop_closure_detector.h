/**
* This file is part of MILD.
*
* Copyright (C) Lei Han(lhanaf@connect.ust.hk) and Lu Fang(fanglu@sz.tsinghua.edu.cn)
* For more information see <https://github.com/lhanaf/MILD>
*
**/


#ifndef LOOP_CLOSURE_DETECTOR_H
#define LOOP_CLOSURE_DETECTOR_H


#include <stdio.h>
#include <stdlib.h>
#include <opencv/cv.h>
#include <opencv2/opencv.hpp>
#include <bitset>
#include <list>
#include <queue>
#include <array>
#include "mild.hpp"

namespace MILD
{

	struct feature_indicator
	{
		unsigned short image_index;
		unsigned short feature_index;
	};

	struct database_frame
	{
		cv::Mat descriptor;
	};

	typedef gstd::lightweight_vector<feature_indicator> mild_entry;

    class LCDCandidate{
    public:
        //EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		LCDCandidate() = default;
        LCDCandidate(float input_salient_score, int input_index) :salient_score(input_salient_score), index(input_index){}
        bool operator < (const LCDCandidate &m) const
        {
            return salient_score< m.salient_score;
        }
        bool operator > (const LCDCandidate &m)const 
        {
            return salient_score> m.salient_score;
        }
        float salient_score;
        int index;
    };
	class LoopClosureDetector
	{
	public:
		LoopClosureDetector(int feature_type = FEATURE_TYPE_ORB,
			int para_table_num = 16,
			int input_depth_level = 0,
			int input_distance_threshold = DEFAULT_HAMMING_DISTANCE_THRESHOLD,
			int input_max_num_per_entry = DEFAULT_MAX_UNIT_NUM_PER_ENTRY);
		~LoopClosureDetector();
		void displayParameters();
		int construct_database(cv::Mat descriptor);		// return the index of image
		int query_database(cv::Mat descriptor, std::vector<float> &score);
		int insert_and_query_database(cv::Mat descriptor, std::vector<float> &score);
        int calculate_hamming_distance_256bit(uint64_t * f1, uint64_t * f2);
		int count_feature_in_database();
		int statistics_num_distance_calculation;	// for statistic information
        void search_entry(uint64_t * f1, unsigned long search_entry_idx, std::vector<float> &score);
		std::vector<database_frame>			features_descriptor;
		std::vector<mild_entry>				features_buffer;
	private:

		int	descriptor_type;							// ORB feature or not
		unsigned int descriptor_length;				// feature descriptor length
		unsigned int depth_level;					// depth of MIH
		unsigned int bits_per_substring;			// substring length
		unsigned int hash_table_num;				// basic parameters of MIH, hash table num
		unsigned int entry_num_per_hash_table;		// entry num per hash table
		unsigned int buckets_num;					// total entry num
		unsigned int max_unit_num_per_entry;		// max num of features to store in each entry
		std::vector<float> lut_feature_similarity;	//feature_similarity look up table based on hamming distance
		float distance_threshold;					// pre-defined parameters for image similarity measurement
	};

}



#endif
