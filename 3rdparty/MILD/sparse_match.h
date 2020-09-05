/**
* This file is part of MILD.
*
* Copyright (C) Lei Han(lhanaf@connect.ust.hk) and Lu Fang(fanglu@sz.tsinghua.edu.cn)
* For more information see <https://github.com/lhanaf/MILD>
*
**/

#ifndef SPARSE_MATCH_H
#define SPARSE_MATCH_H


#include <stdio.h>
#include <stdlib.h>
#include <opencv/cv.h>
#include <opencv2/opencv.hpp>
#include "mild.hpp"
namespace MILD
{
	typedef gstd::lightweight_vector<unsigned short> sparse_match_entry;

	// fast frame match based on binary features, using MILD
	class SparseMatcher
	{
	public:

		SparseMatcher(int feature_type, int input_hash_table_num, int input_depth_level, float input_distance_threshold);
		~SparseMatcher();
		void displayParameters();
		void displayStatistics();

		void train(cv::Mat d1);				
		void search(cv::Mat desc, std::vector<cv::DMatch> &matches);
		// special case, when hash table num equal to 32
		void train_8(cv::Mat d1);
		void search_8(cv::Mat desc, std::vector<cv::DMatch> &matches,int hamming_distance);

		// search feature within a range
		void search_8_with_range(cv::Mat desc, std::vector<cv::DMatch> &matches,
			const std::vector<cv::KeyPoint> &train_features,
			const std::vector<cv::KeyPoint> &query_features,
			float range, int hamming_distance);

		void match(cv::Mat d1, cv::Mat d2, std::vector<cv::DMatch> &matches);
		void BFMatch(cv::Mat d1, cv::Mat d2, std::vector<cv::DMatch> &matches);
		int calculate_hamming_distance_256bit(uint64_t * f1, uint64_t * f2);
		void search_entry(uint64_t * f1, unsigned long search_entry_idx, unsigned short &min_distance, unsigned short &corr);
		uint64_t *features_descriptor;
		std::vector<sparse_match_entry>				features_buffer;
		int statistics_num_distance_calculation;	// for statistic information
	private:

		int	descriptor_type;						// ORB feature or not
		unsigned int descriptor_length;				// feature descriptor length

		unsigned int bits_per_substring;			// substring length
		unsigned int hash_table_num;				// basic parameters of MIH, hash table num
		unsigned int depth_level;					// depth of MIH

		unsigned int entry_num_per_hash_table;		// entry num per hash table
		unsigned int buckets_num;					// total entry num
		int bits_per_chunk;
		float distance_threshold;
	};

}

#endif
