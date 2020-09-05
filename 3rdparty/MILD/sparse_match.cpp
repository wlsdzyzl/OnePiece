/**
* This file is part of MILD.
*
* Copyright (C) Lei Han(lhanaf@connect.ust.hk) and Lu Fang(fanglu@sz.tsinghua.edu.cn)
* For more information see <https://github.com/lhanaf/MILD>
*
**/

#include "sparse_match.h"
#include "mild.hpp"
#include <fstream>
#include <immintrin.h>
#include <list>
#include <bitset>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <malloc.h>



using namespace std;
using namespace cv;

namespace MILD
{

	SparseMatcher::~SparseMatcher()
	{
	}
	//  default parameter settings
	// 256bit descriptor, 64bit per unit, 12bit per chunk
	// 5chunk in a unit, 4 unit in a descriptor
	// 20chunks total
	// the hash size is 4096

	SparseMatcher::SparseMatcher(int feature_type, int input_hash_table_num, int input_depth_level, float input_distance_threshold)
	{
		switch (feature_type)
		{
			case FEATURE_TYPE_ORB:
			{
									 descriptor_length = ORB_DESCRIPTOR_LEN;
									 break;
			}
			case FEATURE_TYPE_BRISK:
			{
									   descriptor_length = BRISK_DESCRIPTOR_LEN;
									   break;
			}
			default:
			{
					   cout << "unknown descriptor" << endl;
					   return;
			}
		}
		descriptor_type = feature_type;
		depth_level = input_depth_level;
		bits_per_substring = (int)(descriptor_length / input_hash_table_num);
		if (bits_per_substring > sizeof(size_t)* 8)
		{
			cout << "substring too large !, invalied" << endl;
			return;
		}
		hash_table_num = input_hash_table_num;
		entry_num_per_hash_table = pow(float(2), float(bits_per_substring));
		buckets_num = entry_num_per_hash_table * hash_table_num;
		distance_threshold = input_distance_threshold;

		features_buffer = std::vector<sparse_match_entry>(entry_num_per_hash_table * hash_table_num);
		for (int i = 0; i < entry_num_per_hash_table * hash_table_num; i++)
		{
			features_buffer[i].clear();
		}
		statistics_num_distance_calculation = 0;
	}
	void SparseMatcher::displayParameters()
	{

		cout << "parameters: " << endl
			<< "unit length :	" << descriptor_length << endl
			<< "chunk_num_per_unit :	" << depth_level << endl
			<< "bits_per_substring :	" << bits_per_substring << endl
			<< "hash_table_num :	" << hash_table_num << endl
			<< "entry_num_per_hash_table :	" << entry_num_per_hash_table << endl
			<< "buckets_num :	" << buckets_num << endl;
	}
	void SparseMatcher::displayStatistics()
	{
		cout << "num of distance calculation : " << statistics_num_distance_calculation << endl;
	}

	int SparseMatcher::calculate_hamming_distance_256bit(uint64_t * f1, uint64_t * f2)
	{
		int hamming_distance = (__builtin_popcountll(*(f1) ^ *(f2)) +
				__builtin_popcountll(*(f1 + 1) ^ *(f2 + 1)) +
				__builtin_popcountll(*(f1 + 2) ^ *(f2 + 2)) +
				__builtin_popcountll(*(f1 + 3) ^ *(f2 + 3)));
#if DEBUG_MODE_MILD
		statistics_num_distance_calculation++;
#endif 
		return hamming_distance;
	}
	void SparseMatcher::BFMatch(cv::Mat d2, cv::Mat d1, std::vector<cv::DMatch> &matches)
	{

		int feature_f1_num = d1.rows;
		int feature_f2_num = d2.rows;
		cout << "f1 " << feature_f1_num << endl << "f2 " << feature_f2_num << endl;
		unsigned short * delta_distribution = new unsigned short[feature_f1_num * feature_f2_num];
		uint64_t current_descriptor[4];
		for (int f1 = 0; f1 < feature_f1_num; f1++)
		{
			uint64_t *feature1_ptr = (d1.ptr<uint64_t>(f1));
			int best_corr_fid = 0;
			int min_distance = 256;
			for (int f2 = 0; f2 < feature_f2_num; f2++)
			{
				int hamming_distance = calculate_hamming_distance_256bit(feature1_ptr, d2.ptr<uint64_t>(f2));
				delta_distribution[f1 * feature_f2_num + f2] = hamming_distance;
				if (hamming_distance < min_distance)
				{
					min_distance = hamming_distance;
					best_corr_fid = f2;
				}
			}

			if (min_distance <= distance_threshold)
			{
				DMatch m;
				m.queryIdx = f1;
				m.trainIdx = best_corr_fid;
				m.distance = min_distance;
				matches.push_back(m);
			}
		}

		FILE * fp = fopen("data_file.bin", "wb+");
		fwrite(delta_distribution, sizeof(unsigned short), feature_f1_num * feature_f2_num, fp);
		fclose(fp);
	}


	void SparseMatcher::train(cv::Mat desc)
	{
		features_descriptor = (uint64_t *)desc.data;
		int feature_num = desc.rows;
		if (descriptor_type == FEATURE_TYPE_ORB)
		{
			int descriptor_length = desc.cols * 8;
			if (descriptor_length != descriptor_length)
			{
				cout << "error ! feature descriptor length doesn't match" << endl;
			}
		}
		std::vector<unsigned long> hash_entry_index = std::vector<unsigned long>(hash_table_num);
		for (unsigned short feature_idx = 0; feature_idx < feature_num; feature_idx++)
		{
			unsigned int *data = desc.ptr<unsigned int>(feature_idx);
			multi_index_hashing(hash_entry_index, data, hash_table_num, bits_per_substring);
			for (int hash_table_id = 0; hash_table_id < hash_table_num; hash_table_id++)
			{
				int entry_pos = hash_table_id*entry_num_per_hash_table + hash_entry_index[hash_table_id];
				features_buffer[entry_pos].push_back(feature_idx);
			}
		}
	}

	void SparseMatcher::search_entry(uint64_t * f1, unsigned long search_entry_idx, unsigned short &min_distance, unsigned short &corr)
	{

	}
	// d1: input feature
	// d2: output feature
	void SparseMatcher::search(cv::Mat desc, std::vector<cv::DMatch> &matches)
	{
		int feature_num = desc.rows;
		matches.clear();
		matches.reserve(feature_num);
		std::vector<unsigned long> hash_entry_index = std::vector<unsigned long>(hash_table_num);
		for (unsigned short feature_idx = 0; feature_idx < feature_num; feature_idx++)
		{
			unsigned int *data = desc.ptr<unsigned int>(feature_idx);
			unsigned short min_distance = 256;
			unsigned short best_corr_fid = 0;
			uint64_t * f1 = desc.ptr<uint64_t>(feature_idx);
			multi_index_hashing(hash_entry_index, data, hash_table_num, bits_per_substring);
			for (int hash_table_id = 0; hash_table_id < hash_table_num; hash_table_id++)
			{
				unsigned long entry_idx = hash_table_id*entry_num_per_hash_table + hash_entry_index[hash_table_id];
				int conflict_num = features_buffer[entry_idx].size();
				for (int i = 0; i < conflict_num; i++)
				{
					unsigned int feature_index = features_buffer[entry_idx][i];
					uint64_t * f2 = features_descriptor + feature_index * 4;
					int hamming_distance = calculate_hamming_distance_256bit(f1, f2);
					if (hamming_distance < min_distance)
					{
						min_distance = hamming_distance;
						best_corr_fid = feature_index;
					}
				}
				// may not be the most efficient implementation
				// to be refine generate_neighbor_candidates
				if (depth_level > 0)
				{
					std::vector<unsigned long> neighbor_entry_idx;
					generate_neighbor_candidates(depth_level, entry_idx, neighbor_entry_idx, bits_per_substring);
					for (int iter = 0; iter < neighbor_entry_idx.size(); iter++)
					{
						search_entry(f1, neighbor_entry_idx[iter], min_distance, best_corr_fid);
					}
				}
			}

				DMatch m;
				m.queryIdx = feature_idx;
				m.trainIdx = best_corr_fid;
				m.distance = min_distance;
				matches.push_back(m);


		}
		
	}
	void SparseMatcher::train_8(cv::Mat desc)
	{
		features_descriptor = (uint64_t *)desc.data;
		int feature_num = desc.rows;
		if (descriptor_type == FEATURE_TYPE_ORB)
		{
			int descriptor_length = desc.cols * 8;
			if (descriptor_length != descriptor_length)
			{
				cout << "error ! feature descriptor length doesn't match" << endl;
			}
		}
		for (unsigned short feature_idx = 0; feature_idx < feature_num; feature_idx++)
		{
			unsigned char *data = (unsigned char *)features_descriptor + 32 * feature_idx;
			for (int hash_table_id = 0; hash_table_id < hash_table_num; hash_table_id++)
			{
				int entry_pos = hash_table_id*entry_num_per_hash_table + data[hash_table_id];
				features_buffer[entry_pos].push_back(feature_idx);
			}
		}
	}
#if 1
	void SparseMatcher::search_8(cv::Mat desc, std::vector<cv::DMatch> &matches, int hamming_threshold)
	{
		int feature_num = desc.rows;
		matches.clear();
		matches.reserve(feature_num);
		for (unsigned short feature_idx = 0; feature_idx < feature_num; feature_idx++)
		{
			unsigned char *data = (unsigned char *)desc.data + feature_idx * 32;
			uint64_t * f1 = (uint64_t *)desc.data + feature_idx * 4;
			unsigned short min_distance = 256;
			unsigned short best_corr_fid = 0;
			for (int hash_table_id = 0; hash_table_id < hash_table_num; hash_table_id++)
			{
				unsigned long entry_idx = hash_table_id*entry_num_per_hash_table + data[hash_table_id];
				int conflict_num = features_buffer[entry_idx].size();
				for (int i = 0; i < conflict_num; i++)
				{
					unsigned int feature_index = features_buffer[entry_idx][i];
					uint64_t * f2 = features_descriptor + feature_index * 4;
					int hamming_distance = calculate_hamming_distance_256bit(f1, f2);
					if (hamming_distance < min_distance)
					{
						min_distance = hamming_distance;
						best_corr_fid = feature_index;
					}
				}
			}
			if(min_distance < hamming_threshold)
			{

					cv::DMatch m;
					m.queryIdx = feature_idx;
					m.trainIdx = best_corr_fid;
					m.distance = min_distance;
					matches.push_back(m);
			}
		}

	}

	// search feature within a range
    void SparseMatcher::search_8_with_range(cv::Mat desc,
                             std::vector<cv::DMatch> &matches,
                             const std::vector<cv::KeyPoint> &train_features,
                             const std::vector<cv::KeyPoint> &query_features,
                             float range,
                             int hamming_distance_threshold)
    {
		int feature_num = desc.rows;
        float euclidean_distance_threshold = range*range;
		matches.clear();
		for (unsigned short feature_idx = 0; feature_idx < feature_num; feature_idx++)
		{
			unsigned char *data = (unsigned char *)desc.data + feature_idx * 32;
            uint64_t * f1 = (uint64_t *)desc.data + feature_idx * 4;
            cv::Point2f query_feature = query_features[feature_idx].pt;
			unsigned short min_distance = 256;
			unsigned short best_corr_fid = 0;
			for (int hash_table_id = 0; hash_table_id < hash_table_num; hash_table_id++)
			{
				unsigned long entry_idx = hash_table_id*entry_num_per_hash_table + data[hash_table_id];
				int conflict_num = features_buffer[entry_idx].size();
				for (int i = 0; i < conflict_num; i++)
				{
					unsigned int train_feature_index = features_buffer[entry_idx][i];
                    uint64_t * f2 = features_descriptor + train_feature_index * 4;
                    cv::Point2f train_feature = train_features[train_feature_index].pt;

                    // 根据三维空间的位置来判断是否是同一个特征点
                    if ((train_feature.x - query_feature.x)*(train_feature.x - query_feature.x) +
                        (train_feature.y - query_feature.y)*(train_feature.y - query_feature.y)
                                < euclidean_distance_threshold)
                    {
                        int hamming_distance = calculate_hamming_distance_256bit(f1, f2);
                        if (hamming_distance < min_distance)
                        {
                            min_distance = hamming_distance;
                            best_corr_fid = train_feature_index;
                        }
                    }
					
				}
			}
            if (min_distance <= hamming_distance_threshold)
			{
                cv::DMatch m;
				m.queryIdx = feature_idx;
				m.trainIdx = best_corr_fid;
				m.distance = min_distance;
				matches.push_back(m);
			}

		}

	}
#else

	bool less_match_id(DMatch m1, DMatch m2)
	{
		return m1.trainIdx < m2.trainIdx;
	}
	bool less_match_score(DMatch m1, DMatch m2)
	{
		return m1.distance < m2.distance;
	}
	bool equal_match_feature_id(DMatch m1, DMatch m2)
	{
		return m1.trainIdx == m2.trainIdx;
	}
	void SparseMatcher::search_8(cv::Mat desc, std::vector<cv::DMatch> &matches)
	{
		int feature_num = desc.rows;
		matches.clear();
		for (unsigned short feature_idx = 0; feature_idx < feature_num; feature_idx++)
		{
			unsigned char *data = (unsigned char *)desc.data + feature_idx * 32;
			uint64_t * f1 = (uint64_t *)desc.data + feature_idx * 4;
			unsigned short min_distance = 256;
			unsigned short best_corr_fid = 0;
			std::vector<cv::DMatch> feature_matches;
			for (int hash_table_id = 0; hash_table_id < hash_table_num; hash_table_id++)
			{
				unsigned long entry_idx = hash_table_id*entry_num_per_hash_table + data[hash_table_id];
				int conflict_num = features_buffer[entry_idx].size();
				for (int i = 0; i < conflict_num; i++)
				{
					unsigned int feature_index = features_buffer[entry_idx][i];
					uint64_t * f2 = features_descriptor + feature_index * 4;
					int hamming_distance = calculate_hamming_distance_256bit(f1, f2);

					if (hamming_distance < distance_threshold)
					{
						DMatch m;
						m.queryIdx = feature_idx;
						m.trainIdx = feature_index;
						m.distance = hamming_distance;
						feature_matches.push_back(m);
					}
				}
			}
			sort(feature_matches.begin(), feature_matches.end(), less_match_id);
			feature_matches.erase(unique(feature_matches.begin(), feature_matches.end(), equal_match_feature_id), feature_matches.end());
			sort(feature_matches.begin(), feature_matches.end(), less_match_score);			//  distance in ascending order
			int feature_pair_num = min((int)feature_matches.size(),3);
			for (int i = 0; i < feature_pair_num; i++)
			{
				matches.push_back(feature_matches[i]);
			}

		}

	}
#endif
	/*
	void SparseMatcher::match(cv::Mat d1, cv::Mat d2, std::vector<cv::DMatch> &matches)
	{
		int feature_f1_num = d1.rows;
		int feature_f2_num = d2.rows;
		num_distance_calculation = 0;
		if (features_buffer == NULL)
		{
			cout << "feature buffer not initiallized!" << endl;
			return;
		}
		//	ippsZero_64s((Ipp64s*)features, buckets_num * max_buckets_num * sizeof(short) / sizeof(Ipp64s));
		//	ippsSet_8u(0, features, sizeof(short)* buckets_num * max_buckets_num);
		//_intel_fast_memset();
		// hash process 
		unsigned char * feature1 = d1.data;
		unsigned char * feature2 = d2.data;

		for (int feature_id = 0; feature_id < feature_f1_num; feature_id++)
		{
			HASH_UNIT * feautres_f1 = (HASH_UNIT *)((feature1 + 32 * feature_id));
			for (int unit_id = 0; unit_id < unit_num; unit_id++)
			{
				for (int chunk_id = 0; chunk_id < chunk_num_per_unit; chunk_id++)
				{
					int hash_table_id = unit_id * chunk_num_per_unit + chunk_id;		//	the order of hash table
					int chunk_pos = (unsigned int)((*(feautres_f1) >> (bits_per_chunk * chunk_id)) & hash_mask) + (hash_table_id)* offset_per_hash_table;
					chunk_pos = chunk_pos * max_buckets_num;
					if (features_buffer[chunk_pos] < max_buckets_num - 1)
					{
						features_buffer[chunk_pos] += 1;
						features_buffer[chunk_pos + features_buffer[chunk_pos]] = feature_id;
					}
				}
				feautres_f1++;
			}
		}
		//	unsigned char *visited_flag = new unsigned char[feature_f1_num];
		// match process
		for (int feature_id = 0; feature_id < feature_f2_num; feature_id++)
		{
			HASH_UNIT * feautres_f2 = (HASH_UNIT *)(feature2 + 32 * feature_id);
			uint64_t * feature2_ptr = d2.ptr<uint64_t>(feature_id);
			int min_distance = 256;
			int best_corr_fid = 0;
			for (int unit_id = 0; unit_id < unit_num; unit_id++)
			{
				for (int chunk_id = 0; chunk_id < chunk_num_per_unit; chunk_id++)
				{
					int hash_table_id = unit_id * chunk_num_per_unit + chunk_id;		//	the order of hash table
					int chunk_pos = ((*(feautres_f2) >> (bits_per_chunk * chunk_id)) & hash_mask)
						+ (hash_table_id)* offset_per_hash_table;
					int candidate_num = features_buffer[chunk_pos * max_buckets_num];
					unsigned short *candidate_pos = &features_buffer[chunk_pos * max_buckets_num + 1];

					for (int candidate_index = 0; candidate_index < candidate_num; candidate_index++)
					{
						int corresponding_fid = *(candidate_pos + candidate_index);
						// 					if (visited_flag[corresponding_fid] == 0)
						{
							//						visited_flag[corresponding_fid] = 1;
							uint64_t *feature1_ptr = (uint64_t*)d1.data + (corresponding_fid)* 4;
							int hamming_distance = ORB_HammingDistance_uint64(feature1_ptr, feature2_ptr);
							num_distance_calculation++;
							if (hamming_distance < min_distance)
							{
								min_distance = hamming_distance;
								best_corr_fid = corresponding_fid;
							}
						}
					}
#if 0
					for (int iter = 0; iter < bits_per_chunk; iter++)
					{
						chunk_pos = ((*(feautres_f2) >> (bits_per_chunk * chunk_id)) & hash_mask) ^ (1 << iter)
							+ (hash_table_id)* offset_per_hash_table;
						candidate_num = features_buffer[chunk_pos * max_buckets_num];
						unsigned short *candidate_pos = &features_buffer[chunk_pos * max_buckets_num + 1];
						for (int candidate_index = 0; candidate_index < candidate_num; candidate_index++)
						{
							int corresponding_fid = *(candidate_pos + candidate_index);
							//							if (visited_flag[corresponding_fid] == 0)
							{
								//								visited_flag[corresponding_fid] = 1;
								uint64_t *feature1_ptr = d1.ptr<uint64_t>(corresponding_fid);
								int hamming_distance = ORB_HammingDistance_uint64(feature1_ptr, feature2_ptr);
								num_distance_calculation++;
								if (hamming_distance < min_distance)
								{
									min_distance = hamming_distance;
									best_corr_fid = corresponding_fid;
								}
							}
						}
					}
#endif
				}
				feautres_f2++;
			}
			if (min_distance <= distance_threshold)
			{
				DMatch m;
				m.queryIdx = feature_id;
				m.trainIdx = best_corr_fid;
				m.distance = min_distance;
				matches.push_back(m);
			}
		}
	}
	*/
}
