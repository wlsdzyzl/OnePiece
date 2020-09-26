/**
* This file is part of MILD.
*
* Copyright (C) Lei Han(lhanaf@connect.ust.hk) and Lu Fang(fanglu@sz.tsinghua.edu.cn)
* For more information see <https://github.com/lhanaf/MILD>
*
**/


#include "loop_closure_detector.h"

#include <iostream>
#include "smmintrin.h"

using namespace std;


namespace MILD
{




	LoopClosureDetector::~LoopClosureDetector()
	{
	}
	LoopClosureDetector::LoopClosureDetector(int feature_type, 
		int para_table_num, 
		int input_depth_level, 
		int input_distance_threshold,
		int input_max_num_per_entry)
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
		bits_per_substring = (int)(descriptor_length / para_table_num);		
		if (bits_per_substring > sizeof(size_t)* 8)
		{
			cout << "substring too large !, invalied" << endl;
			return;
		}
		hash_table_num = para_table_num;
		entry_num_per_hash_table = pow(2, bits_per_substring);
		buckets_num = entry_num_per_hash_table * hash_table_num;
		max_unit_num_per_entry = input_max_num_per_entry;
		distance_threshold = input_distance_threshold;

		features_buffer = std::vector<mild_entry>(entry_num_per_hash_table * hash_table_num);
		for (unsigned int i = 0; i < entry_num_per_hash_table * hash_table_num; i++)
		{
			features_buffer[i].clear();
		}
		statistics_num_distance_calculation = 0;
		lut_feature_similarity = std::vector<float>(512);
		for (int i = 0; i < 512; i++)
		{
			float hamming_distance = i;
			if (hamming_distance < 10)
			{
				hamming_distance = 10;
			}
			lut_feature_similarity[i] = expf(-hamming_distance  * hamming_distance / HAMMING_COVARIANCE);
		}
	}
	int LoopClosureDetector::count_feature_in_database()
	{
		int database_feature_num = 0;
		size_t database_size = features_descriptor.size();
		for (size_t i = 0; i < database_size; i++)
		{
			database_feature_num += features_descriptor[i].descriptor.rows;
		}
		return database_feature_num;
	}


	int LoopClosureDetector::construct_database(cv::Mat desc)
	{
		
		int feature_num = desc.rows;
		if (descriptor_type == FEATURE_TYPE_ORB)
		{
			int descriptor_length = desc.cols * 8;
			if (descriptor_length != descriptor_length)
			{
				cout << "error ! feature descriptor length doesn't match" << endl;
			}
		}
		
		size_t image_index = features_descriptor.size(); 
		std::vector<unsigned long> hash_entry_index = std::vector<unsigned long>(hash_table_num);

		database_frame df;
		df.descriptor = desc;
		for (int feature_idx = 0; feature_idx < feature_num; feature_idx++)
		{
			unsigned int *data = desc.ptr<unsigned int>(feature_idx);
			multi_index_hashing(hash_entry_index,data,hash_table_num,bits_per_substring);
  			feature_indicator f;
			f.image_index = image_index;
			f.feature_index = feature_idx;
			for (unsigned int hash_table_id = 0; hash_table_id < hash_table_num; hash_table_id++)
			{
				int entry_pos = hash_table_id*entry_num_per_hash_table + hash_entry_index[hash_table_id];
				if (features_buffer[entry_pos].size() == 0 || 
					(features_buffer[entry_pos].size() < DEFAULT_MAX_UNIT_NUM_PER_ENTRY &&
					features_buffer[entry_pos].back().image_index != image_index))
				{
					features_buffer[entry_pos].push_back(f);
				}
			}
		}
		features_descriptor.push_back(df);
		return features_descriptor.size();
	}

    int LoopClosureDetector::calculate_hamming_distance_256bit(uint64_t * f1, uint64_t * f2)
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

	

	int LoopClosureDetector::insert_and_query_database(cv::Mat desc, std::vector<float> &score)
	{
		if (descriptor_type == FEATURE_TYPE_ORB)
		{
			int descriptor_length = desc.cols * 8;
			if (descriptor_length != descriptor_length)
			{
				cout << "error ! feature descriptor length doesn't match" << endl;
			}
		}
		int feature_num = desc.rows;
		database_frame df;
		df.descriptor = desc;
		features_descriptor.push_back(df);

		size_t database_size = features_descriptor.size();
		score.clear();
		score = std::vector<float>(database_size);
		std::vector<float> feature_score = std::vector<float>(database_size);
		for (size_t i = 0; i < database_size; i++)
		{
			score[i] = 0;
		}

		int image_index = features_descriptor.size() - 1;
		std::vector<unsigned long> hash_entry_index = std::vector<unsigned long>(hash_table_num);

		for (int feature_idx = 0; feature_idx < feature_num; feature_idx++)
		{
			for (size_t cnt = 0; cnt < database_size; cnt++)
			{
				feature_score[cnt] = 0;
			}
			unsigned int *data = desc.ptr<unsigned int>(feature_idx);
            uint64_t * f1 = desc.ptr<uint64_t>(feature_idx);
			multi_index_hashing(hash_entry_index, data, hash_table_num, bits_per_substring);
			feature_indicator f;
			f.image_index = image_index;
			f.feature_index = feature_idx;
			for (unsigned int hash_table_id = 0; hash_table_id < hash_table_num; hash_table_id++)
			{
				int entry_pos = hash_table_id*entry_num_per_hash_table + hash_entry_index[hash_table_id];
				if (features_buffer[entry_pos].size() == 0 ||
					(features_buffer[entry_pos].size() < DEFAULT_MAX_UNIT_NUM_PER_ENTRY &&
					features_buffer[entry_pos].back().image_index != image_index))
				{
					unsigned long entry_idx = entry_pos;
					search_entry(f1, entry_idx, feature_score);
					// may not be the most efficient implementation
					// to be refine generate_neighbor_candidates
					if (depth_level > 0)
					{
						std::vector<unsigned long> neighbor_entry_idx;
						neighbor_entry_idx.clear();
						generate_neighbor_candidates(depth_level, entry_idx, neighbor_entry_idx, bits_per_substring);
						for (size_t iter = 0; iter < neighbor_entry_idx.size(); iter++)
						{
							search_entry(f1, neighbor_entry_idx[iter], feature_score);
						}
					}
					// query feature;
					features_buffer[entry_pos].push_back(f);
				}
			}
			int similar_feature_count = 0;
			float total_feature_energy = lut_feature_similarity[20];
			for (size_t cnt = 0; cnt < database_size; cnt++)
			{
				total_feature_energy += feature_score[cnt];
				similar_feature_count += (feature_score[cnt] > 0);
			}
			similar_feature_count = max(1, similar_feature_count);
			float idf_freq = (float)database_size / similar_feature_count;
			idf_freq = idf_freq > 1 ? idf_freq : 1;
			idf_freq = log(idf_freq);
			for (size_t cnt = 0; cnt < database_size; cnt++)
			{
				score[cnt] += feature_score[cnt] / total_feature_energy * idf_freq;
			}
		}
		return features_descriptor.size();
	}

	int LoopClosureDetector::query_database(cv::Mat desc, std::vector<float> &score)
	{
		int feature_num = desc.rows;
		// int descriptor_length = desc.cols * 8;
		// int database_feature_num = count_feature_in_database();
		size_t database_size = features_descriptor.size(); 
		score.clear();
		score = std::vector<float>(database_size);
		std::vector<float> feature_score = std::vector<float>(database_size);
		for (size_t i = 0; i < database_size; i++)
		{
			score[i] = 0;
		}
		std::vector<unsigned long> hash_entry_index = std::vector<unsigned long>(hash_table_num);
		for (int feature_idx = 0; feature_idx < feature_num; feature_idx++)
		{

			memset(&feature_score[0], 0, feature_score.size() * sizeof feature_score[0]);
			unsigned int *data = desc.ptr<unsigned int>(feature_idx);
            uint64_t * f1 = desc.ptr<uint64_t>(feature_idx);
			multi_index_hashing(hash_entry_index, data, hash_table_num, bits_per_substring);
			for (unsigned int hash_table_id = 0; hash_table_id < hash_table_num; hash_table_id++)
			{
				unsigned long entry_idx = hash_table_id*entry_num_per_hash_table + hash_entry_index[hash_table_id];
				search_entry(f1, entry_idx, feature_score);

				
				// may not be the most efficient implementation
				// to be refine generate_neighbor_candidates
				if (depth_level > 0)
				{
					std::vector<unsigned long> neighbor_entry_idx;
					generate_neighbor_candidates(depth_level, entry_idx, neighbor_entry_idx, bits_per_substring);
					for (size_t iter = 0; iter < neighbor_entry_idx.size(); iter++)
					{
						search_entry(f1, neighbor_entry_idx[iter], feature_score);
					}
				}
			}
			int similar_feature_count = 0;
			float total_feature_energy = lut_feature_similarity[20];
			for (size_t cnt = 0; cnt < database_size; cnt++)
			{
				total_feature_energy += feature_score[cnt];
				similar_feature_count += (feature_score[cnt] > 0);
			}
			similar_feature_count = max(1, similar_feature_count);
			float idf_freq = (float)database_size / similar_feature_count;
			idf_freq = idf_freq > 1 ? idf_freq : 1;
			idf_freq = log(idf_freq);
			for (size_t cnt = 0; cnt < database_size; cnt++)
			{
				score[cnt] += feature_score[cnt] / total_feature_energy * idf_freq;			
			}
		}
		return 1;
	}

    void LoopClosureDetector::search_entry(uint64_t * f1, unsigned long search_entry_idx, std::vector<float> &score)
	{
		mild_entry &current_entry = features_buffer[search_entry_idx];
		size_t conflict_num = current_entry.size();
		for (size_t i = 0; i < conflict_num; i++)
		{
			feature_indicator &f = current_entry[i];
            uint64_t * f2 = (uint64_t *)features_descriptor[f.image_index].descriptor.data + f.feature_index * 4;
			//features_descriptor[f.image_index].descriptor.ptr<unsigned __int64>(f.feature_index);
            int hamming_distance =  (_mm_popcnt_u64(*(f1) ^ *(f2)) +
                                                        _mm_popcnt_u64(*(f1 + 1) ^ *(f2 + 1)) +
                                                        _mm_popcnt_u64(*(f1 + 2) ^ *(f2 + 2)) +
                                                        _mm_popcnt_u64(*(f1 + 3) ^ *(f2 + 3)));// calculate_hamming_distance_256bit(f1, f2);
			if (hamming_distance < distance_threshold)
			{
				float similarity = lut_feature_similarity[hamming_distance];
				score[f.image_index] += similarity;
			}
		}
	}
	void LoopClosureDetector::displayParameters()
	{

		cout<< "parameters: " << endl
			<< "unit length :	" << descriptor_length << endl
			<< "chunk_num_per_unit :	" << depth_level << endl
			<< "bits_per_substring :	" << bits_per_substring << endl
			<< "hash_table_num :	" << hash_table_num << endl
			<< "entry_num_per_hash_table :	" << entry_num_per_hash_table << endl
			<< "buckets_num :	" << buckets_num << endl;
	}

}
