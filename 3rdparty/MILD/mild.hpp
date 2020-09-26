/**
* This file is part of MILD.
*
* Copyright (C) Lei Han(lhanaf@connect.ust.hk) and Lu Fang(fanglu@sz.tsinghua.edu.cn)
* For more information see <https://github.com/lhanaf/MILD>
*
**/

// MILD : Multi-Index based Loop closure Detection
#pragma once
#ifndef MILD_H
#define MILD_H

#include <inttypes.h>
#include <stdint.h>

#include "lightweight_vector.hpp"


typedef int64_t __int64;
namespace MILD
{



#define FEATURE_TYPE_ORB	1
#define FEATURE_TYPE_BRISK	2
#define ORB_DESCRIPTOR_LEN  256
#define BRISK_DESCRIPTOR_LEN 512

#define DEFAULT_MAX_UNIT_NUM_PER_ENTRY 200
#define DEFAULT_HAMMING_DISTANCE_THRESHOLD (64)
#define HAMMING_COVARIANCE		(900.0f)		// used in calculating feature similarity
	
#define DEBUG_MODE_MILD 0

		// cunstrcut hash table index
		// mapping from desc to entry_idx 
	inline void multi_index_hashing(std::vector<unsigned long> &entry_idx, unsigned int * desc,
		int hash_table_num, int bits_per_substring)
	{

        if(bits_per_substring == 16)
        {

            unsigned short * entry_value = (unsigned short *)desc;
            for (int i = 0; i < hash_table_num; i++)
            {
                entry_idx[i] = entry_value[i];
            }
        }
        else
        {
            unsigned long index = 0;
            unsigned int mask[32];
            for (int j = 0; j < 32; j++)
            {
                mask[j] = 0;
                for (int i = 0; i < j; i++)
                {
                    mask[j] += 1 << i;
                }
            }
            for (int i = 0; i < hash_table_num; i++)
            {
                int si = (bits_per_substring * i) / 32;
                int ei = (bits_per_substring * (i + 1)) / 32;
                int sp = (bits_per_substring * i) % 32;
                int ep = (bits_per_substring * (i + 1)) % 32;
                if (si == ei)
                {
                    index = (desc[si] >> sp) & mask[bits_per_substring];
                }
                if (si < ei)
                {
                    index = ((desc[si] >> sp) & mask[32 - sp]) + ((desc[ei] & mask[ep]) << (bits_per_substring - ep));
                }

                entry_idx[i] = index;
            }
        }

	}

		// output neighbor_entry_idx of entry_idx, whose hamming distance is less than my_depth_leve.
		inline void generate_neighbor_candidates(int my_depth_level, unsigned long entry_idx, std::vector<unsigned long> &neighbor_entry_idx, int bits_per_substring)
		{
			if (my_depth_level <= 0)
			{
				return;
			}
			my_depth_level--;
			for (int i = 0; i < bits_per_substring; i++)
			{
				unsigned long nidx = entry_idx ^ (1 << i);
				int redundancy_flag = 0;
				for (size_t j = 0; j < neighbor_entry_idx.size(); j++)
				{
					if (nidx == neighbor_entry_idx[j])
					{
						redundancy_flag = 1;
					}
				}
				if (redundancy_flag == 0)
				{
					neighbor_entry_idx.push_back(nidx);
				}
				generate_neighbor_candidates(my_depth_level, nidx, neighbor_entry_idx, bits_per_substring);
			}
			return;
		}


}


#endif
