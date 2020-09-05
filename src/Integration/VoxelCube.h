#ifndef VOXEL_CUBE_H
#define VOXEL_CUBE_H

#define CUBE_SIZE 8

#include "TSDFVoxel.h"
#include "Tool/ConsoleColor.h"
#include <vector>
#include <fstream>
#include <iostream>
namespace fucking_cool
{
namespace integration
{
    typedef Eigen::Vector3i CubeID;
    typedef geometry::VoxelGridHasher CubeHasher;
    class CubePara
    {
        public:
        CubePara()
        {
            InitializeVoxelCube();
        }
        std::vector<Eigen::Vector3i> NeighborCubeIDOffset;
        Eigen::Matrix<int, 3, 8> CornerXYZOffset;//x,y,z
        geometry::Point3List VoxelCentroidOffSet;
        float VoxelResolution = 0.01;//meter
        void SetVoxelResolution(float resolution)
        {
            VoxelResolution = resolution;
            std::cout<<"Set Voxel resolution: "<<VoxelResolution<<std::endl;
            InitializeVoxelCube();
        }
        void InitializeVoxelCube()
        {
            NeighborCubeIDOffset.resize(8);
            for( int i = 0; i != 8; ++i)
            {
                int x = (0x01 & i) >> 0;
                int y = (0x02 & i) >> 1;
                int z = (0x04 & i) >> 2;
                NeighborCubeIDOffset[i] = Eigen::Vector3i(x, y, z);
                //std::cout<<NeighborCubeIDOffset[i]<<std::endl;
            }
            CornerXYZOffset <<  0, 1, 1, 0, 0, 1, 1, 0,
                                0, 0, 1, 1, 0, 0, 1, 1,
                                0, 0, 0, 0, 1, 1, 1, 1;
            VoxelCentroidOffSet.resize(CUBE_SIZE * CUBE_SIZE * CUBE_SIZE);
            float half_resolution = VoxelResolution / 2;
            for(int x = 0; x!= CUBE_SIZE; ++x)
            {
                for(int y = 0; y!= CUBE_SIZE; ++y)
                {
                    for(int z = 0; z!= CUBE_SIZE; ++z)
                    {
                        VoxelCentroidOffSet[x + y * CUBE_SIZE + z *CUBE_SIZE * CUBE_SIZE] 
                            = geometry::Point3(x*VoxelResolution + half_resolution,
                            y * VoxelResolution + half_resolution, z * VoxelResolution + half_resolution);
                    }
                }
            }
        }
        CubeID GetCubeID(const geometry::Point3i & point) const
        {
            return CubeID(std::floor((point(0) + 0.0) / CUBE_SIZE), 
                std::floor((point(1) + 0.0) / CUBE_SIZE), std::floor((point(2) + 0.0) / CUBE_SIZE));
        }
        CubeID GetCubeID(const geometry::Point3 &point) const
        {
            geometry::Point3i point_bounded(std::floor(point(0)/VoxelResolution),
                std::floor(point(1) / VoxelResolution), std::floor(point(2) / VoxelResolution));

            return GetCubeID(point_bounded);
        }
        geometry::Point3 GetGlobalPoint(const CubeID &cube_id, int voxel_id) const
        {
            geometry::Point3 start_point = 
                geometry::Point3(cube_id(0), cube_id(1), cube_id(2)) * CUBE_SIZE * VoxelResolution; 
            return start_point + VoxelCentroidOffSet[voxel_id];
        }
        int GetVoxelID(const geometry::Point3i &point) const
        {
            CubeID cube_id = GetCubeID(point);
            geometry::Point3i offset = point - cube_id * CUBE_SIZE;
            return offset(0) + offset(1) * CUBE_SIZE + offset(2) * CUBE_SIZE * CUBE_SIZE;
        }
        int GetVoxelID(const geometry::Point3 &point) const
        {
            geometry::Point3i point_bounded(std::floor(point(0)/VoxelResolution),
                std::floor(point(1) / VoxelResolution), std::floor(point(2) / VoxelResolution));
            return GetVoxelID(point_bounded);
        }
    };

    class VoxelCube
    {
        public:
        VoxelCube()
        {
            voxels.resize(CUBE_SIZE * CUBE_SIZE * CUBE_SIZE, TSDFVoxel());
        }
        VoxelCube(const CubeID & id)
        {
            cube_id = id;
            voxels.resize(CUBE_SIZE * CUBE_SIZE * CUBE_SIZE, TSDFVoxel());
        }

        void IntegrateWithOtherCube(const VoxelCube & other)
        {
            if(cube_id != other.cube_id)
            {
                std::cout<<YELLOW<<"[Integration]::[WARNING]::Integrate two cubes which do not have the same cube_id."<<RESET<<std::endl;
                return;
            }
            for(int i = 0;i!= voxels.size(); ++i)
            {
                voxels[i] += other.voxels[i];
            }
        }
        float GetSDF(int voxel_id) const
        {
            return voxels[voxel_id].sdf;
        }
        TSDFVoxel GetVoxel(int voxel_id) const
        {
            return voxels[voxel_id];
        }
        void WriteToBuffer(std::vector<float> &buffer) const
        {
            buffer.push_back(cube_id(0));
            buffer.push_back(cube_id(1));
            buffer.push_back(cube_id(2));

            for(int i = 0;i!= voxels.size(); ++i)
            {
                if(std::fabs(voxels[i].sdf) <1 && voxels[i].weight!= 0 )
                {
                    buffer.push_back(i);
                    buffer.push_back(voxels[i].sdf);
                    buffer.push_back(voxels[i].weight);
                    buffer.push_back(voxels[i].color(0));
                    buffer.push_back(voxels[i].color(1));
                    buffer.push_back(voxels[i].color(2));
                }
            }
            buffer.push_back(-2.0);
            return;
        }
        geometry::Point3 GetOrigin(const CubePara &c_para) const
        {
            float cube_resolution = CUBE_SIZE * c_para.VoxelResolution;
            return geometry::Point3(cube_id(0) * cube_resolution, cube_id(1) * cube_resolution, cube_id(2) * cube_resolution);
        }
        void ReadFromBuffer(const std::vector<float> & buffer, size_t &ptr)
        {
            while(buffer[ptr]!= -2.0)
            {
                int i = buffer[ptr++];
                voxels[i].sdf = buffer[ptr++];
                voxels[i].weight = buffer[ptr++];
                voxels[i].color(0) = buffer[ptr++];
                voxels[i].color(1) = buffer[ptr++];
                voxels[i].color(2) = buffer[ptr++];
            }
            ptr++;
            return;
        }
        void ReadFromBufferFloat(const std::vector<float> & buffer, size_t &ptr)
        {
            ptr++;//size
            while(buffer[ptr]!= -2.0)
            {
                int i = buffer[ptr++];
                voxels[i].sdf = buffer[ptr++];
                voxels[i].weight = buffer[ptr++];
                //std::cout<<"sdf: "<<voxels[i].sdf<<" weight: "<<voxels[i].weight<<std::endl;
            }
            ptr++;
            size_t count = buffer[ptr++];
            //std::cout<<"count: "<<count<<std::endl;
            int c = 0;
            while(c<count)
            {
                c++;
                int i = buffer[ptr++];
                voxels[i].color(0) = buffer[ptr++]/255.0;
                voxels[i].color(1) = buffer[ptr++]/255.0;
                voxels[i].color(2) = buffer[ptr++]/255.0;
                float color_weight = buffer[ptr++];
                voxels[i].color /= color_weight;
            }
            return;
        }
        public:
        std::vector<TSDFVoxel> voxels;
        Eigen::Vector3i cube_id;
    };
    TSDFVoxel ReadVoxelInterpolate(const std::vector<geometry::Point3i> &_8_neighbor, 
        const std::vector<TSDFVoxel> &_8_voxels, const geometry::Point3 &new_position, const CubePara &c_para);

}
}

#endif