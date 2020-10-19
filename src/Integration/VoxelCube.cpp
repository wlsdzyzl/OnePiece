#include "VoxelCube.h"
namespace one_piece
{
namespace integration
{
    TSDFVoxel ReadVoxelInterpolate(const std::vector<geometry::Point3i> &_8_neighbor, 
        const std::vector<TSDFVoxel> &_8_voxels, const geometry::Point3 &new_position, const CubePara &c_para)
    {
        //return _8_voxels[0];
        //const geometry::Point3 center = new_position; 
        float x_weight = (new_position(0) - _8_neighbor[0](0) * c_para.VoxelResolution ) / c_para.VoxelResolution;
        float y_weight = (new_position(1) - _8_neighbor[0](1) * c_para.VoxelResolution ) / c_para.VoxelResolution;
        float z_weight = (new_position(2) - _8_neighbor[0](2) * c_para.VoxelResolution ) / c_para.VoxelResolution;
        
        TSDFVoxel result1;
        if(_8_voxels[0].weight != 0 || _8_voxels[1].weight != 0)
        result1 = ((_8_voxels[0] * (1 - x_weight)).add( _8_voxels[1] * x_weight))
            / ((1 - x_weight) * (_8_voxels[0].weight != 0.0) + x_weight * (_8_voxels[1].weight != 0));
        TSDFVoxel result2;
        if(_8_voxels[2].weight != 0 || _8_voxels[3].weight != 0)
        result2 = ((_8_voxels[2] * (1 - x_weight)).add( _8_voxels[3] * x_weight))
            / ((1 - x_weight) * (_8_voxels[2].weight != 0.0) + x_weight * (_8_voxels[3].weight != 0));

        TSDFVoxel result_z_1;
        if(result1.weight != 0 || result2.weight != 0)
        result_z_1 = (result1 * (1 - y_weight)).add(result2 * y_weight)
            / ((1 - y_weight) * (result1.weight != 0) + y_weight * (result2.weight != 0));

            
        TSDFVoxel result3 = TSDFVoxel();
        if(_8_voxels[4].weight != 0 || _8_voxels[5].weight != 0)
        result3 = (_8_voxels[4] * (1 - x_weight)).add( _8_voxels[5] * x_weight)
            / ((1 - x_weight) * (_8_voxels[4].weight != 0.0) + x_weight * (_8_voxels[5].weight != 0));
        TSDFVoxel result4 = TSDFVoxel();
        if(_8_voxels[6].weight != 0 || _8_voxels[7].weight != 0)
        result4 = (_8_voxels[6] * (1 - x_weight)).add( _8_voxels[7] * x_weight)
            / ((1 - x_weight) * (_8_voxels[6].weight != 0.0) + x_weight * (_8_voxels[7].weight != 0));

        TSDFVoxel result_z_2;
        if(result3.weight != 0 || result4.weight != 0)
        result_z_2= (result3 * ( 1 - y_weight)).add( result4 * y_weight)
            / ((1 - y_weight) * (result3.weight != 0) + y_weight * (result4.weight != 0));

        TSDFVoxel final_voxel;
        if(result_z_1.weight != 0 || result_z_2.weight != 0)
        final_voxel = (result_z_1 * (1 - z_weight)).add( result_z_2 * z_weight)
            / ((1 - z_weight) * (result_z_1.weight != 0) + z_weight * (result_z_2.weight != 0));
        return final_voxel;
        
    }
}
}