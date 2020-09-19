#ifndef TSDF_VOXEL_H
#define TSDF_VOXEL_H
#include "Geometry/Geometry.h"
namespace fucking_cool
{
namespace integration
{
    class TSDFVoxel
    {
        public:
        TSDFVoxel()=default;
        TSDFVoxel(float _sdf, float _weight, const geometry::Point3 &_color)
        {
            sdf = _sdf;
            weight = _weight;
            color = _color;
        }
        TSDFVoxel(const TSDFVoxel &other)
        {
            sdf = other.sdf;
            weight = other.weight;
            color = other.color;
        }
        TSDFVoxel operator+(const TSDFVoxel &other) const
        {
            TSDFVoxel result;
            if(weight == 0)
            return other;
            if(other.weight == 0)
            return *this;

            result.weight = weight + other.weight;
            if(result.weight != 0)
            {
                result.sdf = (weight * sdf + other.weight * other.sdf)/result.weight;
                result.color = (weight * color + other.weight * other.color)/result.weight;
            }
            return result;
        }
        TSDFVoxel add(const TSDFVoxel &other) const
        {
            //direct addtion
            TSDFVoxel result;
            if(weight == 0)
            return other;
            if(other.weight == 0)
            return *this;

            result.weight = weight + other.weight;
            result.sdf =  sdf +  other.sdf;
            result.color =  color + other.color;
            return result;
        }
        void operator+=(const TSDFVoxel &other) 
        {
            *this = (*this) + other;
        }
        TSDFVoxel operator *(float _weight) const
        {
            TSDFVoxel result = *this;
            if(_weight == 0 || weight == 0)
            {
                return TSDFVoxel();
            }
            result.weight = weight * _weight;
            result.sdf = sdf * _weight;
            result.color = color * _weight;
            return result;
            
        }
        TSDFVoxel operator /(float _weight) const
        {
            return this->operator*(1 / _weight);
        }
        bool IsValid() const
        {
            return !(sdf >= 1||weight <= 0);
        }
        float sdf = 999;
        float weight = 0;
        geometry::Point3 color = geometry::Point3(-1,-1,-1);
    };
}
}
#endif