#ifndef PINHOLE_CAMERA_H
#define PINHOLE_CAMERA_H


#include <Eigen/Core>
#include "Geometry/Geometry.h"
namespace one_piece 
{
namespace camera
{

  enum CameraType{TUM_DATASET, OPEN3D_DATASET, MI_DATASET};
  class PinholeCamera 
  {
        public:

        PinholeCamera()
        {
          SetCameraType(OPEN3D_DATASET);
        }
        PinholeCamera(float _fx, float _fy, float _cx, float _cy, int _width, int _height, float _depth_scale, float *_distortion = nullptr)
        {
            fx = _fx;
            fy = _fy;
            cx = _cx;
            cy = _cy;
            width = _width;
            height = _height;
            depth_scale = _depth_scale;
            if(_distortion != nullptr)
            for(int i = 0; i <5;++i)
            distortion_para[i] = _distortion[i];
            else
            for(int i = 0; i <5;++i)
            distortion_para[i] = 0;

        }
        PinholeCamera GenerateNextPyramid() const
        {
          PinholeCamera next_camera(fx/2,fy/2,cx/2,cy/2,width/2,height/2,depth_scale);
          return next_camera;
        }
        geometry::Matrix3 ToCameraMatrix() const
        {
          geometry::Matrix3 intrinsic;
          intrinsic<<fx,0,cx,
                     0,fy,cy,
                     0, 0, 1;
          return intrinsic;
        }
        // bool IsInFrame(float x, float y) const
        // {
        //   int u = (int)(x + 0.5);
        //   int v = (int)(y + 0.5);
        //   return u >= 0 && u < width && v >= 0 && v < height;
        // }
        float GetFx() const {return fx;}
        float GetFy() const {return fy;}
        float GetCx() const {return cx;}
        float GetCy() const {return cy;}
        float GetWidth() const {return width;}
        float GetHeight() const {return height;}
        float GetDepthScale() const {return depth_scale;}
        void SetPara(float _fx, float _fy, float _cx, float _cy, int _width, int _height, float _depth_scale = -1, float *_distortion = nullptr)
        {
            fx = _fx;
            fy = _fy;
            cx = _cx;
            cy = _cy;
            width = _width;
            height = _height;
            depth_scale = _depth_scale;
            if(_distortion != nullptr)
            for(int i = 0; i <5;++i)
            distortion_para[i] = _distortion[i];
        }
        void SetCameraType(const CameraType &type)
        {
          if(type == CameraType::TUM_DATASET)
          {
            fx = 517.3;
            fy = 516.5;
            cx = 318.6;
            cy = 255.3;
            depth_scale = 5000;
            width = 640;
            height = 480;
            distortion_para[0] = 0.2624;
            distortion_para[1] = -0.9531;
            distortion_para[2] = -0.0054;
            distortion_para[3] = 0.0026;
            distortion_para[4] = 1.1633;
          }
          else if (type == CameraType::OPEN3D_DATASET)
          {
            fx = 514.817;
            fy = 515.375;
            cx = 318.771;
            cy = 238.447;
            depth_scale = 1000;
            width = 640;
            height = 480;
            for(int i = 0; i <5;++i)
            distortion_para[i] = 0;
          }
          else if(type == CameraType::MI_DATASET)
          {
            fx = 2209.84366 ;
            fy = 2210.23057;
            cx = 756.24762;
            cy = 530.00418;
            depth_scale = 1000;
            width = 1440;
            height = 1080;
            for(int i = 0; i <5;++i)
            distortion_para[i] = 0;
          }
          
        }
        protected:

        float fx;
        float fy;
        float cx;
        float cy;
        float depth_scale = 1000.0;
        float distortion_para[5];
        int width;
        int height;
  };
};
}
#endif