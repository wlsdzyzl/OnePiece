#ifndef XTION_READER_H
#define XTION_READER_H
#include <OpenNI.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include "ConsoleColor.h"
#include <string>
namespace one_piece
{
namespace tool
{
    class OpenNIReader
    {
        public:
        bool Init()
        {
            openni::Status rc = openni::OpenNI::initialize(); 
            if(rc != openni::STATUS_OK)
            return false;
            /*
            openni::Array<openni::DeviceInfo> aDeviceList;  
            openni::OpenNI::enumerateDevices( &aDeviceList );
            std::cout<<"Now we have detected "<<aDeviceList.getSize()<<" devices! "<<std::endl;
        
            std::cout<<aDeviceList[0].getName()<<std::endl;
            device_uri =aDeviceList[0].getUri(); //openni::ANY_DEVICE;
            std::cout<<openni::ANY_DEVICE<<std::endl;
            */
            device.open(openni::ANY_DEVICE);
            device_name = device.getDeviceInfo().getName();
            //std::cout<<device_name<<std::endl;
            if(device_name == "PS1080")
            {
                std::cout<<GREEN<<"[Xtion Reader]::[INFO]::We have detected one xtion."<<RESET<<std::endl;
            }
           
            stream_depth.create( device, openni::SENSOR_DEPTH );


            stream_color.create( device, openni::SENSOR_COLOR );
            openni::VideoMode m_mode_depth;
            m_mode_depth.setResolution( width, height );
            m_mode_depth.setFps( fps );
            m_mode_depth.setPixelFormat( openni::PIXEL_FORMAT_DEPTH_1_MM );
            stream_depth.setVideoMode( m_mode_depth);

            openni::VideoMode m_mode_color;
            m_mode_color.setResolution( width, height );
            m_mode_color.setFps( fps );
            m_mode_color.setPixelFormat( openni::PIXEL_FORMAT_RGB888 );

            stream_color.setVideoMode( m_mode_color);

            if( device.isImageRegistrationModeSupported(
                openni::IMAGE_REGISTRATION_DEPTH_TO_COLOR ) )
            {
                device.setImageRegistrationMode( openni::IMAGE_REGISTRATION_DEPTH_TO_COLOR );
            }

            stream_depth.start();
            stream_color.start();
            is_valid = true;
            return true;
        }
        void GetNextRGBD(cv::Mat &rgb, cv::Mat &depth)
        {
            stream_depth.readFrame( &frame_depth );
            stream_color.readFrame( &frame_color );

            cv::Mat depth_image( frame_depth.getHeight(), frame_depth.getWidth(), CV_16UC1, (void*)frame_depth.getData());

            cv::Mat color_image(frame_color.getHeight(), frame_color.getWidth(), CV_8UC3, (void*)frame_color.getData());
            cv::cvtColor( color_image, rgb, CV_RGB2BGR );
            depth = depth_image;
        }
        void Close()
        {
            stream_depth.destroy();
            stream_color.destroy();
            device.close();
            openni::OpenNI::shutdown();
            is_valid = false;
        }
        std::string device_name;
        int width = 640;
        int height = 480;
        int fps = 30;
        bool is_valid = false;
        private:
        //std::string device_uri;
        openni::Device device;
        openni::VideoStream stream_depth;
        openni::VideoStream stream_color;
        openni::VideoFrameRef frame_depth;
        openni::VideoFrameRef frame_color;

    };
}
}
#endif