#ifndef REGISTRATION_RESULT_H
#define REGISTRATION_RESULT_H
#include "Geometry/Geometry.h"
#include "Geometry/PointCloud.h"
namespace one_piece
{
namespace registration
{
    class RegistrationResult
    {
        public:
        geometry::TransformationMatrix T;
        geometry::FMatchSet correspondence_set_index;
        geometry::PointCorrespondenceSet correspondence_set;
        double rmse;
    };
    // class RegistrationResult
    // {
    //     public:
    //     geometry::TransformationMatrix T;
    //     geometry::PointCorrespondenceSet correspondence_set;
    //     geometry::PointCloud down_sample_source;
    //     geometry::PointCloud down_sample_target;
    //     double rmse;
    //     //geometry::FMatchSet matches;
    // };    
}
}
#endif