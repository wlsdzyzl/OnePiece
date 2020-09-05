#include "Ransac.h"

namespace fucking_cool
{
namespace geometry
{
    TransformationMatrix EstimateRigidTransformationRANSAC(const PointCorrespondenceSet &correspondence_set,
        PointCorrespondenceSet & inliers, std::vector<int> &inlier_ids, int max_iteration, float threshold)
    {
        if(correspondence_set.size() < MIN_INLIER_SIZE_RANSAC_TRANSFORMATION)
        {
        std::cout<<YELLOW<<"[Warning]::[FitPlaneRANSAC]::Too few canidate point pair."<<RESET<<std::endl;
        return TransformationMatrix::Zero();
        }
        GRANSAC::RANSAC<TransformationModel, MIN_INLIER_SIZE_RANSAC_TRANSFORMATION> Estimator;
        std::vector<std::shared_ptr<GRANSAC::AbstractParameter>> CandPoints;
        for(int i = 0; i != correspondence_set.size(); ++i)
        {
            std::shared_ptr<GRANSAC::AbstractParameter> CandPt = 
                std::make_shared<Point3fPair>(correspondence_set[i].first, correspondence_set[i].second, i);
            CandPoints.push_back(CandPt);
        }
        Estimator.Initialize(threshold, max_iteration);
        Estimator.Estimate(CandPoints);
        auto bestInliers = Estimator.GetBestInliers();
        for(int i = 0; i < bestInliers.size();++i)
        {
            auto inlier =  std::dynamic_pointer_cast<Point3fPair>(bestInliers[i]);
            inliers.push_back(inlier->_3d_pair);
            inlier_ids.push_back(inlier->id);
        }
        auto bestModel = Estimator.GetBestModel();
        auto bestPoints = bestModel->GetModelParams();
        PointCorrespondenceSet model_points; 
        for(int i = 0; i != MIN_INLIER_SIZE_RANSAC_TRANSFORMATION; ++i )
        {
            model_points.push_back(std::dynamic_pointer_cast<Point3fPair>(bestPoints[i])->_3d_pair);
        }
        return EstimateRigidTransformation(model_points);
    }   

    std::tuple<geometry::Point3, double, double> FitPlaneRANSAC(const Point3List &points, Point3List & inliers, 
        std::vector<int> &inlier_ids, int max_iteration, float threshold )
    {
        if(points.size() < MIN_INLIER_SIZE_RANSAC_PLANEFITTING)
        {
        std::cout<<YELLOW<<"[Warning]::[FitPlaneRANSAC]::Too few canidate points."<<RESET<<std::endl;
        return std::make_tuple(geometry::Point3(0,0,0),0,0);
        }
        GRANSAC::RANSAC<PlaneFittingModel, MIN_INLIER_SIZE_RANSAC_PLANEFITTING> Estimator;      
        std::vector<std::shared_ptr<GRANSAC::AbstractParameter>> CandPoints;
        Point3List model_points;
        for(int i = 0; i != points.size(); ++i)
        {
            CandPoints.push_back(std::make_shared<GPoint3f>(points[i]));
        }          
        Estimator.Initialize(threshold, max_iteration);
        Estimator.Estimate(CandPoints);
        auto bestInliers = Estimator.GetBestInliers();
        for(int i = 0; i < bestInliers.size();++i)
        {
            auto inlier =  std::dynamic_pointer_cast<GPoint3f>(bestInliers[i]);
            inliers.push_back(inlier->p);
            inlier_ids.push_back(inlier->id);
        }

        auto bestModel = Estimator.GetBestModel();
        auto bestPoints = bestModel->GetModelParams(); 

        for(int i = 0; i != MIN_INLIER_SIZE_RANSAC_PLANEFITTING; ++i )
        {
            model_points.push_back(std::dynamic_pointer_cast<GPoint3f>(bestPoints[i])->p);
        }

        //auto bestInliers = Estimator.GetBestInliers();
        return FitPlane(model_points);
    }
}
}