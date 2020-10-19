//use ransac to fit plane
#ifndef PLANE_FITTING_MODEL
#define PLANE_FITTING_MODEL
#include "Geometry.h"
#include "AbstractModel.hpp"
#define MIN_INLIER_SIZE_RANSAC_PLANEFITTING 8
namespace one_piece
{
namespace geometry
{


	typedef std::array<GRANSAC::VPFloat, 2> Vector2VP;

	class GPoint3f:
		public GRANSAC::AbstractParameter
	{
	public:
		GPoint3f(const geometry::Point3 &src, int _id = -1)
		{
			p = src;
			id = _id;
		}

		geometry::Point3 p;
		int id;
	};
	//rigid
	class PlaneFittingModel
		: public GRANSAC::AbstractModel<MIN_INLIER_SIZE_RANSAC_PLANEFITTING>
	{
	protected:
		// Parametric form
		geometry::Point3 n;
		float d;

		virtual GRANSAC::VPFloat ComputeDistanceMeasure(std::shared_ptr<GRANSAC::AbstractParameter> Param) override
		{
			auto point = std::dynamic_pointer_cast<GPoint3f>(Param);
			if (point == nullptr)
				std::cout<<"ERROR::3D RANSAC."<<std::endl;

			// Return distance between passed "point" and this line
			// http://mathworld.wolfram.com/Point-LineDistance2-Dimensional.html
			float dist =std::fabs( point->p.transpose() * n + d) ;

			//// Debug
			//std::cout << "Point: " << ExtPoint2D->m_Point2D[0] << ", " << ExtPoint2D->m_Point2D[1] << std::endl;
			//std::cout << "Line: " << m_a << " x + " << m_b << " y + "  << m_c << std::endl;
			//std::cout << "Distance: " << Dist << std::endl << std::endl;

			return dist;
		};

	public:
		PlaneFittingModel(const std::vector<std::shared_ptr<GRANSAC::AbstractParameter>> &InputParams)
		{
			Initialize(InputParams);
		};

		virtual void Initialize(const std::vector<std::shared_ptr<GRANSAC::AbstractParameter>> &InputParams) override
		{
			if (InputParams.size() != MIN_INLIER_SIZE_RANSAC_PLANEFITTING)
				throw std::runtime_error("PlaneFittingModel - Number of input parameters does not match minimum number required for this model.");

			// Check for AbstractParamter types
			std::copy(InputParams.begin(), InputParams.end(), m_MinModelParams.begin());   
			geometry::Point3List points;
			for(int  i = 0 ; i != MIN_INLIER_SIZE_RANSAC_PLANEFITTING; ++i)
			{
				auto point3 = std::dynamic_pointer_cast<GPoint3f>(InputParams[i]);
				points.push_back(point3->p);
			}  
			auto result = geometry::FitPlane(points);
			n = std::get<0>(result);
			d = std::get<1>(result);
		};

		virtual std::pair<GRANSAC::VPFloat, std::vector<std::shared_ptr<GRANSAC::AbstractParameter>>> Evaluate(const std::vector<std::shared_ptr<GRANSAC::AbstractParameter>>& EvaluateParams, GRANSAC::VPFloat Threshold)
		{
			std::vector<std::shared_ptr<GRANSAC::AbstractParameter>> Inliers;
			int nTotalParams = EvaluateParams.size();
			int nInliers = 0;

			for (auto& Param : EvaluateParams)
			{
				if (ComputeDistanceMeasure(Param) < Threshold)
				{
					Inliers.push_back(Param);
					nInliers++;
				}
			}

			GRANSAC::VPFloat InlierFraction = GRANSAC::VPFloat(nInliers) / GRANSAC::VPFloat(nTotalParams); // This is the inlier fraction

			return std::make_pair(InlierFraction, Inliers);
		};
	};
}
}
#endif