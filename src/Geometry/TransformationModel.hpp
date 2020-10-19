#ifndef TRANSFORMATION_MODEL
#define TRANSFORMATION_MODEL
#include "AbstractModel.hpp"
#include "Geometry.h"
#define MIN_INLIER_SIZE_RANSAC_TRANSFORMATION 8
namespace one_piece
{
namespace geometry
{

	typedef std::array<GRANSAC::VPFloat, 2> Vector2VP;

	class Point3fPair:
		public GRANSAC::AbstractParameter
	{
	public:
		Point3fPair(const geometry::Point3 &src, const geometry::Point3 &dst, int _id = -1)
		{
			_3d_pair.first = src;
			_3d_pair.second = dst;
			id = _id;
		}

		std::pair<geometry::Point3, geometry::Point3> _3d_pair;
		int id = -1;
	};
	//rigid
	class TransformationModel
		: public GRANSAC::AbstractModel<MIN_INLIER_SIZE_RANSAC_TRANSFORMATION>
	{
	protected:
		// Parametric form
		geometry::Matrix3 R;
		geometry::Point3 t;
		GRANSAC::VPFloat m_DistDenominator; // = sqrt(a^2 + b^2). Stored for efficiency reasons
		virtual GRANSAC::VPFloat ComputeDistanceMeasure(std::shared_ptr<GRANSAC::AbstractParameter> Param) override
		{
			auto pair_3d = std::dynamic_pointer_cast<Point3fPair>(Param);
			if (pair_3d == nullptr)
				std::cout<<"ERROR::3D RANSAC."<<std::endl;

			auto dist = R * pair_3d->_3d_pair.first + t - pair_3d->_3d_pair.second;
			
			float error = dist.norm();
			//// Debug
			//std::cout << "Point: " << ExtPoint2D->m_Point2D[0] << ", " << ExtPoint2D->m_Point2D[1] << std::endl;
			//std::cout << "Line: " << m_a << " x + " << m_b << " y + "  << m_c << std::endl;
			//std::cout << "Distance: " << Dist << std::endl << std::endl;

			return error;
		};

	public:
		TransformationModel(const std::vector<std::shared_ptr<GRANSAC::AbstractParameter>> &InputParams)
		{
			Initialize(InputParams);
		};

		virtual void Initialize(const std::vector<std::shared_ptr<GRANSAC::AbstractParameter>> &InputParams) override
		{
			if (InputParams.size() != MIN_INLIER_SIZE_RANSAC_TRANSFORMATION)
				throw std::runtime_error("TransformationModel - Number of input parameters does not match minimum number required for this model.");

			// Check for AbstractParamter types
			std::copy(InputParams.begin(), InputParams.end(), m_MinModelParams.begin());   
			std::vector<std::pair<geometry::Point3, geometry::Point3>> correspondences;
			for(int  i = 0 ; i != MIN_INLIER_SIZE_RANSAC_TRANSFORMATION; ++i)
			{
				auto point3 = std::dynamic_pointer_cast<Point3fPair>(InputParams[i]);
				correspondences.push_back(point3->_3d_pair);
			}  
			auto transformation = geometry::EstimateRigidTransformation(correspondences);
			R = transformation.block<3,3>(0,0);
			t = transformation.block<3,1>(0,3);
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