#ifndef GEOMETRY_H
#define GEOMETRY_H

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>

#include <vector>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include <Eigen/Eigenvalues> 
#include <Eigen/Eigen>
#include <sophus/se3.hpp>
#include "Tool/ConsoleColor.h"
#define DEBUG_MODE 1

namespace fucking_cool 
{
// if you want to use a class A which has been used in current class B
// just declare the class A at current B.h file, and import the A.h file in B.cpp 
namespace camera
{
    class PinholeCamera;
}
namespace geometry { 
    typedef float scalar;
    typedef Eigen::Matrix<scalar, 2, 1> Vector2;
    typedef Eigen::Matrix<scalar, 3, 1> Vector3;
    typedef Eigen::Matrix<scalar, 4, 1> Vector4;
    typedef Eigen::Matrix<scalar, 6, 1> Vector6;

    typedef Eigen::Matrix<scalar, 2, 2> Matrix2;
    typedef Eigen::Matrix<scalar, 3, 3> Matrix3;
    typedef Eigen::Matrix<scalar, 4, 4> Matrix4;
    typedef Eigen::Matrix<scalar, 6, 6> Matrix6;
    typedef Eigen::Matrix<scalar, 3, 6> Matrix3X6;

    typedef Eigen::Matrix<scalar, Eigen::Dynamic, 1> VectorX;
    typedef Eigen::Matrix<scalar, Eigen::Dynamic, Eigen::Dynamic> MatrixX;

    typedef Matrix4 TransformationMatrix;
    typedef Vector3 Point3;
    typedef Vector2 Point2;
    typedef Eigen::Matrix<int, 2, 1> Point2i;
    typedef Eigen::Matrix<int, 3, 1> Point3i;
    typedef Eigen::Matrix<unsigned int, 2, 1> Point2ui;
    typedef Eigen::Matrix<unsigned int, 3, 1> Point3ui;
    //cpp 11 support alias
    template <int T>
        using Vector = Eigen::Matrix<scalar, T, 1>;
    
    // typedef cv::Vec<geometry::scalar,3> Point3CV;
    // typedef cv::Vec<geometry::scalar,2> Point2CV;
    typedef std::pair<Point3, Point3> PointCorrespondence;
    typedef std::vector<PointCorrespondence> PointCorrespondenceSet;
    typedef std::vector<cv::KeyPoint> KeyPointSet;
    typedef std::vector<cv::DMatch> DMatchSet;
    typedef std::pair<int, int> FMatch;
    typedef std::vector<FMatch> FMatchSet;
    typedef cv::Mat Descriptor;
    typedef std::vector<std::pair<Point2ui, Point2ui>> PixelCorrespondenceSet;
    typedef std::vector<Point2, Eigen::aligned_allocator<Point2> > Point2List;
    typedef std::vector<Point3, Eigen::aligned_allocator<Point3> > Point3List;
    typedef std::vector<Point3i, Eigen::aligned_allocator<Point3i> > Point3iList;
    typedef std::vector<Point3ui, Eigen::aligned_allocator<Point3ui> > Point3uiList;
    typedef std::vector<VectorX, Eigen::aligned_allocator<VectorX> > PointXList;
    template <int T>
        using PointList = std::vector<Eigen::Matrix<scalar, T, 1>, 
            Eigen::aligned_allocator<Eigen::Matrix<scalar, T, 1>>>;
    typedef std::vector<Matrix4, Eigen::aligned_allocator<Matrix4> > Mat4List;
    
    typedef std::vector<Point3List> ImageXYZ;
    void TransformToMatXYZ(const cv::Mat &image, const camera::PinholeCamera &camera, geometry::ImageXYZ &imageXYZ);
    //lie group and lie algebra
    //SE3 and se3
    typedef Vector6 Se3;
    typedef Vector3 So3;
    typedef Vector4 Plane;
    typedef Matrix4 SE3;
    typedef Matrix3 SO3;
    typedef Mat4List SE3List;

    // Matrix4 TransformVector6fToMatrix4f(const geometry::Se3 &input);
    // geometry::Se3 TransformMatrix4fToVector6f(const Matrix4 &input);

    Matrix4 Se3ToSE3(const Vector6 &input);
    Vector6 SE3ToSe3(const Matrix4 &input);
    void TransformPoints(const Matrix4 &T, Point3List &points);
    Point3 TransformPoint(const Matrix4 &T, const Point3 &point);
    void TransformNormals(const Matrix4 &T, Point3List &normals);
    double ComputeReprojectionError3D(const PointCorrespondenceSet & correspondence_set,
        const SE3 &camera_pose);

    struct VoxelGridHasher
    {
            // Three large primes are used for spatial hashing.
            static constexpr size_t p1 = 73856093;
            static constexpr size_t p2 = 19349663;
            static constexpr size_t p3 = 83492791;

            std::size_t operator()(const Point3i& key) const
            {
                return ( key(0) * p1 ^ key(1) * p2 ^ key(2) * p3);
            }
    };
    struct PixelGridHasher
    {
            // Three large primes are used for spatial hashing.
            static constexpr size_t p1 = 73856093;
            static constexpr size_t p2 = 19349663;

            std::size_t operator()(const Point2i& key) const
            {
                return ( key(0) * p1 ^ key(1) * p2 );
            }
    };

    TransformationMatrix RandomTransformation();
    Plane GetPlane(const Point3 &p1, const Point3 &p2, const Point3 &p3);
    std::tuple<Point3, double ,double> FitPlane(const Point3List & _points);
    std::tuple<Vector2, double ,double> FitLine(const Point2List & _points);
    TransformationMatrix EstimateRigidTransformation(const  PointCorrespondenceSet & correspondence_set);
    Matrix3 GetSkewSymmetricMatrix(Point3 t);
    void AddToCoefficientTriplet(std::vector<Eigen::Triplet<scalar>> &coefficients,
        int start_row, int start_col, const geometry::MatrixX &JTJ);
    
};
}
#endif