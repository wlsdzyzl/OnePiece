#include "Geometry.h"
#include "Camera/Camera.h"
#include <random>
namespace one_piece
{
namespace geometry 
{

    Matrix4 Se3ToSE3(const Vector6 &input)
    {
        auto tmp = Sophus::SE3Group<scalar>::exp(input);
        return tmp.matrix();
    }
    Vector6 SE3ToSe3(const Matrix4 &input)
    {
        Sophus::SE3Group<scalar>  tmp(input);
        return tmp.log(); 
    }
    void TransformPoints(const Matrix4 &T, Point3List &points)
    {
        for (auto& point : points) 
        {
            Vector4 new_point =
                T *Vector4(point(0), point(1), point(2), 1.0);
            point = new_point.head<3>() / new_point(3);
        }
    }

    Point3 TransformPoint(const Matrix4 &T, const Point3 &point)
    {
            Vector4 new_point =
                T *Vector4(point(0), point(1), point(2), 1.0);
            return new_point.head<3>() / new_point(3);        
    }
    void AddToCoefficientTriplet(std::vector<Eigen::Triplet<scalar>> &coefficients,
        int start_row, int start_col, const MatrixX &JTJ)
    {
        int rows = JTJ.rows();
        int cols = JTJ.cols();
        for(int i = 0; i != rows; ++i)
        {
            for(int j = 0; j != cols; ++j)
            {
                coefficients.push_back(Eigen::Triplet<scalar>(i + start_row, j + start_col, JTJ(i,j)));
            }
        }
    }
    double ComputeReprojectionError3D(const PointCorrespondenceSet & correspondence_set,
        const SE3 &camera_pose)
    {
        //camera pose from source to target.
        double sum_error = 0.0;
        for(size_t j = 0; j != correspondence_set.size(); ++j)
        {
            sum_error += 
                (TransformPoint(camera_pose,
                    correspondence_set[j].first) - 
                    correspondence_set[j].second).squaredNorm();
        }        
        return sqrt(sum_error / correspondence_set.size());
    }  
    void TransformNormals(const Matrix4 &T, Point3List &normals)
    {
        for (auto& normal : normals) 
        {
            Vector4 new_normal =
                T *Vector4(normal(0), normal(1), normal(2), 0.0);
            normal = new_normal.head<3>();
        }
    }
    //transform a depth32f image into xyz image
    void TransformToMatXYZ(const cv::Mat &image, const camera::PinholeCamera &camera, geometry::ImageXYZ &imageXYZ)
    {

        int width = image.cols;
        int height = image.rows;
        imageXYZ.clear();
        if(image.depth() != CV_32FC1)
        {
            std::cout<<RED<<"[ERROR]::[TransformToMatXYZ] Only supports CV_32FC1. Please do the conversion."<<RESET<<std::endl;
            return;
        }
        imageXYZ.resize(height);
        float fx = camera.GetFx(), fy = camera.GetFy(), cx = camera.GetCx(), cy = camera.GetCy();
        for(int i = 0;i!= height; ++i)
        {
            imageXYZ[i].resize(width);
            for(int j = 0; j!=width; ++j)
            {
                float  z;
                z = image.at<float>(i, j) ;
                if (z > 0) 
                {
                    float x = (j - cx) * z / fx;
                    float y =
                            (i - cy) * z / fy;

                    imageXYZ[i][j] = geometry::Point3(x,y,z);
                }
                else imageXYZ[i][j] =geometry::Point3(-1,-1,-1);
            }
        }
        //geometry::PointCloud pcd;
        //pcd.LoadFromXYZ(imageXYZ);
        //pcd.WriteToPLY("FUCKFUCK.ply");
    }
    TransformationMatrix EstimateRigidTransformation(const  PointCorrespondenceSet & correspondence_set)
    {
        MatrixX origin_points, target_points;
        TransformationMatrix T;
        //AT=B
        //ICP algorithm, Compute R, then compute t.
        Matrix3 W;
        Matrix3 R;
        Point3 t;
        Point3 mean_source, mean_target;
        mean_source.setZero();
        mean_target.setZero();
        W.setZero();
        T.setZero();
        
        for(size_t i = 0;i!=correspondence_set.size();++i)
        {

            mean_source += correspondence_set[i].first;
            mean_target += correspondence_set[i].second;
        }            
        mean_source /= correspondence_set.size();
        mean_target /= correspondence_set.size();
        for(size_t i = 0;i!=correspondence_set.size();++i)
        {
            W+=(correspondence_set[i].first - mean_source) * (correspondence_set[i].second - mean_target).transpose();
        }
        Eigen::JacobiSVD<MatrixX> svd(W, Eigen::ComputeThinU | Eigen::ComputeThinV);
        auto UT = svd.matrixU().transpose();
        auto V = svd.matrixV();

        R = V*UT;
        if (R.determinant() < 0)
        {
          V(0, 2) = -V(0, 2);
          V(1, 2) = -V(1, 2);
          V(2, 2) = -V(2, 2);
          R = V * UT;
        }
        t = mean_target - R * mean_source;
        T.block<3,3>(0,0) = R;
        T.block<3,1>(0,3) = t;
        T(3,3) = 1;
        return T;
    }
    TransformationMatrix RandomTransformation()
    {
        std::random_device rd;
        std::default_random_engine e(rd());
        std::uniform_real_distribution<double> u(-1.0, 1.0);
        double a, b, c;
        a = u(e);
        b = u(e);
        c = u(e);
        Se3 se3 = Se3::Zero();
        se3(0) = a; se3(1) = b; se3(2) = c;
        return Se3ToSE3(se3);
    }
    Plane GetPlane(const Point3 &p1, const Point3 &p2, const Point3 &p3)
    {
        Point3 normal = (p2 - p1).cross(p3 - p1);
        normal.normalize();
        double d = -p1.dot(normal);
        return Plane(normal(0), normal(1), normal(2), d);
    }
    std::tuple<Point3, double , double> FitPlane(const Point3List & _points)
    {
        if(_points.size() < 3)
        {
            std::cout<<YELLOW<< "[FitPlane]::[WARNING]::The Number of points is less than 3."<<RESET << std::endl;
            return std::make_tuple(Point3(0,0,0),0,0);
        }
        Point3 mean_point;
        Point3 sum_point;
        sum_point.setZero();
        for(size_t i = 0;i < _points.size(); ++i)
        {
            sum_point+=_points[i];
        }
        mean_point = sum_point / _points.size();
        Matrix3 W, W_tmp;
        W.setZero();
        for(size_t i = 0; i!= _points.size(); ++i)
        {
            W += (_points[i] - mean_point)* (_points[i] - mean_point).transpose();
        }
        W = W / _points.size();
        Eigen::JacobiSVD<MatrixX> svd(W, Eigen::ComputeThinU | Eigen::ComputeThinV);
        auto U = svd.matrixU();
        auto singular_values = svd.singularValues();
        Point3 normal = U.block<3,1>(0,2); 
        //std::cout<<U<<std::endl;   
        normal.normalize();    

/*
        Point3 temp = (_points[0] - mean_point);
        temp.normalize();
        if(temp.dot(normal) >= 0 )
        normal = -normal;
*/
        double d = - mean_point.transpose() * normal;
        
        /*
        double residual = 0;
        
        for(size_t i = 0; i!= _points.size(); ++i)
        {
            residual = residual +  std::fabs((_points[i].transpose() * normal)(0) + d );
        }*/
        double indicator = singular_values(2) / singular_values(1);
        return std::make_tuple(normal,d, indicator);
    }

    std::tuple<Vector2, double, double > FitLine(const Point2List & _points)
    {
        if(_points.size() < 2)
        {
            std::cout<<YELLOW<< "[FitLine]::[WARNING]::The Number of points is less than 2."<<RESET << std::endl;
            return std::make_tuple(Vector2(0,0),0,0);
        }
        Vector2 mean_point;
        Vector2 sum_point;
        sum_point.setZero();
        for(size_t i = 0;i < _points.size(); ++i)
        {
            sum_point+=_points[i];
        }
        mean_point = sum_point / _points.size();
        Matrix2 W, W_tmp;
        W.setZero();
        for(size_t i = 0; i!= _points.size(); ++i)
        {
            W += (_points[i] - mean_point)* (_points[i] - mean_point).transpose();
        }
        W = W / _points.size();
        Eigen::JacobiSVD<MatrixX> svd(W, Eigen::ComputeThinU | Eigen::ComputeThinV);
        auto U = svd.matrixU();
        auto singular_values = svd.singularValues();
        Vector2 normal = U.block<2,1>(0,1); 
        //std::cout<<singular_values(0) << " "<<singular_values(1)<<std::endl;   
        normal.normalize();    
        double d = - mean_point.transpose() * normal;
        /*
        double residual = 0;
        for(size_t i = 0; i!= _points.size(); ++i)
        {
            
            residual = residual +  std::fabs((_points[i].transpose() * normal)(0) + d );
        }
        residual /= _points.size();
        */
        //indicator of the curve, noise, and residual.
        double indicator = singular_values(1)/singular_values(0);
        //std::cout<<residual<<" "<<indicator<<std::endl;
        return std::make_tuple(normal,d, indicator);
    }
    //each 3d vector have a skew symmetric matrix
    Matrix3 GetSkewSymmetricMatrix(Point3 t)
    {
        Matrix3 t_hat;
        t_hat << 0, -t(2), t(1),
                t(2), 0, -t(0),
                -t(1), t(0), 0;
        return t_hat;        
    }
}
}