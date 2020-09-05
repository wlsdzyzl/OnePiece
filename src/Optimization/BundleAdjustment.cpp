#include "BundleAdjustment.h"
#include "Tool/TickTock.h"
#define NO_SCHUR 0
#define INCREASE_SCALE 2
#define DECREASE_SCALE 0.7
#define ERROR_REDUCTION_THRESHOLD 0.5 //if error reduction less than 0.5, increase the lambda
namespace fucking_cool
{
namespace optimization
{
  
    geometry::MatrixX InverseOfSparse(Eigen::SparseMatrix<geometry::scalar> &s_mat)
    {
        Eigen::SimplicialLDLT<Eigen::SparseMatrix<geometry::scalar>> sldlt_solver;
        sldlt_solver.compute(s_mat);
        Eigen::SparseMatrix<geometry::scalar> eye(s_mat.rows(), s_mat.cols());
        eye.setIdentity();
        return sldlt_solver.solve(eye);
    }
    double ComputeError(geometry::Point3List &world_points, 
        std::vector<ProjectedPointsOnFrame> &projected_points, 
        std::vector<ReprojectedPoints> &reprojected_points,
        geometry::SE3List &poses,
        const camera::PinholeCamera &camera)
    {
            auto K = camera.ToCameraMatrix();
            double sum_error = 0.0;
            int count = 0;
            for(int i = 0; i != poses.size(); ++i)
            {
                //std::cout<<"projected_points: "<<i<<" "<<projected_points[i].size()<<std::endl;
                auto pose_inv = poses[i].inverse();
                for(auto iter = projected_points[i].begin(); iter != projected_points[i].end(); ++iter)
                {
                    int pid = iter->first;
                    geometry::Point2 projected_p = iter->second;
                    
                    geometry::Point3 transformed_p = geometry::TransformPoint(pose_inv, world_points[pid]);
                    reprojected_points[i][pid].first = transformed_p;
                    reprojected_points[i][pid].second = (K * (transformed_p / transformed_p(2))).head<2>();

                    count += 1;
                    sum_error += (projected_p - reprojected_points[i][pid].second).squaredNorm();
                }
            }
        return std::sqrt(sum_error / count);
    }
    geometry::scalar MaxDiagonalValue(const std::vector<Eigen::Triplet<geometry::scalar>> &coefficients)
    {
        geometry::scalar max_value = 0;
        for(int i = 0; i != coefficients.size(); ++i)
        {
            if(coefficients[i].col() == coefficients[i].row())
                max_value = std::max(max_value, std::fabs(coefficients[i].value()));
        }
        return max_value;

    }

    void BundleAdjustment(geometry::Point3List &world_points, 
        std::vector<ProjectedPointsOnFrame> &projected_points, 
        geometry::SE3List &poses,
        const camera::PinholeCamera &camera, int max_iteration)
    {
        //the jacobian matrix of JTJ's dimension is  count(world_points) + count(camera_poses)
        //see BA_revisited.pdf
        if(poses.size() < 2) 
        {
            std::cout<<BLUE<<"[INFO]::[BA]::No need to optimize."<<RESET<<std::endl;
            return;
        }

        tool::Timer timer;

        std::vector<Eigen::Triplet<geometry::scalar>> coefficients_pose, coefficients_point, coefficients_pp;
        coefficients_pose.reserve(world_points.size() * poses.size() * (6 * 6 ) + poses.size() * 6 );
        coefficients_point.reserve(world_points.size() * poses.size() * (3 * 3) + world_points.size() * 3 );
        coefficients_pp.reserve(world_points.size() * poses.size() * (6 * 3) );
        Eigen::SimplicialLDLT<Eigen::SparseMatrix<geometry::scalar>> sldlt_solver;
        int iteration = 0;
        double lambda = -1;

        // for(int i = 0; i != poses.size(); ++i)
        // {
        //     //std::cout<<"projected_points: "<<i<<" "<<projected_points[i].size()<<std::endl;
        //     std::cout<<"frame "<<i<<": "<<std::endl;
        //     std::cout<<poses[i]<<std::endl;
        //     for(auto iter = projected_points[i].begin(); iter != projected_points[i].end(); ++iter)
        //     {
        //         int pid = iter->first;
        //         std::cout<<pid<<" "<<iter->second.transpose()<<std::endl;
        //     }
        // }
        Eigen::SparseMatrix<geometry::scalar> U(6 * poses.size(), 6 * poses.size());
        Eigen::SparseMatrix<geometry::scalar> V(3 * world_points.size(), 3 * world_points.size());
        Eigen::SparseMatrix<geometry::scalar> W(6 * poses.size(), 3 * world_points.size());
        //JTr of all the poses and points
        geometry::VectorX r_pose, r_point;
        //delta we computed will be further used to update the world points and camera poses
        geometry::MatrixX delta_pose(6 * poses.size(), 1);
        geometry::MatrixX delta_point(3 * world_points.size(), 1);
        r_pose.resize(6 * poses.size() );
        r_point.resize(3 * world_points.size() );
        double reduction = 0.0;
        std::vector<ReprojectedPoints> reprojected_points;
        reprojected_points.resize(projected_points.size());
        double before_error = ComputeError(world_points, projected_points, reprojected_points, poses, camera);
        double after_error = 0;
#if DEBUG_MODE
        std::cout<<BLUE<<"[INFO]::[BA]::initial reprojection error 2d: "
            <<before_error<<" lambda: "<<lambda<<RESET<<std::endl;
#endif
        while(iteration < max_iteration)
        {

            while(true)
            {
                coefficients_pose.clear();
                coefficients_point.clear();
                coefficients_pp.clear();

                U.setZero();
                V.setZero();
                W.setZero();
                r_pose.setZero();
                r_point.setZero();     
           
                timer.TICK("Make Normal Equation");  

                for(int i = 0; i != poses.size(); ++i)
                {
                    //std::cout<<"projected_points: "<<i<<" "<<projected_points[i].size()<<std::endl;
                    for(auto iter = projected_points[i].begin(); iter != projected_points[i].end(); ++iter)
                    {
                        int pid = iter->first;
                        //if(pid >= 3) continue;
                        geometry::Point2 projected_p = iter->second;
                        geometry::Point2 reprojected_p = reprojected_points[i][pid].second;
                        geometry::MatrixX J_pose, J_point;
                        std::tie(J_pose, J_point) = 
                            ComputeJacobian(world_points[pid], reprojected_points[i][pid].first, poses[i], camera);
                        
                        geometry::Point2 residual = projected_p - reprojected_p;
                        // std::cout<<poses[i]<<std::endl;
                        // std::cout<<world_points[pid]<<std::endl;
                        // std::cout<<"p: "<<projected_p.transpose()<<" reprojected_p: "<<reprojected_p.transpose()<<" residual: "<<residual.transpose()<< std::endl;
                        if(i == 0) J_pose.setZero();
                        
                        auto JTJ_pose = J_pose.transpose() * J_pose;

                        auto JTJ_point = J_point.transpose() * J_point;
                        auto JTJ_pose_point = J_pose.transpose() * J_point;

                        // std::cout<<J_pose.transpose()<<std::endl;
                        // std::cout<<residual<<std::endl;
                        // std::cout<<std::endl;

                        geometry::VectorX JTr_pose = J_pose.transpose() * residual;
                        geometry::VectorX JTr_point = J_point.transpose() * residual;
                        
                        
                        geometry::AddToCoefficientTriplet(coefficients_pose, i * 6, i * 6, JTJ_pose);
                        geometry::AddToCoefficientTriplet(coefficients_point, pid * 3, pid * 3, JTJ_point);
                        geometry::AddToCoefficientTriplet(coefficients_pp, i * 6, pid * 3, JTJ_pose_point);            
                        r_pose.block<6, 1>(6 * i, 0) -= JTr_pose;
                        r_point.block<3, 1>(pid * 3, 0) -= JTr_point;
                    
                    }
                }

            

                int dimension_V = world_points.size() * 3;
                int dimension_U = poses.size() * 6;
                if(lambda < 0)
                {
                    lambda = std::fmax(MaxDiagonalValue(coefficients_pose), MaxDiagonalValue(coefficients_point)) * 1e-5;
                }
                for(int i = 0; i != dimension_U; ++i)
                {
                    coefficients_pose.push_back(Eigen::Triplet<geometry::scalar>(i, i, lambda));
                }

                for(int i = 0; i != dimension_V; ++i)
                {
                    coefficients_point.push_back(Eigen::Triplet<geometry::scalar>(i, i, lambda));
                }
                U.setFromTriplets(coefficients_pose.begin(), coefficients_pose.end());
                V.setFromTriplets(coefficients_point.begin(), coefficients_point.end());
                W.setFromTriplets(coefficients_pp.begin(), coefficients_pp.end());
                auto W_trans = W.transpose();

                
#if NO_SCHUR
                geometry::MatrixX JTJ_all(dimension_V + dimension_U, dimension_V + dimension_U);
                geometry::VectorX JTr_all(dimension_V + dimension_U);
                JTJ_all.setZero();
                JTr_all.setZero();

                JTJ_all.block(0, 0, dimension_U, dimension_U) = geometry::MatrixX(U);
                JTJ_all.block(dimension_U, dimension_U, dimension_V, dimension_V) = geometry::MatrixX(V);
                JTJ_all.block(dimension_U, 0, dimension_V, dimension_U) = geometry::MatrixX(W_trans);
                JTJ_all.block(0, dimension_U, dimension_U, dimension_V) = geometry::MatrixX(W);

                JTr_all.block(0, 0, dimension_U, 1) = geometry::VectorX(r_pose);
                JTr_all.block(dimension_U, 0, dimension_V, 1) = geometry::VectorX(r_point);

                // std::cout<<JTJ_all<<std::endl;
                // std::cout<<JTr_all<<std::endl;
                geometry::VectorX delta_all(dimension_V + dimension_U, 1);
                timer.TOCK("Make Normal Equation");
                timer.TICK("Solve Normal Equation");
                delta_all = JTJ_all.ldlt().solve(JTr_all);
                timer.TOCK("Solve Normal Equation");
                // std::cout<<delta_all.transpose()<<std::endl;
                delta_pose = delta_all.block(0, 0, dimension_U, 1);
                delta_point = delta_all.block(dimension_U, 0, dimension_V, 1);
#else
                //schur complement
                
                geometry::MatrixX V_inv = geometry::MatrixX(V);
                for(int i = 0; i != world_points.size(); ++i)
                {
                    V_inv.block(i*3, i*3, 3, 3) = V_inv.block(i*3, i*3, 3, 3).inverse();
                }
                // auto V_inv = InverseOfSparse(V);
                auto A = U - W * V_inv * W_trans;
                auto x = r_pose - W * V_inv * r_point;        
                timer.TOCK("Make Normal Equation");
                timer.TICK("Solve Normal Equation");
                sldlt_solver.compute(A);
                timer.TOCK("Solve Normal Equation");
                delta_pose = sldlt_solver.solve(x);
                delta_point = V_inv * (r_point -  W_trans * delta_pose);
#endif

                for(int i = 0; i != poses.size(); ++i)
                {
                    geometry::Se3 tmp_delta_pose = delta_pose.block<6, 1>(i * 6, 0);
                    poses[i] =   geometry::Se3ToSE3(tmp_delta_pose) * poses[i];
                }

                for(int i = 0; i != world_points.size(); ++i)
                {
                    geometry::Point3 tmp_delta_point = delta_point.block<3, 1>(i * 3, 0);
                    world_points[i] = world_points[i] + tmp_delta_point;
                }
                timer.TICK("Compute Error");
                double after_error = ComputeError(world_points, projected_points, reprojected_points, poses, camera);
                reduction = before_error - after_error;
                timer.TOCK("Compute Error");
                //modify lambda 
                //timer.LogAll();
                if(reduction < 0)
                {
                    //back to origin poses and position
                    for(int i = 0; i != poses.size(); ++i)
                    {
                        geometry::Se3 tmp_delta_pose = delta_pose.block<6, 1>(i * 6, 0);
                        poses[i] =   geometry::Se3ToSE3(tmp_delta_pose).inverse() * poses[i];
                    }

                    for(int i = 0; i != world_points.size(); ++i)
                    {
                        geometry::Point3 tmp_delta_point = delta_point.block<3, 1>(i * 3, 0);
                        world_points[i] = world_points[i] - tmp_delta_point;
                    }
                    before_error = ComputeError(world_points, projected_points, reprojected_points, poses, camera);
                    lambda *= INCREASE_SCALE;
                }
                else
                {
                    before_error = after_error;
#if DEBUG_MODE
                    std::cout<<BLUE<<"[INFO]::[BA]::iteration "<<iteration<<" reprojection error 2d: "
                        <<after_error<<" lambda: "<<lambda<<RESET<<std::endl;
#endif
                    lambda = (reduction > ERROR_REDUCTION_THRESHOLD) ? lambda * INCREASE_SCALE : lambda * DECREASE_SCALE;    
                    break;
                }
                        
            }

            if(reduction >= 0 && reduction < 1e-10 && after_error < 1e-6) break;
            iteration ++;
            
        }
        //exit(0);
    }
    std::tuple<geometry::MatrixX, geometry::MatrixX> 
        ComputeJacobian(const geometry::Point3 &world_point, const geometry::Point3 &transformed_p, 
        const geometry::SE3 &pose, const camera::PinholeCamera &camera)
    {
        //cost function: uij - (\pi(p_ij))
        

        float fx = camera.GetFx();
        float fy = camera.GetFy();
        auto R_trans = pose.block<3,3>(0,0).transpose();
        float inv_z = 1.0 / transformed_p(2);
        float inv_z_squared = 1 / (transformed_p(2) * transformed_p(2));
        // float x = transformed_p(0), x_squared = x * x;
        // float y = transformed_p(1), y_squared = y * y;
        //compute jacobian matrix for world point and camera
        geometry::MatrixX J_pose, J_point;
        
        J_pose.resize(2, 6);
        J_point.resize(2, 3);

        geometry::MatrixX J_1, J_2;
        J_1.resize(2, 3);
        J_2.resize(3, 6);
        J_1.setZero();
        J_2.setZero();
        J_1(0,0) = fx * inv_z;
        J_1(0,2) = - fx * transformed_p(0) * inv_z_squared;
        J_1(1,1) = fy * inv_z;
        J_1(1,2) = - fy * transformed_p(1) * inv_z_squared;

        J_2.block<3,3>(0,0) = -R_trans;
        J_2.block<3,3>(0,3) = geometry::GetSkewSymmetricMatrix(transformed_p);

        J_pose = -J_1 * J_2;
        J_point = -J_1 * R_trans;
        // J_pose << fx*inv_z, 0, -fx * x*inv_z_squared, -fx*x *y * inv_z_squared, fx*(1.0 + x_squared * inv_z_squared), -fx * y * inv_z,
        //         0, fy*inv_z, -fy*y*inv_z_squared, -fy * (1.0 + y_squared * inv_z_squared), fy * x * y * inv_z_squared, fy * x * inv_z;
        // J_point << fx * inv_z, 0.0, -fx*x * inv_z_squared,
        //         0.0, fy * inv_z, -fy*y * inv_z_squared;
        //J_point = J_point * pose_inv.block<3,3>(0,0);
        return std::make_tuple(J_pose, J_point);
    }
// std::tuple<geometry::MatrixX, geometry::MatrixX, geometry::Vector2> 
//         ComputeJacobianAndReprojection(const geometry::Point3 &world_point, 
//         const geometry::SE3 &pose, const camera::PinholeCamera &camera)
//     {
//         //cost function: uij - (\pi(p_ij))
        
//         auto K = camera.ToCameraMatrix();
//         float fx = camera.GetFx();
//         float fy = camera.GetFy();

//         geometry::Vector4 homo_p(world_point(0), world_point(1), world_point(2), 1);
//         geometry::Point3 transformed_p = (pose * homo_p).head<3>(); 
//         geometry::Point2 uv = (K * transformed_p / transformed_p(2)).head<2>();
//         float inv_z = 1.0 / transformed_p(2);
//         float inv_z_squared = 1 / (transformed_p(2) * transformed_p(2));
//         float x = transformed_p(0), x_squared = x * x;
//         float y = transformed_p(1), y_squared = y * y;
//         //compute jacobian matrix for world point and camera
//         geometry::MatrixX J_pose, J_point;
        
//         J_pose.resize(2, 6);
//         J_point.resize(2, 3);

//         geometry::MatrixX J_1, J_2;
//         J_1.resize(2, 3);
//         J_2.resize(3, 6);
//         J_1.setZero();
//         J_2.setZero();
//         J_1(0,0) = fx * inv_z;
//         J_1(0,2) = - fx * transformed_p(0) * inv_z_squared;
//         J_1(1,1) = fy * inv_z;
//         J_1(1,2) = - fy * transformed_p(1) * inv_z_squared;

//         J_2.block<3,3>(0,0) = geometry::Matrix3::Identity();
//         J_2.block<3,3>(0,3) = - geometry::GetSkewSymmetricMatrix(transformed_p);

//         J_pose = - J_1 * J_2;
//         J_point = - J_1 * pose.block<3,3>(0,0);
//         // J_pose << fx*inv_z, 0, -fx * x*inv_z_squared, -fx*x *y * inv_z_squared, fx*(1.0 + x_squared * inv_z_squared), -fx * y * inv_z,
//         //         0, fy*inv_z, -fy*y*inv_z_squared, -fy * (1.0 + y_squared * inv_z_squared), fy * x * y * inv_z_squared, fy * x * inv_z;
//         // J_point << fx * inv_z, 0.0, -fx*x * inv_z_squared,
//         //         0.0, fy * inv_z, -fy*y * inv_z_squared;
//         // J_point = J_point * pose_inv.block<3,3>(0,0);
//         return std::make_tuple(-J_pose, -J_point, uv);
//     }
}
}