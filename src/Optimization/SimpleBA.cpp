#include "SimpleBA.h"
#include <Eigen/Sparse>
namespace one_piece
{
namespace optimization
{
 
    float ComputeReprojectionError3D(const std::vector<Correspondence> &correspondences, const geometry::SE3List &camera_poses)
    {
        float sum_error = 0;
        //int sum_correspondences = 0;
        for(size_t i = 0; i != correspondences.size(); ++i)
        {
            sum_error += correspondences[i].ComputeReprojectionError3D(camera_poses);
        }
        return sum_error / correspondences.size();
    }
    std::tuple<geometry::Matrix6, geometry::Matrix6, geometry::Matrix6, geometry::Matrix6, geometry::Se3, geometry::Se3 >
        ComputeJTJAndJTr(const Correspondence &correspondence, geometry::SE3List &camera_poses)
    {
        geometry::SE3 source_pose = camera_poses[correspondence.source_id];
        geometry::SE3 target_pose = camera_poses[correspondence.target_id];
        //residual
        
        auto R1 = source_pose.block<3,3>(0,0);
        auto t1 = source_pose.block<3,1>(0,3);

        auto R2 = target_pose.block<3,3>(0,0);
        auto t2 = target_pose.block<3,1>(0,3);

        geometry::Matrix6 JTJ_ss, JTJ_st, JTJ_ts, JTJ_tt;
        geometry::Se3 JTr_s, JTr_t;

        JTJ_ss.setZero();
        JTJ_st.setZero();
        JTJ_ts.setZero();
        JTJ_tt.setZero();
        JTr_s.setZero();
        JTr_t.setZero();

        for(size_t i = 0; i != correspondence.correspondence_set.size(); ++i)
        {
            auto p1 = correspondence.correspondence_set[i].first;
            auto p2 = correspondence.correspondence_set[i].second;
            //residual
            geometry::Point3 r = (R1 * p1 + t1 - R2 * p2 - t2);
            geometry::Matrix3X6 J_source;
            geometry::Matrix3X6 J_target;

            J_source.block<3, 3>(0,0) = geometry::Matrix3::Identity();
            J_source.block<3, 3>(0,3) =  - geometry::GetSkewSymmetricMatrix( R1 * p1 + t1); 

            J_target.block<3, 3>(0,0) = - geometry::Matrix3::Identity();            
            J_target.block<3, 3>(0,3) =  geometry::GetSkewSymmetricMatrix( R2 * p2 + t2); 


            // J_target = - J_target;  
            // compute the JTJ and -JTr
            JTJ_ss += J_source.transpose() * J_source;
            JTJ_tt += J_target.transpose() * J_target;

            JTJ_st += J_source.transpose() * J_target;
            JTJ_ts += J_target.transpose() * J_source;

            JTr_s -= J_source.transpose() * r;
            JTr_t -= J_target.transpose() * r; 

        }
#if 0
        std::cout<<"JTJ_ss: "<<JTJ_ss<<std::endl;
        std::cout<<"JTJ_tt: "<<JTJ_tt<<std::endl;
        std::cout<<"JTJ_st: "<<JTJ_st<<std::endl;
        std::cout<<"JTJ_ts: "<<JTJ_ts<<std::endl;
        std::cout<<"JTr_s: "<<JTr_s<<std::endl;
        std::cout<<"JTr_t: "<<JTr_t<<std::endl;
#endif
        return std::make_tuple(JTJ_ss, JTJ_tt, JTJ_st, JTJ_ts, JTr_s, JTr_t);
    }

    void SimpleBA(const std::vector<Correspondence> &correspondences, geometry::SE3List &camera_poses, int max_iteration)
    {
        //formulate the problem as 
        //pose detla
        if(camera_poses.size() < 3)
        {
            std::cout<<BLUE<<"[INFO]::[SimpleBA]::Too few optimization variables, No need to optimize."<<RESET<<std::endl;
            return;
        }
        if(correspondences.size() < camera_poses.size() - 1)
        {
            std::cout<<RED<<"[ERROR]::[SimpleBA]::There are unconnected components."<<RESET<<std::endl;
            return;            
        }
        int variables_number = camera_poses.size() - 1;
        geometry::MatrixX pose_delta(6 * variables_number, 1), JTr(6 * variables_number, 1);
        Eigen::SparseMatrix<geometry::scalar> JTJ(6 * variables_number, 6 * variables_number);
        std::vector<Eigen::Triplet<geometry::scalar>> coefficients;
        //three iteration
#if DEBUG_MODE
        std::cout<<BLUE<<"before RMSE: "<<ComputeReprojectionError3D(correspondences, camera_poses) <<RESET<<std::endl;
#endif
        for(int iter = 0; iter != max_iteration; ++iter)
        {
            JTr.setZero();
            JTJ.setZero();
            coefficients.clear();
            
            //the number of coefficients will be smaller than 4 * 6 * 6 * |correspondence|
            //because each correspondence can bring 4 * Matrix6f(if the correspondence don't contains the first frame.)
            
            coefficients.reserve(correspondences.size() * 4 * 6 * 6);
            for(size_t i = 0; i != correspondences.size(); ++i)
            {
                geometry::Matrix6 JTJ_ss, JTJ_st, JTJ_ts, JTJ_tt;
                geometry::Se3 JTr_s, JTr_t;

                std::tie(JTJ_ss, JTJ_tt, JTJ_st, JTJ_ts, JTr_s, JTr_t) = 
                    ComputeJTJAndJTr(correspondences[i], camera_poses);
                //Put the Jacobian matrix into the Global JTJ and JTr, in right place.
                //note that the first camera pose will not be changed.
                int source_id = correspondences[i].source_id;
                int target_id = correspondences[i].target_id;

                if(source_id != 0)
                {
                    geometry::AddToCoefficientTriplet(coefficients, (source_id - 1 ) * 6, (source_id - 1 ) * 6, JTJ_ss);
                    geometry::AddToCoefficientTriplet(coefficients, (source_id - 1 ) * 6, (target_id - 1 ) * 6, JTJ_st);
                    geometry::AddToCoefficientTriplet(coefficients, (target_id - 1 ) * 6, (source_id - 1 ) * 6, JTJ_ts);

                    JTr.block<6, 1>((source_id - 1 ) * 6, 0) += JTr_s;
                }

                geometry::AddToCoefficientTriplet(coefficients, (target_id - 1 ) * 6, (target_id - 1 ) * 6, JTJ_tt);
                JTr.block<6, 1>((target_id - 1 ) * 6, 0) += JTr_t;
            }

            JTJ.setFromTriplets(coefficients.begin(), coefficients.end());
            //std::cout<<JTJ<<std::endl;
            Eigen::SimplicialLDLT<Eigen::SparseMatrix<geometry::scalar>> sldlt_solver;
            sldlt_solver.compute(JTJ);
            pose_delta = sldlt_solver.solve(JTr);

            for(size_t i = 1; i != camera_poses.size(); ++i)
            {
                //update the camera pose.
                geometry::Se3 delta_i = pose_delta.block<6,1>((i-1) * 6, 0);
                //Get Transformation Matrix
                //This may not be correct
                geometry::SE3 delta_i_SE3 = geometry::Se3ToSE3(delta_i);
                camera_poses[i] = delta_i_SE3 * camera_poses[i] ;
            }
        }
#if DEBUG_MODE
        std::cout<<BLUE<<"after RMSE: "<<ComputeReprojectionError3D(correspondences, camera_poses) <<RESET<<std::endl;
#endif        
    }
}
}