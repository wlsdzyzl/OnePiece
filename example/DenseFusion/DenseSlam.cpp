#include "DenseSlam.h"
#include "Registration/ICP.h"
#define MAX_ICP_CORRESPONDENCE 500
namespace fucking_cool
{


    void DenseSlam::UpdateFrame(const geometry::RGBDFrame &frame)
    {
        global_frames.push_back(frame);
        auto &current_frame = global_frames.back();
        current_frame.frame_id = global_frames.size() - 1;
        std::cout <<"Process on "<<current_frame.frame_id <<"th image"<<std::endl;

        geometry::TransformationMatrix T = geometry::TransformationMatrix::Identity();
        int tracking_success = 1;
        global_poses.push_back(geometry::SE3::Identity());
        relative_poses.push_back(geometry::SE3::Identity());
        if(current_frame.frame_id > 0)
        {
            //use dense tracking to track the last frame
            auto tracking_result = rgbd_odometry.DenseTracking(global_frames[last_tracking_frame_id], current_frame, T);
            
            float rmse = tracking_result->rmse;
            std::cout << "rmse: " << rmse << std::endl;
            tracking_success = tracking_result-> tracking_success && (rmse < max_reprojection_error_3d);
            if(tracking_success)
            {
                current_frame.tracking_success = true;
                global_poses[current_frame.frame_id] = global_poses[last_tracking_frame_id] * tracking_result->T.inverse();
            }
        }

        if(tracking_success)
        {
            
            last_tracking_frame_id = current_frame.frame_id;
            if(new_submap_flag == true)
            {
                Submap new_submap(submaps.size());
                
                new_submap.containted_frames.push_back(current_frame.frame_id);
                submaps.push_back(new_submap);
                submap_poses.push_back(global_poses.back());
                //relative_poses[current_frame.frame_id] = geometry::SE3::Identity(); 
                new_submap_flag = false;
                current_submap_id += 1;
            }        
            relative_poses[current_frame.frame_id] = submap_poses[current_submap_id].inverse() * global_poses.back();
            submaps[current_submap_id].containted_frames.push_back(current_frame.frame_id);
            if(submaps[current_submap_id].containted_frames.size() % step == 0)
            {
                new_submap_flag = true;
                //register this submap
                //matches all the previous submaps to generate global correspondences
                
                // submaps[current_submap_id].pcd.WriteToPLY("submap_pcd_"+std::to_string(current_submap_id)
                //     +".ply");
                visible_pcd_updated = true;
                RegisterSubmap(current_submap_id);
                // if(current_submap_id > 0)
                // ReleaseSubmapFrame(current_submap_id-1);
                Optimize();
            }
            std::cout << "tracking successful!"<<std::endl;
        }
        
    }
    void DenseSlam::RegisterSubmap(int submap_id)
    {
        
        registration::ICPParameter icp_para;
        icp_para.max_iteration = 1;
        auto pcd_ptr = submaps[submap_id].GenerateSubmapModel(global_frames, relative_poses, camera);
        std::tie(submaps[submap_id].downsampled_pcd, submaps[submap_id].features) = 
            registration::DownSampleAndExtractFeature(*pcd_ptr, r_para);
        auto &pcd = submaps[submap_id].downsampled_pcd;
        auto &features = submaps[submap_id].features;
        for(size_t i = 0; i != submaps.size(); ++i)
        {
            //same submap or last submap
            if(submaps[i].submap_id == submap_id ) continue;

            std::cout << "Matching "<<submaps[i].submap_id<<" ..."<<std::endl;
            auto &previous_pcd = submaps[i].downsampled_pcd;
            
            if(submaps[i].submap_id == submap_id - 1)
            {
                //just use the relative pose to get some correspondences
                geometry::SE3 init_T = submap_poses[submap_id].inverse() * submap_poses[submap_id - 1];
                auto icp_result = registration::PointToPoint(previous_pcd, pcd, init_T, icp_para);
                optimization::Correspondence submap_correspondence(submaps[i].submap_id, submap_id);
                size_t tmp_step = 1;
                if(icp_result->correspondence_set.size() > MAX_ICP_CORRESPONDENCE)
                tmp_step = icp_result->correspondence_set.size() / MAX_ICP_CORRESPONDENCE;
                size_t ptr = 0;
                for(size_t j = 0; j < icp_result->correspondence_set.size(); j += tmp_step, ptr += 1)
                {
                    icp_result->correspondence_set[ptr] = icp_result->correspondence_set[j];
                }
                icp_result->correspondence_set.resize(ptr);
                submap_correspondence.correspondence_set = icp_result->correspondence_set;
                global_submap_correspondences.push_back(submap_correspondence);                
                continue;
            }            
            auto &previous_features = submaps[i].features;
            auto register_result = registration::RansacRegistration(previous_pcd, pcd, 
                previous_features, features, r_para);
            if(register_result->rmse < max_reprojection_error_3d_ransac && register_result->correspondence_set.size() > min_inliers_ransac)
            {
                std::cout << "Match "<<submaps[i].submap_id<<" successfully!"<<std::endl;
                optimization::Correspondence submap_correspondence(submaps[i].submap_id, submap_id);
                submap_correspondence.correspondence_set = register_result->correspondence_set;
                global_submap_correspondences.push_back(submap_correspondence);
            }

        }
        submaps[submap_id].is_registered = true;
    }
    void DenseSlam::Optimize()
    {
        
        optimizer.FastBA(global_submap_correspondences, submap_poses);
        UpdateAllPoses();
    }

}