#include "FBASlam.h"
namespace one_piece
{

    void FBASlam::UpdateFrame(const geometry::RGBDFrame &frame)
    {
        global_frames.push_back(frame);
        auto &current_frame = global_frames.back();
        current_frame.frame_id = global_frames.size() - 1;
        int frame_id = current_frame.frame_id;
        std::cout <<"Process on "<<current_frame.frame_id <<"th image"<<std::endl;

        geometry::TransformationMatrix T = geometry::TransformationMatrix::Identity();
        geometry::PointCorrespondenceSet PCS;
        bool tracking_success = false;
        int last_keyframe_id = -1;

        if(keyframe_ids.size() > 0) last_keyframe_id = keyframe_ids.back();
        optimization::Correspondence corr(last_keyframe_id, frame_id);
        if(frame_id != 0)
        {
            //tracking
            auto tracking_result = rgbd_odometry.SparseTrackingMILD(global_frames[last_keyframe_id], 
                current_frame);
            T = tracking_result->T;
            tracking_success = tracking_result->tracking_success;
            if(tracking_success) 
            {
                //double check through reprojection_error

                corr.correspondence_set = tracking_result->correspondence_set;
                corr.CalculateAverageDisparity(camera);
                float reprojection_error = tracking_result->rmse;
                if(reprojection_error < max_reprojection_error_3d)
                    current_frame.tracking_success = true;
                if(corr.average_disparity >= max_average_disparity)
                    current_frame.keyframe_kid = keyframe_ids.size();
                // if(frame_id % 7 == 0)
                //     current_frame.keyframe_kid = keyframe_ids.size();
                std::cout<<reprojection_error<<std::endl;
            }
            relative_poses.push_back(T);
            global_poses.push_back(global_poses[last_keyframe_id] * relative_poses.back().inverse());
        }
        else
        {
            current_frame.tracking_success = true;
            current_frame.keyframe_kid = 0;
            relative_poses.push_back(geometry::TransformationMatrix::Identity());
            global_poses.push_back(geometry::TransformationMatrix::Identity());

        }
        //update keyframe and optimization
        if(current_frame.keyframe_kid != -1)
        {
            std::cout<<"Keyframe registration. frame id: "<<current_frame.frame_id<<std::endl;

            current_frame.PrepareDownSamplePointCloud(camera, voxel_len);
            std::vector<int> candidates;
            if(frame_id != 0)
            {
                //the relative poses of keyframe is identity
                relative_poses[frame_id] = geometry::TransformationMatrix::Identity();
                if(current_frame.tracking_success )
                {
                    // std::cout<<corr.source_id<<" "<<corr.target_id<<std::endl;
                    corr.source_id = global_frames[corr.source_id].keyframe_kid;
                    corr.target_id = global_frames[corr.target_id].keyframe_kid;

                    global_correspondences.push_back(corr);

                }
                mild_lcd.SelectCandidates(current_frame, candidates);
                std::cout<<"Candidate frame id:{";
                for(size_t i = 0; i < candidates.size(); ++i)
                {
                    std::cout<<" "<<keyframe_ids[candidates[i]];
                }
                std::cout<<"}"<<std::endl;

                for(size_t i = 0; i != candidates.size(); ++i)
                {
                    //last keyframe has been matched
                    geometry::TransformationMatrix loop_T = geometry::TransformationMatrix::Identity();
                    geometry::PointCorrespondenceSet loop_PCS;
                    int candidate_frame_id = keyframe_ids[candidates[i]];
                    if(candidates[i] == static_cast<int>(keyframe_ids.size() - 1)) continue;
                    auto tracking_result  = 
                        rgbd_odometry.SparseTrackingMILD(global_frames[candidate_frame_id], 
                        current_frame);
                    loop_T = tracking_result->T;
                    loop_PCS = tracking_result->correspondence_set;
                    bool success_matched = tracking_result->tracking_success;
                    if(success_matched)
                    {
                        optimization::Correspondence loop_corr(candidate_frame_id, frame_id);
                        loop_corr.correspondence_set = loop_PCS;
                        float loop_reprojection_error = tracking_result->rmse;
                        if(loop_reprojection_error < max_reprojection_error_3d)
                        success_matched = true;
                        else success_matched = false;

                        if(!current_frame.tracking_success && success_matched)
                        {
                            //use another previous keyframe to locate
                            global_poses[frame_id] = global_poses[candidate_frame_id] * loop_T.inverse();
                            current_frame.tracking_success = true;
                        }
                        if(success_matched)
                        {
                            loop_corr.source_id = global_frames[loop_corr.source_id].keyframe_kid;
                            loop_corr.target_id = global_frames[loop_corr.target_id].keyframe_kid;
                            global_correspondences.push_back(loop_corr);
                        }
                        else
                        std::cout << "Reject frame "<<candidate_frame_id<<std::endl; 
                    }
                    else
                    {
                        std::cout << "Reject frame "<<candidate_frame_id<<std::endl;
                    }
                    
                }
                if(!current_frame.tracking_success)
                {
                    std::cout<<"Keyframe match failed!"<<std::endl;
                    exit(1);
                }
            }

            mild_lcd.Insert(current_frame);
            keyframe_ids.push_back(frame_id);
            global_keyframe_poses.push_back(global_poses.back());
            //after add keyframe, we optimize the frame poses
            Optimize();
        }
        //else current_frame.Release();

    }
    void FBASlam::Optimize()
    {
        optimizer.FastBA(global_correspondences, global_keyframe_poses);
        for(size_t i = 0; i != keyframe_ids.size(); ++i)
        {
            global_poses[keyframe_ids[i]] = global_keyframe_poses[i];
        }
    }

}