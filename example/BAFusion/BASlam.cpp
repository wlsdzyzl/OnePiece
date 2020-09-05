#include "BASlam.h"
#define MAX_MATCHES 30
namespace fucking_cool
{

    void BASlam::UpdateFrame(const geometry::RGBDFrame &frame)
    {
        auto K = camera.ToCameraMatrix();
        global_frames.push_back(frame);
        auto &current_frame = global_frames.back();
        current_frame.frame_id = global_frames.size() - 1;
        int frame_id = current_frame.frame_id;
        std::cout <<"Process on "<<current_frame.frame_id <<"th image"<<std::endl;

        geometry::TransformationMatrix T = geometry::TransformationMatrix::Identity();
        geometry::FMatchSet matches;
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
            if(tracking_result->correspondence_set.size() > MAX_MATCHES)
            {
                tracking_result->correspondence_set.resize(MAX_MATCHES);
                tracking_result->correspondence_set_index.resize(MAX_MATCHES);
            }
            T = tracking_result->T;
            tracking_success = tracking_result->tracking_success;
            matches = tracking_result->correspondence_set_index;
            PCS = tracking_result->correspondence_set;
            if(tracking_success) 
            {
                //double check through reprojection_error

                corr.correspondence_set = PCS;
                corr.CalculateAverageDisparity(camera);
                float reprojection_error = tracking_result->rmse;
                if(reprojection_error < max_reprojection_error_3d)
                    current_frame.tracking_success = true;
                if(corr.average_disparity >= max_average_disparity)
                    current_frame.keyframe_kid = keyframe_ids.size();
                // if(frame_id % 7 == 0)
                //     current_frame.keyframe_kid = keyframe_ids.size();
                //std::cout<<reprojection_error<<std::endl;
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
            projected_points.push_back(optimization::ProjectedPointsOnFrame());
            points_local_to_global.push_back(std::vector<int>());
            current_frame.PrepareDownSamplePointCloud(camera, voxel_len);
            std::vector<int> candidates;
            if(frame_id != 0)
            {
                
                if(last_keyframe_id == 0)
                {
                    points_local_to_global[0].resize(global_frames[0].feature_pcd.points.size(), -1);
                }
                
                points_local_to_global.back().resize(current_frame.feature_pcd.points.size(), -1);
                //the relative poses of keyframe is identity
                relative_poses[frame_id] = geometry::TransformationMatrix::Identity();

                if(current_frame.tracking_success )
                {
                    // std::cout<<corr.source_id<<" "<<corr.target_id<<std::endl;
                    int last_kid = global_frames[last_keyframe_id].keyframe_kid;
                    //std::cout<<"matches: "<<matches.size()<<" "<<PCS.size()<<std::endl;
                    std::cout<<last_keyframe_id<<" "<<global_keyframe_poses[last_kid]<<std::endl;
                    for(int i = 0; i < matches.size(); ++i)
                    {
                        int source_local_id = matches[i].first;
                        int target_local_id = matches[i].second;
                        int &source_global_id = points_local_to_global[last_kid][source_local_id];
                        int &target_global_id = (points_local_to_global.back())[target_local_id];

                        auto &source_points = global_frames[last_keyframe_id].feature_pcd.points;
                        // std::cout<<GREEN<<"source_global_id: "<<source_global_id<<
                        //     " target_global_id: "<<target_global_id<<RESET<<std::endl;
                        geometry::Point2 source_uv;
                        source_uv << global_frames[last_keyframe_id].keypoints[source_local_id].pt.x, 
                            global_frames[last_keyframe_id].keypoints[source_local_id].pt.y;

                        geometry::Point2 target_uv;
                        target_uv << current_frame.keypoints[target_local_id].pt.x, 
                            current_frame.keypoints[target_local_id].pt.y;

                            
                        if(source_global_id == -1 && target_global_id == -1)
                        {
                            //this means the point has not been added into the world points yet.
                            
                            world_points.push_back(geometry::TransformPoint(global_keyframe_poses[last_kid], 
                                        source_points[source_local_id]));
                            // if(std::isnan(world_points.back()(0)))
                            // {
                            //     std::cout<<"NAN!"<<std::endl;
                            //     std::cout<<global_keyframe_poses[last_kid]<<std::endl;
                            //     std::cout<<source_points[source_local_id].transpose()<<std::endl;
                            // }
                            source_global_id = world_points.size() -1;
                            target_global_id = world_points.size() -1;
                            //2d coordinate

                            projected_points[last_kid][source_global_id] = source_uv;
                            (projected_points.back())[target_global_id] = target_uv;
                        }
                        else if(target_global_id == -1)
                        {

                            target_global_id = source_global_id;
                            (projected_points.back())[target_global_id] = target_uv;
                        }
                        else if(source_global_id == -1)
                        {
                            source_global_id = target_global_id;
                            projected_points[last_kid][source_global_id] = source_uv;
                        }
                        else
                        {
                            // std::cout<<YELLOW<<"source_global_id: "<<source_global_id<<
                            //     " target_global_id: "<<target_global_id<<RESET<<std::endl;
                            // std::cout<<YELLOW<<"source_global_point: "<<world_points[source_global_id]<<
                            //     " target_global_point: "<<world_points[target_global_id]<<RESET<<std::endl;
                            projected_points.back()[source_global_id] = target_uv;
                            projected_points[last_kid][target_global_id] = source_uv;
                        }
                        
                        
                    }
                }
                mild_lcd.SelectCandidates(current_frame, candidates);
                std::cout<<"Candidate frame id:{";
                for(int i = 0; i < candidates.size(); ++i)
                {
                    std::cout<<" "<<keyframe_ids[candidates[i]];
                }
                std::cout<<"}"<<std::endl;

                for(int i = 0; i != candidates.size(); ++i)
                {
                    //last keyframe has been matched
                    geometry::TransformationMatrix loop_T = geometry::TransformationMatrix::Identity();
                    geometry::PointCorrespondenceSet loop_PCS;
                    geometry::FMatchSet loop_matches;
                    bool success_matched;
                    int candidate_frame_id = keyframe_ids[candidates[i]];
                    if(candidates[i] == keyframe_ids.size() - 1) continue;
                    auto tracking_result = 
                        rgbd_odometry.SparseTrackingMILD(global_frames[candidate_frame_id], 
                        current_frame);
                    if(tracking_result->correspondence_set.size() > MAX_MATCHES)
                    {
                        tracking_result->correspondence_set.resize(MAX_MATCHES);
                        tracking_result->correspondence_set_index.resize(MAX_MATCHES);
                    }
                    success_matched = tracking_result->tracking_success;
                    loop_T = tracking_result->T;
                    loop_matches = tracking_result->correspondence_set_index;
                    loop_PCS = tracking_result->correspondence_set;
                    if(success_matched)
                    {
                        optimization::Correspondence loop_corr(candidate_frame_id, frame_id);
                        loop_corr.correspondence_set = loop_PCS;
                        float loop_reprojection_error = loop_corr.ComputeReprojectionError3D(loop_T);
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

                            for(int j = 0; j < loop_matches.size(); ++j)
                            {
                                int source_local_id = loop_matches[j].first;
                                int target_local_id = loop_matches[j].second;
                                int &source_global_id = points_local_to_global[candidates[i]][source_local_id];
                                int &target_global_id = (points_local_to_global.back())[target_local_id];
                                auto &source_points = global_frames[candidate_frame_id].feature_pcd.points;
                                geometry::Point2 source_uv;
                                source_uv << global_frames[candidate_frame_id].keypoints[source_local_id].pt.x, 
                                    global_frames[candidate_frame_id].keypoints[source_local_id].pt.y;

                                geometry::Point2 target_uv;
                                target_uv << current_frame.keypoints[target_local_id].pt.x, 
                                    current_frame.keypoints[target_local_id].pt.y;

                                if(source_global_id == -1 && target_global_id == -1)
                                {
                                    //this means the point has not been added into the world points yet.
                                    
                                    world_points.push_back(geometry::TransformPoint(global_keyframe_poses[candidates[i]], 
                                                source_points[source_local_id]));
                                    // if(std::isnan(world_points.back()(0)))
                                    // {
                                    //     std::cout<<"NAN!"<<std::endl;
                                    //     std::cout<<global_keyframe_poses[last_kid]<<std::endl;
                                    //     std::cout<<source_points[source_local_id].transpose()<<std::endl;
                                    // }
                                    source_global_id = world_points.size() - 1;
                                    target_global_id = world_points.size() - 1;
                                    //2d coordinate

                                    projected_points[candidates[i]][source_global_id] = source_uv;
                                    (projected_points.back())[target_global_id] = target_uv;
                                }
                                else if(target_global_id == -1)
                                {

                                    target_global_id = source_global_id;
                                    (projected_points.back())[target_global_id] = target_uv;
                                }
                                else if(source_global_id == -1)
                                {
                                    source_global_id = target_global_id;
                                    projected_points[candidates[i]][source_global_id] = source_uv;
                                }
                                else
                                {
                                    // std::cout<<YELLOW<<"source_global_id: "<<source_global_id<<
                                    //     " target_global_id: "<<target_global_id<<RESET<<std::endl;
                                    // std::cout<<YELLOW<<"source_global_point: "<<world_points[source_global_id]<<
                                    //     " target_global_point: "<<world_points[target_global_id]<<RESET<<std::endl;
                                    projected_points.back()[source_global_id] = target_uv;
                                    projected_points[candidates[i]][target_global_id] = source_uv;
                                }
                                
                            }                        
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
            // if(keyframe_ids.size()%10 == 0)
            //     Optimize();
        }
        //else current_frame.Release();

    }
    void BASlam::Optimize()
    {
        optimizer.BA(world_points, projected_points, 
            global_keyframe_poses, camera);
        for(int i = 0; i != keyframe_ids.size(); ++i)
        {
            global_poses[keyframe_ids[i]] = global_keyframe_poses[i];
        }        
    }

}