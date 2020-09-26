#include "DenseOdometryFunction.h"
#include <Eigen/Cholesky>
#define SHOW_VERBOSE 0
namespace fucking_cool
{
namespace odometry 
{
    void AddElementToCorrespondenceMap(cv::Mat &wraping_map,cv::Mat &wraping_depth,
        int u_s, int v_s, int u_t, int v_t, float transformed_d_s)
    {
        float existing_depth = wraping_depth.at<float>(v_t,u_t);
        if(existing_depth == -1)
        {
            wraping_map.at<cv::Vec2i>(v_s,u_s) = cv::Vec2i(v_t,u_t);
            wraping_depth.at<float>(v_s,u_s) = transformed_d_s;
        }
        else
        {
            if(existing_depth > transformed_d_s)
            {
            wraping_map.at<cv::Vec2i>(v_s,u_s) = cv::Vec2i(v_t,u_t);
            wraping_depth.at<float>(v_s,u_s) = transformed_d_s;
            }
        }
    }
    void ConvertDepthTo32FNaN(const cv::Mat &depth, cv::Mat &refined_depth, float depth_scale)
    {
    refined_depth.create(depth.rows, depth.cols, CV_32FC1);
    //std::cout <<depth.depth()<<std::endl;
    if(depth.depth() == CV_32FC1)
    {
        for(int i = 0; i < depth.rows * depth.cols; ++i)
        {
            if(depth.at<float>(i) > MIN_DEPTH  && depth.at<float>(i) < MAX_DEPTH )
            refined_depth.at<float>(i) = depth.at<float>(i);
            else 
            refined_depth.at<float>(i) = std::numeric_limits<float>::quiet_NaN();
        }
        //tool::PrintMat(refined_depth);
    }
    else if(depth.depth() == CV_16UC1)
    {
        for(int i = 0; i < depth.rows * depth.cols; ++i)
        {
            if(depth.at<unsigned short>(i) > MIN_DEPTH * depth_scale && depth.at<unsigned short>(i) < MAX_DEPTH * depth_scale)
            refined_depth.at<float>(i) = depth.at<unsigned short>(i) /depth_scale;
            else 
            refined_depth.at<float>(i) = std::numeric_limits<float>::quiet_NaN();
        }
    }
    else
    {
        std::cout <<RED<< "[ImageProcessing]::[ERROR]::Unknown depth image type: "<<depth.depth()<<RESET<<std::endl;
        exit(1);
    }
    //tool::PrintMat(refined_depth);
    }

    void ConvertColorToIntensity32F(const cv::Mat &color, cv::Mat &intensity, float scale)
    {
        cv::Mat gray;
        tool::Convert2Gray(color,gray);
        intensity.create(gray.rows, gray.cols, CV_32FC1);
        for(int i = 0; i < gray.rows; ++i)
        {
            for(int j = 0; j < gray.cols;++j)
                intensity.at<float>(i,j) = gray.at<unsigned char>(i,j)/scale;
        }
        //tool::PrintMat(intensity);
        
    }
    void ComputeCorrespondencePixelWise(const cv::Mat &source, const cv::Mat &target, const camera::PinholeCamera &camera,
        const geometry::TransformationMatrix & relative_pose, geometry::PixelCorrespondenceSet &correspondences)
    {


        int width = camera.GetWidth();
        int height = camera.GetHeight();
        // float fx = camera.GetFx(), fy = camera.GetFy(), cx = camera.GetCx(), cy = camera.GetCy();
        //std::cout << width << " " << height << std::endl;
        cv::Mat wraping_depth(height,width,CV_32FC1,cv::Scalar(-1)); 
        cv::Mat wraping_map(height,width,CV_32SC2,cv::Scalar(-1));       
        geometry::Matrix3 K = camera.ToCameraMatrix();
        geometry::Matrix3 R = relative_pose.block<3,3>(0,0);
        geometry::Point3 t = relative_pose.block<3,1>(0,3);
        geometry::Point3 Kt = K * t;
        geometry::Matrix3 K_inv = K.inverse();
        geometry::Matrix3 KRK_inv = K * R * K_inv;

        for(int i = 0; i!=height; ++i)
        {
            for(int j = 0;j!=width; ++j)
            {
                float d_s = source.at<float>(i, j) ;
                if(!std::isnan(d_s))
                {
                    geometry::Point3 uv_in_s =
                            d_s * KRK_inv * geometry::Point3(j, i, 1.0) + Kt;
                    float transformed_d_s = uv_in_s(2);

                    //round up
                    int u_t = (int)(uv_in_s(0) / transformed_d_s + 0.5);
                    int v_t = (int)(uv_in_s(1) / transformed_d_s + 0.5);

                    if (u_t >= 0 && u_t < width && v_t >= 0 &&
                        v_t < height)
                        {
                            float d_t = target.at<float>(v_t, u_t);
                            //std::cout<<u_t<<" "<<v_t<<" "<<d_t<<std::endl;
                            if(!std::isnan(d_t) && std::abs(d_t - transformed_d_s)< MAX_DIFF_DEPTH)
                            {
                                AddElementToCorrespondenceMap(wraping_map,wraping_depth,j,i,u_t,v_t,transformed_d_s);
                            }
                        }
                }
            }
        }
        for(int i = 0;i!=height;++i)//v_s
        {
            cv::Vec2i vu_target;
            for(int j = 0;j!=width;++j)
            if(wraping_depth.at<float>(i,j) != -1)//u_s
            {
                vu_target = wraping_map.at<cv::Vec2i>(i,j);
                correspondences.push_back(std::make_pair(geometry::Point2ui(i,j), geometry::Point2ui(vu_target(0), vu_target(1))));
            }
        }        
    }
    void NormalizeIntensity(cv::Mat &source, cv::Mat & target, const geometry::PixelCorrespondenceSet &correspondence)
    {
        float mean_s = 0.0, mean_t = 0.0;
        for (size_t row = 0; row < correspondence.size(); row++) {
            int v_s = correspondence[row].first(0);
            int u_s = correspondence[row].first(1);
            int v_t = correspondence[row].second(0);
            int u_t = correspondence[row].second(1);
            mean_s += source.at<float>(v_s, u_s);
            mean_t += target.at<float>(v_t, u_t);
        }
        mean_s /= (float)correspondence.size();
        mean_t /= (float)correspondence.size();
        tool::LinearTransform(source, 0.5 / mean_s, 0.0);
        tool::LinearTransform(target, 0.5 / mean_t, 0.0);
    }

    void ComputeJacobianPhotoTerm(
        int row,std::vector<geometry::Se3> &J, std::vector<float> &residual, 
        const cv::Mat &source_color, const cv::Mat & source_depth, 
        const cv::Mat & target_color, const cv::Mat & target_depth, 
        const cv::Mat &target_color_dx, const cv::Mat & target_color_dy, 
        const geometry::ImageXYZ & source_XYZ, const camera::PinholeCamera &camera, 
        const geometry::TransformationMatrix & relative_pose, 
        const geometry::PixelCorrespondenceSet &correspondences)
    {
        float fx = camera.GetFx(), fy = camera.GetFy();
        geometry::Matrix3 R = relative_pose.block<3,3>(0,0);
        geometry::Point3 t = relative_pose.block<3,1>(0,3);

        int v_s = correspondences[row].first(0);
        int u_s = correspondences[row].first(1);
        int v_t = correspondences[row].second(0);
        int u_t = correspondences[row].second(1);
        float diff_photo = target_color.at<float>(v_t, u_t) -
                            source_color.at<float>(v_s, u_s);
        float dIdx = SOBEL_SCALE * (target_color_dx.at<float>(v_t, u_t));
        float dIdy = SOBEL_SCALE * (target_color_dy.at<float>(v_t, u_t));

        geometry::Point3 p3d_mat(source_XYZ[v_s][u_s]);
        geometry::Point3 p3d_trans = R * p3d_mat + t;

        float invz = 1. / p3d_trans(2);
        float c0 = dIdx * fx * invz;
        float c1 = dIdy * fy * invz;
        float c2 = -(c0 * p3d_trans(0) + c1 * p3d_trans(1)) * invz;

        J.resize(1);
        residual.resize(1);
        J[0](0) =  (c0);
        J[0](1) =  (c1);
        J[0](2) =  (c2);
        J[0](3) =  (-p3d_trans(2) * c1 + p3d_trans(1) * c2);
        J[0](4) =  (p3d_trans(2) * c0 - p3d_trans(0) * c2);
        J[0](5) =  (-p3d_trans(1) * c0 + p3d_trans(0) * c1);

        float r_photo =  diff_photo;
        residual[0] = r_photo;
        //std::cout <<J[0]<<" \n"<< std::endl;
    }
    void ComputeJacobianDepthTerm(
        int row,std::vector<geometry::Se3> &J, std::vector<float> &residual, 
        const cv::Mat &source_color, const cv::Mat & source_depth, 
        const cv::Mat & target_color, const cv::Mat & target_depth, 
        const cv::Mat & target_depth_dx, const cv::Mat & target_depth_dy, 
        const geometry::ImageXYZ & source_XYZ, const camera::PinholeCamera &camera, const geometry::TransformationMatrix & relative_pose, 
        const geometry::PixelCorrespondenceSet &correspondences)
    {
        float fx = camera.GetFx(), fy = camera.GetFy();
        geometry::Matrix3 R = relative_pose.block<3,3>(0,0);
        geometry::Point3 t = relative_pose.block<3,1>(0,3);

        int v_s = correspondences[row].first(0);
        int u_s = correspondences[row].first(1);
        int v_t = correspondences[row].second(0);
        int u_t = correspondences[row].second(1);
        float dDdx = SOBEL_SCALE * (target_depth_dx.at<float>(v_t, u_t));
        float dDdy = SOBEL_SCALE * (target_depth_dy.at<float>(v_t, u_t));
        if (std::isnan(dDdx)) dDdx = 0;
        if (std::isnan(dDdy)) dDdy = 0;
        geometry::Point3 p3d_mat(source_XYZ[v_s][u_s]);
        geometry::Point3 p3d_trans = R * p3d_mat + t;

        float diff_geo = target_depth.at<float>(v_t, u_t) - p3d_trans(2);
        float invz = 1. / p3d_trans(2);
        float d0 = dDdx * fx * invz;
        float d1 = dDdy * fy * invz;
        float d2 = -(d0 * p3d_trans(0) + d1 * p3d_trans(1)) * invz;

        J.resize(1);
        residual.resize(1);

        J[0](0) =  (d0);
        J[0](1) =  (d1);
        J[0](2) =  (d2 - 1.0f);
        J[0](3) =  ((-p3d_trans(2) * d1 + p3d_trans(1) * d2) - p3d_trans(1));
        J[0](4) =  ((p3d_trans(2) * d0 - p3d_trans(0) * d2) + p3d_trans(0));
        J[0](5) =  ((-p3d_trans(1) * d0 + p3d_trans(0) * d1));

        residual[0] = diff_geo;
        //std::cout <<J[0]<<" \n"<< std::endl;        
    }
    void ComputeJacobianHybridTerm(
        int row,std::vector<geometry::Se3> &J, std::vector<float> &residual, 
        const cv::Mat &source_color, const cv::Mat & source_depth, 
        const cv::Mat & target_color, const cv::Mat & target_depth, 
        const cv::Mat &target_color_dx, const cv::Mat & target_depth_dx, 
        const cv::Mat & target_color_dy, const cv::Mat & target_depth_dy, 
        const geometry::ImageXYZ & source_XYZ, const camera::PinholeCamera &camera, const geometry::TransformationMatrix & relative_pose, 
        const geometry::PixelCorrespondenceSet &correspondences)
    {
        float sqrt_lamba_dep, sqrt_lambda_img;
        sqrt_lamba_dep = sqrt(LAMBDA_HYBRID_DEPTH);
        sqrt_lambda_img = sqrt(1.0 - LAMBDA_HYBRID_DEPTH);

        float fx = camera.GetFx(), fy = camera.GetFy();
        geometry::Matrix3 R = relative_pose.block<3,3>(0,0);
        geometry::Point3 t = relative_pose.block<3,1>(0,3);

        int v_s = correspondences[row].first(0);
        int u_s = correspondences[row].first(1);
        int v_t = correspondences[row].second(0);
        int u_t = correspondences[row].second(1);
        float diff_photo = target_color.at<float>(v_t, u_t) -
                            source_color.at<float>(v_s, u_s);
        float dIdx = SOBEL_SCALE * (target_color_dx.at<float>(v_t, u_t));
        float dIdy = SOBEL_SCALE * (target_color_dy.at<float>(v_t, u_t));
        float dDdx = SOBEL_SCALE * (target_depth_dx.at<float>(v_t, u_t));
        float dDdy = SOBEL_SCALE * (target_depth_dy.at<float>(v_t, u_t));
        if (std::isnan(dDdx)) dDdx = 0;
        if (std::isnan(dDdy)) dDdy = 0;
        geometry::Point3 p3d_mat(source_XYZ[v_s][u_s]);
        geometry::Point3 p3d_trans = R * p3d_mat + t;

        float diff_geo = target_depth.at<float>(v_t, u_t) - p3d_trans(2);
        float invz = 1. / p3d_trans(2);
        float c0 = dIdx * fx * invz;
        float c1 = dIdy * fy * invz;
        float c2 = -(c0 * p3d_trans(0) + c1 * p3d_trans(1)) * invz;
        float d0 = dDdx * fx * invz;
        float d1 = dDdy * fy * invz;
        float d2 = -(d0 * p3d_trans(0) + d1 * p3d_trans(1)) * invz;

        J.resize(2);
        residual.resize(2);
        J[0](0) = sqrt_lambda_img * (c0);
        J[0](1) = sqrt_lambda_img * (c1);
        J[0](2) = sqrt_lambda_img * (c2);
        J[0](3) = sqrt_lambda_img * (-p3d_trans(2) * c1 + p3d_trans(1) * c2);
        J[0](4) = sqrt_lambda_img * (p3d_trans(2) * c0 - p3d_trans(0) * c2);
        J[0](5) = sqrt_lambda_img * (-p3d_trans(1) * c0 + p3d_trans(0) * c1);

        float r_photo = sqrt_lambda_img * diff_photo;
        residual[0] = r_photo;

        J[1](0) = sqrt_lamba_dep * (d0);
        J[1](1) = sqrt_lamba_dep * (d1);
        J[1](2) = sqrt_lamba_dep * (d2 - 1.0f);
        J[1](3) = sqrt_lamba_dep *
                    ((-p3d_trans(2) * d1 + p3d_trans(1) * d2) - p3d_trans(1));
        J[1](4) = sqrt_lamba_dep *
                   ((p3d_trans(2) * d0 - p3d_trans(0) * d2) + p3d_trans(0));
        J[1](5) = sqrt_lamba_dep * ((-p3d_trans(1) * d0 + p3d_trans(0) * d1));

        float r_geo = sqrt_lamba_dep * diff_geo;
        residual[1] = r_geo;
        //std::cout <<J[0]<<" \n"<< std::endl;
    }
    std::tuple<geometry::Matrix6,geometry::Se3,float> ComputeJTJandJTrHybridTerm(
        const cv::Mat &source_color, const cv::Mat & source_depth, 
        const cv::Mat & target_color, const cv::Mat & target_depth, 
        const cv::Mat &target_color_dx, const cv::Mat & target_depth_dx, 
        const cv::Mat & target_color_dy, const cv::Mat & target_depth_dy, 
        const geometry::ImageXYZ & source_XYZ, const camera::PinholeCamera &camera, const geometry::TransformationMatrix & relative_pose, 
        const geometry::PixelCorrespondenceSet &correspondences)
    {
        geometry::Matrix6 JTJ_private;
        geometry::Se3 JTr_private;
        float r2_sum_private = 0.0;
        JTJ_private.setZero();
        JTr_private.setZero();
        for(size_t i = 0;i!=correspondences.size();++i)
        {
            std::vector<geometry::Se3> J;
            std::vector<float > r;
            ComputeJacobianHybridTerm(i,J,r,source_color,source_depth,target_color,target_depth,target_color_dx ,
                target_depth_dx, target_color_dy, target_depth_dy, source_XYZ, camera, relative_pose,correspondences);            
            for(size_t j = 0; j!=J.size();++j)
            {
                JTJ_private.noalias() += J[j] * J[j].transpose();
                JTr_private.noalias() += J[j] * r[j];
                r2_sum_private += r[j] * r[j];
            }
        }
        //std::cout <<"JTJ: \n"<<JTJ_private<<std::endl;
        return std::make_tuple(std::move(JTJ_private), std::move(JTr_private), r2_sum_private);
    }
    std::tuple<geometry::Matrix6,geometry::Se3,float> ComputeJTJandJTrPhotoTerm(
        const cv::Mat &source_color, const cv::Mat & source_depth, 
        const cv::Mat & target_color, const cv::Mat & target_depth, 
        const cv::Mat &target_color_dx, const cv::Mat & target_color_dy, 
        const geometry::ImageXYZ & source_XYZ, const camera::PinholeCamera &camera, const geometry::TransformationMatrix & relative_pose, 
        const geometry::PixelCorrespondenceSet &correspondences)
    {
        geometry::Matrix6 JTJ_private;
        geometry::Se3 JTr_private;
        float r2_sum_private = 0.0;
        JTJ_private.setZero();
        JTr_private.setZero();
        for(size_t i = 0;i!=correspondences.size();++i)
        {
            std::vector<geometry::Se3> J;
            std::vector<float > r;
            ComputeJacobianPhotoTerm(i,J,r,source_color,source_depth,target_color,target_depth,target_color_dx ,
                 target_color_dy, source_XYZ, camera, relative_pose,correspondences);            
            for(size_t j = 0; j!=J.size();++j)
            {
                JTJ_private.noalias() += J[j] * J[j].transpose();
                JTr_private.noalias() += J[j] * r[j];
                r2_sum_private += r[j] * r[j];
            }
        }
        //std::cout <<"JTJ: \n"<<JTJ_private<<std::endl;
        return std::make_tuple(std::move(JTJ_private), std::move(JTr_private), r2_sum_private);
    }
    std::tuple<geometry::Matrix6,geometry::Se3,float> ComputeJTJandJTrDepthTerm(
        const cv::Mat &source_color, const cv::Mat & source_depth, 
        const cv::Mat & target_color, const cv::Mat & target_depth, 
        const cv::Mat & target_depth_dx, const cv::Mat & target_depth_dy, 
        const geometry::ImageXYZ & source_XYZ, const camera::PinholeCamera &camera, const geometry::TransformationMatrix & relative_pose, 
        const geometry::PixelCorrespondenceSet &correspondences)
    {
        geometry::Matrix6 JTJ_private;
        geometry::Se3 JTr_private;
        float r2_sum_private = 0.0;
        JTJ_private.setZero();
        JTr_private.setZero();
        for(size_t i = 0;i!=correspondences.size();++i)
        {
            std::vector<geometry::Se3> J;
            std::vector<float > r;
            ComputeJacobianDepthTerm(i,J,r,source_color,source_depth,target_color,target_depth,
                target_depth_dx, target_depth_dy, source_XYZ, camera, relative_pose,correspondences);            
            for(size_t j = 0; j!=J.size();++j)
            {
                JTJ_private.noalias() += J[j] * J[j].transpose();
                JTr_private.noalias() += J[j] * r[j];
                r2_sum_private += r[j] * r[j];
            }
        }
        //std::cout <<"JTJ: \n"<<JTJ_private<<std::endl;
        return std::make_tuple(std::move(JTJ_private), std::move(JTr_private), r2_sum_private);
    }
    void DoSingleIteration(
        const cv::Mat &source_color, const cv::Mat & source_depth, 
        const cv::Mat & target_color, const cv::Mat & target_depth, 
        const cv::Mat &target_color_dx, const cv::Mat & target_depth_dx, 
        const cv::Mat & target_color_dy, const cv::Mat & target_depth_dy, 
        const geometry::ImageXYZ & source_XYZ, const camera::PinholeCamera &camera, geometry::TransformationMatrix & relative_pose, 
        geometry::PixelCorrespondenceSet &correspondences)
    {
        //find correspondences based on the current pose
        //std::cout<<"find correspondences[pixel]..."<<std::endl;
        ComputeCorrespondencePixelWise(source_depth,target_depth,camera,relative_pose,correspondences);
        //std::cout<<"correspondences: "<<correspondences.size()<<std::endl;
        
        geometry::Matrix6 JTJ;
        geometry::Se3 JTr;
        float r;
        //std::cout<<"computing Jacobian..."<<std::endl;
        
        std::tie(JTJ,JTr,r) = ComputeJTJandJTrHybridTerm(source_color,source_depth,target_color,target_depth,target_color_dx ,
                target_depth_dx, target_color_dy, target_depth_dy, source_XYZ, camera, relative_pose,correspondences );
        geometry::Se3 delta = JTJ.ldlt().solve(-JTr);
        
        //geometry::TransformationMatrix delta_matrix = geometry::TransformVector6fToMatrix4f(delta);
        geometry::TransformationMatrix delta_matrix = geometry::Se3ToSE3(delta);
#if SHOW_VERBOSE     
        size_t valid_count = correspondences.size(); 
        std::cout<<BLUE <<"[DEBUG]::residual: "<<r/valid_count<<" element_num: "<<valid_count <<RESET<<std::endl;
        //std::cout<<"delta: "<<delta<<std::endl;
#endif
        relative_pose = delta_matrix *  relative_pose;

    }
    void DoSingleIterationPhotoTerm(
        const cv::Mat &source_color, const cv::Mat & source_depth, 
        const cv::Mat & target_color, const cv::Mat & target_depth, 
        const cv::Mat &target_color_dx, const cv::Mat & target_color_dy,
        const geometry::ImageXYZ & source_XYZ, const camera::PinholeCamera &camera, geometry::TransformationMatrix & relative_pose, 
        geometry::PixelCorrespondenceSet &correspondences)
    {
        //find correspondences based on the current pose
        //std::cout<<"find correspondences[pixel]..."<<std::endl;
        ComputeCorrespondencePixelWise(source_depth,target_depth,camera,relative_pose,correspondences);
        //std::cout<<"correspondences: "<<correspondences.size()<<std::endl;

        geometry::Matrix6 JTJ;
        geometry::Se3 JTr;
        float r;
        //std::cout<<"computing Jacobian..."<<std::endl;
        
        std::tie(JTJ,JTr,r) = ComputeJTJandJTrPhotoTerm(source_color,source_depth,target_color,target_depth,target_color_dx ,
                 target_color_dy,  source_XYZ, camera, relative_pose,correspondences );
        geometry::Se3 delta = JTJ.ldlt().solve(-JTr);
        
        //geometry::TransformationMatrix delta_matrix = geometry::TransformVector6fToMatrix4f(delta);
        geometry::TransformationMatrix delta_matrix = geometry::Se3ToSE3(delta);
#if SHOW_VERBOSE    
        size_t valid_count = correspondences.size();  
        std::cout<<BLUE <<"[DEBUG]::residual: "<<r/valid_count<<" element_num: "<<valid_count <<RESET<<std::endl;
        //std::cout<<"delta: "<<delta<<std::endl;
#endif
        relative_pose = delta_matrix *  relative_pose;

    }
    void DoSingleIterationDepthTerm(
        const cv::Mat &source_color, const cv::Mat & source_depth, 
        const cv::Mat & target_color, const cv::Mat & target_depth, 
        const cv::Mat &target_depth_dx, const cv::Mat & target_depth_dy,
        const geometry::ImageXYZ & source_XYZ, const camera::PinholeCamera &camera, geometry::TransformationMatrix & relative_pose, 
        geometry::PixelCorrespondenceSet &correspondences)
    {
        //find correspondences based on the current pose
        //std::cout<<"find correspondences[pixel]..."<<std::endl;
        ComputeCorrespondencePixelWise(source_depth,target_depth,camera,relative_pose,correspondences);
        //std::cout<<"correspondences: "<<correspondences.size()<<std::endl;
        
        geometry::Matrix6 JTJ;
        geometry::Se3 JTr;
        float r;
        //std::cout<<"computing Jacobian..."<<std::endl;
        
        std::tie(JTJ,JTr,r) = ComputeJTJandJTrDepthTerm(source_color,source_depth,target_color,target_depth,target_depth_dx ,
                 target_depth_dy,  source_XYZ, camera, relative_pose,correspondences );
        geometry::Se3 delta = JTJ.ldlt().solve(-JTr);
        
        //geometry::TransformationMatrix delta_matrix = geometry::TransformVector6fToMatrix4f(delta);
        geometry::TransformationMatrix delta_matrix = geometry::Se3ToSE3(delta);
#if SHOW_VERBOSE  
        size_t valid_count = correspondences.size();
        std::cout<<BLUE <<"[DEBUG]::residual: "<<r/valid_count<<" element_num: "<<valid_count <<RESET<<std::endl;
        //std::cout<<"delta: "<<delta<<std::endl;
#endif
        relative_pose = delta_matrix *  relative_pose;

    }
}
}