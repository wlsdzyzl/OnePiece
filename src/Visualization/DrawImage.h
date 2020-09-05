#ifndef MATRIX_IMAGE_H
#define MATRIX_IMAGE_H
#include "Algorithm/DCEL.h"
#include "Algorithm/PatchDetection.h"
#include "Algorithm/Clustering.h"
#include "ColorTab.h"
namespace fucking_cool
{
namespace visualization
{

    cv::Mat MatrixImage(const geometry::MatrixX &m)
    {
        geometry::MatrixX::Index maxRow, maxCol, minRow, minCol;
        float max = std::max( std::fabs(m.maxCoeff(&maxRow,&maxCol)), 
            std::fabs(m.minCoeff(&minRow,&minCol)) );
        
        cv::Mat img(m.rows(), m.cols(), CV_8UC3);
        for(int i = 0; i!=m.rows(); ++i)
        {
            for(int j = 0; j!= m.cols(); ++j)
            {
                float value = m(i,j);
                cv::Point ipt(i,j);
                if(value > 0)
                {
                    //img.at<cv::Vec3b>(i,j) = cv::Scalar( 0, 0, std::fabs(value/max) * 255);
                    img.at<cv::Vec3b>(i, j)[0] = 255 - std::fabs(value/max) * 255;

                    img.at<cv::Vec3b>(i, j)[1] = 255 - std::fabs(value/max) * 255;

                    img.at<cv::Vec3b>(i, j)[2] = 255;;
                }
                else if(value < 0)
                {
                    //img.at<cv::Vec3b>(i,j) = cv::Scalar( 0, 0, std::fabs(value/max) * 255);
                    img.at<cv::Vec3b>(i, j)[0] = 255;

                    img.at<cv::Vec3b>(i, j)[1] = 255 - std::fabs(value/max) * 255;

                    img.at<cv::Vec3b>(i, j)[2] = 255 - std::fabs(value/max) * 255;
                }
                else 
                {
                    img.at<cv::Vec3b>(i, j)[0] = 255;

                    img.at<cv::Vec3b>(i, j)[1] = 255;

                    img.at<cv::Vec3b>(i, j)[2] = 255;
                }
            }
        }
        return img;
    }
    cv::Mat PatchImage(const std::vector<algorithm::LinePatch> &patches)
    {
        float max_c = std::numeric_limits<float>::lowest (), min_c = std::numeric_limits<float>::max ();
        float max_x = max_c, max_y = max_c, min_x = min_c, min_y = min_c;
        for(int i = 0; i != patches.size(); ++i)
        {
            const geometry::Point2List &points = patches[i].items;
            for(int j = 0; j!= points.size(); ++j)
            {
                if(points[j](0) > max_x)
                max_x = points[j](0);
                if(points[j](0) < min_x)
                min_x = points[j](0);
                if(points[j](1) > max_y)
                max_y = points[j](1);
                if(points[j](1) < min_y)
                min_y = points[j](1);
            }
        }

        max_c = std::max(max_x - min_x , max_y - min_y);
        min_c = std::min(min_x, min_y);
    

        cv::Mat img_line(700, 700, CV_8UC3);
        img_line = cv::Scalar::all(0);
        float scalar =600 / max_c;

        for( int i = 0; i < patches.size(); ++i )
        {
            //std::cout<<clusters[i].items.size()<<std::endl;
            for(int j = 0; j!=patches[i].items.size(); ++j)
            {

                cv::Point ipt = cv::Point2i((patches[i].items[j](0) - min_x ) *scalar, (patches[i].items[j](1)-min_y)* scalar);
                //std::cout<<patches[i].items[j]<<std::endl;
                cv::circle( img_line, ipt, 2, color_tab[i], CV_FILLED, CV_AA );
            }
        }
        return img_line;
    }
    cv::Mat ClusterImage(const std::vector<algorithm::Cluster<2>> &clusters)
    {
        float max_c = std::numeric_limits<float>::lowest (), min_c = std::numeric_limits<float>::max ();
        float max_x = max_c, max_y = max_c, min_x = min_c, min_y = min_c;
        for(int i = 0; i != clusters.size(); ++i)
        {
            const geometry::Point2List &points = clusters[i].items;
            for(int j = 0; j!= points.size(); ++j)
            {
                if(points[j](0) > max_x)
                max_x = points[j](0);
                if(points[j](0) < min_x)
                min_x = points[j](0);
                if(points[j](1) > max_y)
                max_y = points[j](1);
                if(points[j](1) < min_y)
                min_y = points[j](1);
            }
        }

        max_c = std::max(max_x - min_x , max_y - min_y);
        min_c = std::min(min_x, min_y);
    

        cv::Mat img(700, 700, CV_8UC3);
        img = cv::Scalar::all(0);
        float scalar =600 / max_c;

        for( int i = 0; i < clusters.size(); ++i )
        {
            //std::cout<<clusters[i].items.size()<<std::endl;
            for(int j = 0; j!=clusters[i].items.size(); ++j)
            {

                cv::Point ipt = cv::Point2i((clusters[i].items[j](0) - min_x ) *scalar, (clusters[i].items[j](1)-min_y)* scalar);
                cv::circle( img, ipt, 2, color_tab[i], CV_FILLED, CV_AA );
            }
        }
        return img;
    }
    cv::Mat DCELImage(const algorithm::DCEL & dcel)
    {
        cv::Mat img(700, 700, CV_8UC3);
        img = cv::Scalar::all(0);
        geometry::Point2 max_xy = dcel.right_up - dcel.left_bottom;
        float max_c = std::max(max_xy(0), max_xy(1));
        float scalar = 600 / max_c; 
#if DEBUG_MODE
    std::cout<<BLUE<<"[DrawDCEL]::[DEBUG]::Vertexs: "<<dcel.vertexs.size() - dcel.vid_pool.size()<<" "
        <<"Edges: "<<dcel.edges.size() - dcel.eid_pool.size()<<" Faces: "<<dcel.faces.size() - dcel.fid_pool.size()<<RESET<<std::endl;
#endif
        for(int i = 0; i!=dcel.faces.size(); ++i)
        {
            if(dcel.faces[i].is_valid == false)
            continue;
            std::vector<cv::Point> poly; 
            int start_eid = dcel.faces[i].inc_eid;
            int start_vid = dcel.edges[start_eid].origin_vid;
            poly.push_back(cv::Point2i((dcel.vertexs[start_vid].coor(0) - dcel.left_bottom(0) ) *scalar, 
                (dcel.vertexs[start_vid].coor(1)-dcel.left_bottom(1))* scalar));
            start_eid = dcel.edges[start_eid].succ_eid;
            start_vid = dcel.edges[start_eid].origin_vid;
            while(start_eid != dcel.faces[i].inc_eid)
            {

                poly.push_back(cv::Point2i((dcel.vertexs[start_vid].coor(0) - dcel.left_bottom(0) ) *scalar, 
                    (dcel.vertexs[start_vid].coor(1)-dcel.left_bottom(1))* scalar));
                start_eid = dcel.edges[start_eid].succ_eid;
                start_vid = dcel.edges[start_eid].origin_vid;
            }
            cv::fillConvexPoly(img,&poly[0],poly.size(),color_tab[i]);
        }
        
        for(int i = 0; i!= dcel.edges.size(); ++i)
        {
            if(dcel.edges[i].is_valid == false)
            continue;
            
            //continue;
            int svid = dcel.edges[i].origin_vid;
            int evid = dcel.edges[i].des_vid;

            cv::Point point_s = cv::Point2i((dcel.vertexs[svid].coor(0) - dcel.left_bottom(0) ) *scalar, 
                (dcel.vertexs[svid].coor(1)-dcel.left_bottom(1))* scalar);
            cv::Point point_e = cv::Point2i((dcel.vertexs[evid].coor(0) - dcel.left_bottom(0) ) *scalar, 
                (dcel.vertexs[evid].coor(1)-dcel.left_bottom(1))* scalar);
            if(dcel.edges[i].line_id <4)
            cv::line(img, point_s, point_e, cv::Scalar(0, 255, 255));
            else
            cv::line(img, point_s, point_e, cv::Scalar(0, 0, 0));
            
        }


        for(int i = 0; i !=dcel.vertexs.size(); ++i)
        {
            if(dcel.vertexs[i].is_valid == false)
            continue;
            cv::Point ipt = cv::Point2i((dcel.vertexs[i].coor(0) - dcel.left_bottom(0) ) *scalar, 
                (dcel.vertexs[i].coor(1)-dcel.left_bottom(1))* scalar);
            cv::circle( img, ipt, 2, cv::Scalar(255, 255, 255), CV_FILLED, CV_AA );
        }
        return img;
    }
}
}
#endif