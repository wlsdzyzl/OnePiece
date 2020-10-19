#include "Geometry/PointCloud.h"
#include "Algorithm/PatchDetection.h"
#include "Algorithm/Arrangements.h"
#include "Visualization/Visualizer.h"
#include "Visualization/DrawImage.h"
#include "Visualization/DrawPointCloud.h"
using namespace one_piece;
//2564786688 floor
//2932336640 wall
//3592890368 door

class Building
{
    public:
    std::shared_ptr<algorithm::DCEL> dcel;
    std::vector<algorithm::LinePatch> lines;
    geometry::MatrixX affinity_matrix;
    geometry::MatrixX diffusion_matrix;
    geometry::MatrixX distance_matrix;
    geometry::PointXList embeddings;//embeddings for each face
    std::vector<std::vector<int>> rooms;
    float t = 40;//diffusion time
    int m = 40;//length of embedding
    void SetEmbeddingDimension(int _d = 0)
    {
        if(_d == 0)
        {
            int f_n = dcel->GetFaceSize();
            m = std::min(f_n, 80);
        }
        else m = _d;
    }
    void ComputeWeightsForEachEdge()
    {
        for(size_t i = 0; i!=dcel->edges.size(); ++i)
        {
            if(dcel->edges[i].weight != 0 || dcel->edges[i].line_id < 4 || dcel->edges[i].is_valid == false)
                continue;
            int l_id = dcel->edges[i].line_id - 4;
            algorithm::LinePatch &line = lines[l_id];
            geometry::LineSegment ls = dcel->EdgeSegment(i); 
            int points_on_line = 0;
            for(size_t j = 0; j!= line.items.size(); ++j)
            {
                geometry::Vector2 &inlier_point = line.items[j];
                geometry::Vector2 pro_point = 
                    geometry::ProjectionPointToLineSegment(ls, inlier_point);
                if(geometry::InSegBounding(ls, pro_point ))
                {
                    ++points_on_line;
                }
            }
            dcel->edges[i].weight = (float)points_on_line / ls.Length();
            if(dcel->edges[i].weight < 500) dcel->edges[i].weight = 0;
            else dcel->edges[i].weight = 60;
            dcel->edges[dcel->edges[i].twin_eid].weight = dcel->edges[i].weight;
            std::cout<<"weights: "<<dcel->edges[i].weight<<std::endl;
        }
        /*
        int start_eid = dcel->faces[5].inc_eid;
        int eid = start_eid;
        std::cout<<"weights: "<<dcel->edges[eid].weight <<" "<<dcel->edges[dcel->edges[eid].twin_eid].left_fid<<std::endl;
        eid = dcel->edges[eid].succ_eid;
        while(eid != start_eid)
        {
            std::cout<<"weights: "<<dcel->edges[eid].weight<<" "<<dcel->edges[dcel->edges[eid].twin_eid].left_fid<<std::endl;   
            eid = dcel->edges[eid].succ_eid;         
        }*/
    }

    void ComputeEmbedding()
    {
        affinity_matrix.resize(dcel->faces.size() + 1, dcel->faces.size()+1);
        affinity_matrix.setZero();
        int infty_id = dcel->faces.size();

        for(size_t i = 0; i!=dcel->faces.size(); ++i)
        {
            affinity_matrix(i, i) = 1;
            if(dcel->faces[i].is_valid == false)
            continue;
            int eid = dcel->faces[i].inc_eid;
            int start_eid = eid;
            int fid = dcel->edges[dcel->edges[eid].twin_eid].left_fid;
            float weight = dcel->edges[eid].weight;
            if(fid != -1)
                affinity_matrix(i,fid) = std::exp(-weight);
            else
            {
                affinity_matrix(i, infty_id) = 1;
                affinity_matrix(infty_id, i) = 1;
            }
            eid = dcel->edges[eid].succ_eid;
            while(eid != start_eid)
            {
                fid = dcel->edges[dcel->edges[eid].twin_eid].left_fid;  
                weight = dcel->edges[eid].weight;
                // double free or corruption! Looking for this bug spend me 1 hour.
                if(fid != -1)
                    affinity_matrix(i, fid) = std::exp(-weight);
                else
                {
                    affinity_matrix(i, infty_id) = 1;
                    affinity_matrix(infty_id, i) = 1;
                }
                eid = dcel->edges[eid].succ_eid;            
            }
        }
        affinity_matrix(infty_id, infty_id) = 1;
        
        geometry::MatrixX D;// = Eigen::Matrix<float, dcel->faces.size(), dcel->faces.size()>();
        D.resize(dcel->faces.size()+1, dcel->faces.size()+1);
        D.setZero();
        for(size_t i = 0; i != dcel->faces.size() + 1 ; ++i)
        {
            D(i,i) = affinity_matrix.block(i,0,1, dcel->faces.size() + 1).sum();
        }
        diffusion_matrix = D.inverse() * affinity_matrix;
        //std::cout<<diffusion_matrix<<std::endl<<std::endl;

        Eigen::EigenSolver<geometry::MatrixX> eig(diffusion_matrix);       
        geometry::MatrixX eigenvalue = eig.eigenvalues().real().asDiagonal().toDenseMatrix();                
        geometry::MatrixX eigenvector = eig.eigenvectors().real();
        //std::cout<<eig.eigenvalues()<<std::endl<<std::endl;
        //std::cout<<eigenvalue<<std::endl<<std::endl;

        //std::cout<< eigenvector * eigenvalue * eigenvector.inverse()<<std::endl;
        
        for(size_t i = 0; i!=dcel->faces.size() + 1; ++i)
            eigenvalue(i,i) = std::pow(eigenvalue(i,i),t);       
    
        geometry::MatrixX help = eigenvalue * eigenvector.transpose();
        for(size_t i = 0; i!=dcel->faces.size() + 1; ++i)
        {
            auto tmp = help.block(0,i, m, 1);
            embeddings.push_back(tmp);
            //std::cout<<"embedding "<<i<<": "<<embeddings[i].transpose()<<std::endl;
        }

        distance_matrix.resize(dcel->faces.size() + 1, dcel->faces.size() + 1);

        for(size_t i = 0; i!= dcel->faces.size()+ 1; ++i)
        {
            for(size_t j = 0; j!= dcel->faces.size() + 1; ++j)
            distance_matrix(i,j) = (embeddings[i] - embeddings[j]).norm();
        }
        //std::cout<<distance_matrix<<std::endl;
        cv::Mat img_matrix = visualization::MatrixImage(distance_matrix);
        cv::imwrite("./distance_matrix.png",img_matrix);
    }

    void SeparateRoom()
    {
        //Use kmedoids to separate room
        //SeparateRoom
        int room_number = 1;
        std::set<int> un_labeled_face;
        for(size_t i = 0; i != dcel->faces.size()+1; ++i)
        un_labeled_face.insert(i);
        int infty_id = dcel->faces.size();
        for(int i = 0; i != room_number; ++i)
        {
            std::cout<<"separating the "<<i<<"th room..."<<std::endl;
            std::vector<int> un_labeled_face_v(un_labeled_face.begin(), un_labeled_face.end());
            float max_distance = std::numeric_limits<float>::lowest ();
            geometry::PointXList wait_to_clustering;
            std::vector<algorithm::ClusterDynamic> cluster_result;
            int max_index = -1;
            for(size_t j = 0; j != un_labeled_face_v.size(); ++j)
            {
                float distance = distance_matrix(infty_id,un_labeled_face_v[j]);
                if( distance > max_distance)
                {
                    max_index = j;
                    max_distance = distance; 
                }
                wait_to_clustering.push_back(embeddings[un_labeled_face_v[j]]);
            }                                                                                                                                                                                                                                               
            std::vector<int> initial_index;
            initial_index.push_back(max_index);
            initial_index.push_back(un_labeled_face_v.size()-1);
            algorithm::KMedoidsClusteringDynamic(wait_to_clustering, cluster_result, 2, true, initial_index);
            
            rooms.push_back(std::vector<int>());
            for(size_t j = 0; j != cluster_result[0].indexs.size(); ++j)
            {
                //std::cout<<cluster_result[0].indexs[j]<<std::endl;
                un_labeled_face.erase(un_labeled_face_v[cluster_result[0].indexs[j]]);
                rooms.back().push_back(un_labeled_face_v[cluster_result[0].indexs[j]]);
            }
        }
        /*
        for(size_t i = 0; i != rooms.size(); ++i)                                                                                                      
        {
            std::cout<<"Room "<<i<<":";
            for(size_t j = 0; j != rooms[i].size(); ++j)
            std::cout<<" "<<rooms[i][j];
            std::cout<<std::endl;

            for(size_t j = 0; j != rooms[i].size(); ++j)
            {
                for(size_t k = 0; k != rooms[i].size(); ++k)
                    std::cout<<distance_matrix(rooms[i][j], rooms[i][k])<<" ";
                    std::cout<<distance_matrix(rooms[i][j], infty_id);
                std::cout<<std::endl;
            }
        }*/
    }

    cv::Mat RoomImage()
    {
        cv::Mat img (700, 700, CV_8UC3);
        img = cv::Scalar::all(0);

        geometry::Point2 max_xy = dcel->right_up - dcel->left_bottom;
        float max_c = std::max(max_xy(0), max_xy(1));
        float scalar = 600 / max_c; 
        for(size_t i = 0; i != rooms.size(); ++i)
        {
            for(size_t j = 0; j != rooms[i].size(); ++ j)
            {
                int fid = rooms[i][j];
                std::vector<cv::Point> poly; 
                int start_eid = dcel->faces[fid].inc_eid;
                int start_vid = dcel->edges[start_eid].origin_vid;
                poly.push_back(cv::Point2i((dcel->vertexs[start_vid].coor(0) - dcel->left_bottom(0) ) *scalar, 
                    (dcel->vertexs[start_vid].coor(1)-dcel->left_bottom(1))* scalar));
                start_eid = dcel->edges[start_eid].succ_eid;
                start_vid = dcel->edges[start_eid].origin_vid;
                while(start_eid != dcel->faces[fid].inc_eid)
                {

                    poly.push_back(cv::Point2i((dcel->vertexs[start_vid].coor(0) - dcel->left_bottom(0) ) *scalar, 
                        (dcel->vertexs[start_vid].coor(1)-dcel->left_bottom(1))* scalar));
                    start_eid = dcel->edges[start_eid].succ_eid;
                    start_vid = dcel->edges[start_eid].origin_vid;
                }
                cv::fillConvexPoly(img,&poly[0],poly.size(),visualization::color_tab[i % visualization::color_tab.size()]);                
            }
        }
        return img;
    }
};

int main(int argc, char* argv[]) 
{
    if(argc != 2)
    {
        std::cout << "Usage: RoomDetection [filename.ply]"<<std::endl;
        return 0;
    }
    geometry::PointCloud pcd;
    pcd.LoadFromPLY(argv[1]);

    //compute the floor plane
    std::map<unsigned int, std::string> color_to_label;
    color_to_label[2564786688] = "floor";
    color_to_label[2932336640] = "wall";
    color_to_label[3592890368] = "door";
    geometry::Point3List f_points,w_points,d_points;
    geometry::Point2List _2d_points;
    geometry::Point3List _3d_points;
    for(size_t i = 0; i!=pcd.points.size(); ++i)
    {
            unsigned int c = pcd.colors[i][0] * 255;
            c = c << 8;
            c += pcd.colors[i][1] * 255;
            c = c << 8;
            c += pcd.colors[i][2] * 255;
            c = c << 8;
            if(color_to_label.find(c) != color_to_label.end() )
            {
                if(color_to_label[c] == "floor")
                f_points.push_back(pcd.points[i]);
                if(color_to_label[c] == "wall")
                w_points.push_back(pcd.points[i]);
                if(color_to_label[c] == "door")
                d_points.push_back(pcd.points[i]);
            }
    }
    geometry::Point3 n;
    float d, planar_indicator;
    std::tie(n,d,planar_indicator) = geometry::FitPlane(f_points);
    //std::cout<<n(0)<<" "<<n(1)<<" "<<n(2)<<" "<<d<<std::endl;
    geometry::Matrix3 W = n * geometry::Point3(0,0,1).transpose();
    Eigen::JacobiSVD<geometry::MatrixX> svd(W, Eigen::ComputeThinU | Eigen::ComputeThinV);
    auto UT = svd.matrixU().transpose();
    auto V = svd.matrixV();

    geometry::Matrix3 R =  V*UT;
    //std::cout<<R*n<<std::endl;
    _3d_points.insert(_3d_points.end(),w_points.begin(), w_points.end());
    _3d_points.insert(_3d_points.end(),d_points.begin(), d_points.end());


    //std::cout<<max_x<<" "<<min_x<<" "<<max_y<<" "<<min_y<<std::endl;
    //project the wall and door to the floor
    for(size_t i = 0; i!=w_points.size(); ++i)
    {
        _2d_points.push_back((R * w_points[i]).head<2>());
    }
    for(size_t i = 0; i!=d_points.size(); ++i)
    {
        _2d_points.push_back((R * d_points[i]).head<2>());
    }
    std::vector<algorithm::LinePatch> l_patches;
    std::vector<algorithm::PlanePatch> p_patches;
   
    algorithm::LineDetection(_2d_points,l_patches);
    algorithm::PlaneDetection(_3d_points,p_patches);
    std::cout<<"find "<<l_patches.size()<<" lines."<<std::endl;
    std::cout<<"find "<<p_patches.size()<<" planes."<<std::endl;    
    cv::Mat img_line = visualization::PatchImage(l_patches);
    cv::imwrite("Patches.png", img_line);
    geometry::PointCloud pcd_3d;
    std::vector<geometry::Line> lines;
    for(size_t i = 0; i != l_patches.size(); ++i)
    {
        lines.push_back(geometry::Line(l_patches[i].rep));
    }
    geometry::Point2List inter_points;
    cv::Mat img_cell;
    algorithm::ComputeIntersect(lines, inter_points);
    std::cout<<inter_points.size()<<std::endl;
    //for(size_t i = 0; i!=inter_points.size(); ++i)
    //std::cout<<"["<<inter_points[i](0)<<" "<<inter_points[i](1)<<"]"<<std::endl;
    auto dcel = algorithm::CreateBoundingBoxDcel(inter_points);
    for(size_t i = 0; i!=lines.size(); ++i)
        dcel->IncrementLine(lines[i]);

    // img_cell = dcel->Draw();
    // cv::imwrite("cells0.png", img_cell);    
    // dcel->ReductLine(dcel->lines.size()-1);
    // img_cell = dcel->Draw();
    // cv::imwrite("cells1.png", img_cell);
    // dcel->ReductLine(6);
    // img_cell = dcel->Draw();
    // cv::imwrite("cells2.png", img_cell);
    // /*
    // dcel->ReductLine(6);
    // img_cell = dcel->Draw();
    // cv::imwrite("cells1.png", img_cell);
    // */
    // dcel->IncrementLine(lines.back());
    // dcel->IncrementLine(lines[2]);
    img_cell = dcel->Draw();
    cv::imwrite("cell_complex.png", img_cell);
    
    // geometry::LineSegment line_seg(dcel->left_bottom, dcel->right_up);
    // geometry::Line _line = geometry::LineFromSeg(line_seg);
    // dcel->IncrementLine(_line);
    // img_cell = dcel->Draw();
    // cv::imwrite("cells4.png", img_cell);
    // dcel->ReductLine(dcel->lines.size()-1);
    // img_cell = dcel->Draw();
    // cv::imwrite("cells5.png", img_cell);    
        

    // geometry::Point2 check_p = (dcel->left_bottom + dcel->right_bottom + dcel-> right_up + dcel->left_up)/4;
    // int face_id = dcel->GetFaceID(check_p);
    // std::cout<<"face_id: "<<face_id<<std::endl;
    // cv::imshow("fuck", img_cell);
    // cv::waitKey(0);
    Building building;
    building.dcel = dcel;
    building.lines = l_patches;
    building.SetEmbeddingDimension();
    building.ComputeWeightsForEachEdge();
    building.ComputeEmbedding();
    building.SeparateRoom(); 
    cv::Mat img_room = building.RoomImage();
    cv::imwrite("rooms.png",img_room);
    //ComputeWeightsForEachEdge(l_patches, edge_weights);
    

    std::cout<<"Programe finish."<<std::endl;
    
    pcd_3d = *visualization::PatchPointCloud(p_patches);


    pcd_3d.WriteToPLY("Patches.ply");
    

    /*
    visualization::Visualizer visualizer;
    visualizer.AddPointCloud(pcd_3d);
    visualizer.Show();
    */
/*
    for(auto iter = color_to_label.begin(); iter != color_to_label.end(); ++iter)
    {
        geometry::PointCloud help_pcd = pcd;
        unsigned int now = iter->first;
        for(size_t i = 0; i!=pcd.points.size(); ++i)
        {
                unsigned int c = pcd.colors[i][0] * 255;
                c = c << 8;
                c += pcd.colors[i][1] * 255;
                c = c << 8;
                c += pcd.colors[i][2] * 255;
                c = c << 8;
            if(c != now)
            {
                help_pcd.colors[i] = geometry::Point3(0,0,0);
            }
        }
        help_pcd.WriteToPLY("./label_"+std::to_string(now)+".ply");
    }
*/

    //get the plane parameter of floor

    return 0;
}