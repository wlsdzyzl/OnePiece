#include "DCEL.h"
#include "Visualization/DrawImage.h"
namespace one_piece
{
namespace algorithm
{
    cv::Mat DCEL::Draw()
    {
        return visualization::DCELImage(*this);
    }
    int DCEL::GetFaceID(const geometry::Point2 &point)
    {
        // compute the face id where the point is

        int in_bounding = 0;
        geometry::Point2List poly;
        poly.push_back(left_bottom);
        poly.push_back(right_bottom);
        poly.push_back(right_up);
        poly.push_back(left_up);
        in_bounding = geometry::CheckPointInConvexPoly(poly, point);
        if(in_bounding == 0)
        {
            std::cout<<"Not in the bounding box!"<<std::endl;
            return -1;
        }
        for(size_t i = 0; i != faces.size(); ++ i)
        {
            if(faces[i].is_valid == false)
            continue;
            poly.clear();
            int eid = faces[i].inc_eid;
            int start_eid = eid;
            poly.push_back(vertexs[edges[eid].origin_vid].coor);
            eid = edges[eid].succ_eid;
            while(eid != start_eid)
            {
                poly.push_back(vertexs[edges[eid].origin_vid].coor);  
                eid = edges[eid].succ_eid;              
            }
            
            in_bounding = geometry::CheckPointInConvexPoly(poly,point);
            if(in_bounding == 1)
            return i;
            
        }
        return -1;

    }
    void DCEL::InitialWithBB(const geometry::Point2 & _left_bottom, const geometry::Point2 &_right_bottom,
    const geometry::Point2 &_right_up, const geometry::Point2 &_left_up)
    {
        left_bottom = _left_bottom;
        right_bottom = _right_bottom;
        left_up  = _left_up;
        right_up = _right_up;

        Vertex v0(left_bottom, 0), v1(right_bottom, 1), v2(right_up, 2), v3(left_up, 3);
        v0.inter_count = 1;
        v1.inter_count = 1;
        v2.inter_count = 1;
        v3.inter_count = 1;
        // add vertex
        InsertVertex(v0);
        InsertVertex(v1);
        InsertVertex(v2);
        InsertVertex(v3);
        // add edge
        int eid_1, eid_2, eid_3, eid_4, eid_5, eid_6, eid_7, eid_8;
        DCELLine l0(0), l1(1), l2(2), l3(3);
        InsertLine(l0);
        InsertLine(l1);
        InsertLine(l2);
        InsertLine(l3); 
        eid_1 = AddEdge(v0.id, v1.id);
        edges[eid_1].line_id = 0;
        eid_2 = AddEdge(v1.id, v2.id);
        edges[eid_2].line_id = 1;
        eid_3 = AddEdge(v2.id, v3.id);
        edges[eid_3].line_id = 2;
        eid_4 = AddEdge(v3.id, v0.id);
        edges[eid_4].line_id = 3;
        Face f(0);
        f.inc_eid = eid_1;
        InsertFace(f);
        edges[eid_1].left_fid = f.id;

        ConnectEdge(eid_1, eid_2);
        ConnectEdge(eid_2, eid_3);
        ConnectEdge(eid_3, eid_4);
        ConnectEdge(eid_4, eid_1);

        eid_5 = AddEdge(v1.id, v0.id);
        edges[eid_5].line_id = 0;
        eid_6 = AddEdge(v2.id, v1.id);
        edges[eid_6].line_id = 1;
        eid_7 = AddEdge(v3.id, v2.id);
        edges[eid_7].line_id = 2;
        eid_8 = AddEdge(v0.id, v3.id);
        edges[eid_8].line_id = 3;
        MakeTwin(eid_1, eid_5);
        MakeTwin(eid_2, eid_6);
        MakeTwin(eid_3, eid_7);
        MakeTwin(eid_4, eid_8);
        
    }
    //Only compare x, to find the leftmost edge
    bool CompareWithEdgeIndex(const std::pair<geometry::Vector2, int> & a, const std::pair<geometry::Vector2, int> &b)
    {
        return a.first(0) < b.first(0);
    }
    void DCEL::IncrementLine(const geometry::Line &line)
    {
        std::set<int> edge_ids;
        for(size_t i = 0; i!=edges.size(); ++i)
        {
            if(edges[i].is_valid == false) continue;
            if(edge_ids.find(edges[i].twin_eid) == edge_ids.end())
                edge_ids.insert(edges[i].id);
        }
        std::vector<std::pair<geometry::Vector2, int>> inter_points;
        for(auto iter = edge_ids.begin(); iter != edge_ids.end(); ++iter)
        {
            geometry::LineSegment line_seg = EdgeSegment(*iter);
            if(geometry::IsIntersecting(line, line_seg))
            {
                geometry::Point2 point = LineSegIntersect(line, line_seg);
                auto item = std::make_pair(point,*iter);

                inter_points.push_back(item);
            }
        }
        //from left to right
        std::sort(inter_points.begin(), inter_points.end(), CompareWithEdgeIndex);
        
        auto tmp_inter_points = inter_points;
        inter_points.clear();
        for(size_t i = 0; i != tmp_inter_points.size(); ++i)
        {

            if(inter_points.size() == 0 || 
                geometry::Distance(tmp_inter_points[i].first, inter_points.back().first) > EPS)
            {
                inter_points.push_back(tmp_inter_points[i]);
            }
        }
        int new_eid0 = -1;
        int old_eid1 = -1;
        int vid1 = -1;
        int vid2 = -1;
        //std::cout<<"Add "<<inter_points.size()<<" vertexs."<<std::endl;
        int line_id = AllocateLID();
        DCELLine d_line(line_id, line);

        InsertLine(d_line);
        
        bool restart = true;
        for(size_t i = 0; i < inter_points.size(); ++i)
        {
            //std::cout<<i<<std::endl;
            //Check if the point is on the edge
            int on_edge = false;
            if(geometry::Distance(inter_points[i].first, vertexs[edges[inter_points[i].second].des_vid].coor) < EPS)
            {
                vid2 = edges[inter_points[i].second].des_vid;
                //vertexs[vid2].inter_count += 1;
                on_edge = 1;
            }
            else if(geometry::Distance(inter_points[i].first, vertexs[edges[inter_points[i].second].origin_vid].coor) < EPS)
            {
                vid2 = edges[inter_points[i].second].origin_vid;
                
                on_edge = 2;
            }
            else
            {
                Vertex v(inter_points[i].first);
                v.id = AllocateVID();
                InsertVertex(v);
                vid2 = v.id;
            }
#if DEBUG_MODE
            if(on_edge != 0)
            {
                std::cout<<YELLOW<<"[LineArrangement]::[WARNNING]::The intersect point is on a existing vertex."<<RESET<<std::endl;
            }
#endif
            vertexs[vid2].inter_count += 1;
            if(restart)
            {
                restart = false;
                old_eid1 = inter_points[i].second;
                if(edges[old_eid1].left_fid == -1)
                    old_eid1 = edges[old_eid1].twin_eid;
                if(on_edge == 0)
                {
                    new_eid0 = AddEdge(vid2, edges[old_eid1].des_vid);
                    int new_eid = AddEdge(vid2, edges[old_eid1].origin_vid);
                    int tmp_line_id = edges[old_eid1].line_id;
                    edges[new_eid0].line_id = tmp_line_id;
                    edges[new_eid].line_id = tmp_line_id;
                    if(tmp_line_id >= 4)
                    {
                        if(line_to_edge[tmp_line_id].find(old_eid1) == line_to_edge[tmp_line_id].end())
                        line_to_edge[tmp_line_id].insert(new_eid);
                        else line_to_edge[tmp_line_id].insert(new_eid0);
                        //line_to_edge[edges[old_eid1].line_id].insert(new_eid);
                    }
                    MakeTwin(new_eid0, edges[old_eid1].twin_eid);
                    MakeTwin(old_eid1, new_eid);
                    edges[old_eid1].des_vid = vid2;
                    edges[edges[new_eid0].twin_eid].des_vid = vid2;
                }
                else if(on_edge == 1)
                {
                    new_eid0 = edges[old_eid1].succ_eid;
                }
                else if(on_edge == 2)
                {
                    new_eid0 = old_eid1;
                    old_eid1 = edges[old_eid1].pred_eid;
                }
                vid1 = vid2;
                //std::cout<<old_eid1<<" " <<edges[old_eid1].left_fid<<std::endl; 
                continue;
            }

            Vertex &v1 = vertexs[vid1];
            Vertex &v2 = vertexs[vid2];

            int face_id = edges[old_eid1].left_fid;
            int old_eid2 = inter_points[i].second;


            if(edges[old_eid1].left_fid != edges[old_eid2].left_fid)
            {
                old_eid2 = edges[old_eid2].twin_eid;
            }

            int des2 = edges[old_eid2].des_vid;
            int origin2 = edges[old_eid2].origin_vid;
            int new_eid1, new_eid2, new_eid3, new_eid4;
            if(on_edge == 0)
            {
                new_eid1 = AddEdge(v2.id, des2);
                new_eid2 = AddEdge(v2.id, origin2);
                int tmp_line_id = edges[old_eid2].line_id;
                edges[new_eid1].line_id = tmp_line_id;
                edges[new_eid2].line_id = tmp_line_id;
                if(tmp_line_id >= 4)
                {
                    if(line_to_edge[tmp_line_id].find(old_eid2) == line_to_edge[tmp_line_id].end())
                    line_to_edge[tmp_line_id].insert(new_eid2);
                    else
                    line_to_edge[tmp_line_id].insert(new_eid1);
                    //line_to_edge[edges[old_eid2].line_id].insert(new_eid2);
                }
            }
            else if(on_edge == 1)
            {
                new_eid1 = edges[old_eid2].succ_eid;
                new_eid2 = edges[old_eid2].twin_eid;
            }
            else 
            {
                new_eid1 = old_eid2;
                new_eid2 = edges[edges[old_eid2].twin_eid].succ_eid;       
                old_eid2 = edges[old_eid2].pred_eid;         
            }
                
            new_eid3 = AddEdge(v2.id, v1.id);
            new_eid4 = AddEdge(v1.id, v2.id);

            edges[new_eid3].line_id = line_id;
            edges[new_eid4].line_id = line_id;
            line_to_edge[line_id].insert(new_eid3);
            //line_to_edge[line_id].insert(new_eid4);


            MakeTwin(new_eid3, new_eid4);

            if(on_edge == 0)
            {
                MakeTwin(new_eid1, edges[old_eid2].twin_eid);  
                MakeTwin(old_eid2, new_eid2);
                edges[old_eid2].des_vid = v2.id; 
                edges[edges[new_eid1].twin_eid].des_vid = v2.id;
            }


            Edge & old_edge1 = edges[old_eid1];
            Edge & old_edge2 = edges[old_eid2];
            
            //std::cout<<"on_edge: "<<on_edge<<std::endl;
            //if(on_edge != 1)
            ConnectEdge(new_eid0, old_edge1.succ_eid);
            //if(on_edge != 2)
            ConnectEdge(new_eid1, old_edge2.succ_eid);

            ConnectEdge(old_eid2, new_eid3);
            ConnectEdge(new_eid3, new_eid0);

            ConnectEdge(old_eid1, new_eid4);
            ConnectEdge(new_eid4, new_eid1);

            faces[face_id].inc_eid = old_edge1.id;
            int new_face_id = AllocateFID();
            Face new_face(new_face_id);

            new_face.inc_eid = old_edge2.id;
            InsertFace(new_face);
            //faces.push_back(new_face);
            
            old_edge1.left_fid = face_id;
            old_edge2.left_fid = new_face_id;
            //std::cout<<old_eid1<<" " <<edges[old_eid1].left_fid<<std::endl; 
            int tmp_eid = old_edge1.succ_eid;
            while(tmp_eid != old_edge1.id)
            {
                edges[tmp_eid].left_fid = face_id;
                tmp_eid = edges[tmp_eid].succ_eid;
                //std::cout<<tmp_eid<<std::endl;
            }
            tmp_eid = old_edge2.succ_eid;
            //std::cout<<tmp_eid<<" "<<old_edge2.id<<" "<<new_eid3<<std::endl;
            while(tmp_eid != old_edge2.id)
            {
                edges[tmp_eid].left_fid = new_face_id;
                tmp_eid = edges[tmp_eid].succ_eid;
                //std::cout<<tmp_eid<<std::endl;
            }
            //std::cout<<old_eid1<<" " <<edges[old_eid1].left_fid<<std::endl; 
            if(on_edge == 0)
            {
                new_eid0 = new_eid2;
                old_eid1 = edges[new_eid1].twin_eid;
            }
            else
            {
                new_eid0 = -1;
                old_eid1 = -1;
                int next_eid;
                if(i < inter_points.size() - 1)
                {
                    next_eid = inter_points[i+1].second;
                    if(edges[next_eid].des_vid == vid2 || edges[next_eid].origin_vid == vid2)
                        i+=1;
                }
                if(i < inter_points.size() - 1)
                {
                    next_eid = inter_points[i+1].second;
                    int next_fid = edges[next_eid].left_fid;
                    if(next_fid == -1)
                        next_fid = edges[edges[next_eid].twin_eid].left_fid;
                    else if(edges[edges[next_eid].twin_eid].left_fid == -1);
                    else
                    {
                        int tmp_eid = faces[next_fid].inc_eid;
                        int start_eid = tmp_eid;

                        if(edges[tmp_eid].origin_vid == vid2)
                        new_eid0 = tmp_eid;
                        else if(edges[tmp_eid].des_vid == vid2)
                        old_eid1 = tmp_eid;
                        
                        tmp_eid = edges[start_eid].succ_eid;
                        
                        while(tmp_eid != start_eid)
                        {
                        
                            if(edges[tmp_eid].origin_vid == vid2)
                            new_eid0 = tmp_eid;
                            else if(edges[tmp_eid].des_vid == vid2)
                            old_eid1 = tmp_eid;  
                            tmp_eid = edges[tmp_eid].succ_eid;                                                      
                        
                        }
                        if(new_eid0 == -1 &&  old_eid1 == -1)
                        {
                            next_fid = edges[edges[next_eid].twin_eid].left_fid;
                        }
                    }
                    if(new_eid0 == -1 && old_eid1 == -1)
                    {
                        int tmp_eid = faces[next_fid].inc_eid;
                        int start_eid = tmp_eid;

                        if(edges[tmp_eid].origin_vid == vid2)
                        new_eid0 = tmp_eid;
                        else if(edges[tmp_eid].des_vid == vid2)
                        old_eid1 = tmp_eid;
                        
                        tmp_eid = edges[start_eid].succ_eid;
                        
                        while(tmp_eid != start_eid)
                        {
                        
                            if(edges[tmp_eid].origin_vid == vid2)
                            new_eid0 = tmp_eid;
                            else if(edges[tmp_eid].des_vid == vid2)
                            old_eid1 = tmp_eid;  
                            tmp_eid = edges[tmp_eid].succ_eid;                                                      
                        
                        }
                    }
                    if(new_eid0 == -1 &&  old_eid1 == -1)
                    {
                        std::cout<<RED<<"[LineArrangement]::[ERROR]::Cannot find the correct face! "<<RESET<<std::endl;
                        std::exit(0);
                    }
                }
            }
            vid1 = vid2;
        }
    }

    void DCEL::ReductLine(int lid)
    {
        //erase line lid from dcel
        int eid, t_eid, eid_succ, eid_pred, t_eid_succ, t_eid_pred;
        std::set<int> wait_to_erase;
        std::set<int> pass_vid;
        std::cout<<"edge relied on line "<<lid<<": "<<line_to_edge[lid].size()<<std::endl;
        for(auto iter = line_to_edge[lid].begin(); 
            iter != line_to_edge[lid].end(); ++iter)
        {
            
            eid = *iter;
            if(edges[eid].is_valid == false)
            continue;
            t_eid = edges[eid].twin_eid;
#if DEBUG_MODE
            if(edges[eid].line_id != lid || edges[t_eid].line_id != lid)
            {
                std::cout<<YELLOW<<"[LineArrangement]::[WARNNING]::Edge not on the line which will be reducted."<<RESET<<std::endl;
            }
#endif
            eid_pred = edges[eid].pred_eid;
            eid_succ = edges[eid].succ_eid;
            t_eid_pred = edges[t_eid].pred_eid;
            t_eid_succ = edges[t_eid].succ_eid;
            pass_vid.insert(edges[eid].origin_vid);
            pass_vid.insert(edges[eid].des_vid); 
            if( edges[t_eid_pred].line_id == edges[eid_succ].line_id)
            {
                wait_to_erase.insert(eid_succ);
                wait_to_erase.insert(edges[t_eid_pred].twin_eid);
            }
            else
            std::cout<<"FUCK"<< edges[t_eid_pred].line_id <<" "<<edges[eid_succ].line_id<<std::endl;
            if(edges[eid_pred].line_id == edges[t_eid_succ].line_id)
            {
            wait_to_erase.insert(edges[eid_pred].twin_eid);
            wait_to_erase.insert(t_eid_succ);
            }
            else std::cout<<"FUCK"<< edges[eid_pred].line_id <<" "<<edges[t_eid_succ].line_id<<std::endl;
            
            wait_to_erase.insert(eid);
            wait_to_erase.insert(t_eid);
            int tmp_fid = edges[eid].left_fid;
            EraseFace(edges[t_eid].left_fid);
            faces[tmp_fid].inc_eid = eid_pred;
            int tmp_eid = t_eid_pred;
            while(edges[tmp_eid].left_fid != tmp_fid)
            {
                edges[tmp_eid].left_fid = tmp_fid;
                tmp_eid = edges[tmp_eid].pred_eid;
            }
        }
        for(auto iter = line_to_edge[lid].begin(); 
            iter != line_to_edge[lid].end(); ++iter)
        {
            eid = *iter;
            t_eid = edges[eid].twin_eid;
            eid_pred = edges[eid].pred_eid;
            eid_succ = edges[eid].succ_eid;
            t_eid_pred = edges[t_eid].pred_eid;
            t_eid_succ = edges[t_eid].succ_eid;
            if( edges[t_eid_pred].line_id == edges[eid_succ].line_id)
            {
                if(edges[eid_succ].line_id < 4 )
                {
                    //std::cout<<eid<<" "<<t_eid<<" "<<eid_succ<<std::endl;
                    MergeEdge(edges[eid_succ].twin_eid, edges[t_eid_pred].twin_eid);
                }
                MergeEdge(t_eid_pred, eid_succ);
                MakeTwin(t_eid_pred, edges[eid_succ].twin_eid);
            }
            else
            {
                ConnectEdge(t_eid_pred, eid_succ);
                if(edges[eid_succ].line_id < 4 )
                {
                    //std::cout<<eid<<" "<<t_eid<<" "<<eid_succ<<std::endl;
                    ConnectEdge(edges[eid_succ].twin_eid, edges[t_eid_pred].twin_eid);
                }
            }
            if(edges[eid_pred].line_id == edges[t_eid_succ].line_id)
            {
                if(edges[eid_pred].line_id < 4)
                {
                    //std::cout<<eid<<" "<<t_eid<<" "<<eid_pred<<std::endl;
                    MergeEdge( edges[t_eid_succ].twin_eid,edges[eid_pred].twin_eid);
                }
                MergeEdge(eid_pred, t_eid_succ);
                MakeTwin(eid_pred, edges[t_eid_succ].twin_eid);
            }
            else
            {
                if(edges[eid_pred].line_id < 4)
                {
                    //std::cout<<eid<<" "<<t_eid<<" "<<eid_pred<<std::endl;
                    ConnectEdge( edges[t_eid_succ].twin_eid,edges[eid_pred].twin_eid);
                }
                ConnectEdge(eid_pred, t_eid_succ);              
            }
            
            
        }
        for(auto iter = wait_to_erase.begin(); 
            iter != wait_to_erase.end(); ++iter)
        {
            //std::cout<<*iter<<std::endl;
            EraseEdge(*iter);
        }

        for(auto iter = pass_vid.begin(); iter != pass_vid.end();
            ++iter)
        {
            int vid = *iter;
            vertexs[vid].inter_count-=1;
            if(vertexs[vid].inter_count == 0)
            EraseVertex(vid); 
        }
        EraseLine(lid);
    }
}
}