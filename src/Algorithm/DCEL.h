#ifndef DCEL_H
#define DCEL_H

#include "Geometry/Geometry2d.h"

namespace fucking_cool
{
namespace algorithm
{
    struct Vertex;
    struct Edge;
    struct Face;
    //LineSegment
    struct Vertex
    {
        int id;//index
        geometry::Point2 coor;//coordinate of v
        int inc_eid = -1;//Reference to the first outgoing incident half-edge
        int inter_count = 0;//if inter count == 0, the vertex can be deleted.
        Vertex() = default;
        bool is_valid = true;
        Vertex(const geometry::Point2 &c, int _id = 0)
        {
            coor = c;
            id = _id;
        }
    };

    struct Edge//Half Edge
    {
        int id;//index
        bool is_valid = true;
        int twin_eid = -1;// Another twin 
        int left_fid = -1;//reference to left incident face;
        int origin_vid = -1;// origin vertex
        int des_vid = -1;//destination vertex
        int pred_eid = -1;// previous half edge
        int succ_eid = -1;// next half edge
        int line_id = -1;//$(line_id) line create this edge
        float weight = 0;
        Edge() = default;
        Edge(int _id)
        {
            id = _id;
        }
    };
    struct Face
    {
        int id;//index
        int inc_eid = -1;//Reference to the first incident half-edge
        bool is_valid = true;
        Face()=default;
        Face(int _id)
        {
            id = _id;
        }
    };

    struct DCELLine
    {
        int id = -1;
        geometry::Line para;
        DCELLine(int _id)
        {
            id = _id;
        }
        DCELLine(int _id, const geometry::Line &_para)
        {
            id = _id;
            para = _para;
        }
    };
    class DCEL
    {
        public:
        //Initial With Bounding Box
        void InitialWithBB(const geometry::Point2 & _left_bottom, const geometry::Point2 & _right_bottom,
            const geometry::Point2 &_right_up, const geometry::Point2 &_left_up);
        int AddEdge(int vid1, int vid2)
        {
            if(vid1 == vid2)
            return -1;
            Edge e1;
            e1.id = AllocateEID();

            e1.origin_vid = vid1;
            e1.des_vid = vid2;

            if(vertexs[vid1].inc_eid == -1)
                vertexs[vid1].inc_eid = e1.id;
            InsertEdge(e1);
            return e1.id;
        }
        int AllocateEID()
        {
            if(eid_pool.empty())
            return edges.size();
            else
            {
                int eid = *(eid_pool.begin());
                eid_pool.erase(eid);
                return eid;
            }
        }
        int AllocateFID()
        {
            if(fid_pool.empty())
            return faces.size();
            else
            {
                int fid = *(fid_pool.begin());
                fid_pool.erase(fid);
                return fid;
            }
        }
        int AllocateVID()
        {
            if(vid_pool.empty())
            return vertexs.size();
            else
            {
                int vid = *(vid_pool.begin());
                vid_pool.erase(vid);
                return vid;
            }
        }
        int AllocateLID()
        {
            if(lid_pool.empty())
            return lines.size();
            else
            {
                int lid = *(lid_pool.begin());
                lid_pool.erase(lid);
                return lid;
            }
        }
        void InsertFace(const Face &f)
        {
            if(f.id == static_cast<int>(faces.size()))
            faces.push_back(f);
            else if(f.id < static_cast<int>(faces.size()))
            faces[f.id] = f;
            else 
            {
                std::cout<<RED<<"[LineArrangement]::[ERROR]::Wrong face id!"<<RESET<<std::endl;
                exit(1);               
            }
        }
        void InsertEdge(const Edge &e)
        {
            if(e.id == static_cast<int>(edges.size()))
            edges.push_back(e);
            else if(e.id < static_cast<int>(edges.size()))
            edges[e.id] = e;
            else 
            {
                std::cout<<RED<<"[LineArrangement]::[ERROR]::Wrong edge id!"<<RESET<<std::endl;
                exit(1);               
            }
        }
        void InsertLine(const DCELLine &l)
        {
            if(l.id == static_cast<int>(lines.size()))
            {
            lines.push_back(l);
            line_to_edge.push_back(std::set<int>());
            }
            else if(l.id< static_cast<int>(lines.size()))
            {
            lines[l.id] = l;
            line_to_edge[l.id].clear();
            }
            else
            {
                std::cout<<RED<<"[LineArrangement]::[ERROR]::Wrong line id!"<<RESET<<std::endl;
                exit(1);                  
            }
            
        }
        void InsertVertex(const Vertex & v)
        {
            if(v.id == static_cast<int>(vertexs.size()))
            vertexs.push_back(v);
            else if(v.id < static_cast<int>(vertexs.size()))
            vertexs[v.id] = v;
            else 
            {
                std::cout<<RED<<"[LineArrangement]::[ERROR]::Wrong vertex id!"<<RESET<<std::endl;
                exit(1);               
            }
        }
        void MakeTwin(int eid1, int eid2)
        {
            if(eid1 == eid2) return;
            edges[eid1].twin_eid = eid2;
            edges[eid2].twin_eid = eid1;
        }
        void EraseEdge(int eid)
        {
            if(eid >=static_cast<int>( edges.size()) || eid_pool.find(eid) != eid_pool.end())
            {
                std::cout<<YELLOW<<"[LineArrangement]::[WARNING]::Edge id does not exist!"<<RESET<<std::endl;
                return;                
            }
            eid_pool.insert(eid);
            edges[eid].is_valid = false;
        }
        void EraseLine(int lid)
        {
            if(lid >= static_cast<int>(lines.size()) || lid_pool.find(lid) != lid_pool.end())
            {
                std::cout<<YELLOW<<"[LineArrangement]::[WARNING]::Line id does not exist!"<<RESET<<std::endl;
                return;                
            }
            lid_pool.insert(lid);     
            line_to_edge[lid].clear();       
        }
        void EraseFace(int fid)
        {
            if(fid >= static_cast<int>(faces.size()) || fid_pool.find(fid) != fid_pool.end())
            {
                std::cout<<YELLOW<<"[LineArrangement]::[WARNING]::face id does not exist!"<<RESET<<std::endl;
                return;
            }
            fid_pool.insert(fid);
            faces[fid].is_valid = false;
        }
        void EraseVertex(int vid)
        {
            if(vid >=static_cast<int>( vertexs.size()) || vid_pool.find(vid) != vid_pool.end())
            {
                std::cout<<YELLOW<<"[LineArrangement]::[WARNING]::vertex id does not exist!"<<RESET<<std::endl;
                return;                
            }
            vid_pool.insert(vid);
            vertexs[vid].is_valid = false;
        }
        void MergeEdge(int t_eid_pred, int eid_succ)
        {
#if DEBUG_MODE
            if(edges[t_eid_pred].line_id  !=  edges[eid_succ].line_id)
            {
                std::cout<<YELLOW<<"[LineArrangement]::[WARNING]::The two edge are not on the same line."<<RESET<<std::endl;
                return ;
            }
#endif
            edges[t_eid_pred].des_vid = edges[eid_succ].des_vid;
            edges[t_eid_pred].succ_eid = edges[eid_succ].succ_eid;

            edges[edges[eid_succ].succ_eid].pred_eid = t_eid_pred;
        }
        void ConnectEdge(int eid1, int eid2)
        {
            if(eid1 == eid2) return;
            edges[eid1].succ_eid = eid2;

            edges[eid2].pred_eid = eid1;

            edges[eid2].left_fid = edges[eid1].left_fid;
        }
        int GetFaceID(const geometry::Point2 &point);
        cv::Mat Draw();
        geometry::LineSegment EdgeSegment(int eid)
        {
            return geometry::LineSegment(vertexs[edges[eid].origin_vid].coor, vertexs[edges[eid].des_vid].coor);
        }

        int GetFaceSize()
        {
            return faces.size() - fid_pool.size();
        }
        int GetEdgeSize()
        {
            return edges.size() - eid_pool.size();
        }
        int GetVertexSize()
        {
            return vertexs.size() - vid_pool.size();
        }
        int GetLineSize()
        {
            return lines.size() - lid_pool.size();
        }
        void IncrementLine(const geometry::Line &line);
        void ReductLine(int line_id);
        std::vector<Vertex> vertexs;
        std::vector<Edge> edges;
        std::vector<Face> faces;
        std::vector<DCELLine> lines;
        std::vector<std::set<int>> line_to_edge;
        std::set<int> fid_pool;
        std::set<int> eid_pool;
        std::set<int> vid_pool;
        std::set<int> lid_pool;
        geometry::Vector2 left_up, left_bottom, right_up, right_bottom;

    };
}
}
#endif