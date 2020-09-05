#include "MarchingCube.h"
#include <iostream>
namespace fucking_cool
{
namespace integration
{
    
    geometry::Point3 InterpolateEdgeVetex(const geometry::Point3 &corner1, const geometry::Point3 &corner2, float sdf1, float sdf2)
    {
        float sdf_diff = sdf2 - sdf1;
        float t = sdf1 / sdf_diff;
        //std::cout<<"sdf diff: "<<sdf_diff<<" corner1: "<<corner1<<" corner2: "<<corner2<<std::endl;
        return  corner1 - t*(corner2 - corner1);
    }

    int DetermineCase(const std::vector<TSDFVoxel > &corner_sdf)
    {
        if(corner_sdf.size() != 8) return -1;
        return  (corner_sdf[0].sdf > 0 ? 1<< 0 : 0)|
                (corner_sdf[1].sdf > 0 ? 1<< 1 : 0)|
                (corner_sdf[2].sdf > 0 ? 1<< 2 : 0)|
                (corner_sdf[3].sdf > 0 ? 1<< 3 : 0)|
                (corner_sdf[4].sdf > 0 ? 1<< 4 : 0)|
                (corner_sdf[5].sdf > 0 ? 1<< 5 : 0)|
                (corner_sdf[6].sdf > 0 ? 1<< 6 : 0)|
                (corner_sdf[7].sdf > 0 ? 1<< 7 : 0);
    }

    void MarchingCube(const geometry::Point3List &corners, const std::vector<TSDFVoxel> &corner_sdf, geometry::TriangleMesh &mesh)
    {
        //choose the right cases
        int CaseIndex = DetermineCase(corner_sdf);
        int * Edges = MCLookTable[CaseIndex];
        std::map<int, geometry::Point3> edge_vertex;
        std::map<int, geometry::Point3> edge_color;
        int index = mesh.points.size();
        Eigen::Vector3i triangle;
        for(int i = 0;i < 16; ++i)
        {

            if(Edges[i] == -1)
            break;
            if(edge_vertex.find(Edges[i]) != edge_vertex.end())
                ;
            else
            {
                int *pair = EdgeIndexPairs[Edges[i]];
                geometry::Point3 corner1 = corners[pair[0]];
                geometry::Point3 corner2 = corners[pair[1]];
                edge_vertex[Edges[i]] = InterpolateEdgeVetex(corner1, corner2, corner_sdf[pair[0]].sdf, corner_sdf[pair[1]].sdf);
                edge_color[Edges[i]] = (corner_sdf[pair[0]].color+ corner_sdf[pair[1]].color)/2;

            }

            mesh.points.push_back(edge_vertex[Edges[i]]);
            mesh.colors.push_back(edge_color[Edges[i]]);
            triangle(i%3) = index ++;
            if(i % 3 == 2)
            {
                mesh.triangles.push_back(triangle);
                //std::cout<<triangle<<std::endl<<std::endl;
            }
        }
    }    
    
}
}