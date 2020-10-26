#include "OBJManager.h"
//#define TINYOBJLOADER_IMPLEMENTATION
#include "tiny_obj_loader.h"
#include <iostream>
#include <fstream>
#include <numeric>
namespace one_piece
{
namespace tool
{
    bool ReadOBJ(const std::string &filename, geometry::Point3List &points, geometry::Point3List &normals, 
        geometry::Point3List &colors)
    {
        tinyobj::attrib_t attrib;
        std::vector<tinyobj::shape_t> shapes;
        std::vector<tinyobj::material_t> materials;
        std::string warn;
        std::string err;
        bool ret = tinyobj::LoadObj(&attrib, &shapes, &materials, &warn, &err,
                                    filename.c_str());
        if (!warn.empty()) {
            std::cout<<YELLOW<<"[WARNNING]::[LoadFromOBJ]::"<<warn<<" (Notice that currently we don't support texture, uv coordinate in OPLib, because I know little about it, I never working on it)"<<RESET<<std::endl;
        }
        if (!err.empty()) {
            std::cout<<RED<<"[ERROR]::[LoadFromOBJ]::Read OBJ failed: "<<err<<RESET<<std::endl;
        }

        if (!ret) {
            return false;
        }

        //std::cout<<shapes.size()<<std::endl;
        // copy vertex and data
        for (size_t vidx = 0; vidx < attrib.vertices.size(); vidx += 3) {
            tinyobj::real_t vx = attrib.vertices[vidx + 0];
            tinyobj::real_t vy = attrib.vertices[vidx + 1];
            tinyobj::real_t vz = attrib.vertices[vidx + 2];
            points.push_back(geometry::Point3(vx, vy, vz));
            
        }

        for (size_t vidx = 0; vidx < attrib.colors.size(); vidx += 3) {
            tinyobj::real_t r = attrib.colors[vidx + 0];
            tinyobj::real_t g = attrib.colors[vidx + 1];
            tinyobj::real_t b = attrib.colors[vidx + 2];
            colors.push_back(geometry::Point3(r, g, b));
        }
        //std::cout<<points.size()<<std::endl;

        // resize normal data and create bool indicator vector
        normals.resize(points.size());
        std::vector<bool> normals_indicator(points.size(), false);

        // copy face data and copy normals data
        // append face-wise uv data
        for (size_t s = 0; s < shapes.size(); s++) 
        {
            size_t index_offset = 0;
            for (size_t f = 0; f < shapes[s].mesh.num_face_vertices.size(); f++) 
            {
                int fv = shapes[s].mesh.num_face_vertices[f];
                //std::cout<<shapes[s].mesh.num_face_vertices.size()<<std::endl;
                if (fv != 3) 
                {
                    std::cout<<RED<< "Read OBJ failed: facet with number of vertices not equal to 3"<<RESET<<std::endl;
                    return false;
                }

                geometry::Point3ui facet;
                for (int v = 0; v < fv; v++) 
                {
                    tinyobj::index_t idx = shapes[s].mesh.indices[index_offset + v];
                    int vidx = idx.vertex_index;
                    facet(v) = vidx;
                    
                    if (!attrib.normals.empty() && !normals_indicator[vidx] &&
                        (3 * idx.normal_index + 2) < int(attrib.normals.size())) 
                    {
                        tinyobj::real_t nx =
                                attrib.normals[3 * idx.normal_index + 0];
                        tinyobj::real_t ny =
                                attrib.normals[3 * idx.normal_index + 1];
                        tinyobj::real_t nz =
                                attrib.normals[3 * idx.normal_index + 2];
                        normals[vidx](0) = nx;
                        normals[vidx](1) = ny;
                        normals[vidx](2) = nz;
                        normals_indicator[vidx] = true;

                    }
                }
                //triangles.push_back(facet);
                
                index_offset += fv;
            }
        }

        // if not all normals have been set, then remove the vertex normals
        bool all_normals_set =
                std::accumulate(normals_indicator.begin(), normals_indicator.end(),
                                true, [](bool a, bool b) { return a && b; });
        if (!all_normals_set) {
            normals.clear();
        }
        std::cout <<BLUE<<"[OBJManager]::[INFO]::vertex: "<<points.size()<<RESET<<std::endl;
        
        return true;
    }
    bool ReadOBJ(const std::string &filename, geometry::Point3List &points, geometry::Point3List &normals, 
        geometry::Point3List &colors, geometry::Point3uiList &triangles)
    {
        tinyobj::attrib_t attrib;
        std::vector<tinyobj::shape_t> shapes;
        std::vector<tinyobj::material_t> materials;
        std::string warn;
        std::string err;
        bool ret = tinyobj::LoadObj(&attrib, &shapes, &materials, &warn, &err,
                                    filename.c_str());
        
        if (!warn.empty()) {
            std::cout<<YELLOW<<"[WARNNING]::[LoadFromOBJ]::"<<warn<<" (Notice that currently we don't support texture, uv coordinate in OPLib, because I know little about it, I never working on it)"<<RESET<<std::endl;
        }
        
        if (!err.empty()) {
            std::cout<<RED<<"[ERROR]::[LoadFromOBJ]::Read OBJ failed: "<<err<<RESET<<std::endl;
        }

        if (!ret) {
            return false;
        }
        //std::cout<<shapes.size()<<std::endl;
        // copy vertex and data
        for (size_t vidx = 0; vidx < attrib.vertices.size(); vidx += 3) {
            tinyobj::real_t vx = attrib.vertices[vidx + 0];
            tinyobj::real_t vy = attrib.vertices[vidx + 1];
            tinyobj::real_t vz = attrib.vertices[vidx + 2];
            points.push_back(geometry::Point3(vx, vy, vz));
            
        }

        for (size_t vidx = 0; vidx < attrib.colors.size(); vidx += 3) {
            tinyobj::real_t r = attrib.colors[vidx + 0];
            tinyobj::real_t g = attrib.colors[vidx + 1];
            tinyobj::real_t b = attrib.colors[vidx + 2];
            colors.push_back(geometry::Point3(r, g, b));
        }
        //std::cout<<points.size()<<std::endl;

        // resize normal data and create bool indicator vector
        normals.resize(points.size());
        std::vector<bool> normals_indicator(points.size(), false);

        // copy face data and copy normals data
        // append face-wise uv data
        for (size_t s = 0; s < shapes.size(); s++) 
        {
            size_t index_offset = 0;
            for (size_t f = 0; f < shapes[s].mesh.num_face_vertices.size(); f++) 
            {
                int fv = shapes[s].mesh.num_face_vertices[f];
                //std::cout<<shapes[s].mesh.num_face_vertices.size()<<std::endl;
                if (fv != 3) 
                {
                    std::cout<<RED<< "Read OBJ failed: facet with number of vertices not equal to 3"<<RESET<<std::endl;
                    return false;
                }

                geometry::Point3ui facet;
                for (int v = 0; v < fv; v++) 
                {
                    tinyobj::index_t idx = shapes[s].mesh.indices[index_offset + v];
                    int vidx = idx.vertex_index;
                    facet(v) = vidx;
                    
                    if (!attrib.normals.empty() && !normals_indicator[vidx] &&
                        (3 * idx.normal_index + 2) < int(attrib.normals.size())) 
                    {
                        tinyobj::real_t nx =
                                attrib.normals[3 * idx.normal_index + 0];
                        tinyobj::real_t ny =
                                attrib.normals[3 * idx.normal_index + 1];
                        tinyobj::real_t nz =
                                attrib.normals[3 * idx.normal_index + 2];
                        normals[vidx](0) = nx;
                        normals[vidx](1) = ny;
                        normals[vidx](2) = nz;
                        normals_indicator[vidx] = true;

                    }
                }
                triangles.push_back(facet);
                
                index_offset += fv;
            }
        }

        // if not all normals have been set, then remove the vertex normals
        bool all_normals_set =
                std::accumulate(normals_indicator.begin(), normals_indicator.end(),
                                true, [](bool a, bool b) { return a && b; });
        if (!all_normals_set) {
            normals.clear();
        }
        std::cout <<BLUE<<"[OBJManager]::[INFO]::"<<"face: "<<triangles.size()<<" vertex: "<<points.size()<<RESET<<std::endl;
        return true;
    }
    bool WriteOBJ(const std::string &filename, const geometry::Point3List &points, const geometry::Point3List &normals, 
        const geometry::Point3List &colors)
    {
        bool write_vertex_normals = true;
        bool write_vertex_colors  = true;
        std::ofstream file(filename.c_str(), std::ios::out | std::ios::binary);
        if (!file) {
            std::cout<<RED<< "Write OBJ failed: unable to open file."<<RESET<<std::endl;
            return false;
        }
        file << "# Created by OPLib " << std::endl;
        //file << "# object name: " << object_name << std::endl;
        file << "# number of vertices: " << points.size() << std::endl;

        // always write material name in obj file, regardless of uvs or textures
        //file << "mtllib " << object_name << ".mtl" << std::endl;
        //file << "usemtl " << object_name << std::endl;

        write_vertex_normals = write_vertex_normals && normals.size() != 0 && normals.size() == points.size();
        write_vertex_colors = write_vertex_colors && colors.size()!=0 && colors.size() == points.size();
        for (size_t vidx = 0; vidx < points.size(); ++vidx) {
            const geometry::Point3& vertex = points[vidx];
            file << "v " << vertex(0) << " " << vertex(1) << " " << vertex(2);
            if (write_vertex_colors) {
                const geometry::Point3 & color = colors[vidx];
                file << " " << color(0) << " " << color(1) << " " << color(2);
            }
            file << std::endl;

            if (write_vertex_normals) {
                const geometry::Point3 & normal = normals[vidx];
                file << "vn " << normal(0) << " " << normal(1) << " " << normal(2)
                    << std::endl;
            }
        }
        file.close();
        return true;        
    }
    bool WriteOBJ(const std::string &filename, const geometry::Point3List &points, const geometry::Point3List &normals, 
        const geometry::Point3List &colors, const geometry::Point3uiList &triangles)
    {
        bool write_vertex_normals = true;
        bool write_vertex_colors  = true;
        std::ofstream file(filename.c_str(), std::ios::out | std::ios::binary);
        if (!file) {
            std::cout<<RED<< "Write OBJ failed: unable to open file."<<RESET<<std::endl;
            return false;
        }
        file << "# Created by OPLib " << std::endl;
        //file << "# object name: " << object_name << std::endl;
        file << "# number of vertices: " << points.size() << std::endl;
        file << "# number of triangles: " << triangles.size() << std::endl;

        // always write material name in obj file, regardless of uvs or textures
        //file << "mtllib " << object_name << ".mtl" << std::endl;
        //file << "usemtl " << object_name << std::endl;

        write_vertex_normals = write_vertex_normals && normals.size() != 0 && normals.size() == points.size();
        write_vertex_colors = write_vertex_colors && colors.size()!=0 && colors.size() == points.size();
        for (size_t vidx = 0; vidx < points.size(); ++vidx) {
            const geometry::Point3& vertex = points[vidx];
            file << "v " << vertex(0) << " " << vertex(1) << " " << vertex(2);
            if (write_vertex_colors) {
                const geometry::Point3 & color = colors[vidx];
                file << " " << color(0) << " " << color(1) << " " << color(2);
            }
            file << std::endl;

            if (write_vertex_normals) {
                const geometry::Point3 & normal = normals[vidx];
                file << "vn " << normal(0) << " " << normal(1) << " " << normal(2)
                    << std::endl;
            }
        }

        // we are less strict and allows writing to uvs without known material
        // potentially this will be useful for exporting conformal map generation

        for (size_t tidx = 0; tidx < triangles.size(); ++tidx) {
            const geometry::Point3ui& triangle = triangles[tidx];
            if (write_vertex_normals) 
            {
                file << "f " << triangle(0) + 1 << "//" << triangle(0) + 1 << " "
                    << triangle(1) + 1 << "//" << triangle(1) + 1 << " "
                    << triangle(2) + 1 << "//" << triangle(2) + 1 << std::endl;
            }
            else 
            {
                file << "f " << triangle(0) + 1 << " " << triangle(1) + 1 << " "
                    << triangle(2) + 1 << std::endl;
            }
        }
        file.close();
        return true;
    }
}
}