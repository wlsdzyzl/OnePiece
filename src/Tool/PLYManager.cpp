#include "PLYManager.h"
#include <iostream>
#include <fstream>

namespace one_piece
{
namespace tool
{
    using namespace tinyply;

    void PlyDataToPoint3List(const std::shared_ptr<PlyData> &p_data, geometry::Point3List &point_list)
    {
        if (p_data->t == tinyply::Type::FLOAT32)
        {
            size_t count = p_data->count;
            //std::cout<<count<<std::endl;
            point_list.resize(count);
            float *data_ptr = (float *)p_data->buffer.get(); 
            for(size_t i = 0; i < count; ++i)
            {
                point_list[i] = geometry::Point3((geometry::scalar)(*(data_ptr + i*3 + 0)), 
                    (geometry::scalar)(*(data_ptr + i*3 + 1)), (geometry::scalar)(*(data_ptr + i*3 + 2)));
            }
        }
        else if (p_data->t == tinyply::Type::FLOAT64)
        {
            size_t count = p_data->count;
            point_list.resize(count);
            double *data_ptr = (double *)p_data->buffer.get(); 
            for(size_t i = 0; i < count; ++i)
            {
                point_list[i] = geometry::Point3((geometry::scalar)(*(data_ptr + i*3 + 0)), 
                    (geometry::scalar)(*(data_ptr + i*3 + 1)), (geometry::scalar)(*(data_ptr + i*3 + 2)));
            }
        }
        else if(p_data->t == tinyply::Type::UINT8)//from uchar to float
        {
            size_t count = p_data->count;
            point_list.resize(count);
            unsigned char *data_ptr = (unsigned char *)p_data->buffer.get(); 
            for(size_t i = 0; i < count; ++i)
            {
                point_list[i] = geometry::Point3((*(data_ptr + i*3 + 0))/255.0, 
                    (*(data_ptr + i*3 + 1))/255.0, (*(data_ptr + i*3 + 2))/255.0);
            }
        }
        else
        {
            std::cout<<RED<<"[ERROR]::[ReadPLY]::Unknown Type."<<RESET<<std::endl;
        }
        // std::cout<<p_data->count<<" "<<point_list.size()<<std::endl;
    }
    void PlyDataToPoint3uiList(const std::shared_ptr<PlyData> &p_data, geometry::Point3uiList &point_list)
    {
        if (p_data->t == tinyply::Type::UINT32 || p_data->t == tinyply::Type::INT32)
        {
            size_t count = p_data->count;
            point_list.resize(count);
            unsigned int *data_ptr = (unsigned int *)p_data->buffer.get(); 
            for(size_t i = 0; i < count; ++i)
            {
                point_list[i] = geometry::Point3ui((unsigned int)(*(data_ptr + i*3 + 0)), 
                    (unsigned int)(*(data_ptr + i*3 + 1)), (unsigned int)(*(data_ptr + i*3 + 2)));
            }
        }

    }


    bool ReadPLY(const std::string &filename, geometry::Point3List &_points, geometry::Point3List &_normals,
        geometry::Point3List &_colors, geometry::Point3uiList &_triangles, 
        std::vector<AdditionalElement> & additional_labels)
    {
        std::unique_ptr<std::istream> file_stream;
        std::vector<uint8_t> byte_buffer;

        try
        {
            file_stream.reset(new std::ifstream(filename, std::ios::binary));
            if (!file_stream || file_stream->fail()) 
            {
                std::cout<<RED<<"[ERROR]::[ReadPLY]::failed to open "<<filename<<RESET<<std::endl;
                return false;
            }
            file_stream->seekg(0, std::ios::end);
            const float size_mb = file_stream->tellg() * float(1e-6);
            file_stream->seekg(0, std::ios::beg);

            PlyFile file;
            file.parse_header(*file_stream);
            std::cout << BLUE<<"[INFO]::[ReadPLY]::"<<std::endl;
            std::cout<< "\t[ply_header] Size: " << size_mb<<"MB" << std::endl;
            std::cout<< "\t[ply_header] Type: " << (file.is_binary_file() ? "binary" : "ascii") << std::endl;
            for (const auto & c : file.get_comments()) std::cout << "\t[ply_header] Comment: " << c << std::endl;
            for (const auto & c : file.get_info()) std::cout << "\t[ply_header] Info: " << c  << std::endl;

            for (const auto & e : file.get_elements())
            {
                std::cout << "\t[ply_header] element: " << e.name << " (" << e.size << ")" << std::endl;
                for (const auto & p : e.properties)
                {
                    std::cout << "\t[ply_header] \tproperty: " << p.name << " (type=" << tinyply::PropertyTable[p.propertyType].str << ")";
                    if (p.isList) std::cout << " (list_type=" << tinyply::PropertyTable[p.listType].str << ")";
                    std::cout << std::endl;
                }
            }
            std::cout<<RESET<<std::endl;
            // Because most people have their own mesh types, tinyply treats parsed data as structured/typed byte buffers. 
            // See examples below on how to marry your own application-specific data structures with this one. 
            std::shared_ptr<PlyData> vertices, normals, colors, faces;
            std::vector<std::shared_ptr<PlyData>> additional_elements_ptr(additional_labels.size());
            // The header information can be used to programmatically extract properties on elements
            // known to exist in the header prior to reading the data. For brevity of this sample, properties 
            // like vertex position are hard-coded: 
            try { vertices = file.request_properties_from_element("vertex", { "x", "y", "z" }); }
            catch (const std::exception & e) { std::cout << RED << "[ERROR]::[ReadPLY]::" << e.what() << RESET<< std::endl; return false;}

            try { normals = file.request_properties_from_element("vertex", { "nx", "ny", "nz" }); }
            catch (const std::exception & e) { }

            try { colors = file.request_properties_from_element("vertex", { "red", "green", "blue" }); }
            catch (const std::exception & e) { }
            if(!colors)
            {
                try { colors = file.request_properties_from_element("vertex", { "r", "g", "b" }); }
                catch (const std::exception & e) { }
            }
            try { faces = file.request_properties_from_element("face", { "vertex_indices" }, 3); }
            catch (const std::exception & e) { }
            if(!faces)
            {
                try { faces = file.request_properties_from_element("face", { "vertex_index" }, 3); }
                catch (const std::exception & e) { }
            }
            for(size_t i = 0; i != additional_labels.size(); ++i)
            {
                additional_elements_ptr[i] = file.request_properties_from_element(additional_labels[i].element_key, 
                    additional_labels[i].element_property);
            }

            file.read(*file_stream);

            std::cout << BLUE <<"[INFO]::[ReadPLY]::"<<std::endl;
            if (vertices)   std::cout << "\tRead " << vertices->count  << " total vertices "<< std::endl;
            if (normals)    std::cout << "\tRead " << normals->count   << " total vertex normals " << std::endl;
            if (colors)     std::cout << "\tRead " << colors->count << " total vertex colors " << std::endl;
            if (faces)     std::cout << "\tRead " << faces->count << " total faces" << std::endl;
            for(size_t i = 0; i != additional_labels.size(); ++i)
            {
                if(additional_elements_ptr[i])
                std::cout << "\tRead " << additional_elements_ptr[i]->count << " total "
                    <<additional_labels[i].element_key<<" "<<" additional elements "<< std::endl;
            }
            std::cout << RESET <<std::endl;

            {
                //processing points, normals, colors, triangles
                if(vertices)
                PlyDataToPoint3List(vertices, _points);
                if(normals)
                PlyDataToPoint3List(normals, _normals);
                if(colors)
                PlyDataToPoint3List(colors, _colors);
                if(faces)
                PlyDataToPoint3uiList(faces, _triangles);
                for(size_t i = 0; i != additional_labels.size(); ++i)
                {
                    if(additional_elements_ptr[i])
                    {
                        additional_labels[i].count = additional_elements_ptr[i]->count;
                        additional_labels[i].type = additional_elements_ptr[i]->t;
                        size_t num_bytes = additional_elements_ptr[i]->buffer.size_bytes();
                        additional_labels[i].data = new unsigned char[num_bytes];
                        additional_labels[i].byte_size = num_bytes;
                        //remember to delete!!
                        memcpy(additional_labels[i].data, additional_elements_ptr[i]->buffer.get(), num_bytes);
                    }
                }
            }
        }
        catch (const std::exception & e)
        {
            std::cout << RED << "[ERROR]::[ReadPLY]::Caught " << e.what() <<RESET<< std::endl;
            return false;
        }
        return true;        
    }
    bool WritePLY(const std::string &filename, const geometry::Point3List&points, 
        const geometry::Point3List &normals, const geometry::Point3List &colors, 
        const geometry::Point3uiList &triangles, 
        const std::vector<std::string> & comments,
        const std::vector<AdditionalElement> & additional_labels,
        bool use_ascii)
    {
        //size_t numPoints = points.size();
        bool has_normals = normals.size()>0 && normals.size() == points.size();
        bool has_colors = colors.size() > 0 && colors.size() == points.size();
        bool has_faces = triangles.size() > 0;

        std::filebuf fb;
        if(use_ascii)
        fb.open(filename , std::ios::out);
        else 
        fb.open(filename , std::ios::out | std::ios::binary);
        std::ostream outstream(&fb);
        if (outstream.fail())
        {
            std::cout <<RED << "[PLYReader]::[ERROR]::failed to open "<<filename<<RESET<<std::endl;
            return false;
        }

        PlyFile geometry_file;
        std::vector<geometry::scalar> vertices_buffer, normals_buffer;
        std::vector<unsigned char> colors_buffer;
        std::vector<unsigned int> faces_buffer;
        for(size_t i = 0; i != points.size(); ++i)
        {
            vertices_buffer.push_back(points[i](0));
            vertices_buffer.push_back(points[i](1));
            vertices_buffer.push_back(points[i](2));
        }
        if(has_normals)
        for(size_t i = 0; i != normals.size(); ++i)
        {
            normals_buffer.push_back(normals[i](0));
            normals_buffer.push_back(normals[i](1));
            normals_buffer.push_back(normals[i](2));
        }
        if(has_colors)
        for(size_t i = 0; i != colors.size(); ++i)
        {
            colors_buffer.push_back((unsigned char)(colors[i](0) * 255));
            colors_buffer.push_back((unsigned char)(colors[i](1) * 255));
            colors_buffer.push_back((unsigned char)(colors[i](2) * 255));
        }
        if(has_faces)
        for(size_t i = 0; i != triangles.size(); ++i)
        {
            faces_buffer.push_back((unsigned int)triangles[i](0));
            faces_buffer.push_back((unsigned int)triangles[i](1));
            faces_buffer.push_back((unsigned int)triangles[i](2));
        }        
        Type float_type = sizeof(geometry::scalar) == sizeof(float)? Type::FLOAT32 : Type::FLOAT64;
        geometry_file.add_properties_to_element("vertex", { "x", "y", "z" }, 
            float_type, vertices_buffer.size() / 3, reinterpret_cast<uint8_t*>(vertices_buffer.data()), Type::INVALID, 0);
        if(has_normals)
        geometry_file.add_properties_to_element("vertex", { "nx", "ny", "nz" },
            float_type, normals_buffer.size() / 3, reinterpret_cast<uint8_t*>(normals_buffer.data()), Type::INVALID, 0);
        if(has_colors)
        geometry_file.add_properties_to_element("vertex", { "red", "green", "blue" },
            Type::UINT8, colors_buffer.size() / 3, reinterpret_cast<uint8_t*>(colors_buffer.data()), Type::INVALID, 0);
        if(has_faces)
        geometry_file.add_properties_to_element("face", { "vertex_indices" },
            Type::UINT32, faces_buffer.size() / 3, reinterpret_cast<uint8_t*>(faces_buffer.data()), Type::UINT8, 3);

        for(size_t i = 0; i != additional_labels.size(); ++i)
        {   
            geometry_file.add_properties_to_element(additional_labels[i].element_key, additional_labels[i].element_property,
                additional_labels[i].type, additional_labels[i].count, 
                reinterpret_cast<uint8_t*>(additional_labels[i].data), additional_labels[i].list_type, additional_labels[i].list_count);            
        }

        geometry_file.get_comments().push_back("generated by OnePiece");
        for(size_t i = 0;  i != comments.size(); ++i)
        {
            geometry_file.get_comments().push_back(comments[i]);
        }

        if(use_ascii)
        geometry_file.write(outstream, false);
        else
        geometry_file.write(outstream, true);
        return true;
    }

}
}