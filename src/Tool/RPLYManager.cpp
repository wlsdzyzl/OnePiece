

#include <iostream>
#include <fstream>
#include "RPLYManager.h"

namespace fucking_cool
{
namespace tool
{
//cb means callback
//create by Guoqing Zhang 2019.12.18
    typedef geometry::Point3List VertexSet;
    typedef geometry::Point3List NormalSet;
    typedef std::vector<Eigen::Vector3i> TriangleSet;
    typedef geometry::Point3List ColorSet;

    struct PlyState
    {
        int vertex_index;
        int vertex_num;
        int normal_index;
        int normal_num;
        int color_index;
        int color_num;
        int triangle_index;
        int triangle_num;
        VertexSet *vertex_ptr;
        NormalSet *normal_ptr;
        ColorSet *color_ptr;
        TriangleSet *triangle_ptr;
    };


    int vertex_cb(p_ply_argument argument) 
    {
        long index;
        PlyState *plystate_ptr;
        ply_get_argument_user_data(argument, reinterpret_cast<void **>(&plystate_ptr), &index);
        if(plystate_ptr->vertex_index >= plystate_ptr->vertex_num)
        return 0;
        double value = ply_get_argument_value(argument);
        (*plystate_ptr->vertex_ptr)[plystate_ptr->vertex_index](index) = value;
        if(index == 2)
        plystate_ptr->vertex_index++;    
        return 1;
    }
    int normal_cb(p_ply_argument argument) 
    {
        long index;
        PlyState *plystate_ptr;
        ply_get_argument_user_data(argument, reinterpret_cast<void **>(&plystate_ptr), &index);
        if(plystate_ptr->normal_index >= plystate_ptr->normal_num)
        return 0;
        double value = ply_get_argument_value(argument);
        (*plystate_ptr->normal_ptr)[plystate_ptr->normal_index](index) = value;
        if(index == 2)
        plystate_ptr->normal_index++;    
        return 1;
    }

    int color_cb(p_ply_argument argument)
    {
        long index;
        PlyState *plystate_ptr;
        ply_get_argument_user_data(argument, reinterpret_cast<void **>(&plystate_ptr), &index);
        if(plystate_ptr->color_index >= plystate_ptr->color_num)
        return 0;
        double value = ply_get_argument_value(argument);
        (*plystate_ptr->color_ptr)[plystate_ptr->color_index](index) = value/255.0;
        if(index == 2)
        plystate_ptr->color_index++;    
        return 1;
    }



    int triangle_cb(p_ply_argument argument) 
    {
        PlyState *plystate_ptr;
        long dummy, length, index;
        ply_get_argument_user_data(argument, reinterpret_cast<void **>(&plystate_ptr),
                                &dummy);
        double value = ply_get_argument_value(argument);
        if(plystate_ptr->triangle_index >= plystate_ptr->triangle_num) {
            return 0;
        }

        ply_get_argument_property(argument, NULL, &length, &index);
        if(index == -1) ;
        else
            (*plystate_ptr->triangle_ptr)[plystate_ptr->triangle_index](index) = value;    
        if(index == length-1) {
            plystate_ptr->triangle_index++;
        }
        return 1;
    }

    bool ReadPLY(const std::string &filename, geometry::Point3List &points, geometry::Point3List &normals, 
        geometry::Point3List &colors, std::vector<Eigen::Vector3i> &triangles)
    {
        p_ply ply_file = ply_open(filename.c_str(), NULL, 0, NULL);
        if(!ply_file) {
            std::cout <<RED << "[PLYReader]::[ERROR]::Cannot open file "<<filename<<RESET<<std::endl;
            return false;
        }
        if(!ply_read_header(ply_file)) {
            std::cout <<RED << "[PLYReader]::[ERROR]::Cannot Parse file header "<<RESET<<std::endl;
            ply_close(ply_file);
            return false;
        }

        PlyState state;


        state.vertex_num = ply_set_read_cb(ply_file, "vertex", "x",
                                        vertex_cb, &state, 0);
        ply_set_read_cb(ply_file, "vertex", "y", vertex_cb, &state, 1);
        ply_set_read_cb(ply_file, "vertex", "z", vertex_cb, &state, 2);

        state.normal_num = ply_set_read_cb(ply_file, "vertex", "nx",
                                        normal_cb, &state, 0);
        ply_set_read_cb(ply_file, "vertex", "ny", normal_cb, &state, 1);
        ply_set_read_cb(ply_file, "vertex", "nz", normal_cb, &state, 2);

        state.color_num = ply_set_read_cb(ply_file, "vertex", "red",
                                        color_cb, &state, 0);
        ply_set_read_cb(ply_file, "vertex", "green", color_cb, &state, 1);
        ply_set_read_cb(ply_file, "vertex", "blue", color_cb, &state, 2);

        if(state.vertex_num <= 0) {
            std::cout <<RED << "[PLYReader]::[ERROR]::vertex_num < 0"<<RESET<<std::endl;
            ply_close(ply_file);
            return false;
        }

        state.triangle_num = ply_set_read_cb(ply_file, "face", "vertex_indices",
                                        triangle_cb, &state, 0);
        if(state.triangle_num == 0) {
            state.triangle_num = ply_set_read_cb(ply_file, "face", "vertex_index",
                                            triangle_cb, &state, 0);
        }

        state.vertex_index = 0;
        state.normal_index = 0;
        state.color_index = 0; 
        state.triangle_index = 0;

        points.resize(state.vertex_num);
        normals.resize(state.normal_num);
        triangles.resize(state.triangle_num);
        colors.resize(state.color_num);

        state.vertex_ptr = &points;
        state.normal_ptr = &normals; 
        state.triangle_ptr = &triangles;
        state.color_ptr = &colors;
        if(!ply_read(ply_file)) 
        {
            std::cout <<RED << "[PLYReader]::[ERROR]::failed to read "<<filename<<RESET<<std::endl;
            ply_close(ply_file);
            return false;
        }
        std::cout <<BLUE<<"[PLYReader]::[INFO]::"<<"face: "<<state.triangle_num<<" vertex: "<<state.vertex_num<<RESET<<std::endl;
        ply_close(ply_file);
        return true;
    }

    bool ReadPLY(const std::string &filename, geometry::Point3List &points, geometry::Point3List &normals, 
        geometry::Point3List &colors)
    {
        p_ply ply_file = ply_open(filename.c_str(), NULL, 0, NULL);
        if(!ply_file) {
            std::cout <<RED << "[PLYReader]::[ERROR]::Cannot open file "<<filename<<RESET<<std::endl;
            return false;
        }
        if(!ply_read_header(ply_file)) {
            std::cout <<RED << "[PLYReader]::[ERROR]::Cannot Parse file header "<<RESET<<std::endl;
            ply_close(ply_file);
            return false;
        }

        PlyState state;

        state.vertex_num = ply_set_read_cb(ply_file, "vertex", "x",
                                        vertex_cb, &state, 0);
        ply_set_read_cb(ply_file, "vertex", "y", vertex_cb, &state, 1);
        ply_set_read_cb(ply_file, "vertex", "z", vertex_cb, &state, 2);

        state.normal_num = ply_set_read_cb(ply_file, "vertex", "nx",
                                        normal_cb, &state, 0);
        ply_set_read_cb(ply_file, "vertex", "ny", normal_cb, &state, 1);
        ply_set_read_cb(ply_file, "vertex", "nz", normal_cb, &state, 2);

        state.color_num = ply_set_read_cb(ply_file, "vertex", "red",
                                        color_cb, &state, 0);
        ply_set_read_cb(ply_file, "vertex", "green", color_cb, &state, 1);
        ply_set_read_cb(ply_file, "vertex", "blue", color_cb, &state, 2);

        if(state.vertex_num <= 0) {
            std::cout <<RED << "[PLYReader]::[ERROR]::vertex_num < 0"<<RESET<<std::endl;
            ply_close(ply_file);
            return false;
        }

        state.vertex_index = 0;
        state.normal_index = 0;
        state.color_index = 0; 

        points.resize(state.vertex_num);
        normals.resize(state.normal_num);
        colors.resize(state.color_num);

        state.vertex_ptr = &points;
        state.normal_ptr = &normals; 
        state.color_ptr = &colors;
        if(!ply_read(ply_file)) 
        {
            std::cout <<RED << "[PLYReader]::[ERROR]::failed to read "<<filename<<RESET<<std::endl;
            ply_close(ply_file);
            return false;
        }
        std::cout <<BLUE<<"[PLYReader]::[INFO]::vertex: "<<state.vertex_num<<RESET<<std::endl;
        ply_close(ply_file);
        return true;
    }

    bool WritePLY(const std::string &filename, const geometry::Point3List&points, 
        const geometry::Point3List &normals, const geometry::Point3List &colors, 
        const std::vector<AdditionalLabel> & additional_labels,
        bool use_ascii)
    {
        size_t numPoints = points.size();
        bool has_normals = normals.size()>0 && normals.size() == points.size();
        bool has_colors = colors.size() > 0 && colors.size() == points.size();
        //write ply with instance and semantic label
        // if(points.size() == 0)
        // {
        //     return true;
        // }

        p_ply ply_file = ply_create(filename.c_str(), (use_ascii? PLY_ASCII: PLY_LITTLE_ENDIAN),
                                    NULL, 0, NULL);
        if (!ply_file)
        {
            std::cout<<RED << "[PLYWriter]::[ERROR] Cannot open file "<<filename<<RESET<<std::endl;
            return false;
        }
        ply_add_comment(ply_file, "Created by FCLib");
        ply_add_element(ply_file, "vertex",
                        static_cast<long>(points.size()));
        ply_add_property(ply_file, "x", PLY_DOUBLE, PLY_DOUBLE, PLY_DOUBLE);
        ply_add_property(ply_file, "y", PLY_DOUBLE, PLY_DOUBLE, PLY_DOUBLE);
        ply_add_property(ply_file, "z", PLY_DOUBLE, PLY_DOUBLE, PLY_DOUBLE);
        if(has_normals)
        {
            ply_add_property(ply_file, "nx", PLY_DOUBLE, PLY_DOUBLE, PLY_DOUBLE);
            ply_add_property(ply_file, "ny", PLY_DOUBLE, PLY_DOUBLE, PLY_DOUBLE);
            ply_add_property(ply_file, "nz", PLY_DOUBLE, PLY_DOUBLE, PLY_DOUBLE);
        }
        if(has_colors) 
        {
            ply_add_property(ply_file, "red", PLY_UCHAR, PLY_UCHAR, PLY_UCHAR);
            ply_add_property(ply_file, "green", PLY_UCHAR, PLY_UCHAR, PLY_UCHAR);
            ply_add_property(ply_file, "blue", PLY_UCHAR, PLY_UCHAR, PLY_UCHAR);
        }
        for(int i = 0; i != additional_labels.size(); ++i)
        {
            std::string label_name = std::get<0>(additional_labels[i]);
            e_ply_type type = std::get<1>(additional_labels[i]);
            ply_add_property(ply_file, label_name.c_str(), 
                type, type, type);
        }
        /*
        if(labels.size() > 0)
        {
            ply_add_property(ply_file, "semantic", PLY_INT32, PLY_INT32, PLY_INT32);
            ply_add_property(ply_file, "instance", PLY_INT32, PLY_INT32, PLY_INT32);
        }*/

        if(!ply_write_header(ply_file)) 
        {
            std::cout<<"Unable to write header."<<std::endl;
            return false;
        }
        for(size_t i = 0; i < points.size(); i++) 
        {
            const geometry::Point3 &point = points[i];
            ply_write(ply_file, point(0));
            ply_write(ply_file, point(1));
            ply_write(ply_file, point(2));
            if(has_normals) 
            {
                const geometry::Point3 &normal = normals[i];
                ply_write(ply_file, normal(0));
                ply_write(ply_file, normal(1));
                ply_write(ply_file, normal(2));
            }
            if (has_colors) 
            {
                const geometry::Point3 &color = colors[i];
                ply_write(ply_file,
                        std::min(255.0, std::max(0.0, color(0) * 255.0)));
                ply_write(ply_file,
                        std::min(255.0, std::max(0.0, color(1) * 255.0)));
                ply_write(ply_file,
                        std::min(255.0, std::max(0.0, color(2) * 255.0)));
            }
            for(int j = 0; j != additional_labels.size(); ++j)
            {
                e_ply_type type = std::get<1>(additional_labels[j]);
                void *ptr = std::get<2>(additional_labels[j]);
                if(type == PLY_INT32)
                {
                    int value = *((int *)ptr + i);
                    ply_write(ply_file, value);
                }   
                else if(type == PLY_UCHAR)
                {
                    unsigned char value = *((unsigned char *)ptr + i);
                    ply_write(ply_file, value);
                }
                else if(type == PLY_USHORT)
                {
                    unsigned short value = *((unsigned short *)ptr + i);
                    ply_write(ply_file, value);
                }
                else if(type == PLY_FLOAT32)
                {
                    float value = *((float *)ptr + i);
                    ply_write(ply_file, value);
                }
                else if(type == PLY_FLOAT64)
                {
                    double value = *((double *)ptr + i);
                    ply_write(ply_file, value);
                }
            }
            /*
            if(labels.size())
            {
                int semantic_label = get_semantic_label(labels[i]);
                int instance_label = get_instance_label(labels[i]);
                ply_write(ply_file, semantic_label);
                ply_write(ply_file, instance_label);
            }*/
        }

        ply_close(ply_file);
        return true;
    }
    bool WritePLY(const std::string &filename, const geometry::Point3List&points, 
        const geometry::Point3List &normals, const geometry::Point3List &colors,
        const std::vector<Eigen::Vector3i> &triangles, 
        const std::vector<AdditionalLabel> & additional_labels,
        bool use_ascii)
    {
        size_t numPoints = points.size();
        bool has_normals = normals.size()>0 && normals.size() == points.size();
        bool has_colors = colors.size() > 0 && colors.size() == points.size();
        //write ply with instance and semantic label
        // if(points.size() == 0) 
        // {
        //     return true;
        // }

        p_ply ply_file = ply_create(filename.c_str(), (use_ascii? PLY_ASCII: PLY_LITTLE_ENDIAN),
                                    NULL, 0, NULL);
        if(!ply_file) {
            std::cout<<RED << "[PLYWriter]::[ERROR] Cannot open file "<<filename<<RESET<<std::endl;
            return false;
        }
        ply_add_comment(ply_file, "Created by FCLib");
        ply_add_element(ply_file, "vertex",
                        static_cast<long>(points.size()));
        ply_add_property(ply_file, "x", PLY_DOUBLE, PLY_DOUBLE, PLY_DOUBLE);
        ply_add_property(ply_file, "y", PLY_DOUBLE, PLY_DOUBLE, PLY_DOUBLE);
        ply_add_property(ply_file, "z", PLY_DOUBLE, PLY_DOUBLE, PLY_DOUBLE);
        if(has_normals)
        {
            ply_add_property(ply_file, "nx", PLY_DOUBLE, PLY_DOUBLE, PLY_DOUBLE);
            ply_add_property(ply_file, "ny", PLY_DOUBLE, PLY_DOUBLE, PLY_DOUBLE);
            ply_add_property(ply_file, "nz", PLY_DOUBLE, PLY_DOUBLE, PLY_DOUBLE);
        }
        if(has_colors) 
        {
            ply_add_property(ply_file, "red", PLY_UCHAR, PLY_UCHAR, PLY_UCHAR);
            ply_add_property(ply_file, "green", PLY_UCHAR, PLY_UCHAR, PLY_UCHAR);
            ply_add_property(ply_file, "blue", PLY_UCHAR, PLY_UCHAR, PLY_UCHAR);
        }
        for(int i = 0; i != additional_labels.size(); ++i)
        {
            std::string label_name = std::get<0>(additional_labels[i]);
            e_ply_type type = std::get<1>(additional_labels[i]);
            ply_add_property(ply_file, label_name.c_str(), 
                type, type, type);
        }
        ply_add_element(ply_file, "face",
                        static_cast<long>(triangles.size()));
        ply_add_property(ply_file, "vertex_indices", PLY_LIST, PLY_UCHAR, PLY_UINT);
        /*
        if(labels.size() > 0)
        {
            ply_add_property(ply_file, "semantic", PLY_INT32, PLY_INT32, PLY_INT32);
            ply_add_property(ply_file, "instance", PLY_INT32, PLY_INT32, PLY_INT32);
        }*/

        if(!ply_write_header(ply_file))
        {
            std::cout<<"Unable to write header."<<std::endl;
            return false;
        }
        for (size_t i = 0; i < points.size(); i++)
        {
            const geometry::Point3 &point = points[i];
            ply_write(ply_file, point(0));
            ply_write(ply_file, point(1));
            ply_write(ply_file, point(2));
            if(has_normals) 
            {
                const geometry::Point3 &normal = normals[i];
                ply_write(ply_file, normal(0));
                ply_write(ply_file, normal(1));
                ply_write(ply_file, normal(2));
            }
            if(has_colors) 
            {
                const geometry::Point3 &color = colors[i];
                ply_write(ply_file,
                        std::min(255.0, std::max(0.0, color(0) * 255.0)));
                ply_write(ply_file,
                        std::min(255.0, std::max(0.0, color(1) * 255.0)));
                ply_write(ply_file,
                        std::min(255.0, std::max(0.0, color(2) * 255.0)));
            }
            for(int j = 0; j != additional_labels.size(); ++j)
            {
                e_ply_type type = std::get<1>(additional_labels[j]);
                void *ptr = std::get<2>(additional_labels[j]);
                if(type == PLY_INT32)
                {
                    int value = *((int *)ptr + i);
                    ply_write(ply_file, value);
                }   
                else if(type == PLY_UCHAR)
                {
                    unsigned char value = *((unsigned char *)ptr + i);
                    ply_write(ply_file, value);
                }
                else if(type == PLY_USHORT)
                {
                    unsigned short value = *((unsigned short *)ptr + i);
                    ply_write(ply_file, value);
                }
                else if(type == PLY_FLOAT32)
                {
                    float value = *((float *)ptr + i);
                    ply_write(ply_file, value);
                }
                else if(type == PLY_FLOAT64)
                {
                    double value = *((double *)ptr + i);
                    ply_write(ply_file, value);
                }
            }
            /*
            if(labels.size())
            {
                int semantic_label = get_semantic_label(labels[i]);
                int instance_label = get_instance_label(labels[i]);
                ply_write(ply_file, semantic_label);
                ply_write(ply_file, instance_label);
            }*/
        }
        for (size_t i = 0; i < triangles.size(); i++)
        {
            const auto &triangle = triangles[i];
            ply_write(ply_file, 3);
            ply_write(ply_file, triangle(0));
            ply_write(ply_file, triangle(1));
            ply_write(ply_file, triangle(2));
        }
        ply_close(ply_file);
        return true;
    }

}
}