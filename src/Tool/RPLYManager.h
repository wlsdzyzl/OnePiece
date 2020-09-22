#if 0
#ifndef RPLY_READER_H
#define RPLY_READER_H
#include "ConsoleColor.h"
#include <vector>
#include "../Geometry/Geometry.h"
#include "rply.h"
//this file has been discared and will not be maintained. Try to use functions in PLYManager.
namespace fucking_cool
{
namespace tool
{
    typedef std::tuple<std::string, e_ply_type, void *> AdditionalLabel;

    bool ReadPLY(const std::string &filename, geometry::Point3List &points, geometry::Point3List &normals, 
        geometry::Point3List &colors);
    bool ReadPLY(const std::string &filename, geometry::Point3List &points, geometry::Point3List &normals, 
        geometry::Point3List &colors, geometry::Point3uiList &triangles);

    bool ReadPLY(const std::string &filename, geometry::Point3List &points, geometry::Point3List &normals, 
        geometry::Point3List &colors, geometry::Point3uiList &triangles, 
        std::vector<AdditionalLabel> & additional_labels);
    
    bool ReadPLY(const std::string &filename, geometry::Point3List &points, geometry::Point3List &normals,
        geometry::Point3List &colors, std::vector<AdditionalLabel> & additional_labels)
    
    
    
    bool WritePLY(const std::string &filename, const geometry::Point3List&points, 
        const geometry::Point3List &normals, const geometry::Point3List &colors, 
        const std::vector<AdditionalLabel> & additional_labels = std::vector<AdditionalLabel>(),
        bool use_ascii = true);
    bool WritePLY(const std::string &filename, const geometry::Point3List&points, 
        const geometry::Point3List &normals, const geometry::Point3List &colors,
        const geometry::Point3uiList &triangles, 
        const std::vector<AdditionalLabel> & additional_labels = std::vector<AdditionalLabel>(),
        bool use_ascii = true);
    //for read a arbitrary label of vertex from state
    template<typename T>
    struct LabelState
    {
        int label_index;
        int label_num;
        std::vector<T> *label_ptr;
    };
    template<typename T>
    int label_cb(p_ply_argument argument)
    {
        long index;
        LabelState<T> *label_state_ptr;
        ply_get_argument_user_data(argument, reinterpret_cast<void **>(&label_state_ptr), &index);
        if(label_state_ptr->label_index >= label_state_ptr->label_num)
        return 0;
        double value = ply_get_argument_value(argument);
        (*label_state_ptr->label_ptr)[label_state_ptr->label_index] = value;
        label_state_ptr->label_index++;    
        return 1;
    };
    template<typename T>
    bool ReadVertexLabelFromPLY(const std::string &filename, const std::string &label_name,
        std::vector<T> &vertex_labels)
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

        LabelState<T> state;
        state.label_num = ply_set_read_cb(ply_file, "vertex", label_name.c_str(),
                                        label_cb<T>, &state, 0);


        if(state.label_num <= 0) {
            std::cout <<RED << "[PLYReader]::[ERROR]::label_num < 0"<<RESET<<std::endl;
            ply_close(ply_file);
            return false;
        }

        state.label_index = 0;
        vertex_labels.resize(state.label_num);

        state.label_ptr = &vertex_labels;
        if(!ply_read(ply_file)) 
        {
            std::cout <<RED << "[PLYReader]::[ERROR]::failed to read "<<filename<<RESET<<std::endl;
            ply_close(ply_file);
            return false;
        }
        std::cout <<BLUE<<"[PLYReader]::[INFO]::Read "<<state.label_num<<" labels."<<RESET<<std::endl;
        ply_close(ply_file);      
        return true;  
    }
}
}
#endif
#endif