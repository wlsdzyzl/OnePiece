#include "Geometry/KDTree.h"
#include "Geometry/TriangleMesh.h"
#include "Tool/PLYManager.h"
#include "Tool/IO.h"
#include "Tool/CppExtension.h"
#include "Visualization/ColorTab.h"
using namespace fucking_cool;

int main(int argc, char** argv)
{
    if(argc < 3)
    {
        std::cout<<"Usage: GetSemanticLabelUsingKDTree [input model] [reference model] [instance_json_path](optional) "<<std::endl;
        return 1;
    }
    geometry::TriangleMesh mesh;
    geometry::TriangleMesh reference_mesh;
    mesh.LoadFromPLY(argv[1]);
    float max_distance = 0.1;
    std::vector<unsigned short> labels;
    std::vector<unsigned short> reference_labels;
    std::vector<tool::AdditionalElement> additional_labels;
    tool::AdditionalElement additional_label;
    additional_label.element_key = "vertex";
    additional_label.element_property = {"label"};
    additional_labels.push_back(additional_label);

    tool::ReadPLY(argv[2], reference_mesh.points, reference_mesh.normals, 
        reference_mesh.colors, reference_mesh.triangles, additional_labels);
    if(additional_labels[0].type == tinyply::Type::UINT16)
    {
        if(additional_labels[0].byte_size != additional_labels[0].count * sizeof(unsigned short))
        {
            std::cout<<"Wrong buffer."<<std::endl;
        }
        reference_labels.resize(additional_labels[0].count);
        memcpy(reference_labels.data(), additional_labels[0].data, additional_labels[0].byte_size);
        delete additional_labels[0].data;
        additional_labels[0].data = nullptr;
    }
    else
    {
       std::cout<<"Error occurs when reading labels."<<std::endl;
    }
    geometry::KDTree<> kdtree;
    kdtree.BuildTree(reference_mesh.points);
    //default semantic label is 0, means unannonated 
    labels.resize(mesh.points.size(), 0);
    for(size_t i = 0; i != mesh.points.size(); ++i)
    {
        geometry::Point3 query_point = mesh.points[i];
        std::vector<int> indices;
        std::vector<float> dists;
        kdtree.KnnSearch(query_point, indices, dists, 1);
        
        if(indices.size() > 0 && dists[0] < max_distance)
        {
            
            labels[i] = reference_labels[indices[0]];
        }
        // else
        // {
        //     std::cout<<"Error: cannot find the nearest points. Try to increase the checks."<<std::endl;
        // }
        if(i % 100000 == 0 )
        std::cout<<"processed "<< (i + 0.0) / mesh.points.size() * 100<<"%..."<<std::endl;
    }    
    std::cout<<"Write to ply."<<std::endl;
    //create label writer

    additional_labels[0].count = labels.size();
    additional_labels[0].data = (unsigned char *)(&labels[0]);
    std::vector<std::string> comments;
    comments.push_back("each vertex will have semantic labels.");
    tool::WritePLY("Labeled_model.ply", mesh.points, mesh.normals, mesh.colors, mesh.triangles, comments, additional_labels);
    mesh.colors.resize(mesh.points.size());
    for(size_t i = 0; i != mesh.points.size(); ++i)
    {
        cv::Scalar label_color = visualization::color_tab[labels[i] % visualization::color_tab.size()];
        mesh.colors[i] = geometry::Point3(label_color(0)/255.0, label_color(1)/255.0, label_color(2)/255.0 );
    }
    mesh.WriteToPLY("colored_model_semantic.ply");

    if(argc == 4)
    {
        //get instance label
        std::vector<int> reference_instance_labels;
        tool::ReadIntanceInfoFromScannet(argv[3], reference_instance_labels);
        geometry::TriangleMesh high_res_mesh; 
        auto strs = tool::RSplit(argv[3], "/", 1);
        if(strs.size() == 2)
        { 
            //std::cout<<std::string(argv[3])+"/"+strs[1] +"_vh_clean.ply"<<std::endl;
            high_res_mesh.LoadFromPLY(std::string(argv[3])+"/"+strs[1] +"_vh_clean.ply");
            
            kdtree.BuildTree(high_res_mesh.points);
            std::vector<int> low_res_labels(reference_mesh.points.size());
            for(size_t i = 0; i != reference_mesh.points.size(); ++i)
            {
                geometry::Point3 query_point = reference_mesh.points[i];
                std::vector<int> indices;
                std::vector<float> dists;
                kdtree.KnnSearch(query_point, indices, dists, 1);
                
                if(indices.size() > 0 && dists[0] < max_distance)
                {
                    low_res_labels[i] = reference_instance_labels[indices[0]];
                }
                // else
                // {
                //     std::cout<<"Error: cannot find the nearest points. Try to increase the checks."<<std::endl;
                // }
                if(i % 10000 == 0 )
                std::cout<<"processed "<< (i + 0.0) / reference_mesh.points.size() * 100<<"%..."<<std::endl;
            } 
            std::vector<int> instance_labels;
            //default instance label is -1, means this point does not belong to any object 
            instance_labels.resize(mesh.points.size(), -1);
            kdtree.BuildTree(reference_mesh.points);

            for(size_t i = 0; i != mesh.points.size(); ++i)
            {
                geometry::Point3 query_point = mesh.points[i];
                std::vector<int> indices;
                std::vector<float> dists;
                kdtree.KnnSearch(query_point, indices, dists, 1);
                
                if(indices.size() > 0 && dists[0] < max_distance)
                {
                    
                    instance_labels[i] = low_res_labels[indices[0]];
                }
                // else
                // {
                //     std::cout<<"Cannot find the nearest points (). "<<std::endl;
                // }
                if(i % 100000 == 0 )
                std::cout<<"processed "<< (i + 0.0) / mesh.points.size() * 100<<"%..."<<std::endl;
            }
            for(size_t i = 0; i != mesh.points.size(); ++i)
            {
                cv::Scalar label_color = cv::Scalar(0, 0, 0); 
                if(instance_labels[i] >= 0)
                    label_color= visualization::color_tab[instance_labels[i] % visualization::color_tab.size()];
                mesh.colors[i] = geometry::Point3(label_color(0)/255.0, label_color(1)/255.0, label_color(2)/255.0 );
            }
            mesh.WriteToPLY("colored_model_instance.ply");               
        }
        else
        {
            std::cout<<"Something wrong when parsing the path. If scene dir is in current dir, use \"./sceneXXXX_XX\"." 
                "Do not add '/' in the end."<<std::endl;
        }        
        
        
    }
    return 1;

}