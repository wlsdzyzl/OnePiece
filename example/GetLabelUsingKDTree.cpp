#include "Geometry/KDTree.h"
#include "Geometry/TriangleMesh.h"
#include "Tool/PLYManager.h"
#include "Visualization/ColorTab.h"
using namespace fucking_cool;
int main(int argc, char** argv)
{
    if(argc != 3)
    {
        std::cout<<"Usage: GetLabelUsingKDTree [input model] [reference model]"<<std::endl;
        return 1;
    }
    geometry::TriangleMesh mesh;
    geometry::TriangleMesh reference_mesh;
    mesh.LoadFromPLY(argv[1]);
    
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

    for(int i = 0; i != mesh.points.size(); ++i)
    {
        geometry::Point3 query_point = mesh.points[i];
        std::vector<int> indices;
        std::vector<float> dists;
        kdtree.KnnSearch(query_point, indices, dists, 1);
        
        if(indices.size() > 0)
        {
            
            labels.push_back(reference_labels[indices[0]]);
        }
        else
        {
            std::cout<<"Error: cannot find the nearest points. Try to increase the checks."<<std::endl;
        }
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
    for(int i = 0; i != mesh.points.size(); ++i)
    {
        cv::Scalar label_color = visualization::color_tab[labels[i] % visualization::color_tab.size()];
        mesh.colors[i] = geometry::Point3(label_color(0)/255.0, label_color(1)/255.0, label_color(2)/255.0 );
    }
    mesh.WriteToPLY("Colored_model.ply");
    return 1;

}