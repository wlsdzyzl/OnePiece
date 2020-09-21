#include "Geometry/KDTree.h"
#include "Geometry/TriangleMesh.h"
#include "Tool/RPLYManager.h"
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
    reference_mesh.LoadFromPLY(argv[2]);
    std::vector<unsigned short> labels;

    std::vector<unsigned short> reference_labels;
    tool::ReadVertexLabelFromPLY<unsigned short>(argv[2], "label", reference_labels);
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
        if(i % 1000 == 0 )
        std::cout<<"processed "<< (i + 0.0) / mesh.points.size() * 100<<"%..."<<std::endl;
    }    
    std::cout<<"Write to ply."<<std::endl;
    //create label writer
    std::string label_name = "label";
    e_ply_type type = PLY_USHORT;
    void * label_ptr = (void *)(&labels[0]);
    tool::AdditionalLabel additional_label = std::tie(label_name, type, label_ptr);
    std::vector<tool::AdditionalLabel> additional_labels;
    additional_labels.push_back(additional_label);
    tool::WritePLY("Labeled_model.ply", mesh.points, mesh.normals, mesh.colors, mesh.triangles, additional_labels);
    mesh.colors.resize(mesh.points.size());
    for(int i = 0; i != mesh.points.size(); ++i)
    {
        cv::Scalar label_color = visualization::color_tab[labels[i] % visualization::color_tab.size()];
        mesh.colors[i] = geometry::Point3(label_color(0)/255.0, label_color(1)/255.0, label_color(2)/255.0 );
    }
    mesh.WriteToPLY("Colored_model.ply");
    return 1;

}