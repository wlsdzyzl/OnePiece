#include "Geometry/Geometry.h"

using namespace one_piece;

int main(int argc, char **argv)
{
    if(argc != 3)
    {
        std::cout << "Usage: EigenTest [trajectory] [pair]"<<std::endl;
        return 0;
    }
    geometry::Mat4List pose_list;
    std::vector<std::pair<int, int>> pairs;
    
    int all;
    {
        std::ifstream ifs(argv[1]);
        
        while(ifs)
        {
            int index, useless;
            ifs>>index;
            ifs>>index;
            ifs>>useless;

            geometry::TransformationMatrix T;
            ifs>>T(0,0)>>T(0,1)>>T(0,2)>>T(0,3)
                >>T(1,0)>>T(1,1)>>T(1,2)>>T(1,3)
                >>T(2,0)>>T(2,1)>>T(2,2)>>T(2,3)
                >>T(3,0)>>T(3,1)>>T(3,2)>>T(3,3);
            pose_list.push_back(T);
        }
        ifs.close();
    }
    {
        std::ifstream ifs(argv[2]);
        
        while(ifs)
        {
            int ref_id, new_id;
            ifs>>ref_id;
            ifs>>new_id;
            ifs>>all;

            geometry::TransformationMatrix T;
            ifs>>T(0,0)>>T(0,1)>>T(0,2)>>T(0,3)
                >>T(1,0)>>T(1,1)>>T(1,2)>>T(1,3)
                >>T(2,0)>>T(2,1)>>T(2,2)>>T(2,3)
                >>T(3,0)>>T(3,1)>>T(3,2)>>T(3,3);
            pairs.push_back(std::make_pair(ref_id,new_id));
        }
    }
    std::vector<geometry::TransformationMatrix> relative_poses;
    for(size_t i = 0; i != pairs.size(); ++i)
    {
        int ref_id = pairs[i].first * 50;
        int new_id = pairs[i].second * 50;
        relative_poses.push_back(pose_list[ref_id].inverse() * pose_list[new_id]);
    }

    std::ofstream ofs("./gt.log");
    for(size_t i = 0; i != pairs.size(); ++i)
    {
        int ref_id = pairs[i].first;
        int new_id = pairs[i].second;
        ofs<<ref_id<<" "<<new_id<<" "<<all<<"\n";
        ofs<<relative_poses[i]<<"\n";
    }
    ofs.close();
    return 0;
}