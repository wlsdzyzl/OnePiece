#ifndef VOXEL_HASHING_H
#define VOXEL_HASHING_H
// voxel hashing table
#include <unordered_map>
#include <memory>
#include "VoxelCube.h"
#include "Camera/Camera.h"
#include "Geometry/Geometry.h"
#include "Geometry/RGBDFrame.h"
#include "Geometry/TriangleMesh.h"
#include "Integrator.h"
#include "MarchingCube.h"
#include "Frustum.h"

#define TRUNCATED_DISTANCES 1.0
namespace one_piece
{
namespace integration
{
    //define Hashing function for undordered_map

    typedef std::unordered_map<CubeID, VoxelCube, CubeHasher> CubeMap;

    class CubeHandler
    {
        public:

        CubeHandler()
        {
            c_para.InitializeVoxelCube();
        }
        CubeHandler(const camera::PinholeCamera &_camera):camera(_camera)
        {
            c_para.InitializeVoxelCube();
        }
        void SetVoxelResolution(float resolution)
        {
            c_para.SetVoxelResolution(resolution);
        }
        bool ReadFromFile(const std::string &filename)
        {
            cube_map.clear();
            std::ifstream ifs(filename, std::ifstream::binary);
            std::vector<float> buffer;
            ifs.seekg (0, ifs.end);
            size_t length = ifs.tellg();
            ifs.seekg (0, ifs.beg);
            buffer.resize(length/sizeof(float));
            ifs.read((char *)&buffer[0],length);
            //bufferSize = buffer[0];
            unsigned int cube_size;
            cube_size = *((unsigned int *)(&buffer[0]));
            size_t ptr = 1;
            size_t i = 0;
            float x,y,z;
            while(i<cube_size)
            {
                x = buffer[ptr++];
                y = buffer[ptr++];
                z = buffer[ptr++];
                CubeID cube_id = CubeID(x,y,z);
                cube_map[cube_id] = VoxelCube(cube_id);
                cube_map[cube_id].ReadFromBuffer(buffer,ptr);
                //chunk->computeOrigin();
                ++i;
            }
            std::cout<<GREEN<<"[CubeHandler]::[INFO]::Load TSDF field done!(From BinaryFile) "<<RESET<<std::endl;            
            return true;
        }
        /*
        This function is nothing but to read a float tsdf file I defined before, which has been abandoned.
        */
        bool ReadFromFileFloat(const std::string &filename)
        {
            cube_map.clear();
            std::ifstream ifs(filename,std::ifstream::binary);
            std::vector<float> buffer;
            /*
            size_t buffer_size;
            ifs.read((char*)&buffer_size, sizeof(buffer_size));
            bufferSize = buffer_size;
            buffer.resize(bufferSize);
            //
            */
            ifs.seekg (0, ifs.end);
            size_t length = ifs.tellg();
            ifs.seekg (0, ifs.beg);
            buffer.resize(length/sizeof(float));
            ifs.read((char *)&buffer[0],length);
            //bufferSize = buffer[0];
            unsigned int cube_size = buffer[1];
            ifs.close();    
            size_t ptr = 2;
            size_t i = 0;
            float x,y,z;
            while(i<cube_size)
            {
                x = buffer[ptr++];
                y = buffer[ptr++];
                z = buffer[ptr++];
                CubeID cube_id = CubeID(x,y,z);
                cube_map[cube_id] = VoxelCube(cube_id);
                cube_map[cube_id].ReadFromBufferFloat(buffer,ptr);
                //chunk->computeOrigin();
                ++i;
            }
            std::cout<<GREEN<<"[CubeHandler]::[INFO]::Load TSDF field done!(From BinaryFile) "<<RESET<<std::endl;            
            return true;
        }
        std::shared_ptr<geometry::PointCloud> GetPointCloud() const;
        void GenerateMeshByCube(const CubeID &cube_id, geometry::TriangleMesh &mesh);
        void ExtractTriangleMesh(geometry::TriangleMesh &mesh);
        bool WriteToFile(const std::string &filename) const
        {

            std::ofstream ofs(filename,std::ios::binary);
            std::vector<float> buffer;
            unsigned int size = cube_map.size();
            buffer.push_back(0);
            *((unsigned int *)(&buffer[0])) = size;
            for( auto iter = cube_map.begin(); iter != cube_map.end(); ++ iter)
            {
                iter->second.WriteToBuffer(buffer);
            }
            ofs.write((char *) &buffer[0],sizeof(float) * buffer.size());            
            std::cout<<GREEN<<"[CubeHandler]::[INFO]::Write TSDF field done!(To BinaryFile) "<<RESET<<std::endl;  
            return true;
        }
        bool HasCube(const CubeID &cube_id) const 
        {
            return cube_map.find(cube_id) != cube_map.end();
        }
        void Clear()
        {
            cube_map.clear();
        }       
        void SetCamera(const camera::PinholeCamera &_camera)
        {
            camera = _camera;
        }
        void SetTruncation(float trunc)
        {
            integrator.SetTruncation(trunc);
        }
        void Merge(const CubeHandler &another)
        {
            if(c_para.VoxelResolution != another.c_para.VoxelResolution)
            {
                std::cout<<YELLOW<<"[Warning]::[MergeVoxelHash]::Voxel resolution is not identical."<<RESET<<std::endl;
                return;
            }
            for(auto iter = another.cube_map.begin(); iter != another.cube_map.end(); ++iter)
            {
                if(cube_map.find(iter->first) == cube_map.end())
                {
                    AddCube(iter->first);
                    cube_map[iter->first] = iter->second;
                }
                else
                {
                    for(size_t voxel_id = 0; voxel_id != iter->second.voxels.size(); ++voxel_id)
                    {
                        cube_map[iter->first].voxels[voxel_id] += iter->second.voxels[voxel_id];
                    }
                }
            }
        } 
        void Merge(const CubeHandler &another, const geometry::TransformationMatrix & trans)
        {
            if(c_para.VoxelResolution != another.c_para.VoxelResolution)
            {
                std::cout<<YELLOW<<"[Warning]::[MergeVoxelHash]::Voxel resolution is not identical."<<RESET<<std::endl;
                return;
            }
            auto another_ptr = another.Transform(trans);
            Merge(*another_ptr);
        }
        void ComputeBounding(const cv::Mat &depth, const geometry::TransformationMatrix & pose, 
            geometry::Point3 & max_pos, geometry::Point3 &min_pos);
        void PrepareCubes(const cv::Mat &depth, const geometry::TransformationMatrix &pose, 
            std::vector<CubeID> &cube_id_list);
        // A volumetric method for building complex models from range images, B. Curless and M. Levoy, 1996 SIGGRAPH
        void IntegrateImage(const cv::Mat &depth, const cv::Mat &rgb, const geometry::TransformationMatrix & pose);
        void IntegrateImage(const geometry::RGBDFrame &rgbd, const geometry::TransformationMatrix & pose);
        CubeID GetCubeID(const geometry::Point3 &point) const
        {
            //float cube_resolution = CUBE_SIZE * c_para.VoxelResolution;
            return c_para.GetCubeID(point);
        }

        //Add a cube after transformed.
        void AddCube(const CubeID &cube_id)
        {
            if(cube_map.find(cube_id) == cube_map.end())
            {
                cube_map[cube_id] = VoxelCube(cube_id);
            }
        }
        void AddTransformedCube(const VoxelCube &v_cube, const geometry::TransformationMatrix &trans)
        {
            for(size_t voxel_id = 0; voxel_id != v_cube.voxels.size(); ++voxel_id)
            {
                geometry::Point3 center_position = c_para.GetGlobalPoint(v_cube.cube_id, voxel_id);
                geometry::Vector4 tmp_position = 
                    trans * geometry::Vector4( center_position(0), center_position(1), center_position(2), 1);
                geometry::Point3 new_position = tmp_position.head<3>() / tmp_position(3) - 
                     geometry::Point3(c_para.VoxelResolution/2, c_para.VoxelResolution/2, c_para.VoxelResolution/2);
                std::vector<geometry::Point3i> _8_neighbor(8);
                _8_neighbor[0] = 
                    geometry::Point3i(std::floor(new_position(0) / c_para.VoxelResolution), 
                    std::floor(new_position(1) / c_para.VoxelResolution), std::floor(new_position(2)/ c_para.VoxelResolution) );                
                _8_neighbor[1] = _8_neighbor[0] + geometry::Point3i(1,0,0);
                _8_neighbor[2] = _8_neighbor[0] + geometry::Point3i(0,1,0);
                _8_neighbor[3] = _8_neighbor[0] + geometry::Point3i(1,1,0);
                _8_neighbor[4] = _8_neighbor[0] + geometry::Point3i(0,0,1);
                _8_neighbor[5] = _8_neighbor[0] + geometry::Point3i(1,0,1);
                _8_neighbor[6] = _8_neighbor[0] + geometry::Point3i(0,1,1);
                _8_neighbor[7] = _8_neighbor[0] + geometry::Point3i(1,1,1);
                for(size_t i = 0; i != 8; ++i)
                {
                    CubeID cube_id = c_para.GetCubeID(_8_neighbor[i]);
                    AddCube(cube_id);
                }
            }
        }
        void AddTransformedCubeNearest(const VoxelCube &v_cube, const geometry::TransformationMatrix &trans)
        {
            for(size_t voxel_id = 0; voxel_id != v_cube.voxels.size(); ++voxel_id)
            {
                geometry::Point3 center_position = c_para.GetGlobalPoint(v_cube.cube_id, voxel_id);
                geometry::Vector4 tmp_position = 
                    trans * geometry::Vector4( center_position(0), center_position(1), center_position(2), 1);
                geometry::Point3 new_position = tmp_position.head<3>() / tmp_position(3);
                geometry::Point3i chosen_voxel = 
                    geometry::Point3i(std::floor(new_position(0) / c_para.VoxelResolution), 
                    std::floor(new_position(1) / c_para.VoxelResolution), std::floor(new_position(2)/ c_para.VoxelResolution) );

                CubeID cube_id = c_para.GetCubeID(chosen_voxel);
                AddCube(cube_id);
            }
        }        
        std::shared_ptr<CubeHandler> Transform(const geometry::TransformationMatrix &trans) const
        {
            std::shared_ptr<CubeHandler> after_trans = std::make_shared<CubeHandler>(camera);
            //after_trans->camera = camera;
            after_trans->far = far; 
            after_trans->near = near;
            after_trans->integrator = integrator;
            after_trans->c_para = c_para;
            for(auto iter = cube_map.begin(); iter != cube_map.end(); ++iter)
            {
                after_trans->AddTransformedCube(iter->second, trans);
            }
            for(auto iter = after_trans->cube_map.begin(); iter != after_trans->cube_map.end(); ++iter)
            {
                //read each voxel from eight neighbor
                auto &v_cube = iter->second;
                for(size_t voxel_id = 0; voxel_id != v_cube.voxels.size(); ++voxel_id)
                {
                    geometry::Point3 center_position = c_para.GetGlobalPoint(v_cube.cube_id, voxel_id);
                    geometry::Vector4 tmp_position = 
                        trans.inverse() * geometry::Vector4( center_position(0), center_position(1), center_position(2), 1);
                    geometry::Point3 new_position = tmp_position.head<3>() / tmp_position(3) - 
                        geometry::Point3(c_para.VoxelResolution/2, c_para.VoxelResolution/2, c_para.VoxelResolution/2);
                    std::vector<geometry::Point3i> _8_neighbor(8);
                    std::vector<TSDFVoxel> _8_voxels(8);
                    _8_neighbor[0] = 
                        geometry::Point3i(std::floor(new_position(0) / c_para.VoxelResolution), 
                        std::floor(new_position(1) / c_para.VoxelResolution), std::floor(new_position(2)/ c_para.VoxelResolution) );                
                    _8_neighbor[1] = _8_neighbor[0] + geometry::Point3i(1,0,0);
                    _8_neighbor[2] = _8_neighbor[0] + geometry::Point3i(0,1,0);
                    _8_neighbor[3] = _8_neighbor[0] + geometry::Point3i(1,1,0);
                    _8_neighbor[4] = _8_neighbor[0] + geometry::Point3i(0,0,1);
                    _8_neighbor[5] = _8_neighbor[0] + geometry::Point3i(1,0,1);
                    _8_neighbor[6] = _8_neighbor[0] + geometry::Point3i(0,1,1);
                    _8_neighbor[7] = _8_neighbor[0] + geometry::Point3i(1,1,1);

                    
                    for(size_t i = 0; i != 8; ++i)
                    {
                        auto cube_id = c_para.GetCubeID(_8_neighbor[i]);
                        auto _voxel_id = c_para.GetVoxelID(_8_neighbor[i]);
                        auto the_cube = cube_map.find(cube_id);
                        if(the_cube!= cube_map.end())
                        {
                            _8_voxels[i] = the_cube->second.GetVoxel(_voxel_id);
                        }
                    }
                    auto result = ReadVoxelInterpolate(_8_neighbor, _8_voxels, new_position, c_para);
                    //use some weighted function to reconpute the voxel
                    //std::cout<<result.sdf<<std::endl;
                    v_cube.voxels[voxel_id] += result;
                    
                }                
            }
            //we can add garbage collection
            return after_trans;
        }
        std::shared_ptr<CubeHandler> TransformNearest(const geometry::TransformationMatrix &trans)
        {
            std::shared_ptr<CubeHandler> after_trans = std::make_shared<CubeHandler>(camera);
            //after_trans->camera = camera;
            after_trans->far = far; 
            after_trans->near = near;
            after_trans->integrator = integrator;

            for(auto iter = cube_map.begin(); iter != cube_map.end(); ++iter)
            {
                after_trans->AddTransformedCubeNearest(iter->second, trans);
            }
            
            for(auto iter = after_trans->cube_map.begin(); iter != after_trans->cube_map.end(); ++iter)
            {
                //read each voxel from eight neighbor
                auto &v_cube = iter->second;
                for(size_t voxel_id = 0; voxel_id != v_cube.voxels.size(); ++voxel_id)
                {
                    geometry::Point3 center_position = c_para.GetGlobalPoint(v_cube.cube_id, voxel_id);
                    geometry::Vector4 tmp_position = 
                        trans.inverse() * geometry::Vector4( center_position(0), center_position(1), center_position(2), 1);
                    geometry::Point3 new_position = tmp_position.head<3>() / tmp_position(3) ;
                    geometry::Point3i chosen_voxel = geometry::Point3i(std::floor(new_position(0) / c_para.VoxelResolution), 
                        std::floor(new_position(1) / c_para.VoxelResolution), std::floor(new_position(2)/ c_para.VoxelResolution) );                
                    TSDFVoxel tsdf_voxel;
                    auto cube_id = c_para.GetCubeID(chosen_voxel);
                    auto _voxel_id = c_para.GetVoxelID(chosen_voxel);
                    auto the_cube = cube_map.find(cube_id);
                    if(the_cube!= cube_map.end())
                    {
                        tsdf_voxel = the_cube->second.GetVoxel(_voxel_id);
                    }
                    //use some weighted function to reconpute the voxel
                    v_cube.voxels[voxel_id] += tsdf_voxel;
                }                
            }
            //we can add garbage collection
            return after_trans;
        }      
        CubeMap GetCubeMap()
        {
            
            return cube_map;
        }
        void SetCubeMap(const CubeMap & _cube_map)
        {
            std::cout<<YELLOW<<"[WARNING]::[SetCubeMap]::Note that you are changing the hashing map directly."<<RESET<<std::endl;
            cube_map = _cube_map;
        }
        void SetFarPlane(float _far)
        {
            far = _far;
        }
        void SetNearPlane(float _near)
        {
            near = _near;
        }
        void CollectGarbage();
        protected:
        CubeMap cube_map;
        camera::PinholeCamera camera;
        Integrator integrator;
        CubePara c_para;
        float far = 5.0;
        float near = 0.5;

    };
}
}
#endif