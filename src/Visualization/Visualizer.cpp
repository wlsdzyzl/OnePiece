
#include "Visualizer.h"

namespace fucking_cool
{
namespace visualization 
{
    void Visualizer::ShowOnce()
    {
        //std::cout<<"point_step: "<<point_step<<std::endl;
        if(buffer_data_updated)
        {
            glBindBuffer(GL_ARRAY_BUFFER,vbo);
            glBufferSubData(GL_ARRAY_BUFFER, 0, point_buffer_size * sizeof(scalar), &point_buffer[0]);     

            glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo);
            glBufferSubData(GL_ELEMENT_ARRAY_BUFFER, 0, index_buffer_size * sizeof(int), &index_buffer[0]);

            buffer_data_updated = false;
        }
        PreCall();
        if(has_lines)
        {
            //Draw lines here
            //Use the most naive way.
            
            glLineWidth(0.5f);
            glBegin(GL_LINES);

            for(size_t i = 0; i < line_indexs.size(); ++ i)
            {
                size_t p_s = line_indexs[i].first;
                size_t p_d = line_indexs[i].second;
                
                glColor4f(line_colors[i](0), line_colors[i](1), line_colors[i](2), 0.0);
                glVertex3f(line_points[p_s](0), line_points[p_s](1), line_points[p_s](2));
                glVertex3f(line_points[p_d](0), line_points[p_d](1), line_points[p_d](2));
                
            }
            glEnd();
            glColor4f(1.0f, 1.0f, 1.0f, 0.0f);
        }
        if(has_cameras)
        {
            //draw camera
            for(size_t i = 0; i != camera_poses.size(); ++i)
            DrawCamera(camera_poses[i], camera_colors[i]);
        }
        std::shared_ptr<Shader> program = GetShader();
        // set this program as current program
        
        program->Bind();
        ConfigProgram(program, draw_normal, draw_color);
        if(point_step == 0)
            std::cout<<"[Visualizer]::[WARNING]::Nothing is in the buffer"<<std::endl;
        else
        {
        
            glEnableVertexAttribArray(0);
            glVertexAttribPointer(0, 3, data_type, GL_TRUE, point_step * sizeof(scalar), reinterpret_cast<GLvoid*>(0));//vertex
            if(point_step > 3)
            {
                glEnableVertexAttribArray(1);
                glVertexAttribPointer(1, 3, data_type, GL_TRUE,point_step * sizeof(scalar), reinterpret_cast<GLvoid*>(3*sizeof(scalar)));                
            }

            if(point_step > 6)
            {
                glEnableVertexAttribArray(2);
                glVertexAttribPointer(2, 3, data_type, GL_TRUE, point_step * sizeof(scalar),reinterpret_cast< GLvoid*>(6*sizeof(scalar)));
            }
                
            if(geometry_type == GeometryType::TRIANGLE_MESH)
                glDrawElements(GL_TRIANGLES,index_buffer_size, GL_UNSIGNED_INT,0);
            
            if(geometry_type == GeometryType::POINTCLOUD)
                glDrawArrays(GL_POINTS, 0, point_buffer_size/point_step);

            glDisableVertexAttribArray(0);
            if(point_step > 3) glDisableVertexAttribArray(1);
            if(point_step > 6) glDisableVertexAttribArray(2);
        }
        //glBindBuffer(GL_ARRAY_BUFFER, 0);

        program->Unbind();
        //pangolin::glDrawAxis(3);
        PostCall();
    }
    void Visualizer::Show()
    {
        if(point_step == 0)
        {
            std::cout<<"[Visualizer]::[WARNING]::Nothing is in the buffer"<<std::endl;
            return;
        }
        Initialize();
        
        while(!pangolin::ShouldQuit())
        {
            ShowOnce();
        }
        
    }
    void Visualizer::ConfigProgram(const std::shared_ptr<Shader> &program, const bool drawNormals, const bool drawColors)
    {                
        program->setUniform(Uniform("MVP", s_cam.GetProjectionModelViewMatrix()));
        //std::cout<<"mvp: "<<mvp<<std::endl;
        int color_type = (drawNormals ? 1 : drawColors ? 2 : 0);
        if(draw_color_phong) color_type = 3;
        //std::cout<<"color_type: "<<color_type<<std::endl;
        program->setUniform(Uniform("colorType", color_type));
        float s_materialShininess = 8.0f;
        Eigen::Vector4f s_materialAmbient   = Eigen::Vector4f(0.85f, 0.85f, 0.85f, 1.0f);
        Eigen::Vector4f s_materialDiffuse   = Eigen::Vector4f(0.85f, 0.85f, 0.85f, 1.0f);
        Eigen::Vector4f s_materialSpecular  = Eigen::Vector4f(1.0f, 1.0f, 1.0f, 1.0f);
        Eigen::Vector4f s_lightAmbient 	    = Eigen::Vector4f(0.4f, 0.4f, 0.4f, 1.0f);
        Eigen::Vector4f s_lightDiffuse 		= Eigen::Vector4f(0.6f, 0.52944f, 0.4566f, 0.6f);
        Eigen::Vector4f s_lightSpecular 	= Eigen::Vector4f(0.3f, 0.3f, 0.3f, 1.0f);
        Eigen::Vector3f lightDir 	= Eigen::Vector3f(0.0f, -1.0f, 2.0f);

        program->setUniform(Uniform("materialShininess", s_materialShininess));
        program->setUniform(Uniform("materialAmbient", s_materialAmbient));
        program->setUniform(Uniform("materialDiffuse", s_materialDiffuse));
        program->setUniform(Uniform("materialSpecular", s_materialSpecular));
        program->setUniform(Uniform("lightAmbient", s_lightAmbient));
        program->setUniform(Uniform("lightDiffuse", s_lightDiffuse));
        program->setUniform(Uniform("lightSpecular", s_lightSpecular));
        program->setUniform(Uniform("lightDir", lightDir));        
    }
    void Visualizer::AddPointCloud(const geometry::PointCloud &pcd)
    {
        if(dynamic_first_view)
        {
            ChooseCameraPoseFromPoints(pcd.points);
        }
        if(point_step != 0 &&geometry_type != GeometryType::POINTCLOUD)
        {
            std::cout<<YELLOW<<"[Visualizer]::[WARNING]::Geometry type is not pointcloud, the visualizer will clear buffer."<<RESET<<std::endl;
            Reset();
        }
        size_t size = pcd.GetSize();
        size_t step = 3;
        
        if(pcd.HasColors()) step += 3;
        if(pcd.HasNormals()) step += 3;
        if((step != point_step || has_normals != pcd.HasNormals() || has_colors != pcd.HasColors()) && point_step != 0)
        {
            std::cout<<YELLOW<<"[Visualizer]::[WARNING]::Different types of pointcloud, the visualizer will clear buffer."<<RESET<<std::endl;
            Reset();            
        }
        for(size_t i = 0;i!=size;++i)
        {
            //position
            size_t start = 0;
            for(size_t j = 0;j<3; ++j, ++start)
                point_buffer[point_buffer_size + i*step+start] = pcd.points[i](j);

            //normal
            if(pcd.HasNormals())
            {
            for(size_t j = 0;j<3; ++j, ++start)
                point_buffer[point_buffer_size + i*step+start] = pcd.normals[i](j);
            }
            //color
            if(pcd.HasColors())
            {
            for(size_t j = 0;j<3; ++j, ++start)
                point_buffer[point_buffer_size + i*step+start] = pcd.colors[i](j);
            }

        }
        buffer_data_updated = true;
        point_step = step;
        
        point_buffer_size +=step*size;

        geometry_type = GeometryType::POINTCLOUD;

        has_colors = pcd.HasColors();

        has_normals = pcd.HasNormals();

        SetShaderPath();
#if DEBUG_MODE
        std::cout<<BLUE<<"[Visualizer]::[INFO]::Point Buffer Size: "<<point_buffer_size;
        std::cout<<" Points: "<<point_buffer_size/point_step<<RESET<<std::endl;
#endif
        if(point_buffer_size > MAX_BUFFER_SIZE || index_buffer_size > MAX_BUFFER_SIZE)
        std::cout<<YELLOW<<"[Visualizer]::[WARNING]::Overflow."<<RESET<<std::endl;

    }
    void Visualizer::AddLineSet(const geometry::Point3List & _points, const std::vector<std::pair<int, int>> &_index,
        const geometry::Point3List &_line_colors)
    {
        // points: a, b, c, d, e, f
        // index:0,1,1,2,2,3,3,4,4,5, so index may be even
        // line:a-b b-c c-d d-e e-f
        if( _index.size() != _line_colors.size() || _index.size() == 0)
        {
#if DEBUG_MODE
        std::cout<<YELLOW<<"[Visualizer]::[WARNING]::The number of line colors must matches the lines, and cannot be zero."<<RESET<<std::endl;
#endif
            return;
        }
        line_points = _points;
        line_colors = _line_colors;
        line_indexs = _index;
        has_lines = true;
    }
    void Visualizer::AddCameraSet(const geometry::SE3List & _camera_poses, const geometry::Point3List &_camera_colors)
    {
        if( _camera_colors.size() != _camera_poses.size() || _camera_colors.size() == 0)
        {
#if DEBUG_MODE
        std::cout<<YELLOW<<"[Visualizer]::[WARNING]::The number of camera colors must matches the camera poses, and cannot be zero."<<RESET<<std::endl;
#endif
            return;
        }        
        camera_poses = _camera_poses;
        camera_colors = _camera_colors;
        has_cameras = true;
    }
    void Visualizer::AddTriangleMesh(const geometry::TriangleMesh &mesh)
    {
        if(dynamic_first_view)
        {
            ChooseCameraPoseFromPoints(mesh.points);
        }
        if( point_step != 0 &&geometry_type != GeometryType::TRIANGLE_MESH)
        {
            std::cout<<YELLOW<<"[Visualizer]::[WARNING]::Geometry type is not mesh, the visualizer will clear buffer."<<RESET<<std::endl;
            Reset();
        }
        size_t point_size = mesh.GetPointSize();
        size_t triangle_size = mesh.GetTriangleSize();
        size_t step = 3;
        if((step != point_step || has_normals != mesh.HasNormals() || has_colors != mesh.HasColors()) && point_step != 0)
        {
            std::cout<<YELLOW<<"[Visualizer]::[WARNING]::Different types of mesh, the visualizer will clear buffer."<<RESET<<std::endl;
            Reset();            
        }        
        if(mesh.HasColors()) step += 3;
        if(mesh.HasNormals()) step +=3;

        for(size_t i = 0;i!=point_size;++i)
        {
            //position
            size_t start = 0;
            for(size_t j = 0;j<3; ++j, ++start)
                point_buffer[point_buffer_size + i*step+start] = mesh.points[i](j);

            //normal
            if(mesh.HasNormals())
            {
            for(size_t j = 0;j<3; ++j, ++start)
                point_buffer[point_buffer_size + i*step+start] = mesh.normals[i](j);
            }
            //color
            if(mesh.HasColors())
            {
            for(size_t j = 0;j<3; ++j, ++start)
                point_buffer[point_buffer_size + i*step+start] = mesh.colors[i](j);
            }

        }

        for(size_t i = 0; i!=triangle_size; ++i)
        {
            for(size_t j = 0;j<3; ++j)
                index_buffer[i*3 + j] = mesh.triangles[i](j);
            //std::cout <<mesh.triangles[i]<<std::endl;
        }
        buffer_data_updated = true;
        point_step = step;
        point_buffer_size += step * point_size;
        index_buffer_size += 3 * triangle_size;  

        geometry_type = GeometryType::TRIANGLE_MESH;      
        has_colors = mesh.HasColors();
        has_normals = mesh.HasNormals();
        SetShaderPath();
#if DEBUG_MODE
        std::cout<<BLUE<<"[Visualizer]::[INFO]::Point Buffer Size: "<<point_buffer_size;
        std::cout<<" Points: "<<point_buffer_size/point_step<<std::endl;
        std::cout<<"[Visualizer]::[INFO]::Index Buffer Size: "<<index_buffer_size;
        std::cout<<" Triangles: "<<index_buffer_size/3<<RESET<<std::endl;
#endif

        if(point_buffer_size > MAX_BUFFER_SIZE || index_buffer_size > MAX_BUFFER_SIZE)
        std::cout<<YELLOW<<"[Visualizer]::[WARNING]::Overflow."<<RESET<<std::endl;

    }

}
}