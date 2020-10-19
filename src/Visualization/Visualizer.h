#ifndef VISUALIZER_H
#define VISUALIZER_H
#include <pangolin/pangolin.h>
#include <Eigen/Core>
#include "Geometry/PointCloud.h"
#include "Geometry/TriangleMesh.h"
#include "Tool/ConsoleColor.h"
#include "./Shaders/Shaders.h"
#include <pangolin/gl/gl.h>
#include <pangolin/gl/gldraw.h>
#include <GL/freeglut.h>
#include <GL/glut.h>
#include <GL/glew.h>
#define MAX_BUFFER_SIZE 1024*1024*30
namespace one_piece
{
namespace visualization 
{
    enum GeometryType
    {
        POINTCLOUD, TRIANGLE_MESH
    };
    class Visualizer
    {
        typedef geometry::scalar scalar;
        public:
        Visualizer()
        {
            if(sizeof(scalar) == sizeof(double))
            data_type = GL_DOUBLE;
            else data_type = GL_FLOAT;
            Reset();
            point_buffer = new scalar[MAX_BUFFER_SIZE];
            index_buffer = new int[MAX_BUFFER_SIZE];
            memset(point_buffer,0,MAX_BUFFER_SIZE*sizeof(scalar));
            memset(index_buffer,0,MAX_BUFFER_SIZE*sizeof(int));

        }
        ~Visualizer()
        {
            delete[] point_buffer;
            delete[] index_buffer;
            glDeleteBuffers(1, &ebo);
            glDeleteBuffers(1, &vbo);
        }
        void AddPointCloud( const geometry::PointCloud &pcd);
        void AddTriangleMesh(const geometry::TriangleMesh &mesh);
        void Show();
        void ShowOnce();
        void SetGeometryType(GeometryType type)
        {
            geometry_type = type;
        }
        void Initialize(const std::string &name = "OnePiece")
        {
            if(is_initialized) return;
            pangolin::CreateWindowAndBind(name,640,480); 
            glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
            glPixelStorei(GL_PACK_ALIGNMENT, 1);
            glEnable(GL_DEPTH_TEST);
            glDepthMask(GL_TRUE);
            glDepthFunc(GL_LESS);
            s_cam = pangolin::OpenGlRenderState(pangolin::ProjectionMatrix(640, 480, 420, 420, 320, 240, 0.1, 1000),
                                                pangolin::ModelViewLookAt(0, 0, -1, 0, 0, 1, pangolin::AxisNegY));
            pangolin::Display("cam").SetBounds(0, 1.0f, 0, 1.0f, -640 / 480.0)
                                    .SetHandler(new pangolin::Handler3D(s_cam));
            if(dynamic_first_view)
            SetModelViewMatrix(camera_pose_for_view);
            InitializeGlut();
            is_initialized = true;
            buffer_data_updated = false;
        }
        void PreCall()
        {
            glClearColor(1.0f,1.0f, 1.0f,1.0f);
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
            pangolin::Display("cam").Activate(s_cam);
        }
        void PostCall()
        {
            pangolin::FinishFrame();
            glFinish();
        }
        void SetDrawColor(bool dc)
        {
            draw_color = dc;
        }
        void SetModelViewMatrix(const geometry::TransformationMatrix &camera_pose, 
            bool reversed_model = true)
        {
            //transform to camera coordinate system
            geometry::Matrix3 camera_rotation = camera_pose.block<3, 3>(0, 0);
            
            geometry::Vector3 camera_position = camera_pose.block<3, 1>(0, 3);
            //in OpenGL
            //z
            geometry::Vector3 forward_gl = - camera_rotation.block<3, 1>(0, 2);
            //y
            geometry::Vector3 up_gl = camera_rotation.block<3, 1>(0, 1);
            if(reversed_model) up_gl = -up_gl;
            //x
            geometry::Vector3 right_gl = camera_rotation.block<3, 1>(0, 0);
            up_gl.normalize();
            right_gl.normalize();
            forward_gl.normalize();
            
            geometry::Matrix3 camera_rotation_gl;
            // axis of OpenGL coordinate system
            camera_rotation_gl.block<1, 3>(0, 0) = right_gl.transpose();
        
            camera_rotation_gl.block<1, 3>(1, 0) = up_gl.transpose();

            camera_rotation_gl.block<1, 3>(2, 0) = forward_gl.transpose();
            // change the zero point
            // we want the camera can be more far away to the object
            // because in OpenGL coordinate, the Z axis is towards the camera, so if camera wants to move far away from the 
            // objects, it need to add the z axis vector.
            auto camera_position_gl = - camera_rotation_gl * camera_position + 2 * forward_gl;
            
            
            Eigen::Matrix4d mv_eigen = Eigen::Matrix4d::Zero();
            mv_eigen.block<3, 3>(0, 0) = camera_rotation_gl.cast<double>();
            mv_eigen.block<3, 1>(0, 3) = camera_position_gl.cast<double>();
            mv_eigen(3, 3) = 1.0;
            pangolin::OpenGlMatrix mv;
            memcpy(&mv.m[0], mv_eigen.data(), sizeof(Eigen::Matrix4d));  

            s_cam.SetModelViewMatrix(mv);

        }
        void ChooseCameraPoseFromPoints(const geometry::Point3List &points)
        {
            geometry::Point3 average_point =  geometry::Point3::Zero();
            for(size_t i = 0; i != points.size(); ++i)
            {
                average_point += points[i];
            }
            average_point /= points.size();
            //just change the y value
            geometry::TransformationMatrix camera_pose = geometry::TransformationMatrix::Identity();
            camera_pose.block<3, 1>(0, 3) = average_point + geometry::Point3(0, -5, 0);
            //let camera
            // camera_pose.block<3, 1>(0, 0) = geometry::Point3(1, 0, 0);
            camera_pose.block<3, 1>(0, 1) = geometry::Point3(0, 0, 1);
            camera_pose.block<3, 1>(0, 2) = geometry::Point3(0, 1, 0);
            camera_pose_for_view = camera_pose;
        }
        void SetDrawNormal(bool dn)
        {
            draw_normal = dn;
        }
        void DrawPhongRendering()
        {
            draw_normal = false;
            draw_color = false;
        }
        void Reset()
        {
            point_step = 0;
            point_buffer_size = 0;
            index_buffer_size = 0;
            geometry_type = GeometryType::POINTCLOUD;
            has_lines = false;
            line_points.clear();
            line_colors.clear();
            line_indexs.clear();
            has_cameras = false;
            camera_poses.clear();
            camera_colors.clear();
        }
        void ConfigProgram(const std::shared_ptr<Shader> &program, 
            const bool drawNormals, const bool drawColors);
        std::shared_ptr<Shader> GetShader()
        {
            return loadProgramFromFile(shader_path,shader_vert,shader_frag);
        }
        void DrawCamera(const geometry::TransformationMatrix &camera_pose, 
            const geometry::Point3 &camera_color)
        {
            glPushMatrix();
            //glColor4f(1.0, 0.0f, 0.0f, 1.0f);
            Eigen::Matrix4f camera_pose_f = camera_pose.cast<float>();
            glMultMatrixf(camera_pose_f.data());
            DrawPyramidWireframe(camera_color);
            glPopMatrix();
            glColor4f(1.0f,1.0f,1.0f,0.0f);    
        }
        void DrawPyramidWireframe(const geometry::Point3 &rgb) {
            float pyrH = 0.4f;
            float pyrW = 0.2f;
            glLineWidth(1.5);
            glColor4f(rgb(0), rgb(1), rgb(2), 0.0f);
            glBegin(GL_LINE_LOOP);
            glVertex3f(pyrW,-pyrW,pyrH);
            glVertex3f(pyrW,pyrW,pyrH);
            glVertex3f(-pyrW,pyrW,pyrH);
            glVertex3f(-pyrW,-pyrW,pyrH);
            glEnd();

            glBegin(GL_LINES);
            glVertex3f(pyrW,-pyrW,pyrH);
            glVertex3f(0.0,0.0,0.0);
            glVertex3f(pyrW,pyrW,pyrH);
            glVertex3f(0.0,0.0,0.0);
            glVertex3f(-pyrW,pyrW,pyrH);
            glVertex3f(0.0,0.0,0.0);
            glVertex3f(-pyrW,-pyrW,pyrH);
            glVertex3f(0.0,0.0,0.0);
            glEnd();
        }
        void AddLineSet(const geometry::Point3List & _points, const std::vector<std::pair<int, int>> &_index,
        const geometry::Point3List &_line_colors);
        void AddCameraSet(const geometry::SE3List & _camera_poses, const geometry::Point3List &_camera_colors);
        void InitializeGlut()
        {
            int argc = 1;
            char ProjectName[256] = "one_piece";
            char *argv= ProjectName;
            glutInit(&argc, &argv);

            glutInitDisplayMode(GLUT_SINGLE);

            GLenum err=glewInit();

            glGenVertexArrays(1, &vao);
            glBindVertexArray(vao);

            glGenBuffers(1, &vbo);
            glBindBuffer(GL_ARRAY_BUFFER,vbo);
            glBufferData(GL_ARRAY_BUFFER, MAX_BUFFER_SIZE * sizeof(geometry::scalar), &point_buffer[0], GL_DYNAMIC_DRAW);     

            glGenBuffers(1, &ebo);
            glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo);
            glBufferData(GL_ELEMENT_ARRAY_BUFFER,MAX_BUFFER_SIZE *sizeof(int), &index_buffer[0], GL_DYNAMIC_DRAW);

            if(err!=GLEW_OK) 
            {
                // Problem: glewInit failed, something is seriously wrong.
                std::cout<<RED << "[Visualizer]::[ERROR]::glewInit failed: " << glewGetErrorString(err)<<RESET << std::endl;
                exit(1);
            }
            std::cout<<GREEN<<"[Visualizer]::[INFO]::Initialize successfully."<<RESET << std::endl;
        }

        void SetShaderPath(const std::string &vert_path, const std::string &frag_path)
        {
            shader_vert = vert_path;
            shader_frag = frag_path;
        }
        void SetShaderPath()
        {
            if(!has_colors && !has_normals)
            shader_vert = "draw_point.vert";
            if(has_colors && !has_normals)
            shader_vert = "draw_color.vert";
            if(!has_colors && has_normals)
            shader_vert = "draw_normal.vert";
            if(has_colors && has_normals)
            shader_vert = "draw_all.vert";

            std::cout<<BLUE<<"[Visualizer]::[INFO]::Using shader: "<<shader_vert<<RESET<<std::endl;
        }
        protected:
        size_t point_step;
        scalar * point_buffer;
        int * index_buffer;
        size_t point_buffer_size;
        size_t index_buffer_size;
        bool buffer_data_updated = false;
        //for draw line.
        geometry::Point3List line_points;
        geometry::Point3List line_colors;
        geometry::SE3List camera_poses;
        geometry::Point3List camera_colors;
        std::vector<std::pair<int, int>> line_indexs;
        
        //for model view
        geometry::TransformationMatrix camera_pose_for_view;
        
        bool has_lines = false;
        bool has_cameras = false;
        GeometryType geometry_type;
        bool dynamic_first_view = true;
        //vertex shader
        std::string shader_vert = "draw_all.vert";
        //fragment shader
        std::string shader_frag = "draw_feedback.frag";

        std::string shader_path = "../../src/Visualization/Shaders";
        bool draw_normal = false;
        bool draw_color = false;
        bool draw_color_phong = false;
        bool has_colors = true;
        bool has_normals = true;
        bool is_initialized = false;
        int data_type = GL_FLOAT;
        protected:
        
        GLuint vbo;
        GLuint ebo;
        GLuint vao;
        pangolin::OpenGlRenderState s_cam;
    };
}
}
#endif