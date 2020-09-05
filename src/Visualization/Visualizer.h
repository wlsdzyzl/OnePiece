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
namespace fucking_cool
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
        void Initialize(const std::string &name = "FuckingCool")
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
        void ConfigProgram(const std::shared_ptr<Shader> &program, const pangolin::OpenGlMatrix &mvp,
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
            char ProjectName[256] = "fucking_cool";
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
        int point_step;
        scalar * point_buffer;
        int * index_buffer;
        int point_buffer_size;
        int index_buffer_size;
        bool buffer_data_updated = false;
        //for draw line.
        geometry::Point3List line_points;
        geometry::Point3List line_colors;
        geometry::SE3List camera_poses;
        geometry::Point3List camera_colors;
        std::vector<std::pair<int, int>> line_indexs;

        bool has_lines = false;
        bool has_cameras = false;
        GeometryType geometry_type;
         
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
        GLuint vbo;
        GLuint ebo;
        GLuint vao;
        pangolin::OpenGlRenderState s_cam;
    };
}
}
#endif