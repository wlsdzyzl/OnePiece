cmake_minimum_required(VERSION 3.1)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}")
set (CMAKE_BUILD_TYPE "Release")
set (CMAKE_BINARY_DIR "./example")
find_package(OpenGL)
find_package(GLUT)
# set (EIGEN3_INCLUDE_DIRS "")
# set (OpenCV_INCLUDE_DIRS "")
# set (Pangolin_INCLUDE_DIR "")
# set(OPENNI2_LIBRARY "/home/zheng/Desktop/exp_for_leo/instance_demo/deps/OpenNI-Linux-x64-2.2/Redist/libOpenNI2.so")
# set(OPENNI2_INCLUDE_DIR "/home/zheng/Desktop/exp_for_leo/instance_demo/deps/OpenNI-Linux-x64-2.2/Include")
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Pangolin REQUIRED)
find_package(OpenNI2 REQUIRED)
find_package(Sophus REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${Pangolin_INCLUDE_DIR})
include_directories(${GLUT_INCLUDE_DIRS})
include_directories(${OPENGL_INCLUDE_DIRS})
include_directories(${OPENNI2_INCLUDE_DIR})
include_directories(${SOPHUS_INCLUDE_DIR})
# include_directories(3rdparty/rply)

include_directories(3rdparty/tinyply/source)
include_directories(3rdparty/tinyobjloader)
include_directories(3rdparty/MILD)
include_directories(3rdparty/GRANSAC)
include_directories(3rdparty/nanoflann/include)
include_directories(src)

#message("tinyobjloader")

file(GLOB Odometry_SRC
"src/Odometry/*.cpp"
"src/Odometry/*.h"
)

file(GLOB Geometry_SRC
"src/Geometry/*.cpp"
"src/Geometry/*.h"
"src/Geometry/*.hpp"
)

file(GLOB_RECURSE Visualization_SRC
"src/Visualization/Shaders/*.h"
"src/Visualization/*.cpp"
"src/Visualization/*.h"
)


file(GLOB Integration_SRC
"src/Integration/*.cpp"
"src/Integration/*.h"
)

file(GLOB Tool_SRC
"src/Tool/*.cpp"
"src/Tool/*.h"
)
#file(GLOB Rply_SRC
#"3rdparty/rply/*.c"
#"3rdparty/rply/*.h"
#)
file(GLOB Objloader_SRC
"3rdparty/tinyobjloader/*.c"
"3rdparty/tinyobjloader/*.cpp"
"3rdparty/tinyobjloader/*.cc"
"3rdparty/tinyobjloader/*.h"
)

file(GLOB TinyPly_SRC
"3rdparty/tinyply/source/tinyply.h"
"3rdparty/tinyply/source/tinyply.cpp"
)

file(GLOB Registration_SRC
"src/Registration/*.cpp"
"src/Registration/*.h"
)
file(GLOB Algorithm_SRC
"src/Algorithm/*.cpp"
"src/Algorithm/*.h"
)
file(GLOB LCDetection_SRC
"src/LCDetection/*.cpp"
"src/LCDetection/*.h"
)
file(GLOB Optimization_SRC
"src/Optimization/*.cpp"
"src/Optimization/*.h"
)
file(GLOB MILD_SRC
"3rdparty/MILD/*.hpp"
"3rdparty/MILD/*.h"
"3rdparty/MILD/*.cpp"
)

file(GLOB GRANSAC_SRC
"3rdparty/GRANSAC/*.hpp"
)
file(GLOB FBAFusion_SRC
"example/FBAFusion/*.cpp"
"example/FBAFusion/*.h"
)
file(GLOB BAFusion_SRC
"example/BAFusion/*.cpp"
"example/BAFusion/*.h"
)
file(GLOB DenseFusion_SRC
"example/DenseFusion/*.cpp"
"example/DenseFusion/*.h"
)

#add_library(rply SHARED ${Rply_SRC})
add_library(tinyply SHARED ${TinyPly_SRC})
add_library(objloader SHARED ${Objloader_SRC})
add_library(mild SHARED ${MILD_SRC})

#add_library(gransac SHARED ${GRANSAC_SRC})

add_library(lcdetection SHARED ${LCDetection_SRC})
add_library(integration SHARED ${Integration_SRC})
add_library(geometry SHARED ${Geometry_SRC})
add_library(visualization SHARED ${Visualization_SRC})
add_library(odometry SHARED ${Odometry_SRC})
add_library(tool SHARED ${Tool_SRC})
add_library(algorithm SHARED ${Algorithm_SRC})
add_library(regitration SHARED ${Optimization_SRC})
add_library(optimization SHARED ${Registration_SRC})
add_library(fucking_cool SHARED ${Geometry_SRC}  ${GRANSAC_SRC}
    ${Visualization_SRC} ${Odometry_SRC} ${Tool_SRC} ${LCDetection_SRC}  
    ${Integration_SRC} ${Algorithm_SRC} ${Registration_SRC} ${Optimization_SRC})

set(CMAKE_CXX_FLAGS " -std=c++11 ")
set(CMAKE_CXX_FLAGS " ${CMAKE_CXX_FLAGS} -O0 -g -msse4.2 -Wall -fopenmp") 
set (CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})

add_executable(SparseOdometry "example/SparseOdometry.cpp"   )
add_executable(SparseOdometryMild "example/SparseOdometryMild.cpp"   )
add_executable(DenseOdometry "example/DenseOdometry.cpp")
add_executable(ReadPLYPCD "example/ReadPLYPointCloud.cpp")
add_executable(ReadRGBD "example/ReadRGBD.cpp")
add_executable(ReadPLYMESH "example/ReadPLYMesh.cpp")
add_executable(SimplifyMeshQuadric "example/SimplifyMeshQuadric.cpp")
add_executable(SimplifyMeshClustering "example/SimplifyMeshClustering.cpp")
add_executable(MCGenerateMesh "example/MCGenerateMesh.cpp")
add_executable(EstimateNormals "example/EstimateNormals.cpp")
add_executable(ImageIntegration "example/ImageIntegration.cpp")
add_executable(ImageSequenceIntegration "example/ImageSequenceIntegration.cpp")
add_executable(RoomDetection "example/RoomDetection.cpp")
add_executable(Clustering "example/Clustering.cpp")
add_executable(ConvertImageSequenceToPCD "example/ConvertImageSequenceToPCD.cpp")
add_executable(PruneMesh "example/PruneMesh.cpp")
add_executable(AcquireLiveData "example/AcquireLiveData.cpp")
add_executable(DetectPlane "example/DetectPlane.cpp")
add_executable(MergeMultipleSubmaps "example/MergeMultipleSubmaps.cpp")
add_executable(RansacTest "example/RansacTest.cpp")
add_executable(ICPTest "example/ICPTest.cpp")
add_executable(BATest "example/BATest.cpp")
add_executable(GenerateModelFromScannet "example/GenerateModelFromScannet.cpp")
add_executable(GetLabelUsingKDTree "example/GetLabelUsingKDTree.cpp")
add_executable(FBAFusion ${FBAFusion_SRC})
add_executable(BAFusion ${BAFusion_SRC})
add_executable(DenseFusion ${DenseFusion_SRC})

target_link_libraries(SparseOdometry
    fucking_cool
    tinyply
    objloader
    mild
    ${OpenCV_LIBS} 
    ${Eigen_LIBRARIES}
    ${Pangolin_LIBRARIES}
    ${OPENGL_LIBRARIES} 
	${GLUT_LIBRARY}
)
target_link_libraries(SparseOdometryMild
    fucking_cool
    tinyply
    objloader
    mild
    ${OpenCV_LIBS} 
    ${Eigen_LIBRARIES}
    ${Pangolin_LIBRARIES}
    ${OPENGL_LIBRARIES} 
	${GLUT_LIBRARY}
)
target_link_libraries(DenseOdometry 
    fucking_cool
    tinyply
    objloader
    mild
    ${OpenCV_LIBS} 
    ${Eigen_LIBRARIES}
    ${Pangolin_LIBRARIES}
    ${OPENGL_LIBRARIES} 
	${GLUT_LIBRARY}
)
target_link_libraries(ReadPLYPCD
    fucking_cool
    tinyply
    objloader
    mild
    ${OpenCV_LIBS}
    ${Eigen_LIBRARIES}
    ${Pangolin_LIBRARIES}
    ${OPENGL_LIBRARIES} 
	${GLUT_LIBRARY}
)
target_link_libraries(ReadRGBD
    fucking_cool
    tinyply
    objloader
    mild
    ${OpenCV_LIBS}
    ${Eigen_LIBRARIES}
    ${Pangolin_LIBRARIES}
    ${OPENGL_LIBRARIES} 
	${GLUT_LIBRARY}
)
target_link_libraries(ReadPLYMESH
    fucking_cool
    tinyply
    objloader
    mild
    ${OpenCV_LIBS}
    ${Eigen_LIBRARIES}
    ${Pangolin_LIBRARIES}
    ${OPENGL_LIBRARIES} 
	${GLUT_LIBRARY}
)
target_link_libraries(SimplifyMeshQuadric
    fucking_cool
    tinyply
    objloader
    mild
    ${OpenCV_LIBS}
    ${Eigen_LIBRARIES}
    ${Pangolin_LIBRARIES}
    ${OPENGL_LIBRARIES} 
	${GLUT_LIBRARY}
)
target_link_libraries(SimplifyMeshClustering
    fucking_cool
    tinyply
    objloader
    mild
    ${OpenCV_LIBS}
    ${Eigen_LIBRARIES}
    ${Pangolin_LIBRARIES}
    ${OPENGL_LIBRARIES} 
	${GLUT_LIBRARY}
)
target_link_libraries(MCGenerateMesh
    fucking_cool
    tinyply
    objloader
    mild
    ${OpenCV_LIBS}
    ${Eigen_LIBRARIES}
    ${Pangolin_LIBRARIES}
    ${OPENGL_LIBRARIES} 
	${GLUT_LIBRARY}
)
target_link_libraries(EstimateNormals
    fucking_cool
    tinyply
    objloader
    mild
    ${OpenCV_LIBS}
    ${Eigen_LIBRARIES}
    ${Pangolin_LIBRARIES}
    ${OPENGL_LIBRARIES} 
	${GLUT_LIBRARY}
)
target_link_libraries(ImageIntegration
    fucking_cool
    tinyply
    objloader
    mild
    ${OpenCV_LIBS}
    ${Eigen_LIBRARIES}
    ${Pangolin_LIBRARIES}
    ${OPENGL_LIBRARIES} 
	${GLUT_LIBRARY}
)
target_link_libraries(ImageSequenceIntegration
    fucking_cool
    tinyply
    objloader
    mild
    ${OpenCV_LIBS}
    ${Eigen_LIBRARIES}
    ${Pangolin_LIBRARIES}
    ${OPENGL_LIBRARIES} 
	${GLUT_LIBRARY}
)
target_link_libraries(RoomDetection
    fucking_cool
    tinyply
    objloader
    mild
    ${OpenCV_LIBS}
    ${Eigen_LIBRARIES}
    ${Pangolin_LIBRARIES}
    ${OPENGL_LIBRARIES} 
	${GLUT_LIBRARY}
)
target_link_libraries(Clustering
    fucking_cool
    tinyply
    objloader
    mild
    ${OpenCV_LIBS}
    ${Eigen_LIBRARIES}
    ${Pangolin_LIBRARIES}
    ${OPENGL_LIBRARIES} 
	${GLUT_LIBRARY}
)
target_link_libraries(ConvertImageSequenceToPCD
    fucking_cool
    tinyply
    objloader
    mild
    ${OpenCV_LIBS}
    ${Eigen_LIBRARIES}
    ${Pangolin_LIBRARIES}
    ${OPENGL_LIBRARIES} 
	${GLUT_LIBRARY}
)
target_link_libraries(PruneMesh
    fucking_cool
    tinyply
    objloader
    mild
    ${OpenCV_LIBS}
    ${Eigen_LIBRARIES}
    ${Pangolin_LIBRARIES}
    ${OPENGL_LIBRARIES} 
	${GLUT_LIBRARY}
)

target_link_libraries(AcquireLiveData
    fucking_cool
    tinyply
    objloader
    mild
    ${OpenCV_LIBS} 
    ${Eigen_LIBRARIES}
    ${Pangolin_LIBRARIES}
    ${OPENGL_LIBRARIES} 
	${GLUT_LIBRARY}
    ${OPENNI2_LIBRARY}
)

target_link_libraries(DetectPlane
    fucking_cool
    tinyply
    objloader
    mild
    ${OpenCV_LIBS} 
    ${Eigen_LIBRARIES}
    ${Pangolin_LIBRARIES}
    ${OPENGL_LIBRARIES} 
	${GLUT_LIBRARY}
)

target_link_libraries(MergeMultipleSubmaps
    fucking_cool
    tinyply
    objloader
    mild
    ${OpenCV_LIBS} 
    ${Eigen_LIBRARIES}
    ${Pangolin_LIBRARIES}
    ${OPENGL_LIBRARIES} 
	${GLUT_LIBRARY}
)
target_link_libraries(RansacTest
    fucking_cool
    tinyply
    objloader
    mild
    ${OpenCV_LIBS} 
    ${Eigen_LIBRARIES}
    ${Pangolin_LIBRARIES}
    ${OPENGL_LIBRARIES} 
	${GLUT_LIBRARY}
)
target_link_libraries(ICPTest
    fucking_cool
    tinyply
    objloader
    mild
    ${OpenCV_LIBS} 
    ${Eigen_LIBRARIES}
    ${Pangolin_LIBRARIES}
    ${OPENGL_LIBRARIES} 
	${GLUT_LIBRARY}
)
target_link_libraries(BATest
    fucking_cool
    tinyply
    objloader
    mild
    ${OpenCV_LIBS} 
    ${Eigen_LIBRARIES}
    ${Pangolin_LIBRARIES}
    ${OPENGL_LIBRARIES} 
	${GLUT_LIBRARY}
)
target_link_libraries(GenerateModelFromScannet
    fucking_cool
    tinyply
    objloader
    mild
    ${OpenCV_LIBS} 
    ${Eigen_LIBRARIES}
    ${Pangolin_LIBRARIES}
    ${OPENGL_LIBRARIES} 
	${GLUT_LIBRARY}
)
target_link_libraries(GetLabelUsingKDTree
    fucking_cool
    tinyply
    objloader
    mild
    ${OpenCV_LIBS} 
    ${Eigen_LIBRARIES}
    ${Pangolin_LIBRARIES}
    ${OPENGL_LIBRARIES} 
	${GLUT_LIBRARY}
)
target_link_libraries(FBAFusion
    fucking_cool
    tinyply
    objloader
    mild
    ${OpenCV_LIBS} 
    ${Eigen_LIBRARIES}
    ${Pangolin_LIBRARIES}
    ${OPENGL_LIBRARIES} 
	${GLUT_LIBRARY}
)
target_link_libraries(BAFusion
    fucking_cool
    tinyply
    objloader
    mild
    ${OpenCV_LIBS} 
    ${Eigen_LIBRARIES}
    ${Pangolin_LIBRARIES}
    ${OPENGL_LIBRARIES} 
	${GLUT_LIBRARY}
)
target_link_libraries(DenseFusion
    fucking_cool
    tinyply
    objloader
    mild
    ${OpenCV_LIBS} 
    ${Eigen_LIBRARIES}
    ${Pangolin_LIBRARIES}
    ${OPENGL_LIBRARIES} 
	${GLUT_LIBRARY}
)