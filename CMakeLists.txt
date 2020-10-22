cmake_minimum_required(VERSION 3.1)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}")
set (CMAKE_BUILD_TYPE "Release")
set (CMAKE_BINARY_DIR "./example")

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(OpenGL REQUIRED)
find_package(GLUT REQUIRED)
find_package(GLEW REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
# optional
find_package(OpenNI2)

add_subdirectory(3rdparty/Pangolin)
link_directories(${Pangolin_DIRS})

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${GLUT_INCLUDE_DIRS})
include_directories(${OPENGL_INCLUDE_DIRS})
include_directories(${OPENNI2_INCLUDE_DIR})
include_directories(${EIGEN3_INCLUDE_DIR})

include_directories(${Pangolin_INCDIRS})
include_directories(3rdparty/tinyply/source)
include_directories(3rdparty/tinyobjloader)
include_directories(3rdparty/MILD)
include_directories(3rdparty/GRANSAC)
include_directories(3rdparty/nanoflann/include)
include_directories(3rdparty/jsoncpp/include)
include_directories(3rdparty/Sophus)

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
file(GLOB Jsoncpp_SRC
"3rdparty/jsoncpp/include/json/*.h"
"3rdparty/jsoncpp/src/lib_json/json_tool.h"
"3rdparty/jsoncpp/src/lib_json/json_reader.cpp"
"3rdparty/jsoncpp/src/lib_json/json_valueiterator.inl"
"3rdparty/jsoncpp/src/lib_json/json_value.cpp"
"3rdparty/jsoncpp/src/lib_json/json_writer.cpp"
"3rdparty/jsoncpp/src/lib_json/version.h.in"
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
add_library(jsoncpp SHARED ${Jsoncpp_SRC})
add_library(mild SHARED ${MILD_SRC})

#add_library(gransac SHARED ${GRANSAC_SRC})




add_library(one_piece SHARED ${Geometry_SRC}  ${GRANSAC_SRC}
    ${Visualization_SRC} ${Odometry_SRC} ${Tool_SRC} ${LCDetection_SRC}  
    ${Integration_SRC} ${Algorithm_SRC} ${Registration_SRC} ${Optimization_SRC})



set(CMAKE_CXX_FLAGS " -std=c++11 ")
set(CMAKE_CXX_FLAGS " ${CMAKE_CXX_FLAGS} -O3 -msse4.2 -Wall -fopenmp") 
if(OpenNI2_FOUND)
    set(CMAKE_CXX_FLAGS " ${CMAKE_CXX_FLAGS} -DWITH_OPENNI2")
endif()
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
if(OpenNI2_FOUND)
    add_executable(AcquireLiveData "example/AcquireLiveData.cpp")
endif()
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

set(THIRD_LIB 
    tinyply 
    objloader
    mild
    jsoncpp
    pangolin
    ${OpenCV_LIBS} 
    ${GLUT_LIBRARIES}  
    ${GLEW_LIBRARIES} 
    ${OPENGL_LIBRARIES} 
)
target_link_libraries(SparseOdometry
    one_piece
    ${THIRD_LIB}
)
target_link_libraries(SparseOdometryMild
    one_piece
    ${THIRD_LIB}
)
target_link_libraries(DenseOdometry 
    one_piece
    ${THIRD_LIB}
)
target_link_libraries(ReadPLYPCD
    one_piece
    ${THIRD_LIB}
)
target_link_libraries(ReadRGBD
    one_piece
    ${THIRD_LIB}
)
target_link_libraries(ReadPLYMESH
    one_piece
    ${THIRD_LIB}
)
target_link_libraries(SimplifyMeshQuadric
    one_piece
    ${THIRD_LIB}
)
target_link_libraries(SimplifyMeshClustering
    one_piece
    ${THIRD_LIB}
)
target_link_libraries(MCGenerateMesh
    one_piece
    ${THIRD_LIB}
)
target_link_libraries(EstimateNormals
    one_piece
    ${THIRD_LIB}
)
target_link_libraries(ImageIntegration
    one_piece
    ${THIRD_LIB}
)
target_link_libraries(ImageSequenceIntegration
    one_piece
    ${THIRD_LIB}
)
target_link_libraries(RoomDetection
    one_piece
    ${THIRD_LIB}
)
target_link_libraries(Clustering
    one_piece
    ${THIRD_LIB}
)
target_link_libraries(ConvertImageSequenceToPCD
    one_piece
    ${THIRD_LIB}
)
target_link_libraries(PruneMesh
    one_piece
    ${THIRD_LIB}
)
if(OpenNI2_FOUND)
target_link_libraries(AcquireLiveData
    one_piece
    ${THIRD_LIB}
)
endif()

target_link_libraries(DetectPlane
    one_piece
    ${THIRD_LIB}
)

target_link_libraries(MergeMultipleSubmaps
    one_piece
    ${THIRD_LIB}
)
target_link_libraries(RansacTest
    one_piece
    ${THIRD_LIB}
)
target_link_libraries(ICPTest
    one_piece
    ${THIRD_LIB}
)
target_link_libraries(BATest
    one_piece
    ${THIRD_LIB}
)
target_link_libraries(GenerateModelFromScannet
    one_piece
    ${THIRD_LIB}
)
target_link_libraries(GetLabelUsingKDTree
    one_piece
    ${THIRD_LIB}
)
target_link_libraries(FBAFusion
    one_piece
    ${THIRD_LIB}
)
target_link_libraries(BAFusion
    one_piece
    ${THIRD_LIB}
)
target_link_libraries(DenseFusion
    one_piece
    ${THIRD_LIB}
)