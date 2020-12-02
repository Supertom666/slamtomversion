# 声明要求的 cmake 最低版本
cmake_minimum_required(VERSION 3.2)
# 声明一个 cmake 工程
project(0_1)

# 设置编译模式# 设置用debug还是release模式。debug允许断点，而release更快
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_BUILD_TYPE "Debug")
# 设置编译选项
# 允许c++11标准、O3优化、多线程。match选项可避免一些cpu上的问题
set( CMAKE_CXX_FLAGS "-std=c++11 -march=native -O3 -pthread" )
list( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules )

set( EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin )
set( LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib )
#set(Sophus_LIBRARIES libSophus.so)

# Eigen
FIND_PACKAGE(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
# OpenCV
FIND_PACKAGE(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
# Sophus
find_package(Sophus REQUIRED)
include_directories(${Sophus_INCLUDE_DIRS})
#Pangolin
find_package(Pangolin REQUIRED)
include_directories(${Pangolin_INCLUDE_DIRS})

# glog
find_package(Glog REQUIRED)
include_directories(${GLOG_INCLUDE_DIRS})

# gtest
find_package(GTest REQUIRED)
include_directories(${GTEST_INCLUDE_DIRS})

# gflags
find_package(GFlags REQUIRED)
include_directories(${GFLAGS_INCLUDE_DIRS})

# csparse
find_package(CSparse REQUIRED)
include_directories(${CSPARSE_INCLUDE_DIR})

#find_package(G2O REQUIRED)
include_directories(/home/tom/code/slambook2/3rdparty/g2o)

include_directories(/home/tom/code/slambook2/3rdparty/g2o/build)
set( THIRD_PARTY_LIBS
        ${OpenCV_LIBS}
        ${Sophus_LIBRARIES}
        g2o_core g2o_stuff g2o_types_sba g2o_solver_csparse g2o_csparse_extension
        ${Pangolin_LIBRARIES} GL GLU GLEW glut
        ${GTEST_BOTH_LIBRARIES}
        ${GLOG_LIBRARIES}
        ${GFLAGS_LIBRARIES}
        pthread
        ${CSPARSE_LIBRARY}
        )
#g2o_core g2o_stuff g2o_types_sba
############### source and test ##################3####
# 添加一个可执行程序
# 语法：add_executable( 程序名 源代码文件 ）
#add_executable(main main.cpp )


#add_executable( run_vo run_vo.cpp )
#${PROJECT_SOURCE_DIR}/src/config.cpp ${PROJECT_SOURCE_DIR}/src/visual_odometry.cpp
#MESSAGE("OpenCV_LIBS: " ${OpenCV_LIBS} )  #display opencv libs found
#include_directories( ${PROJECT_SOURCE_DIR}/include )
link_directories("/home/tom/code/slambook2/3rdparty/g2o/lib")
enable_testing()
include_directories(${PROJECT_SOURCE_DIR}/include)
add_subdirectory( src )
add_subdirectory( test )
add_subdirectory( app)
#target_link_libraries(run_vo ${OpenCV_LIBS})
#target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS} ${Sophus_LIBRARIES} ${EIGEN3_LIBRARIES} ${Pangolin_LIBRARIES} )
