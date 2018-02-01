cmake_minimum_required(VERSION 2.8.3)
project(optimal_function)
set( CMAKE_BUILD_TYPE "Release" )
set( CMAKE_CXX_FLAGS "-std=c++11 -O3" )

# 添加cmake模块以使用ceres库
list( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules )

# 寻找Ceres库并添加它的头文件
find_package( Ceres REQUIRED )


# OpenCV
find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_DIRS} )
find_package(catkin REQUIRED COMPONENTS
        
        geometry_msgs
        nav_msgs
        sensor_msgs
        roscpp
        rospy
        std_msgs
        tf
        pcl_ros
        pcl_conversions
        )



catkin_package(

)

include_directories( ${CERES_INCLUDE_DIRS} )
include_directories(
        # include
        ${catkin_INCLUDE_DIRS}
)
include_directories(include)

add_executable(optimal_function src/optimal_funtion.cpp)


target_link_libraries(optimal_function
        ${catkin_LIBRARIES}
        ${CERES_LIBRARIES} 
        ${OpenCV_LIBS}
        )
