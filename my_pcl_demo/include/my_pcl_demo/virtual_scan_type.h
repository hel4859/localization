#ifndef VIRTUAL_SCAN_TYPE_H
#define VIRTUAL_SCAN_TYPE_H

#include <mrpt/gui/CDisplayWindow3D.h>
#include <mrpt/maps/CSimpleMap.h>

#include <mrpt/maps/COccupancyGridMap2D.h>
#include <mrpt/maps/CMultiMetricMap.h>
#include <mrpt/maps/COctoMap.h>
#include"mrpt/gui.h"
#include"mrpt/opengl.h"
#include <mrpt/math.h>
#include"opencv2/opencv.hpp"

#include"mrpt/system/os.h"
#include"math.h"
#include"mrpt/utils/CImage.h"
#include"dead_reckoning.h"


#include"ros/ros.h"
#define Pi 3.1415926
#define HalfPi Pi/2
#define Car_width   4  //车辆宽度
#define scan_point_num  361  //虚拟扫描的点数
#define  half_scan_num  (scan_point_num/2)
#define scan_angle_area 90
#define Max_range   40.0    //虚拟扫描极限距离值
#define grid_map_resolution 0.2     //栅格图分辨率
#define variance_threshold  200     //距离方差阈值
#define average_angle_threshole 35  //平均角度偏离阈值
#define gray_threshold  0.7             //栅格图灰度阈值

#define filter_length   5       //路口判断结果中值滤波器长度
#define start_angle 180
#define end_angle   360
#define middle_angle    ((start_angle + end_angle)/2)

float get_arch_dimension(float radius,float rad);


typedef struct phase_angle_math
{
    float   variance;//方差
    float average_angle;//中间角度的平均值
    float start_average ;//右边边沿角度的平均值
    float end_average ;//左边边沿角度的平均值
}phase_angle_math_type;

typedef struct admissable_space
{
    int start;
    int middle;
    int end;
    float arch_radius;
    float dimension;
}admissable_space_type;
typedef struct pose
{
    float x;
    float y;
}pose_type;
typedef struct road_edge
{
    pose_type left;

    pose_type right;
}road_edge_type;

typedef struct road_middle
{
    pose_type middle;
}road_middle_type;

typedef struct line_disappear
{
    bool is_left_line_disappear;
    bool is_right_line_disappear;
}line_disappear_type;

class virtual_scan_type
{
private:
    ros::NodeHandle nh;
    ros::Subscriber sub_error_angle;//用来订阅消息

    enum intersection_type {left_T_style, right_T_style,cross_style,road_segment,left_angle,right_angle};
    std::vector <float> virtual_scan_result;//虚拟扫描出的距离最终结果
    std::vector <float> car_based_virtual_scan_result;
    char is_intersection_raw[filter_length];//判断结果fifo
    phase_angle_math_type   math_characteristic;//可行域的数学特征
    void math_characteristic_process();
    void get_virtual_scan_val(mrpt::obs::CObservation2DRangeScan &range_scan, float gray_scale_threshold);//获取虚拟扫描的结果
    std::vector<admissable_space_type> get_passable_area(std::vector <float> &scan_result,float min_radius,float max_radius,float delat_radius);//以给定的参数获得可行域
    void passable_area_filter(std::vector<admissable_space_type> &input_space,std::vector<admissable_space_type> &out_space,float direction_angle,float error_angle);
    void passable_area_filter(std::vector<admissable_space_type> &input_space,std::vector<admissable_space_type> &out_space,float error_angle);
    intersection_type intersection_detection_type;  //路口的类型
    dead_reckoning  DR;
    float road_width  ;

public:
    double car_yaw;
    double target_angle;
    double intersectiont_error_angle;
    double target_distance;
    line_disappear_type is_line_disappear;
    mrpt::poses::CPose3D senser_pose; //虚拟扫描点位置
    mrpt::poses::CPose2D    curRobotPoseEst;//车辆所在的位置

    std::vector<admissable_space_type> admisbale_space;//可行域
    std::vector<admissable_space_type> admisbale_space_filtered;//可行域

    std::vector<admissable_space_type> car_based_admisbale_space;//可行域
    std::vector<admissable_space_type> car_based_admisbale_space_filtered;//可行域
    mrpt::maps::COccupancyGridMap2D gridmap_local;
    mrpt::obs::CObservation2DRangeScan gridmap_virtual_scan;
    mrpt::obs::CObservation2DRangeScan car_based_gridmap_virtual_scan;
    virtual_scan_type();

    void multi_angle_distance_callback(std_msgs::Float64MultiArray msg);
    void set_robot_position(mrpt::poses::CPose2D robot_pos);
    void set_senser_position(mrpt::poses::CPose3D senser_pos);
    void intersection_process(mrpt::maps::COccupancyGridMap2D &grid_map);

    void get_line_state();
    bool is_intesection();
    void calculate_senser_pose();
    void get_road_width();
};




#endif // VIRTUAL_SCAN_TYPE_H
