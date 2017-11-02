#ifndef DEAD_RECKONING_H
#define DEAD_RECKONING_H
#include"ros/ros.h"
#include"std_msgs/Float64MultiArray.h"
#include"std_msgs/Int8MultiArray.h"
#include"sensor_msgs/Imu.h"
#include"mrpt/poses/CPoint3D.h"
#include"mrpt/poses/CPoint2D.h"
#include "mrpt/obs/CActionRobotMovement2D.h"
#include"std_msgs/Float64.h"
#include"std_msgs/Float32.h"
#include"std_msgs/Float32MultiArray.h"
#include<tf/transform_datatypes.h>
//using namespace mrpt;
//using namespace mrpt::poses;

#define my_pi 3.1415926
#define encoder_line_num 100
#define  cal_factor  0.0210386
#define pulse2meter_radio cal_factor
#define laserscan2car_pitch   -0.2*M_PI /180.0
#define laserscan2car_roll  0.7*M_PI /180.0
#define laserscan2car_yaw   0.0*M_PI /180.0
#define pitch_offset -0.003534
#define roll_offset 0.057116

typedef struct euler_angle
{
    tfScalar roll;
    tfScalar pitch;
    tfScalar yaw;
} euler_angle_type;

class dead_reckoning
{

    ros::NodeHandle nh_;
    ros::NodeHandle nh_test;
    ros::Subscriber velocity_sub_;//用来订阅消息
    ros::Subscriber pulse_sub;
    ros::Publisher car_angle_pub;//用来发布消息
    ros::Subscriber imu_sub_;

public:
    std_msgs::Float64MultiArray velocity_data;
    std_msgs::Int8MultiArray pulse_data;
    sensor_msgs::Imu imu_data;
    std_msgs::Float32MultiArray car_angle_msg;
    mrpt::poses::CPose2D robot_pose_inc ;
    mrpt::poses::CPose2D robot_global_pose;
    euler_angle_type car_angle;
    bool zhendong_flag;
    int zhendong_count;
    float car2N_matrix[3][3];//车身坐标系到参考坐标系的转换
    float sensor2car_matrix[3][3];
    float sensor2N_matrix[3][3];
    double yaw_angle_veolcity_sum;
    double pulse_sum;

    dead_reckoning();
    ~dead_reckoning()
    {

    }
    void velocity_callback(std_msgs::Float64MultiArray msg);
    void xsens_callback(const sensor_msgs::Imu & msg);
    void pulse_callback(std_msgs::Int8MultiArray msg );
    bool calculate_pose_inc();
    void clear_sum();
    euler_angle_type quaternions2euler();
    void  quaternions2rotation_matrix();
    float safe_asin(float v);
    void publish_car_angle();
};

#endif // DEAD_RECKONING_H
