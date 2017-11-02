#include "dead_reckoning.h"
#include"stdio.h"
#include"math.h"
#include"message_filters/synchronizer.h"
using namespace std;

dead_reckoning::dead_reckoning()
{
    imu_sub_ = nh_.subscribe("/imu_torso/xsens/data",1,&dead_reckoning::xsens_callback,this);
    velocity_sub_ = nh_.subscribe("velocity",1,&dead_reckoning::velocity_callback,this);
    pulse_sub = nh_.subscribe("pulse",1,&dead_reckoning::pulse_callback,this);
    car_angle_pub =nh_.advertise<std_msgs::Float32MultiArray>("car_angle",1000);

    car_angle.pitch=0.0;
    car_angle.roll =0.0;
    car_angle.yaw =0.0;
    yaw_angle_veolcity_sum =0;
    pulse_sum =0;
    robot_pose_inc = mrpt::poses::CPose2D(0, 0, 0);
    robot_global_pose= mrpt::poses::CPose2D(0, 0, 0);

    sensor2car_matrix[0][0]= cos( laserscan2car_pitch) *cos(laserscan2car_yaw);
    sensor2car_matrix[0][1]=-cos(laserscan2car_roll)*sin(laserscan2car_yaw) + sin(laserscan2car_roll)*sin(laserscan2car_pitch)*cos(laserscan2car_yaw);//-cos(roll)sin(yaw)+sin(roll)sin(pitch)cos(yaw)
    sensor2car_matrix[0][2]=sin(laserscan2car_roll)*sin(laserscan2car_yaw)+cos(laserscan2car_roll)*sin(laserscan2car_pitch)*cos(laserscan2car_yaw);
    sensor2car_matrix[1][0]=cos(laserscan2car_pitch)*sin(laserscan2car_yaw);
    sensor2car_matrix[1][1]=cos(laserscan2car_roll)*cos(laserscan2car_yaw) + sin(laserscan2car_roll)*sin(laserscan2car_pitch)*sin(laserscan2car_yaw);
    sensor2car_matrix[1][2]=-sin(laserscan2car_roll)*cos(laserscan2car_yaw) + cos(laserscan2car_roll)*sin(laserscan2car_pitch)*sin(laserscan2car_yaw);
    sensor2car_matrix[2][0]= -sin(laserscan2car_pitch);
    sensor2car_matrix[2][1]=sin(laserscan2car_roll)* cos(laserscan2car_pitch);
    sensor2car_matrix[2][2]= cos(laserscan2car_roll) * cos(laserscan2car_pitch);

    car2N_matrix[0][0] = 0;//cos(pitch)cos(yaw)
    car2N_matrix[0][1] =0;//-cos(roll)sin(yaw)+sin(roll)sin(pitch)cos(yaw)
    car2N_matrix[0][2] = 0;
    car2N_matrix[1][0] = 0;
    car2N_matrix[1][1] = 0;
    car2N_matrix[1][2] = 0;
    car2N_matrix[2][0] = 0;
    car2N_matrix[2][1] = 0;
    car2N_matrix[2][2] = 0;

    sensor2N_matrix[0][0] = 0;//cos(pitch)cos(yaw)
    sensor2N_matrix[0][1] =0;//-cos(roll)sin(yaw)+sin(roll)sin(pitch)cos(yaw)
    sensor2N_matrix[0][2] = 0;
    sensor2N_matrix[1][0] = 0;
    sensor2N_matrix[1][1] = 0;
    sensor2N_matrix[1][2] = 0;
    sensor2N_matrix[2][0] = 0;
    sensor2N_matrix[2][1] = 0;
    sensor2N_matrix[2][2] = 0;
    zhendong_flag = false;
    zhendong_count=0;
}

void dead_reckoning::xsens_callback(const sensor_msgs::Imu &msg)
{
//    ROS_INFO("i receive the IMU mesage!");
    static double time_present =ros::Time::now().toSec();
    static double time_previous =ros::Time::now().toSec();

    static double time_delat=0.0;
    #define filter_factor   0.5
#define zhendong_threshold  0.04
    imu_data = msg;

    if(fabs(imu_data.angular_velocity.y) > zhendong_threshold)
    {
        zhendong_count++;
        zhendong_flag = true;
    }
//    if(zhendong_count >3)
//    {
//        zhendong_flag = true;
//    }
//    else
//    {
//        zhendong_flag =false;
//    }
#undef zhendong_threshold
    time_present = ros::Time::now().toSec();//单位是秒
    time_delat = time_present - time_previous;
//    ROS_INFO("delat time is %f",time_delat);
    time_previous = time_present;
    double yaw_angle_inc = msg.angular_velocity.z * time_delat;
    yaw_angle_veolcity_sum += yaw_angle_inc; //积分，单位是弧度/s

    quaternions2rotation_matrix();
//    publish_car_angle();
//    printf("time_delat is %.10f \n",time_delat);
//    printf("yaw_angle_inc is %.10f \n",yaw_angle_inc);
//    printf("yaw_angle_speed is %.10f \n",msg.angular_velocity.z);
//    ROS_INFO(" IMU mesage end!");
#undef filter_factor
}
void dead_reckoning::publish_car_angle()
{
    car_angle_msg.data.clear();
    car_angle_msg.data.push_back(car_angle.pitch);
    car_angle_msg.data.push_back(car_angle.roll);
    car_angle_msg.data.push_back(car_angle.yaw);
    car_angle_pub.publish(car_angle_msg);
}
void dead_reckoning::velocity_callback(std_msgs::Float64MultiArray msg)
{
    int size=0;
    static double time_present =ros::Time::now().toSec();
    static double time_previous =ros::Time::now().toSec();
    static double time_delat=0.0;
    velocity_data = msg;
//    ROS_INFO("the  velocity is %f\n",msg.data[0]);
//     printf("the  velocity is %f\n",msg.data[0]);
}
void dead_reckoning::pulse_callback(std_msgs::Int8MultiArray msg )
{
//    ROS_INFO("i heard the pulse msgs!");
    static double time_present =ros::Time::now().toSec();
    static double time_previous =ros::Time::now().toSec();
    static double time_delat=0.0;
    pulse_data = msg;
    float pulse_temp=(pulse_data.data[0] + pulse_data.data[1])/2.0;//两轮的平均脉冲数
    pulse_sum +=pulse_temp*pulse2meter_radio ; //脉冲数积分得到位移值
//    printf("the  pulse is %d\n",pulse_temp );
//    ROS_INFO(" pulse mesage end!");
}
bool dead_reckoning::calculate_pose_inc()
{
    if(pulse_sum >0.14)//速度太慢，则点云不再叠加
    {
        robot_pose_inc.x(pulse_sum * cos(yaw_angle_veolcity_sum /2));
        robot_pose_inc.y(pulse_sum * sin(yaw_angle_veolcity_sum /2));
        robot_pose_inc.phi(yaw_angle_veolcity_sum);
        robot_global_pose +=robot_pose_inc;
    //    ROS_INFO(" the  pulse is %f\n",pulse_sum);
    //    printf("yaw angle is %.10f \n",yaw_angle_veolcity_sum);
    //    printf("the  pulse is %f\n",pulse_sum );
    //    printf("new pose is %.10f,  %.10f  %.10f\n",robot_pose_inc.x(),robot_pose_inc.y(),robot_pose_inc.phi());
        clear_sum();//清除积分
        return true;
    }
    else
    {
        return false;
    }
}
void dead_reckoning::clear_sum()
{
    pulse_sum =0.0;
    yaw_angle_veolcity_sum =0.0;
}

euler_angle_type dead_reckoning::quaternions2euler()
{
    tf::Quaternion q;
    tf::quaternionMsgToTF(imu_data.orientation, q);
    tf::Matrix3x3(q).getRPY(car_angle.roll, car_angle.pitch, car_angle.yaw);
/**********
    car_angle.roll = (atan2(2.0*(imu_data.orientation.x*imu_data.orientation.w + imu_data.orientation.y*imu_data.orientation.z),
                             1 - 2.0*(imu_data.orientation.y*imu_data.orientation.y + imu_data.orientation.x*imu_data.orientation.x)));
    car_angle.pitch =safe_asin(2.0*(imu_data.orientation.w*imu_data.orientation.y - imu_data.orientation.z*imu_data.orientation.x));
    car_angle.yaw = atan2(2.0*(imu_data.orientation.z*imu_data.orientation.w + imu_data.orientation.y*imu_data.orientation.x),
                          1 - 2.0*(imu_data.orientation.y*imu_data.orientation.y + imu_data.orientation.z*imu_data.orientation.z));
    car_angle.roll -=roll_offset;
    car_angle.pitch -=pitch_offset;
//        printf("roll %f,  pitch %f,  yaw %f \n",car_angle.roll,car_angle.pitch,car_angle.yaw);
*********/
}

float dead_reckoning::safe_asin(float v)
{
    if (isnan(v)) {
        return 0.0;
    }
    if (v >= 1.0) {
        return M_PI/2;
    }
    if (v <= -1.0) {
        return -M_PI/2;
    }
    return asin(v);
}
void  dead_reckoning::quaternions2rotation_matrix()
{
       quaternions2euler();//
//    car2N_matrix[0][0] = imu_data.orientation.w*imu_data.orientation.w + imu_data.orientation.x*imu_data.orientation.x - imu_data.orientation.y*imu_data.orientation.y - imu_data.orientation.z*imu_data.orientation.z ;//cos(pitch)cos(yaw)
//    car2N_matrix[0][1] =2*(imu_data.orientation.x*imu_data.orientation.y - imu_data.orientation.w*imu_data.orientation.z);//-cos(roll)sin(yaw)+sin(roll)sin(pitch)cos(yaw)
//    car2N_matrix[0][2] = 2*(imu_data.orientation.x * imu_data.orientation.z +imu_data.orientation.w*imu_data.orientation.y);
//    car2N_matrix[1][0] = 2*(imu_data.orientation.x * imu_data.orientation.y + imu_data.orientation.w*imu_data.orientation.z);
//    car2N_matrix[1][1] =imu_data.orientation.w*imu_data.orientation.w - imu_data.orientation.x*imu_data.orientation.x + imu_data.orientation.y*imu_data.orientation.y - imu_data.orientation.z*imu_data.orientation.z  ;
//    car2N_matrix[1][2] = 2*(imu_data.orientation.y*imu_data.orientation.z - imu_data.orientation.w*imu_data.orientation.x);
//    car2N_matrix[2][0] = 2*(imu_data.orientation.x*imu_data.orientation.z - imu_data.orientation.w*imu_data.orientation.y);
//    car2N_matrix[2][1] = 2*(imu_data.orientation.y*imu_data.orientation.z + imu_data.orientation.w*imu_data.orientation.x);
//    car2N_matrix[2][2] = imu_data.orientation.w*imu_data.orientation.w - imu_data.orientation.x*imu_data.orientation.x - imu_data.orientation.y*imu_data.orientation.y + imu_data.orientation.z*imu_data.orientation.z ;

//    car2N_matrix[0][0]= cos( car_angle.pitch) ;
//     car2N_matrix[0][1]= sin(car_angle.roll)*sin(car_angle.pitch);//-cos(roll)sin(yaw)+sin(roll)sin(pitch)cos(yaw)
//    car2N_matrix[0][2]=cos(car_angle.roll)*sin(car_angle.pitch);
//    car2N_matrix[1][0]=0;
//   car2N_matrix[1][1]=cos(car_angle.roll) ;
//    car2N_matrix[1][2]= -sin(car_angle.roll);
//    car2N_matrix[2][0]= -sin(car_angle.pitch);
//     car2N_matrix[2][1]=sin(car_angle.roll)* cos(car_angle.pitch);
//    car2N_matrix[2][2]= cos(car_angle.roll) * cos(car_angle.pitch);

    car2N_matrix[0][0]=1;
     car2N_matrix[0][1]= 0;//-cos(roll)sin(yaw)+sin(roll)sin(pitch)cos(yaw)
    car2N_matrix[0][2]=0;
    car2N_matrix[1][0]=0;
   car2N_matrix[1][1]=1;
    car2N_matrix[1][2]= 0;
    car2N_matrix[2][0]= 0;
     car2N_matrix[2][1]=0;
    car2N_matrix[2][2]=1;

    for(int j=0;j<3;j++)
    {
        for(int i=0;i<3;i++)
        {
            sensor2N_matrix[j][i] = car2N_matrix[j][0] * sensor2car_matrix[0][i] + car2N_matrix[j][1] * sensor2car_matrix[1][i] + car2N_matrix[j][2] * sensor2car_matrix[2][i];
        }
    }
}




