//
// Created by hl on 18-1-22.
//

#ifndef PROJECT_OPTIMAL_FUNCTION_H
#define PROJECT_OPTIMAL_FUNCTION_H

#include <ceres/ceres.h>
#include <ceres/jet.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include "quaterniontorpy.h"
#include <nav_msgs/Odometry.h>
#include <thread>
#include <mutex>
#include <iostream>
#include <chrono>



class Constraint_Pose
{
public:
   // Constraint_Pose(Eigen::Vector4d rotation_,Eigen::Vector3d translation_): rotation_(rotation),translation_(translation){};
    double translation_weight=10.0;
    double rotation_weight=12.0;
    Eigen::Vector4d rotation;
    Eigen::Vector3d translation;

};

class Constraint_Node
{
public:
    Node node_;
    int node_id_;
    Constraint_Node(int node_id,Node node): node_id_(node_id),node_(node){};
    Constraint_Node(){};

};

class Opt_Node
{
public:

    Opt_Node(int node_id,int trajectory_id,
         ros::Time time,
             std::array<double, 3> pose_translation_array,
             std::array<double, 4> pose_rotation_array):node_id_(node_id),
                                           trajectory_id_(trajectory_id),
                                           time_(time),pose_translation_array_(pose_translation_array),
                                           pose_rotation_array_(pose_rotation_array){};
    Opt_Node(){};
    int node_id_;
    int trajectory_id_;
    ros::Time time_;
    std::array<double, 3> pose_translation_array_;
    std::array<double, 4> pose_rotation_array_;

};






std::ostream &operator<<(std::ostream &out, Node &c1)
{
    std::cout<<"node"<<std::endl;
    std::cout<<"trajectory_id: "<<c1.trajectory_id_<<","<<"pose_translation: "<<c1.pose_translation_<<"pose_rotation: "<<c1.pose_rotation_<<std::endl;
    return out;
}






class Constraint
{
public:
    int first_id_;
    int second_id_;
    int trajectory_id_;
    Eigen::Vector3d pose_translation_;
    Eigen::Quaterniond pose_rotation_;
    Constraint(int first_id, int second_id,
               int trajectory_id,
         Eigen::Vector3d pose_translation,
         Eigen::Quaterniond pose_rotation):first_id_(first_id),second_id_(second_id),trajectory_id_(trajectory_id),pose_translation_(pose_translation),pose_rotation_(pose_rotation){};

   friend std::ostream& operator<<(std::ostream &out, Constraint &c1);
};

std::ostream  &operator<<(std::ostream &out, Constraint &c1)
{
    std::cout<<"constraint:"<<std::endl;
    std::cout<<"first_id: "<<c1.first_id_<<"second_id: "<<c1.second_id_<<","<<"trajectory_id: "<<c1.trajectory_id_<<","<<"pose_translation: "<<c1.pose_translation_<<"pose_rotation: "<<c1.pose_rotation_<<std::endl;
    return out;
}




class Submap
{
public:
    int trajectory_id_;
    Eigen::Vector3d global_submap_pose_translation_;
    Eigen::Quaterniond global_submap_pose_rotation_;
};

class SpaCostFunction
{
public:
    static ceres::CostFunction* CreateAutoDiffCostFunction(const Constraint_Pose &pose)
    {
        return new ceres::AutoDiffCostFunction<SpaCostFunction,6,4,3,4,3>(new SpaCostFunction(pose));
    }

    template <typename T>
            bool operator()(const T* const c_i_rotation, const T*const c_i_translation,
                    const T*const c_j_rotation, const T*const c_j_translation,
            T*const e)const {
        ComputeScaledError(pose_,c_i_rotation,c_i_translation,
                           c_j_rotation,c_j_translation,e);
        return true;

    }

    template <typename T>
    static std::array<T,6> ComputeUnscaledError(const Constraint_Pose &pose,
                                                const T* const c_i_rotation,const T* const c_i_translation,
                                                const T* const c_j_rotation, const T* const c_j_translation)
    {
        const Eigen::Quaternion<T> Ri_inverse(c_i_rotation[3],-c_i_rotation[0], -c_i_rotation[1], -c_i_rotation[2]);
        const Eigen::Matrix<T, 3, 1> Tr(c_j_translation[0] - c_i_translation[0],
                                 c_j_translation[1] - c_i_translation[1],
                                 c_j_translation[2] - c_i_translation[2]);

        const Eigen::Matrix<T, 3, 1> h_translation =Ri_inverse*Tr;
        const Eigen::Quaternion<T> h_rotation=
                Ri_inverse*Eigen::Quaternion<T> (c_j_rotation[3],c_j_rotation[0], c_j_rotation[1], c_j_rotation[2]);
        const Eigen::Quaternion<T> rotation_different= Eigen::Quaternion<T>(T(pose.rotation[0]),T(-pose.rotation[1]),
                                                                            T(-pose.rotation[2]),T(-pose.rotation[3]))*h_rotation;


        const Eigen::Matrix<T,3,1> angle_axis_difference=RotationQuaternionToAngleAxisVector(rotation_different);

//       / const Eigen::Matrix<T,3,1> angle_axis_difference(T(1.0),T(1.0),T(1.0));
        const Eigen::Matrix<T, 3, 1> translation_difference(T(pose.translation[0])-h_translation[0],
                                                            T(pose.translation[1])-h_translation[1],
                                                     T(pose.translation[2])-h_translation[2]);
        return {{translation_difference[0],translation_difference[1],translation_difference[2],
                angle_axis_difference[0], angle_axis_difference[1], angle_axis_difference[2]}};

    }

    template <typename T>
    static void ComputeScaledError(const Constraint_Pose& pose,
                                   const T*const c_i_rotation,
                                   const T*const c_i_translation,
                                   const T*const c_j_rotation,
                                   const T*const c_j_translation,
                                   T*const e)
    {
        const  std::array<T,6> e_ij=
                ComputeUnscaledError(pose,c_i_rotation,c_i_translation,
                                     c_j_rotation,c_j_translation);
        for (int ij :{0,1,2})
        {
            e[ij]=e_ij[ij]*T(pose.translation_weight);
        }
        for (int ij:{3,4,5})
        {
            e[ij]=e_ij[ij]*T(pose.rotation_weight);
        }
    }




private:
    explicit SpaCostFunction(const Constraint_Pose& pose): pose_(pose){}
    const Constraint_Pose pose_;


};
class Optimal_Function
{
    ros::NodeHandle nh;
    ros::Subscriber imu_odom;
    ros::Subscriber laser_odom;
    ros::Subscriber fix_odom;
public:
    Optimal_Function();

    ~Optimal_Function();

    void imu_odom_callback(const nav_msgs::Odometry::ConstPtr &ImuOdomIn);
    void laser_odom_callback(const nav_msgs::Odometry::ConstPtr &LaserOdomIn);
    void fix_odom_callback(const nav_msgs::Odometry::ConstPtr &FixOdomIn);

    void AddDeadReckoning(ros::Time time,
                          const Eigen::Vector3d& pose_translation,
                          const Eigen::Quaterniond& pose_rotation);

    void OptimalSolve(const std::vector<Node> &nodes,const std::vector<Constraint> &constraints);


    void AddLocalNode(int trajectory_id,ros::Time time,
                      const Eigen::Vector3d& pose_translation,
                      const Eigen::Quaterniond& pose_rotation);

    void AddGlobalNode(std::vector<Node> &imu_odom_1,std::vector<Node> &laser_odom_1,std::vector<Node> &fix_odom_1);

    void AddSubmap(int trajectory_id,
                   const Eigen::Vector3d& global_submap_pose_translation,
                   const Eigen::Quaterniond& global_submap_pose_rotation);
    void Local_Constraint(std::vector<Node> &imu_odom_1,std::vector<Node> &fix_odom_1,std::vector<Node>& global_node);
    void Global_Constraint(std::vector<Node> imu_odom_1,std::vector<Node> laser_odom_1);
    void Fix_Constraint(std::vector<Node> &fix_odom_1,std::vector<Node>& global_node);

    void Submap_Constraint(int trajectory_id,
                           const Eigen::Vector3d& global_submap_pose_translation,
                           const Eigen::Quaterniond& global_submap_pose_rotation);
    Node Interpolation (int id,std::vector<Node>odom,ros::Time time);


    //进程
    void AddImuNodeConstraintThread();
    void OptimalThread();
    void fixThread(){};


    //id
    int node_id=0;
    int constraint_first_id=0;
    int fix_constraint_first_id=0;

//private:

    //
    int trajectory_id_=0;
    int laser_odom_id=0;

    int solve_node_id=0;
    int solve_constraint_id=0;

    int compare_node_id=0;
    int compare_constraint_id=0;

    int node_trajectory=0;
    int constraint_trajectory=0;

    std::vector<Node> imu_odom_;
    std::vector<Node> fix_odom_;
    std::vector<Node> laser_odom_;
    std::vector<Constraint> constraint_;
    std::vector<Constraint> fix_constraint_;




    std::vector<Node> local_node_;
    std::vector<Node> global_node_;
    std::vector<Node> constraint_node_;
    std::vector<Constraint_Node> fix_constraint_node_;
    std::vector<Node> optimal_node_;
    std::vector<Submap> submap_;



    //线程定义
    pthread_t imu_thread_id_;
    pthread_t optimal_thread_id_;
    pthread_t fix_thread_id_;

    //线程锁
    pthread_mutex_t imu_mutex_;
    pthread_mutex_t laser_mutex_;
    pthread_mutex_t fix_mutex_;
    pthread_mutex_t node_mutex;
    pthread_mutex_t constraint_mutex;
    pthread_mutex_t fix_constraint_mutex;



    //flag
    bool first_pose_flag=true;


    //优化结果
    std::vector<Node> opt_node_;




};


#endif //PROJECT_OPTIMAL_FUNCTION_H
