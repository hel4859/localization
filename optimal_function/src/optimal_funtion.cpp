//
// Created by hl on 18-1-22.
//
#include "optimal_function.h"


Optimal_Function::Optimal_Function() {

    std::cout<<"Optimal_Function::Optimal_Function()"<<std::endl;

    imu_mutex_= PTHREAD_MUTEX_INITIALIZER;
    laser_mutex_=PTHREAD_MUTEX_INITIALIZER;
    fix_mutex_=PTHREAD_MUTEX_INITIALIZER;
    node_mutex=PTHREAD_MUTEX_INITIALIZER;
    constraint_mutex=PTHREAD_MUTEX_INITIALIZER;

    //订阅惯导里程计的数据
    imu_odom=nh.subscribe("/dead_reconking_3d",10,&Optimal_Function::imu_odom_callback,this);

    //订阅SLAM的结果
    laser_odom=nh.subscribe("/aft_mapped_to_init",10,&Optimal_Function::laser_odom_callback,this);

    //GPS结果,间歇性gps值
    fix_odom=nh.subscribe("/gps_constraint",10,&Optimal_Function::fix_odom_callback,this);

    typedef void* (*FUNC)(void*);//定义FUNC类型是一个指向函数的指针，该函数参数为void*，返回值为void*
    FUNC AddImuNodeConstraintThread = (FUNC)&Optimal_Function::AddImuNodeConstraintThread;//强制转换func()的类型
    FUNC OptimalThread = (FUNC)&Optimal_Function::OptimalThread;//强制转换func()的类型

    pthread_create(&imu_thread_id_,NULL, AddImuNodeConstraintThread ,this);
    pthread_create(&optimal_thread_id_,NULL, OptimalThread ,this);


}
Optimal_Function::~Optimal_Function() {

    std::cout<<"Optimal_Function::~Optimal_Function()"<<std::endl;

}


void Optimal_Function::AddLocalNode(int trajectory_id, ros::Time time, const Eigen::Vector3d &pose_translation,
                                    const Eigen::Quaterniond &pose_rotation) {

    Node node;
    node.trajectory_id_=trajectory_id;
    node.time_=time;
    node.pose_rotation_=pose_rotation;
    node.pose_translation_=pose_translation;
    local_node_.push_back(node);

}


void Optimal_Function::AddGlobalNode(std::vector<Node> &imu_odom_1,
                                     std::vector<Node> &laser_odom_1,
                                     std::vector<Node> &fix_odom_1) {
    pthread_mutex_lock(&laser_mutex_);
    pthread_mutex_lock(&imu_mutex_);
    pthread_mutex_lock(&fix_mutex_);

    if (first_pose_flag) {

        if( laser_odom_1[0].time_<= std::max(imu_odom_1[0].time_,
                                                     laser_odom_1[0].time_))
        {
            for (int i=0;i!=laser_odom_1.size();i++) {
                    if(laser_odom_1[i].time_>=std::max(imu_odom_1[0].time_,
                                                              laser_odom_1[0].time_)) {
                        Optimal_Function::global_node_.push_back(Node(laser_odom_1[i].trajectory_id_, laser_odom_1[i].time_,
                                                                      laser_odom_1[i].pose_translation_,
                                                                      laser_odom_1[i].pose_rotation_));
                        Optimal_Function::laser_odom_id=i;
                        first_pose_flag=false;
                        pthread_mutex_unlock(&laser_mutex_);
                        pthread_mutex_unlock(&imu_mutex_);
                        pthread_mutex_unlock(&fix_mutex_);
                        return;
                    }
                }
        }
        else
        {
            Optimal_Function::global_node_.push_back(Node(laser_odom_1[0].trajectory_id_, laser_odom_1[0].time_,
                                                              laser_odom_1[0].pose_translation_,
                                                              laser_odom_1[0].pose_rotation_));
                Optimal_Function::laser_odom_id=0;
                first_pose_flag=false;
        }
    }
    else {

        if (laser_odom_1.size()-1>Optimal_Function::laser_odom_id) {
            for (int i=laser_odom_id+1;i!=laser_odom_1.size();i++)
            {
                Optimal_Function::global_node_.push_back(Node(laser_odom_1[i].trajectory_id_, laser_odom_1[i].time_,
                                                              laser_odom_1[i].pose_translation_,
                                                              laser_odom_1[i].pose_rotation_));
            }
            laser_odom_id=laser_odom_1.size()-1;

        }
    }

    pthread_mutex_unlock(&laser_mutex_);
    pthread_mutex_unlock(&imu_mutex_);
    pthread_mutex_unlock(&fix_mutex_);

}

void Optimal_Function::AddSubmap(int trajectory_id, const Eigen::Vector3d &global_submap_pose_translation,
                                 const Eigen::Quaterniond &global_submap_pose_rotation) {
    Submap submap;
    submap.global_submap_pose_rotation_=global_submap_pose_rotation;
    submap.global_submap_pose_translation_=global_submap_pose_translation;
    (Optimal_Function::submap_).push_back(submap);

}

void Optimal_Function::Local_Constraint(std::vector<Node> &imu_odom_1,std::vector<Node> &fix_odom_1,std::vector<Node>& global_node) {
    if(global_node.size()>=((Optimal_Function::trajectory_id_+1)*10)+1 &&
            global_node.size()<((Optimal_Function::trajectory_id_+2)*10)+1)


    {
        pthread_mutex_lock(&imu_mutex_);
        //  pthread_mutex_lock(&fix_mutex_);
        pthread_mutex_lock(&node_mutex);
        for (int j=Optimal_Function::trajectory_id_*10;j!=(Optimal_Function::trajectory_id_+1)*10+1;j++)
        {
            global_node[j].trajectory_id_=Optimal_Function::trajectory_id_;
            Optimal_Function::local_node_.push_back(global_node[j]);
        }

        pthread_mutex_unlock(&node_mutex);
        //for imu
        for(int m=0+Optimal_Function::trajectory_id_*10;m!=11+Optimal_Function::trajectory_id_*10;m++)
        {
            for(int p=Optimal_Function::constraint_first_id;p!=imu_odom_1.size();p++)
            {
                imu_odom_1[p].trajectory_id_=Optimal_Function::trajectory_id_;
                if(imu_odom_1[p].time_>=Optimal_Function::global_node_[m].time_)
                {
                    Optimal_Function::constraint_node_.push_back(
                            Optimal_Function::Interpolation(p,imu_odom_1,Optimal_Function::global_node_[m].time_)) ;



                    Optimal_Function::constraint_first_id=p;
                    break;
                }
            }

        }

        pthread_mutex_unlock(&imu_mutex_);
        //for gps
//        for(int m=0+Optimal_Function::trajectory_id_*10;m!=11+Optimal_Function::trajectory_id_*10;m++)
//        {
//            for(int p=Optimal_Function::fix_constraint_first_id;p!=fix_odom_1.size();p++)
//            {
//                fix_odom_1[p].trajectory_id_=Optimal_Function::trajectory_id_;
//                if(fix_odom_1[p].time_>=local_node_[m].time_&&(fix_odom_1[p].time_.toSec()-fix_odom_1[p-1].time_.toSec())<0.1)
//                {
//                    Optimal_Function::fix_constraint_node_.push_back(
//                            Constraint_Node(m,Optimal_Function::Interpolation(p,fix_odom_1,Optimal_Function::local_node_[m].time_))) ;
//
//                    Optimal_Function::fix_constraint_first_id=p;
//                    break;
//                }
//                else {
//                    std::cout<<"dfix_odom_time_"<<(fix_odom_1[p].time_.toSec()-fix_odom_1[p-1].time_.toSec())
//                             <<std::endl;
//                }
//            }
//
//        }
        //    pthread_mutex_unlock(&fix_mutex_);
        pthread_mutex_lock(&constraint_mutex);
        if(Optimal_Function::constraint_node_.size()>=2)
        {
            for(int j=0;j!=11;j++)
            {

                Optimal_Function::constraint_.push_back(
                        Constraint(j,j+1,Optimal_Function::constraint_node_[j].trajectory_id_,
                                   Optimal_Function::constraint_node_[j].pose_rotation_.inverse()*
                                   (Optimal_Function::constraint_node_[j+1].pose_translation_
                                    -Optimal_Function::constraint_node_[j].pose_translation_),
                                   (Optimal_Function::constraint_node_[j].pose_rotation_.inverse()*
                                    Optimal_Function::constraint_node_[j+1].pose_rotation_)));
            }

        }

        pthread_mutex_unlock(&constraint_mutex);
        // pthread_mutex_lock(&fix_constraint_mutex);

//        for(int j=0;j!=Optimal_Function::fix_constraint_node_.size()-1;j++)
//        {
//
//            Optimal_Function::fix_constraint_.push_back(
//                    Constraint(fix_constraint_node_[j].node_id_,
//                               fix_constraint_node_[j+1].node_id_,
//                               Optimal_Function::fix_constraint_node_[j].node_.trajectory_id_,
//                               Optimal_Function::fix_constraint_node_[j].node_.pose_rotation_.inverse()*
//                               (Optimal_Function::fix_constraint_node_[j+1].node_.pose_translation_
//                                -Optimal_Function::fix_constraint_node_[j].node_.pose_translation_),
//                               (Optimal_Function::fix_constraint_node_[j].node_.pose_rotation_.inverse()*
//                                Optimal_Function::fix_constraint_node_[j+1].node_.pose_rotation_)));
//        }


        // pthread_mutex_unlock(&fix_constraint_mutex);
        Optimal_Function::trajectory_id_++;
        constraint_node_.clear();
    }


    }




void Fix_Constraint(std::vector<Node> &fix_odom_1,std::vector<Node>& global_node){


}
void Optimal_Function::Global_Constraint(std::vector<Node> imu_odom_1,std::vector<Node> laser_odom_1) {
    int laser_id = 0;
    int imu_id = 0;

}

void Optimal_Function::imu_odom_callback(const nav_msgs::Odometry::ConstPtr &ImuOdomIn) {

    pthread_mutex_lock(&imu_mutex_);

    Node imu_odom;
    imu_odom.trajectory_id_=0;
    imu_odom.time_=ImuOdomIn->header.stamp;
    imu_odom.pose_translation_ = Eigen::Vector3d(ImuOdomIn->pose.pose.position.x,
                                               ImuOdomIn->pose.pose.position.y,
                                               ImuOdomIn->pose.pose.position.z);
    imu_odom.pose_rotation_ = Eigen::Quaterniond(ImuOdomIn->pose.pose.orientation.w,
                                               ImuOdomIn->pose.pose.orientation.x,
                                               ImuOdomIn->pose.pose.orientation.y,
                                               ImuOdomIn->pose.pose.orientation.z);
    imu_odom_.push_back(imu_odom);

    pthread_mutex_unlock(&imu_mutex_);
}

void Optimal_Function::laser_odom_callback(const nav_msgs::Odometry::ConstPtr &LaserOdomIn) {

    pthread_mutex_lock(&laser_mutex_);
    Node laser_odom;
    laser_odom.trajectory_id_=0;
    laser_odom.time_=LaserOdomIn->header.stamp;
    laser_odom.pose_translation_ = Eigen::Vector3d(LaserOdomIn->pose.pose.position.x,
                                  LaserOdomIn->pose.pose.position.y,
                                  LaserOdomIn->pose.pose.position.z);
    laser_odom.pose_rotation_ = Eigen::Quaterniond(LaserOdomIn->pose.pose.orientation.w,
                                                 LaserOdomIn->pose.pose.orientation.x,
                                                 LaserOdomIn->pose.pose.orientation.y,
                                                 LaserOdomIn->pose.pose.orientation.z);
    laser_odom_.push_back(laser_odom);
    pthread_mutex_unlock(&laser_mutex_);
}

void Optimal_Function::fix_odom_callback(const nav_msgs::Odometry::ConstPtr &FixOdomIn) {
    pthread_mutex_lock(&fix_mutex_);
    Node fix_odom;
    fix_odom.trajectory_id_=0;
    fix_odom.time_=FixOdomIn->header.stamp;
    fix_odom.pose_translation_ = Eigen::Vector3d(FixOdomIn->pose.pose.position.x,
                                                 FixOdomIn->pose.pose.position.y,
                                                 FixOdomIn->pose.pose.position.z);
    fix_odom.pose_rotation_ = Eigen::Quaterniond(FixOdomIn->pose.pose.orientation.w,
                                                 FixOdomIn->pose.pose.orientation.x,
                                                 FixOdomIn->pose.pose.orientation.y,
                                                 FixOdomIn->pose.pose.orientation.z);
    fix_odom_.push_back(fix_odom);
    pthread_mutex_unlock(&fix_mutex_);

}

//线性插值函数

Node Optimal_Function::Interpolation(int id,std::vector<Node> odom,ros::Time time) {

    const double duration=(odom[id].time_).toSec()-(odom[id-1].time_).toSec();
    const double factor = ((time).toSec()-(odom[id-1].time_).toSec())/duration;
    const Eigen::Vector3d origin =odom[id-1].pose_translation_ +
            factor * (odom[id].pose_translation_ - odom[id-1].pose_translation_) ;
    const Eigen::Quaterniond rotation =
            Eigen::Quaterniond(odom[id-1].pose_rotation_)
                    .slerp(factor, Eigen::Quaterniond(odom[id].pose_rotation_));
    return Node(odom[id].trajectory_id_,time,origin,rotation);


}

void Optimal_Function::OptimalSolve(const std::vector<Node> &nodes,
                                    const std::vector<Constraint> &constraints) {

    ceres::Problem problem;

    bool first_flag=true;
    for (const Constraint& constraint : constraints) {

        Constraint_Pose pose;
        pose.rotation= Eigen::Vector4d(constraint.pose_rotation_.w(),
                                       constraint.pose_rotation_.x(),
                                       constraint.pose_rotation_.y(),
                                       constraint.pose_rotation_.z());
        pose.translation=Eigen::Vector3d(constraint.pose_translation_);
        std::array<double, 4> temp1{{nodes[constraint.first_id_].pose_rotation_.w(),
                                            nodes[constraint.first_id_].pose_rotation_.x(),
                                            nodes[constraint.first_id_].pose_rotation_.y(),
                                            nodes[constraint.first_id_].pose_rotation_.z()}};

        std::array<double, 4> temp2{{nodes[constraint.second_id_].pose_rotation_.w(),
                                            nodes[constraint.second_id_].pose_rotation_.x(),
                                            nodes[constraint.second_id_].pose_rotation_.y(),
                                            nodes[constraint.second_id_].pose_rotation_.z()}};

        std::array<double, 3> temp3{{nodes[constraint.first_id_].pose_translation_[0],
                                            nodes[constraint.first_id_].pose_translation_[1],
                                            nodes[constraint.first_id_].pose_translation_[2]}};

        std::array<double, 3> temp4{{nodes[constraint.second_id_].pose_translation_[0],
                                            nodes[constraint.second_id_].pose_translation_[1],
                                            nodes[constraint.second_id_].pose_translation_[2]}};
        problem.AddParameterBlock(temp1.data(),4);
        problem.AddParameterBlock(temp3.data(),3);
        problem.AddParameterBlock(temp2.data(),4);
        problem.AddParameterBlock(temp4.data(),3);
        if (first_flag)
        {
            problem.SetParameterBlockConstant(temp1.data());
            problem.SetParameterBlockConstant(temp3.data());
            first_flag=false;
        }
        problem.AddResidualBlock(
                SpaCostFunction::CreateAutoDiffCostFunction(pose),
                nullptr /* loss function */,
                temp1.data(),
                temp3.data(),
                temp2.data(),
                temp4.data());

    }

    ceres::Solver::Options options;
    options.linear_solver_type=ceres::DENSE_QR;
    options.minimizer_progress_to_stdout=true;

    ceres::Solver::Summary summary;
    std::chrono::steady_clock::time_point t1=std::chrono::steady_clock::now();
    ceres::Solve(options,&problem,&summary);
    //储存优化结果
    for (auto node :nodes)
    {
        opt_node_.push_back(node);
    }
    std::chrono::steady_clock::time_point t2=
            std::chrono::steady_clock::now();
    std::chrono::duration<double> time_used =
            std::chrono::duration_cast<std::chrono::duration<double>>(t2-t1);
    std::cout<<"solve time cost= "<<time_used.count()<<"seconds."<<std::endl;
    std::cout<<summary.BriefReport() <<std::endl;
    std::cout<<"estimated a,b,c = ";
    for ( auto node :nodes ) std::cout<<node<<" ";
    std::cout<<std::endl;

}


void Optimal_Function::AddImuNodeConstraintThread() {

 std::cout<<"AddImuNodeConstraintThread"<<std::endl;

    while(ros::ok())
    {
        if(Optimal_Function::imu_odom_.size()>0 && Optimal_Function::laser_odom_.size()>0)
        {
           Optimal_Function::AddGlobalNode(Optimal_Function::imu_odom_,
                                           Optimal_Function::laser_odom_,
                                           Optimal_Function::fix_odom_);
           Optimal_Function::Local_Constraint(Optimal_Function::imu_odom_,Optimal_Function::fix_odom_,
                                              Optimal_Function::global_node_);
           usleep(1000);
        }

    }

}



void Optimal_Function::OptimalThread() {

    std::cout<<"OptimalThread"<<std::endl;

    while(ros::ok())
    {
        if (Optimal_Function::constraint_.size()!=0 &&
                Optimal_Function::local_node_.size()!=0)

        {

            if(Optimal_Function::constraint_[Optimal_Function::constraint_.size()-1].trajectory_id_
               >= solve_constraint_id &&
               Optimal_Function::local_node_[Optimal_Function::local_node_.size()-1].trajectory_id_
               >=solve_node_id ) {

                std::vector<Node> nodes_temp;
                std::vector<Constraint> constraints_temp;

                pthread_mutex_lock(&node_mutex);
              //  std::cout<<node_trajectory<<std::endl;
                for (int i=node_trajectory;i!=Optimal_Function::local_node_.size()-1;i++)
                {


                    if (Optimal_Function::local_node_[i].trajectory_id_==solve_node_id)
                    {
                       nodes_temp.push_back(Optimal_Function::local_node_[i]);
                    }
                    else
                    {
                        //std::cout<<"i:"<<i<<std::endl;
                        node_trajectory=i;
                        solve_node_id++;
                        break;
                    }

                }

                pthread_mutex_unlock(&node_mutex);
                pthread_mutex_lock(&constraint_mutex);
                for (int j=constraint_trajectory;j!=Optimal_Function::constraint_.size()-1;j++)
                {
                    if(Optimal_Function::constraint_[j].trajectory_id_==solve_constraint_id)
                    {
                        constraints_temp.push_back(Optimal_Function::constraint_[j]);
                    }
                    else
                    {
                        constraint_trajectory=j;
                        solve_constraint_id++;
                        break;
                    }

                }
                pthread_mutex_unlock(&constraint_mutex);

                if(nodes_temp[nodes_temp.size()-1].trajectory_id_==compare_node_id)
                {
                    std::cout<<"opt_node"<<opt_node_.size()<<std::endl;

                    if(opt_node_.size()!=0)
                    {
                        nodes_temp[0]=opt_node_[opt_node_.size()-1];
                        nodes_temp[0].trajectory_id_++;
                    }
                    compare_node_id++;
                    Optimal_Function::OptimalSolve(nodes_temp,constraints_temp);
                }

                nodes_temp.clear();
                constraints_temp.clear();


            }

        }

        usleep(1000);

    }
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "Optimal_Function");
    Optimal_Function opt;
    ros::MultiThreadedSpinner spinner(0); // Use all threads
    spinner.spin(); // spin() will not return until the node has been shutdown


}