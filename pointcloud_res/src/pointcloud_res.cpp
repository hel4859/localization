#include <nav_msgs/Odometry.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <iostream>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
double time_front= 0;
double time_back=0;
bool new_front= false;
bool new_back= false;
ros::Publisher pubLaser;
typedef pcl::PointXYZ PointType;
pcl::PointCloud<PointType>::Ptr front_pointCloud(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr back_pointCloud(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr all_pointCloud(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr voxel_point(new pcl::PointCloud<PointType>());


inline Eigen::Matrix4f GetMatrixLH(double x,double y,double z,double w)
{
    Eigen::Matrix4f ret;
    double xx = x*x;
    double yy = y*y;
    double zz = z*z;
    double xy = x*y;
    double wz = w*z;
    double wy = w*y;
    double xz = x*z;
    double yz = y*z;
    double wx = w*x;

    ret (0,0) = 1.0f-2*(yy+zz);
    ret(0,1)= 2*(xy-wz);
    ret(0,2) = 2*(wy+xz);
    ret(0,3) = 0.0f;

    ret(1,0)= 2*(xy+wz);
    ret(1,1) = 1.0f-2*(xx+zz);
    ret(1,2) = 2*(yz-wx);
    ret(1,3)= 0.0f;

    ret(2,0) = 2*(xz-wy);
    ret(2,1) = 2*(yz+wx);
    ret(2,2) = 1.0f-2*(xx+yy);
    ret(2,3) = 0.0f;

    ret(3,0) = 0.0f;
    ret(3,1) = 0.0f;
    ret(3,2) = 0.0f;
    ret(3,3)= 1.0f;

    return ret;
};
void front_callback(const sensor_msgs::PointCloud2ConstPtr& front_point)//去除前面的点云的离群点
{
    time_front = front_point->header.stamp.toSec();
    front_pointCloud->clear();
    pcl::fromROSMsg(*front_point, *front_pointCloud);
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*front_pointCloud,*front_pointCloud, indices);
    new_front = true;
    try
    {
        static tf::TransformListener tf_listener1;
        tf::StampedTransform transform1;
        tf_listener1.waitForTransform("/base_link", "/imu_link", ros::Time(0), ros::Duration(3.0));
        tf_listener1.lookupTransform("/base_link", "/imu_link", ros::Time(0), transform1);
        //姿态变换矩阵4x4,用于将每一帧点云按照机器人的位姿进行坐标转换
        Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
        transform=GetMatrixLH(transform1.getRotation().x(),transform1.getRotation().y(),transform1.getRotation().z(),transform1.getRotation().w());
        transform(0,3)=transform1.getOrigin().x();
        transform(1,3)=transform1.getOrigin().y();
        transform(2,3)=transform1.getOrigin().z();
        pcl::transformPointCloud(*front_pointCloud, *front_pointCloud,transform);
    }
    catch (tf::TransformException ex)
    {
        ROS_INFO("%s", ex.what());
        ros::Duration(0.01).sleep();
    }

}
void rear_callback(const sensor_msgs::PointCloud2ConstPtr& back_point)//去除后面的点云的离群点
{
    time_back = back_point->header.stamp.toSec();
    back_pointCloud->clear();
    pcl::fromROSMsg(*back_point, *back_pointCloud);
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*back_pointCloud,*back_pointCloud, indices);
    new_back = true;
    try
    {
        tf::StampedTransform transform1;
        static tf::TransformListener tf_listener1;
        tf_listener1.waitForTransform("/base_link", "/velodyne_rear", ros::Time(0), ros::Duration(3.0));
        tf_listener1.lookupTransform("/base_link", "/velodyne_rear", ros::Time(0), transform1);
        //姿态变换矩阵4x4,用于将每一帧点云按照机器人的位姿进行坐标转换
        Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
        transform=GetMatrixLH(transform1.getRotation().x(),transform1.getRotation().y(),transform1.getRotation().z(),transform1.getRotation().w());
        transform(0,3)=transform1.getOrigin().x();
        transform(1,3)=transform1.getOrigin().y();
        transform(2,3)=transform1.getOrigin().z();
        pcl::transformPointCloud(*back_pointCloud, *back_pointCloud,transform);
    }
    catch (tf::TransformException ex)
    {
        ROS_INFO("%s", ex.what());
        ros::Duration(0.01).sleep();
    }
}
int main(int argc, char** argv)
{
    ros::init(argc, argv, "pointcloud_res");
    ros::NodeHandle nh;
    all_pointCloud->header.frame_id = "map";
     pubLaser = nh.advertise<sensor_msgs::PointCloud2>
            ("velodyne_points", 2);
    ros::Subscriber sub_front = nh.subscribe("/front/velodyne_points",2,front_callback);
    ros::Subscriber sub_back=nh.subscribe("/rear/velodyne_points",2,rear_callback);
    //使用多线程
    //ros::MultiThreadedSpinner spinner(0); // Use all threads
    //spinner.spin(); // spin() will not return until the node has been shutdow
    //ros::Rate rate(10);
    bool status = ros::ok();
    while(status)
    {
        ros::spinOnce();

        if(new_back && new_front)
        {
            //std::cout<<"11"<<std::endl;
            all_pointCloud->clear();
            *all_pointCloud=*back_pointCloud+*front_pointCloud;
            pcl::VoxelGrid<PointType> downPoint;
            downPoint.setLeafSize(0.2, 0.2, 0.2);
            downPoint.setInputCloud(all_pointCloud);
            pcl::PointCloud<PointType>::Ptr all_pointCloud1(new pcl::PointCloud<PointType>());
            downPoint.filter(*all_pointCloud1);
            sensor_msgs::PointCloud2 output;
            pcl::toROSMsg(*all_pointCloud1, output);
            pubLaser.publish(output);
            new_front= false;
            new_back= false;
        }


    }
    return 0;
}
