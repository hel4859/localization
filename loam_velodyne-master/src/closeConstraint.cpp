//
// Created by hl on 17-10-4.
//
#include <math.h>

#include <loam_velodyne/common.h>
#include <nav_msgs/Odometry.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>
//#include "spa_cost.h"
#include <vector>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/console/time.h>
#include <iostream>

#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
//typedef pcl::PointXYZINormal PointT;
typedef pcl::PointXYZI PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
pcl::PointCloud<PointType>::Ptr laserCloudCornerClose(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr laserCloudCornerCloseStack(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr laserCloudCornerCloseShow(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr laserCloudCornerCloseShow1(new pcl::PointCloud<PointType>());
//std::vector<pcl::PointCloud<pcl::PointXYZINormal> > AllPoint;
std::vector<pcl::PointCloud<pcl::PointXYZI> > AllPoint;
std::vector<nav_msgs::Odometry> AllOdometry;
int m=0;
int p=0;
//double timeMarker= laserOdometry->header.stamp.toSec();
bool closeCloudFlag=false;
bool laserOdometryFlag=false;
double timeLaserOdometry=0;
void closeCloudHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudCornerClose_)
{
    //timeLaserCloudSurfLast = laserCloudCornerClose_->header.stamp.toSec();

    laserCloudCornerClose->clear();
    pcl::fromROSMsg(*laserCloudCornerClose_, *laserCloudCornerClose);
    *laserCloudCornerCloseStack+=*laserCloudCornerClose;

        /**
          * 计算每一帧点云的法向量,并且把点云储存在
          */

       // pcl::NormalEstimation<pcl::PointXYZI, pcl::Normal> n;
      //  pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
        //建立kdtree来进行近邻点集搜索
       // pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>);
        //为kdtree添加点运数据
       // tree->setInputCloud(laserCloudCornerCloseStack);
      //  n.setInputCloud(laserCloudCornerCloseStack);
      //  n.setSearchMethod(tree);
        //点云法向计算时，需要所搜的近邻点大小
      //  n.setKSearch(3);
        //开始进行法向计算
     //   n.compute(*normals);
       // pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointXYZINormal>);
      //  pcl::concatenateFields(*laserCloudCornerCloseStack, *normals, *cloud_with_normals);



        AllPoint.push_back(*laserCloudCornerCloseStack);
      //  cloud_with_normals->clear();
       // normals->clear();
    laserCloudCornerCloseStack->clear();
    closeCloudFlag=true;
        //  for (int i=0;i!=AllPoint.size();i++)
        //  {
        //      std::cout<<"all_size"<<i<<":"<<AllPoint[i].size()<<std::endl;

        //   }




    laserCloudCornerClose->clear();

}
void closeOdometryHandler(const nav_msgs::Odometry::ConstPtr& laserOdometry)
{
    timeLaserOdometry = laserOdometry->header.stamp.toSec();
    if(closeCloudFlag)
    {
        AllOdometry.push_back(*laserOdometry);
        laserOdometryFlag=true;
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "closeConstraint");
    ros::NodeHandle nh;

    ros::Subscriber closeConstraint = nh.subscribe<sensor_msgs::PointCloud2>
            ("/point_to_close",500, closeCloudHandler);//点云输入
    ros::Subscriber closeOdometry = nh.subscribe<nav_msgs::Odometry>
            ("/aft_mapped_to_init", 500, closeOdometryHandler);//点云输入
    //ros::Subscriber subImu = nh.subscribe<sensor_msgs::Imu> ("/imu/data", 50, imuHandler);

    ros::Publisher pubMaker = nh.advertise<visualization_msgs::Marker>("visualization_marker", 1);
    ros::Publisher pubclose = nh.advertise<sensor_msgs::PointCloud2>("closecloud", 1);
    ros::Publisher pubclose1 = nh.advertise<sensor_msgs::PointCloud2>("closecloud1", 1);
   // ros::Publisher pubMaker = nh.advertise<sensor_msgs::PointCloud2>
     //       ("/point_to_close", 2000);
    while (ros::ok())
    {
        ros::spinOnce();
        if(closeCloudFlag &&laserOdometryFlag)
        {
            if (AllPoint.size()>100)
            {
                for (int i=m;i<AllPoint.size()-100;i++)
                {
                    if(sqrt((AllOdometry[i].pose.pose.position.z-AllOdometry[AllPoint.size()-31].pose.pose.position.z)*
                                    (AllOdometry[i].pose.pose.position.z-AllOdometry[AllPoint.size()-31].pose.pose.position.z)+
                                    (AllOdometry[i].pose.pose.position.x-AllOdometry[AllPoint.size()-31].pose.pose.position.x)*
                                    (AllOdometry[i].pose.pose.position.x-AllOdometry[AllPoint.size()-31].pose.pose.position.x)) <60.0&&
                            sqrt((AllOdometry[i].pose.pose.position.z-AllOdometry[AllPoint.size()-31].pose.pose.position.z)*
                                 (AllOdometry[i].pose.pose.position.z-AllOdometry[AllPoint.size()-31].pose.pose.position.z)+
                                 (AllOdometry[i].pose.pose.position.x-AllOdometry[AllPoint.size()-31].pose.pose.position.x)*
                                 (AllOdometry[i].pose.pose.position.x-AllOdometry[AllPoint.size()-31].pose.pose.position.x)) >2.0)
                    {
                        //std::cout<<AllOdometry[i].pose.pose.position.x<<","<<AllOdometry[i].pose.pose.position.y<<","<<AllOdometry[i].pose.pose.position.z<<std::endl;
                        //pcl::console::TicToc time;
                        int iterations = 1;
                        //Eigen::Matrix4d transformation_matrix = Eigen::Matrix4d::Identity ();
                        //time.tic ();
                        // pcl::IterativeClosestPointWithNormals<PointT, PointT> icp;
                        pcl::console::TicToc time;
                        time.tic ();
                       pcl::IterativeClosestPoint<PointT, PointT> icp;
                        //pcl::PointCloud<PointType>::Ptr temp_a;

                        //pcl::PointCloud<PointType>::Ptr temp_b;
                        PointCloudT::Ptr cloud_tr (new PointCloudT);  // Transformed point cloud
                        PointCloudT::Ptr cloud_tr1 (new PointCloudT);
                        PointCloudT::Ptr cloud_icp (new PointCloudT);  // ICP output point cloud
                        PointCloudT::Ptr cloud_allign (new PointCloudT);
                        for (int j=0;j!=30;j++)
                        {
                            *cloud_icp+=AllPoint[i+j];
                            *cloud_tr+=AllPoint[AllPoint.size()-31+j];

                        }
                        Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
                        transform(0,3)=AllOdometry[i].pose.pose.position.x-AllOdometry[AllPoint.size()-31].pose.pose.position.x;
                        transform(1,3)=AllOdometry[i].pose.pose.position.y-AllOdometry[AllPoint.size()-31].pose.pose.position.y;
                        transform(2,3)=AllOdometry[i].pose.pose.position.z-AllOdometry[AllPoint.size()-31].pose.pose.position.z;
                        geometry_msgs::Quaternion geoQuat = AllOdometry[i].pose.pose.orientation;
                        geometry_msgs::Quaternion geoQuat1 = AllOdometry[AllPoint.size()-31].pose.pose.orientation;
                        double roll, pitch, yaw;
                        (tf::Matrix3x3(tf::Quaternion(geoQuat.x, geoQuat.y, geoQuat.z, geoQuat.w))
                                          *tf::Matrix3x3(tf::Quaternion(geoQuat1.x, geoQuat1.y, geoQuat1.z, geoQuat1.w)).inverse()).getRPY(roll, pitch, yaw);

                        float theta =pitch; // The angle of rotation in radians
                        transform (0,0) = cos (theta);
                        transform (0,1) = -sin(theta);
                        transform (1,0) = sin (theta);
                        transform (1,1) = cos (theta);
                        // transform(0,0)=A.xx();
                        //std::cout<<A<<std::endl;
                        pcl::transformPointCloud(*cloud_tr, *cloud_tr1,transform);
                        //pcl::IterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI> icp;
                      //  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree1 (new pcl::search::KdTree<pcl::PointXYZ>);
                       //' tree1->setInputCloud(cloud_source);
                        //pcl::search::KdTree<pcl::PointXYZ>::Ptr tree2 (new pcl::search::KdTree<pcl::PointXYZ>);
                    // pcl::KdTreeFLANN<PointType>::Ptr tree1(new pcl::KdTreeFLANN<PointType>());
                      //  tree1->setInputCloud(cloud_icp);
                        //icp.setSearchMethodSource(tree1);
                       // pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
                        pcl::search::KdTree<PointType>::Ptr tree1 (new pcl::search::KdTree<PointType>);
                        tree1->setInputCloud(cloud_tr1);
                        pcl::search::KdTree<PointType>::Ptr tree2 (new pcl::search::KdTree<PointType>);
                        tree2->setInputCloud(cloud_icp);
                        icp.setSearchMethodSource(tree1);
                        icp.setSearchMethodTarget(tree2);
                        icp.setInputSource(cloud_tr1);
                        icp.setInputTarget(cloud_icp);
                        icp.setMaxCorrespondenceDistance(100);
                        icp.setTransformationEpsilon(1e-10);
                        icp.setEuclideanFitnessEpsilon(0.01);
                        icp.setMaximumIterations(150);
                        icp.align(*cloud_tr1);
                     //   Eigen::Matrix4f transformation = icp.getFinalTransformation();
                      //  std::cout << transformation << std::endl;

                        //pcl::KdTreeFLANN<PointType>::Ptr tree1 (new pcl::KdTreeFLANN<PointType>());
                        //pcl::KdTreeFLANN<PointType>::Ptr tree2 (new pcl::KdTreeFLANN<PointType>());
                       // pcl::IterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI> icp;

                       // pcl::search::KdTree<pcl::PointXYZI>::Ptr tree1 (new pcl::search::KdTree<pcl::PointXYZI>);
                        //tree1->setInputCloud(cloud_tr);
                        //pcl::search::KdTree<pcl::PointXYZI>::Ptr tree2 (new pcl::search::KdTree<pcl::PointXYZI>);
                       //tree2->setInputCloud(cloud_icp);
                      //  icp.setSearchMethodTarget()
                       /*
                        icp.setSearchMethodSource(tree1);
                        icp.setSearchMethodTarget(tree2);
                        icp.setInputSource(cloud_tr);
                        icp.setInputTarget(cloud_icp);
                        icp.setMaxCorrespondenceDistance(1500);
                        icp.setTransformationEpsilon(1e-10);
                        icp.setEuclideanFitnessEpsilon(0.1);
                        icp.setMaximumIterations(300);
                        icp.align(*cloud_icp);
                        Eigen::Matrix4f transformation = icp.getFinalTransformation();
                        std::cout << transformation << std::endl;*/


                        //  std::cout<<"cloud_icp"<<cloud_icp->size()<<std::endl;
                        // std::cout<<"cloud_tr"<<cloud_tr->size()<<std::endl;


                        //    std::cout<<cloud_tr->points[1].x<<std::endl;
                        //   icp.setMaxCorrespondenceDistance(100);
                        // icp.setTransformationEpsilon(0.01);
                        // icp.setEuclideanFitnessEpsilon(0.1);
                     //   icp.setInputSource (cloud_tr);
                      //  icp.setInputTarget (cloud_icp);
                      //  icp.align (*cloud_tr);
                     //   icp.setMaximumIterations (1);  // We set this variable to 1 for the next time we will call .align () function
                    //    std::cout << "Applied " << iterations << " ICP iteration(s) in " << time.toc () << " ms" << std::endl;

                        if (icp.hasConverged ())//1e-10
                        {
                           laserCloudCornerCloseShow->clear();
                            *laserCloudCornerCloseShow=*cloud_tr;
                            sensor_msgs::PointCloud2 closeShow;

                            pcl::toROSMsg(*laserCloudCornerCloseShow, closeShow);
                            closeShow.header.stamp = ros::Time().fromSec(timeLaserOdometry);
                            closeShow.header.frame_id = "/camera_init";
                            pubclose.publish(closeShow);
                            laserCloudCornerCloseShow->clear();

                            laserCloudCornerCloseShow1->clear();
                            *laserCloudCornerCloseShow1=*cloud_icp;
                            sensor_msgs::PointCloud2 closeShow1;

                            pcl::toROSMsg(*laserCloudCornerCloseShow1, closeShow1);
                            closeShow1.header.stamp = ros::Time().fromSec(timeLaserOdometry);
                            closeShow1.header.frame_id = "/camera_init";
                            pubclose1.publish(closeShow1);
                            laserCloudCornerCloseShow1->clear();

                            visualization_msgs::Marker marker;
                            marker.header.frame_id =  "/camera_init";
                            marker.header.stamp = ros::Time().fromSec(timeLaserOdometry);
                            marker.ns = "basic_shapes";
                            marker.id = p;
                           // p++;

                            // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
                            marker.type =  visualization_msgs::Marker::ARROW;

                            // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
                            marker.action = visualization_msgs::Marker::ADD;

                            // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
                            marker.pose.position.x = AllOdometry[AllPoint.size()-31].pose.pose.position.x;
                            marker.pose.position.y = AllOdometry[AllPoint.size()-31].pose.pose.position.y;
                            marker.pose.position.z = AllOdometry[AllPoint.size()-31].pose.pose.position.z;
                            marker.pose.orientation.x = AllOdometry[AllPoint.size()-31].pose.pose.orientation.x;
                            marker.pose.orientation.y = AllOdometry[AllPoint.size()-31].pose.pose.orientation.y;
                            marker.pose.orientation.z = AllOdometry[AllPoint.size()-31].pose.pose.orientation.z;
                            marker.pose.orientation.w = AllOdometry[AllPoint.size()-31].pose.pose.orientation.w;

                            // Set the scale of the marker -- 1x1x1 here means 1m on a side
                            marker.scale.x = 2;
                            marker.scale.y = 2;
                            marker.scale.z = 2;

                            // Set the color -- be sure to set alpha to something non-zero!
                            marker.color.r = 0.0f;
                            marker.color.g = 1.0f;
                            marker.color.b = 0.0f;
                            marker.color.a = 1.0;

                            marker.lifetime = ros::Duration();
                            pubMaker.publish(marker);


                            visualization_msgs::Marker marker2;
                            marker2.header.frame_id =  "/camera_init";
                            marker2.header.stamp = ros::Time().fromSec(timeLaserOdometry);
                            marker2.ns = "basic_shapes2";
                            marker2.id = p;
                            p++;

                            // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
                            marker2.type =  visualization_msgs::Marker::CUBE;

                            // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
                            marker2.action = visualization_msgs::Marker::ADD;

                            // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
                            marker2.pose.position.x = AllOdometry[i].pose.pose.position.x;
                            marker2.pose.position.y = AllOdometry[i].pose.pose.position.y;
                            marker2.pose.position.z = AllOdometry[i].pose.pose.position.z;
                            marker2.pose.orientation.x = AllOdometry[i].pose.pose.orientation.x;
                            marker2.pose.orientation.y = AllOdometry[i].pose.pose.orientation.y;
                            marker2.pose.orientation.z = AllOdometry[i].pose.pose.orientation.z;
                            marker2.pose.orientation.w = AllOdometry[i].pose.pose.orientation.w;

                            // Set the scale of the marker -- 1x1x1 here means 1m on a side
                            marker2.scale.x = 2;
                            marker2.scale.y = 2;
                            marker2.scale.z = 2;

                            // Set the color -- be sure to set alpha to something non-zero!
                            marker2.color.r = 1.0f;
                            marker2.color.g = 0.0f;
                            marker2.color.b = 0.0f;
                            marker2.color.a = 1.0;

                            marker2.lifetime = ros::Duration();
                            pubMaker.publish(marker2);



                            // m=AllPoint.size()-11;
                            std::cout << "Applied " << iterations << " ICP iteration(s) in " << time.toc () << " ms" << std::endl;
                            std::cout << "\nICP has converged, score is " << icp.getFitnessScore () << std::endl;
                            std::cout << "\nICP transformation " << iterations << " : "<<i+1<<" -> "<<AllPoint.size()-30<< std::endl;
                            Eigen::Matrix4f transformation = icp.getFinalTransformation();
                            std::cout << transform*transformation << std::endl;
                            //   transformation_matrix = icp.getFinalTransformation ().cast<double>();
                            // print4x4Matrix (transformation_matrix);
                          //  m=i+20;
                            cloud_icp->clear();
                            cloud_tr->clear();
                            break;
                        }
                        cloud_tr->clear();


                    }


                }
            }

        }


    }

    //ros::spin();

    return 0;
}

