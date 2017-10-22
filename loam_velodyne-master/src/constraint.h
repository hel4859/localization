//
// Created by hl on 17-10-3.
//

#ifndef HELEI_WS_CONSTRAINT_H
#define HELEI_WS_CONSTRAINT_H

#include <vector>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
typedef pcl::PointXYZI PointType;
struct Constraint
{

    int c_first_id;//先没用
    int c_second_id;//
    struct Pose
    {
        double c_relative_pose[3];//只有三个元素，第1、2个是x，y，,第3个是角度值(弧度制)，
        double first_pose[3];
        double second_pose[3];
        //注意这里的x，y的坐标要和loam本身的坐标匹配。
        double c_translation_weight;//随便给的值
        double c_rotation_weight;
    };
    struct Point_index
    {
        int id;
        int first;
        int second;
       // pcl::PointCloud<PointType>::Ptr c_point_cloud;

    };
    Pose pose;
    std::vector<Point_index> first_point;
    std::vector<Point_index> second_point;
    pcl::PointCloud<PointType>::Ptr first_corner;
    pcl::PointCloud<PointType>::Ptr first_surf;
    pcl::PointCloud<PointType>::Ptr second_corner;
    pcl::PointCloud<PointType>::Ptr second_surf;

    enum Tag { no_close, close } tag;

};

#endif //HELEI_WS_CONSTRAINT_H
