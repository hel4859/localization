#ifndef PCL_POINTCLOUD_H
#define PCL_POINTCLOUD_H
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#define MRPT_NO_WARN_BIG_HDR

#include <mrpt/gui/CDisplayWindow3D.h>
#include <mrpt/maps/CSimpleMap.h>
#include <mrpt/maps/COccupancyGridMap2D.h>
#include <mrpt/maps/CMultiMetricMap.h>
#include<mrpt/maps/CColouredPointsMap.h>
#include <mrpt/math/utils.h>

#include <mrpt/maps/COctoMap.h>
#include"mrpt/gui.h"
#include"mrpt/utils/CConfigFile.h"
#include"mrpt/opengl.h"
#include"mrpt/opengl/CTexturedObject.h"
#include"mrpt/utils/bits.h"

// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include<pcl_ros/point_cloud.h>

#include <pcl/common/common_headers.h>
#include <pcl/console/parse.h>
#include <pcl/point_types.h>
#include <pcl/io/io.h>
#include<pcl/filters/filter.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <boost/thread/thread.hpp>
#include <pcl/ModelCoefficients.h>
#include <pcl/features/normal_3d.h>
#include <pcl/common/transforms.h>
#include"dead_reckoning.h"

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Int8.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Int8MultiArray.h"
#include "std_msgs/Int32MultiArray.h"
#include "std_msgs/Float64MultiArray.h"
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include<sensor_msgs/image_encodings.h>
#include<sensor_msgs/NavSatFix.h>
#include <pcl_conversions/pcl_conversions.h>

#include <string>
#include <iostream>
#include <sstream>
#include <math.h>
#include <Eigen/Dense>
#include<opencv2/opencv.hpp>
#include"dead_reckoning.h"
#include"virtual_scan_type.h"
#include"display_data.h"
#include "line_finder.h"

#include<mrpt/nav/planners/PlannerRRT_SE2_TPS.h>

#define min_distBetweenLaserPoints 0.05//点云叠加认为的最小距离
#define pointcloud_add_num 15    //点云叠加次数
/**********************************
 * 点云中点的范围，单位米
 * *******************************/
#define pointcloud_x_min (-10)
#define pointcloud_x_max (40)
#define pointcloud_y_min (-25)
#define pointcloud_y_max (25)
//#define pointcloud_z_min (-2.1)
//#define pointcloud_z_max (-0.5)
/**********************************
 * 栅格图中点坐标最大最小值，单位米
 * *******************************/
#define gridMap_x_min pointcloud_x_min
#define gridMap_x_max pointcloud_x_max
#define  gridMap_y_min pointcloud_y_min
#define gridMap_y_max pointcloud_y_max

#define gridMap_resolution  0.2 //栅格图每个栅格大小，单位m
//#define gridMap_halfSize    40 //栅格图大小，单位m
//#define  gridMap_cell_size  (int) (2*gridMap_halfSize / gridMap_resolution)
/********************************
 *栅格图栅格的个数，相当于图片的分辨率
 * ******************************/
#define gridMap_cell_x_size (int) ((gridMap_x_max - gridMap_x_min)/gridMap_resolution)
#define gridMap_cell_y_size (int) ((gridMap_y_max - gridMap_y_min)/gridMap_resolution)

#define gridMapEnhanceStep 0.01
#define gridMapPzFactor 0.1
#define gridMap_cellPointsThreshold 15
/**************************
 *点云数据所使用的环数
 * ************************/
#define RING_MAX    7
#define X_ZERO  0.05
#define Y_ZERO  0.05
#define  Curve_Radius_Change_Threshold  0.05


/** Euclidean Velodyne coordinate, including intensity and ring number. */
struct PointXYZIR
{
    PCL_ADD_POINT4D;                    // quad-word XYZ
    float    intensity;                 ///< laser intensity reading
    uint16_t ring;                      ///< laser ring number
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW     // ensure proper alignment
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZIR,
                                  (float, x, x)
                                  (float, y, y)
                                  (float, z, z)
                                  (float, intensity, intensity)
                                  (uint16_t, ring, ring))
typedef PointXYZIR VPoint;
typedef pcl::PointCloud<VPoint> VPointCloud;
/********************************
typedef struct point_index
{
    int start;
    int end;
} point_index_type;
*******************************/
/*****************************
 * 点云叠加结构体
 * ***************************/
typedef struct pointcloud_fifo
{
    mrpt::maps::CColouredPointsMap pointcloud;
    mrpt::poses::CPose2D robot_pose_local ;
} pointcloud_fifo_type;

/*类型定义*/
/********************
 *GPS坐标点类型
 * ******************/
//坐标点
typedef struct _COORD_T
{
    double longitude;	//经度
    double latitude;	//维度
}COORD_t;
/****************
 * 目标节点属性定义
 * **************/
//属性1
#define ATTRIBUTE1_START			0	//起点
#define ATTRIBUTE1_ENTRANCE			1	//交叉口入点
#define ATTRIBUTE1_EXIT				2	//交叉口出口
#define ATTRIBUTE1_COMMON			3	//普通路点
#define ATTRIBUTE1_PARK_ENTRANCE	4	//进入停车区
#define ATTRIBUTE1_PARK_EXIT		5	//驶出停车区
#define ATTRIBUTE1_PARK				6	//停车位位置
#define ATTRIBUTE1_END				7	//终点
#define ATTRIBUTE1_INTERPOLATION	8	//插值点

//属性2
#define ATTRIBUTE2_UNKNOWN		0	//未知
#define ATTRIBUTE2_GO_STRAIGHT	1	//直行
#define ATTRIBUTE2_TURN_RIGHT	2	//右转
#define ATTRIBUTE2_TURN_LEFT    	3	//左转
#define ATTRIBUTE2_U_TURN		4	//掉头
#define ATTRIBUTE2_TRAFFIC_SIGN	5	//交通标志
/****************
 * 目标节点类型
 * **************/
//结点
typedef struct _NODE
{
    int id;				//路点序号
    double longitude;	//经度
    double latitude;	//维度
    double elevation;	//海拔
    int attribute1;		//属性1点类型: 0：起点，1：交叉口入点，2；交叉口出点，3：普通路点，4：进入停车区，5：驶出停车区，6：停车位位置，7：终点停车
    int attribute2;		//属性2行驶方向或交通标志： 0：未知，1：直行，2：右转，3：左转，4：掉头，5：交通标志
    double turnAngle;
    double coursingAngle;
    int    onetwoway;
    int    waySum;
    int    targetWay;
    struct _NODE *next_node;	//指向后面连接的路点
    struct _WAY *p_way;
} NODE_t;
/******************************
 * 路径类型
 * ****************************/
//路径
typedef struct _WAY
{
    int id;	//等于
    NODE_t *head_node;
} WAY_t;
//每个毫米波雷达检测到的动态目标，用以下结构体表示，这些是我们能用得到的有用信息
typedef
struct _ESR_RX_OBJECT
{
    unsigned char track_status;		//跟踪状态
    unsigned char occupied_state;	//目标状态,无、静止、运动
    float angle;	//角度，正前方为0度，逆时针为正；单位：deg；
    float rrate;	//径向速度，远离传感器为正；单位：m/s；
    float x;		//笛卡尔坐标系x,单位m
    float y;		//笛卡尔坐标系y,单位m
}
ESR_RX_OBJECT_t;
/******************************
 * 点云处理类
 * ****************************/
class pcl_pointcloud
{
    ros::NodeHandle nh_;
    ros::Subscriber pcl_point_cloud_sub;//用来订阅消息
    ros::Subscriber gps_sub;//用来订阅消息
    ros::Subscriber targetGps_sub;//用来订阅消息
    ros::Subscriber sub_esr;
    image_transport::Subscriber image_sub;
    std::vector<float>pcl_point_radius_different[RING_MAX];
    float   radius_change_threshold[RING_MAX];
    pcl::PointCloud<pcl::PointXYZRGB>  pcl_singleline_colored_pointcloud[RING_MAX];//pcl单线的点云
    mrpt::maps::CColouredPointsMap mrpt_singleline_colored_pointcloud[RING_MAX];//mrpt单线的点云，最后从pcl的单线转换过来
    pcl::PointCloud<pcl::PointXYZRGB> pcl_cloud_ring;//pcl单帧带有环数属性的点云
    mrpt::maps::CColouredPointsMap mrpt_pointcloud_map_raw_color;//mrpt单帧点云数据

    std::vector<float>pcl_point_radius[RING_MAX];//每根线中的点到激光雷达的平面距离，用于剔除路面
    dead_reckoning  DR;//航位推算
    std::vector<pointcloud_fifo_type> pointcloud_map_fifo;//所有点云的fifo
    std::vector<pointcloud_fifo_type> curve_pointcloud_map_fifo;//路沿的点云fifo

    display_data_type display;//显示类，里面包含有虚拟扫描的结果
    NODE_t targetGps_node;//目标节点的信息，包含GPS数据
    sensor_msgs::NavSatFix robot_gps_location;
public:

    //显示部分
//    mrpt::gui::CDisplayWindow3D win3D;
//    mrpt::opengl::CPointCloudColouredPtr obj_pointcloud;
//    mrpt::opengl::COpenGLScenePtr	scene;

    mrpt::maps::COccupancyGridMap2DPtr curve_grid_map;//障碍物栅格图
    pcl_pointcloud();

    pcl::PointCloud<pcl::PointXYZRGB> pcl_curve_pointcloud;//pcl边沿检测后的点云
    mrpt::maps::CColouredPointsMap  mrpt_curve_pointcloud;//mrpt边沿检测后的点云
    mrpt::maps::CColouredPointsMap pointcloud_map_reset_ref_color;//车辆参考系下的点云叠加结果
    mrpt::maps::CColouredPointsMap pointcloud_map_final_color;//最终得到的车辆参考系下一定范围内的点云

    mrpt::maps::CColouredPointsMap curve_pointcloud_map_reset_ref_color;//车辆参考系下的点云叠加结果
    mrpt::maps::CColouredPointsMap curve_pointcloud_map_final_color;//最终得到的车辆参考系下一定范围内的点云
    /*****************************
     *MRPT中自带的RRT算法
     * ***************************/
    mrpt::math::TPose2D local_gridmap_target;
    mrpt::nav::PlannerRRT_SE2_TPS  planner;         //TP-RRT运动规划
    mrpt::nav::PlannerRRT_SE2_TPS::TPlannerInput planner_input;
    mrpt::nav::PlannerRRT_SE2_TPS::TPlannerResult planner_result;
    mrpt::nav::PlannerRRT_SE2_TPS::TRenderPlannedPathOptions render_options;


    cv::Mat curve_grid_map_filtered;//得到的栅格图
    cv::Mat curve_grid_map_filtered_temp;
    cv::Mat dilate_element;
    cv::Mat erode_element;
    cv::Mat task_img;//全局地图
    //毫米波雷达的数据的基本解析已经完成，我将处理之后的结果用Float64MultiArray发布出来，rostopic的名字叫esr_info
    void esr_callback(const std_msgs::Float64MultiArray::ConstPtr& esr_in);
    void gps_msg_callback(const sensor_msgs::NavSatFix::ConstPtr& gps_in);
    void current_targetGps_callback(std_msgs::Float64MultiArray target_node);
    void pcl_pointCloudCallback(const sensor_msgs::PointCloud2 &inMsg);
    void copy2coloredpcl_pointsMap(const VPointCloud::ConstPtr &inMsg, pcl::PointCloud<pcl::PointXYZRGB> &cloud_ring);
    void copy2pcl_pointsMap(const VPointCloud::ConstPtr &inMsg, pcl::PointCloud<pcl::PointXYZI> &cloud_ring);
    void image_callback(const sensor_msgs::ImageConstPtr& msg);
    void pcl_radius_differential();
    void curve_detection_process();
    void global_pointcloud_sum();
    void curve_pointcloud_sum();
    void route_planner();
    void my_route_planner(std::vector<admissable_space_type> admisbale_space_filtered,mrpt::poses::CPose3D senser_pose);
    int curve_pointcloud2gridmap();
};

#endif // PCL_POINTCLOUD_H
