#include"pcl_pointcloud.h"

pcl_pointcloud::pcl_pointcloud()
{
    curve_grid_map = mrpt::maps::COccupancyGridMap2D::Create();
    curve_grid_map->setSize(
                gridMap_x_min,
                gridMap_x_max,
                gridMap_y_min,
                gridMap_y_max,
                gridMap_resolution,
                1);
    dilate_element = cv::getStructuringElement(cv::MORPH_DILATE, cv::Size(3, 3));
    erode_element = cv::getStructuringElement(cv::MORPH_ERODE, cv::Size(3, 3));

pcl_point_cloud_sub= nh_.subscribe("/pcl_output",1,&pcl_pointcloud::pcl_pointCloudCallback,this);
   // pcl_point_cloud_sub= nh_.subscribe("cubicframe",1,&pcl_pointcloud::pcl_pointCloudCallback,this);//改成topic！！
    targetGps_sub = nh_.subscribe("last_target",1,&pcl_pointcloud::current_targetGps_callback,this);
    gps_sub = nh_.subscribe("/fix",1,&pcl_pointcloud::gps_msg_callback,this);
    image_transport::ImageTransport it(nh_);
    image_sub = it.subscribe("/hubing/task_map",1,&pcl_pointcloud::image_callback,this);
    radius_change_threshold[0] = 0.035;
    radius_change_threshold[1] = 0.035;
    radius_change_threshold[2] = 0.035;
    radius_change_threshold[3] = 0.035;
    radius_change_threshold[4] = 0.04;
    radius_change_threshold[5] = 0.1;
    if(RING_MAX == 7)
    {
        radius_change_threshold[6] = 0.15;
    }

}
void pcl_pointcloud::gps_msg_callback(const sensor_msgs::NavSatFix::ConstPtr& gps_in)
{
    robot_gps_location.latitude = gps_in->latitude;
    robot_gps_location.longitude = gps_in->longitude;
}
void pcl_pointcloud::image_callback(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    task_img = cv_ptr->image;//任务图像
}
void  pcl_pointcloud::current_targetGps_callback(std_msgs::Float64MultiArray target_node)
{
    float target_distance=0;
    targetGps_node.id = target_node.data[0];
    targetGps_node.longitude= target_node.data[1];
    targetGps_node.latitude= target_node.data[2];
    target_distance = target_node.data[3];
    targetGps_node.attribute1= target_node.data[4];
    targetGps_node.attribute2= target_node.data[5];
    targetGps_node.coursingAngle= target_node.data[6];
    targetGps_node.onetwoway= target_node.data[7];
    targetGps_node.waySum= target_node.data[8];
    targetGps_node.targetWay= target_node.data[9];
}
void pcl_pointcloud::pcl_pointCloudCallback(const sensor_msgs::PointCloud2 &inMsg)
{
	pcl_cloud_ring.points.clear();
	pcl::fromROSMsg(inMsg, pcl_cloud_ring);

  for (int i=0; i<pcl_cloud_ring.size(); i++)
  {
	/*if(out_frame->points[i].x<30)continue;
	if(out_frame->points[i].x>40)continue;
    if(out_frame->points[i].y>5)continue;
    if(out_frame->points[i].y<-20)continue;*/
    #define zGap (4.0)
#define pointcloud_z_min (0.0)
#define pointcloud_z_max (4.0)
	if(pcl_cloud_ring.points[i].z>pointcloud_z_max)
{
		pcl_cloud_ring.points[i].r= 255;
                  pcl_cloud_ring.points[i].g=0;
                  pcl_cloud_ring.points[i].b= 0;
}
    else if(pcl_cloud_ring.points[i].z>3.0)
              {
                  pcl_cloud_ring.points[i].r= 255;
                  pcl_cloud_ring.points[i].g=255*(4.0-pcl_cloud_ring.points[i].z) ;
                  pcl_cloud_ring.points[i].b= 0;
              }
              else if(pcl_cloud_ring.points[i].z>2.0)
              {
                  pcl_cloud_ring.points[i].r= 255*(pcl_cloud_ring.points[i].z-2.0);
                  pcl_cloud_ring.points[i].g= 255;
                  pcl_cloud_ring.points[i].b= 0;
              }
              else if(pcl_cloud_ring.points[i].z>1.0)
              {
                  pcl_cloud_ring.points[i].r= 0;
                  pcl_cloud_ring.points[i].g= 255;
                  pcl_cloud_ring.points[i].b= 255*(2.0-pcl_cloud_ring.points[i].z);
              }
              else
              {
                  pcl_cloud_ring.points[i].r= 0;
                  pcl_cloud_ring.points[i].g= 255*(pcl_cloud_ring.points[i].z);
                  pcl_cloud_ring.points[i].b= 255;
              }
pcl_cloud_ring.points[i].z+=2;
    #undef zGap
  }
/********
    double time_start =ros::Time::now().toSec();
    mrpt::utils::CImage mrpt_gridmap_img;
    IplImage ipl_gridmap_img;
    mrpt::maps::COccupancyGridMap2DPtr mrpt_grid_map = mrpt::maps::COccupancyGridMap2D::Create();
    pcl_cloud_ring.points.clear();
    mrpt_pointcloud_map_raw_color.clear();
    copy2coloredpcl_pointsMap(inMsg,pcl_cloud_ring);
    curve_detection_process();
    for(int i=0;i<RING_MAX;i++)
    {
        mrpt_singleline_colored_pointcloud[i].setFromPCLPointCloudRGB(pcl_singleline_colored_pointcloud[i]);
    }
*********/
    mrpt_pointcloud_map_raw_color.setFromPCLPointCloudRGB(pcl_cloud_ring);

    display.scene = mrpt::opengl::COpenGLScene::Create();
     /*************mini window****************/
    /***************/

    display.clear_scene();
    mrpt::opengl::CPointCloudColouredPtr obj_pointcloud = mrpt::opengl::CPointCloudColoured::Create();
    obj_pointcloud->setPointSize(3);
    mrpt::opengl::CSetOfObjectsPtr obj_gridMap = mrpt::opengl::CSetOfObjects::Create();
//    mrpt_grid_map->getAs3DObject(obj_gridMap);
//    curve_grid_map->getAs3DObject(obj_gridMap);
//    display.scene->insert(obj_gridMap);

    display.insert_all_obj();

//    display.insert_grid();
//    display.insert_suvModel();
    obj_pointcloud->clear();
    obj_pointcloud->loadFromPointsMap(&mrpt_pointcloud_map_raw_color);
//    obj_pointcloud->loadFromPointsMap(&pointcloud_map_reset_ref_color );
//    obj_pointcloud->loadFromPointsMap(&curve_pointcloud_map_reset_ref_color );


    display.scene->insert(obj_pointcloud);


    display.refresh_3Ddisplay();
//    ROS_INFO("point cloud process end!");
}
void pcl_pointcloud::my_route_planner(std::vector<admissable_space_type> passable_space,mrpt::poses::CPose3D senser_pose)
{
    mrpt::maps::COccupancyGridMap2DPtr mrpt_passable_gridmap = mrpt::maps::COccupancyGridMap2D::Create();
    mrpt_passable_gridmap->setSize(
                gridMap_y_min,
                gridMap_y_max,
                gridMap_x_min,
                gridMap_x_max,
                gridMap_resolution,
               0);

#define delat_radius 0.05
#define yuliang_angle   15
#define target_ring 2
    /*****************************************
     *遍历可行域，获得可行域边沿的栅格图
     * 该栅格图所对应的点云用于RRT算法
     * ***************************************/
    for(int i=0;i<passable_space.size();i++)
    {
        float radius = passable_space.at(i).arch_radius;
        int arch_start_angle= passable_space.at(i).start;
        int arch_end_angle= passable_space.at(i).end;
        {
            if(arch_start_angle != start_angle )
            {
                float start_x =radius * cos((float)(arch_start_angle - start_angle)/180.0 * my_pi)+ senser_pose.x();
                float start_y =radius * sin((float)(arch_start_angle - start_angle)/180.0 * my_pi) + senser_pose.y();
                mrpt_passable_gridmap->setPos(start_x,start_y,1);
            }
            if(arch_end_angle != end_angle)
            {
                float end_x =  radius * cos((float)(arch_end_angle - start_angle)/180.0 * my_pi) +senser_pose.x();
                float end_y = radius * sin((float)(arch_end_angle - start_angle)/180.0 * my_pi) + senser_pose.y();
                mrpt_passable_gridmap->setPos(end_x,end_y,1);

            }
        }
    }
    double localmap_target_y=0.0;
    double localmap_target_x = 0.0;
    double phi=0.0;
#undef yuliang_angle
#undef delat_radius
}

void pcl_pointcloud::copy2pcl_pointsMap(const VPointCloud::ConstPtr &inMsg, pcl::PointCloud<pcl::PointXYZI> &cloud_ring)
{
    pcl::PointXYZI  pcl_point_temp;
    cloud_ring.points.clear();
    for (int i = 0; i < inMsg->points.size(); i++)
    {
        if(inMsg->points[i].ring <RING_MAX && inMsg->points[i].x >0)
        {
            pcl_point_temp.x = inMsg->points[i].x;
            pcl_point_temp.y = inMsg->points[i].y;
            pcl_point_temp.z = inMsg->points[i].z;
            pcl_point_temp.z += fabs(pointcloud_z_min);
            pcl_point_temp.intensity = inMsg->points[i].intensity;
            cloud_ring.push_back(pcl_point_temp);
        }
    }
}
void pcl_pointcloud::copy2coloredpcl_pointsMap(const VPointCloud::ConstPtr &inMsg , pcl::PointCloud<pcl::PointXYZRGB> &cloud_ring)
{
    pcl::PointXYZRGB  pcl_colorpoint_temp;
    cloud_ring.points.clear();
    for(int i=0;i<RING_MAX;i++)
    {
        pcl_singleline_colored_pointcloud[i].clear();
        pcl::PointCloud<pcl::PointXYZRGB> (pcl_singleline_colored_pointcloud[i]).swap(pcl_singleline_colored_pointcloud[i]);
        pcl_point_radius[i].clear();
        std::vector<float>(pcl_point_radius[i]).swap(pcl_point_radius[i]);
    }
    int ring_temp=0;
    for (int i = 0; i < inMsg->points.size(); i++)
    {
        ring_temp = inMsg->points[i].ring;
//        if( inMsg->points[i].x >0 )//保证只有向前的点云
        {
            pcl_colorpoint_temp.x = inMsg->points[i].x;
            pcl_colorpoint_temp.y = inMsg->points[i].y;
            pcl_colorpoint_temp.z = inMsg->points[i].z;
            /********/
            #define zGap (pointcloud_z_max-pointcloud_z_min)
              if(pcl_colorpoint_temp.z>pointcloud_z_min + 3 * zGap /4)
              {
                  pcl_colorpoint_temp.r= 255;
                  pcl_colorpoint_temp.g=255 * (1-(pcl_colorpoint_temp.z *4 - pointcloud_z_min * 4 - 3 * zGap ) / zGap);
                  pcl_colorpoint_temp.b= 0;
              }
              else if(pcl_colorpoint_temp.z>pointcloud_z_min+zGap /2)
              {
                  pcl_colorpoint_temp.r= 255*( pcl_colorpoint_temp.z*4-pointcloud_z_min*4-2* zGap ) / zGap;
                  pcl_colorpoint_temp.g= 255;
                  pcl_colorpoint_temp.b= 0;
              }
              else if(pcl_colorpoint_temp.z>pointcloud_z_min+zGap/4)
              {
                  pcl_colorpoint_temp.r= 0;
                  pcl_colorpoint_temp.g= 255;
                  pcl_colorpoint_temp.b= 255*(1-(pcl_colorpoint_temp.z*4-pointcloud_z_min*4-zGap)/zGap);
              }
              else
              {
                  pcl_colorpoint_temp.r= 0;
                  pcl_colorpoint_temp.g= (pcl_colorpoint_temp.z*4-pointcloud_z_min*4)/zGap;
                  pcl_colorpoint_temp.b= 255;
              }
            #undef zGap
            /*******/

            if( pcl_colorpoint_temp.z >pointcloud_z_min && pcl_colorpoint_temp.z<pointcloud_z_max && pcl_colorpoint_temp.x>pointcloud_x_min && pcl_colorpoint_temp.x<pointcloud_x_max  && pcl_colorpoint_temp.y>pointcloud_y_min && pcl_colorpoint_temp.y<pointcloud_y_max)
            {
                cloud_ring.push_back(pcl_colorpoint_temp);
            }
            if(ring_temp <RING_MAX && pcl_colorpoint_temp.z <pointcloud_z_max && pcl_colorpoint_temp.z > pointcloud_z_min )
            {
                pcl_singleline_colored_pointcloud[ring_temp].push_back(pcl_colorpoint_temp);
                pcl_point_radius[ring_temp].push_back(sqrt(pcl_colorpoint_temp.x * pcl_colorpoint_temp.x + pcl_colorpoint_temp.y * pcl_colorpoint_temp.y));
            }
        }
    }
}

void pcl_pointcloud::pcl_radius_differential()
{
    static float last_radius_change[RING_MAX] = {0.0};
    static float last2_radius_change[RING_MAX] = {0.0};
    static float last3_radius_change[RING_MAX] = {0.0};
    float radius_change=0.0;
    pcl_curve_pointcloud.clear();
    pcl::PointCloud<pcl::PointXYZRGB>(pcl_curve_pointcloud).swap(pcl_curve_pointcloud);
    for(int i=0;i<RING_MAX;i++)
    {
        pcl_point_radius_different[i].clear();
        std::vector<float>(pcl_point_radius_different[i]).swap(pcl_point_radius_different[i]);
        for(int j=1; j<pcl_point_radius[i].size();j++)
        {
            radius_change= pcl_point_radius[i].at(j) - pcl_point_radius[i].at(j-1); //
            radius_change = fabs(radius_change);
            if(fabs(radius_change) >radius_change_threshold[i]
                    && fabs(last_radius_change[i]) > radius_change_threshold[i]
                    && fabs(last2_radius_change[i] >radius_change_threshold[i])
                    ||(pcl_singleline_colored_pointcloud[i].at(j).z - pointcloud_z_min ) >0.7
//                    && fabs(last3_radius_change[i] >radius_change_threshold[i])
                    )//fifo的思想率除掉孤立的点
            {
//                pcl_singleline_colored_pointcloud[i].at(j).g = 255;
                pcl_curve_pointcloud.push_back(pcl_singleline_colored_pointcloud[i].at(j));
            }
            last3_radius_change[i] = last2_radius_change[i];
            last2_radius_change[i] = last_radius_change[i];
            last_radius_change[i] = radius_change;
            pcl_point_radius_different[i].push_back(fabs(radius_change));
        }
    }
}

void pcl_pointcloud::curve_detection_process()
{
    pcl_radius_differential();
}

void pcl_pointcloud::global_pointcloud_sum()
{
//    pointcloud_sum_num_decide();
    pointcloud_fifo_type pointcloud_fifo_temp;
    mrpt::maps::CColouredPointsMap pointcloud_map_added_color;//全局坐标系下的点云叠加结果
    if(!DR.zhendong_flag)
    {
        //    ROS_INFO("pitch %f,  roll %f,  z_acc %f \n",DR.car_angle.pitch/M_PI *180.0,DR.car_angle.roll/M_PI *180.0,DR.imu_data.linear_acceleration.z);
            pointcloud_fifo_temp.pointcloud = mrpt_pointcloud_map_raw_color;
            {
                if(DR.robot_pose_inc.x() >0.00000001 )
                {
                    pointcloud_fifo_temp.robot_pose_local = DR.robot_global_pose;
                    pointcloud_map_fifo.push_back(pointcloud_fifo_temp);
                    //改过，原来为if
                    while( pointcloud_map_fifo.size() > pointcloud_add_num)
                    {
                        pointcloud_map_fifo.erase(pointcloud_map_fifo.begin());
                    }
                    std::vector<pointcloud_fifo_type>(pointcloud_map_fifo).swap(pointcloud_map_fifo);
                    pointcloud_map_added_color.clear();
                    pointcloud_map_added_color.insertionOptions.minDistBetweenLaserPoints = min_distBetweenLaserPoints;

                    for(int i = 0;i<pointcloud_map_fifo.size();i++)
                    {
                        //插入点云的相对坐标，Z方向提高2m，将地面点的高度变为0
                        mrpt::poses::CPose3D pose_test(pointcloud_map_fifo.at(i).robot_pose_local.x(),pointcloud_map_fifo.at(i).robot_pose_local.y(), 0,pointcloud_map_fifo.at(i).robot_pose_local.phi(),0,0);
                        pointcloud_map_added_color.insertAnotherMap(&pointcloud_map_fifo.at(i).pointcloud , pose_test);
                    }
                }
                else
                {
                    return ;
                }
            }
        #define front_car_distance  30
            float x_inc = front_car_distance *cos(DR.robot_global_pose.phi());
            float y_inc = front_car_distance *sin(DR.robot_global_pose.phi());
        #undef front_car_distance
        #define scan_range  60
            pointcloud_map_added_color.clipOutOfRange(mrpt::math::TPoint2D(DR.robot_global_pose.x() +x_inc,DR.robot_global_pose.y() + y_inc),scan_range);
        #undef scan_range
            pointcloud_map_reset_ref_color.clear();
             mrpt::poses::CPose3D pose_temp(DR.robot_global_pose.x(),DR.robot_global_pose.y(),0, DR.robot_global_pose.phi(),0,0);
            pointcloud_map_reset_ref_color.changeCoordinatesReference(pointcloud_map_added_color,-pose_temp);
            pointcloud_map_reset_ref_color.extractPoints(mrpt::math::TPoint3D(pointcloud_x_min,pointcloud_y_min,-5),mrpt::math::TPoint3D(pointcloud_x_max,pointcloud_y_max,5),&pointcloud_map_final_color,0.4,0.2,0);
        //    pointcloud_map_final_color.save3D_to_text_file("/home/hubing/pointcloud_add.txt");
    }
    else
    {
        DR.zhendong_flag = false;
        DR.zhendong_count =0;
    }
}


void pcl_pointcloud::curve_pointcloud_sum()
{
    pointcloud_fifo_type pointcloud_fifo_temp;
    mrpt::maps::CColouredPointsMap curve_pointcloud_map_added_color;//全局坐标系下的点云叠加结果
    if(!DR.zhendong_flag)
    {
        //    ROS_INFO("pitch %f,  roll %f,  z_acc %f \n",DR.car_angle.pitch/M_PI *180.0,DR.car_angle.roll/M_PI *180.0,DR.imu_data.linear_acceleration.z);
            pointcloud_fifo_temp.pointcloud = mrpt_curve_pointcloud;
            {
//                mrpt_curve_pointcloud
                //if(DR.robot_pose_inc.x() >0.00000001 )
                {
                    pointcloud_fifo_temp.robot_pose_local = DR.robot_global_pose;
                    curve_pointcloud_map_fifo.push_back(pointcloud_fifo_temp);
                    if( curve_pointcloud_map_fifo.size() > pointcloud_add_num)
                    {
                        curve_pointcloud_map_fifo.erase(curve_pointcloud_map_fifo.begin());
                    }
                    std::vector<pointcloud_fifo_type>(curve_pointcloud_map_fifo).swap(curve_pointcloud_map_fifo);

                    curve_pointcloud_map_added_color.clear();
//                    curve_pointcloud_map_added_color.insertionOptions.minDistBetweenLaserPoints = min_distBetweenLaserPoints;

                    for(int i = 0;i<curve_pointcloud_map_fifo.size();i++)
                    {
                        //插入点云的相对坐标，Z方向提高2m，将地面点的高度变为0
                        mrpt::poses::CPose3D pose_test(curve_pointcloud_map_fifo.at(i).robot_pose_local.x(),curve_pointcloud_map_fifo.at(i).robot_pose_local.y(), 0,curve_pointcloud_map_fifo.at(i).robot_pose_local.phi(),0,0);
                        curve_pointcloud_map_added_color.insertAnotherMap(&curve_pointcloud_map_fifo.at(i).pointcloud , pose_test);
                    }
                }
            }
        #define front_car_distance  30
            float x_inc = front_car_distance *cos(DR.robot_global_pose.phi());
            float y_inc = front_car_distance *sin(DR.robot_global_pose.phi());
        #undef front_car_distance
        #define scan_range  60
            curve_pointcloud_map_added_color.clipOutOfRange(mrpt::math::TPoint2D(DR.robot_global_pose.x() +x_inc,DR.robot_global_pose.y() + y_inc),scan_range);
        #undef scan_range
            curve_pointcloud_map_reset_ref_color.clear();
             mrpt::poses::CPose3D pose_temp(DR.robot_global_pose.x(),DR.robot_global_pose.y(),0, DR.robot_global_pose.phi(),0,0);
            curve_pointcloud_map_reset_ref_color.changeCoordinatesReference(curve_pointcloud_map_added_color,-pose_temp);
            curve_pointcloud_map_reset_ref_color.extractPoints(mrpt::math::TPoint3D(pointcloud_x_min,pointcloud_y_min,-5),mrpt::math::TPoint3D(pointcloud_x_max,pointcloud_y_max,5),&curve_pointcloud_map_final_color,0.4,0.2,0);
//            curve_pointcloud_map_final_color.save3D_to_text_file("/home/hubing/curve_pointcloud_add.txt");
    }
    else
    {
        DR.zhendong_flag = false;
        DR.zhendong_count =0;
    }
}
int pcl_pointcloud::curve_pointcloud2gridmap()
{
    curve_grid_map_filtered = cv::Mat(gridMap_cell_x_size,gridMap_cell_y_size,CV_8U,cv::Scalar::all(0));
   int img_width = curve_grid_map_filtered.cols;//200
   int img_height = curve_grid_map_filtered.rows;//150
   curve_grid_map->clear();
   curve_grid_map = mrpt::maps::COccupancyGridMap2D::Create();
   curve_grid_map->setSize(
               gridMap_x_min,
               gridMap_x_max,
               gridMap_y_min,
               gridMap_y_max,
               gridMap_resolution,
              1);
   std::vector<mrpt::math::TPoint3D> local_pose;
   curve_pointcloud_map_final_color.getAllPoints(local_pose);
   int pointcloud_size=local_pose.size();
   mrpt::math::TPoint3D pmin,pmax;
   curve_pointcloud_map_final_color.boundingBox(pmin,pmax);
//   ROS_INFO("zmin is %f",pmin.z);
   for (int i = 0; i < pointcloud_size; i++)
   {
       int xx=0;
       int yy=0;
       xx = curve_grid_map->x2idx(local_pose[i].x);
       yy = curve_grid_map->y2idx(local_pose[i].y);
       curve_grid_map->setCell(xx,yy,0);

       curve_grid_map_filtered.data[(img_height-xx -1)*img_width + (img_width - yy -1 )] = 255;//白色
//        if(local_pose[i].z <(pointcloud_z_min + 0.06))
//        {
//            curve_grid_map_filtered.data[(img_height-xx -1)*img_width + (img_width - yy -1 )] = 0;//白色
//        }
   }
//   cv::morphologyEx(curve_grid_map_filtered,curve_grid_map_filtered,cv::MORPH_CLOSE,cv::Mat(6,6,CV_8U),cv::Point(-1,-1),1);
//      cv::dilate(curve_grid_map_filtered,curve_grid_map_filtered, cv::Mat(3,3,CV_8U),cv::Point(-1,-1),1);
//   cv::erode(curve_grid_map_filtered,curve_grid_map_filtered, cv::Mat(5,5,CV_8U),cv::Point(-1,-1),1);

   /********/
   int data_size = curve_grid_map_filtered.cols * curve_grid_map_filtered.rows;
   for(int i=0;i<data_size;i++)
   {
       {
            curve_grid_map_filtered.data[i] = 255- curve_grid_map_filtered.data[i];
       }
   }
   /**********/
   /**************/
   line_finder finder;
   cv::Mat contours;
     //边缘检测
     cv::Canny (curve_grid_map_filtered,contours,50,150);
     finder.setMinVote (30);
     finder.setLineLengthAndGap (10,5);
     finder.findLines (contours);
     finder.drawDetectedLines (curve_grid_map_filtered,cv::Scalar(0));
//    cv::imwrite("/home/hubing/gridmap.jpg",curve_grid_map_filtered);
/****************************/
}
