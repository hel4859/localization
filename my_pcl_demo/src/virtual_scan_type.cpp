#include "virtual_scan_type.h"
#include "my_math.h"

#define VERTICAL_POSE

virtual_scan_type::virtual_scan_type()
{
    senser_pose = mrpt::poses::CPose3D(0,10,0,0,0,0);
    curRobotPoseEst = mrpt::poses::CPose2D(0,0,0);
    gridmap_virtual_scan.maxRange =Max_range;
    gridmap_virtual_scan.rightToLeft = true;
    gridmap_virtual_scan.setSensorPose(senser_pose);
    gridmap_virtual_scan.aperture = Pi*2;
    car_based_gridmap_virtual_scan.maxRange = Max_range;
    car_based_gridmap_virtual_scan.rightToLeft = true;
    car_based_gridmap_virtual_scan.setSensorPose(mrpt::poses::CPose3D(0,0,0,0,0,0));
    car_based_gridmap_virtual_scan.aperture = my_pi*2;
    for(int i=0;i<filter_length;i++)
    {
        is_intersection_raw[i]=false;
    }
    road_width =0;
    car_yaw =0.0;
    target_angle = 0.0;
    intersectiont_error_angle = 0;
    target_distance = 0.0;
    sub_error_angle = nh.subscribe("multi_angle_distance",1,&virtual_scan_type::multi_angle_distance_callback,this);
}
void virtual_scan_type::multi_angle_distance_callback(std_msgs::Float64MultiArray msg)
{
    car_yaw = msg.data[0];
    target_angle = msg.data[1];
    intersectiont_error_angle= msg.data[2];
    target_distance= msg.data[3];
}
void virtual_scan_type::intersection_process(mrpt::maps::COccupancyGridMap2D &grid_map)
{
}
void virtual_scan_type::passable_area_filter(std::vector<admissable_space_type> &input_space,std::vector<admissable_space_type> &out_space,float error_angle)
{
    out_space.clear();
    for(int i = 0;i<input_space.size();i++)
    {
        float angle_temp = middle_angle+error_angle;
        if(angle_temp > input_space[i].start && angle_temp < input_space[i].end )
        {
            out_space.push_back(input_space[i]);
        }
    }
}
void virtual_scan_type::passable_area_filter(std::vector<admissable_space_type> &input_space,std::vector<admissable_space_type> &out_space,float direction_angle,float error_angle)
{
    out_space.clear();
    int size = input_space.size();
    for(int i=0;i<size;i++)
    {
        if(fabs(input_space[i].middle - direction_angle) <error_angle)
        {
            out_space.push_back(input_space[i]);
        }
    }
}

void virtual_scan_type::calculate_senser_pose()
{
    float y,x;
    float cell_val=0.0;
    #define y_step 5*grid_map_resolution //搜索的步进，单位是像素点
    #define x_max   20
    #define y_min   3   //国际单位制距离
    #define y_max   17  //国际单位直距离
    std::vector<road_edge_type> road_edge;
    road_edge_type edge_temp;
    float velocity_temp = DR.velocity_data.data[0] /3.6; //米/s制单位的速度
    //d = d b + v 0 t + d‘
    //d b =v 0 2 / 2μg
#define horizontal_factor   0.2 //处理事件
    float horizontal_distance = 4+ velocity_temp*horizontal_factor +velocity_temp*velocity_temp / (2 * 0.4*9.8) ;
    if(horizontal_distance > 10)
    {
        horizontal_distance = 10;
    }
    //    horizontal_distance = 10;
#undef   horizontal_factor

#ifndef VERTICAL_POSE
senser_pose.setFromValues(0,horizontal_distance,0,0,0,0);
#else
//set_senser_position(senser_pose);
    /****************/
    if(is_intesection() == false)
    {
        for(y = y_min;y< y_max ;y+=y_step)
        {
            x = 0;
            cell_val=gridmap_local.getPos(x,y);
            while(cell_val >gray_threshold && x > -x_max)
            {
                cell_val=gridmap_local.getPos(x,y);
                x -=grid_map_resolution;
            }
            if(x ==-x_max)
            {
                edge_temp.left.y = y;
                edge_temp.left.x = 0;
            }else
            {
                edge_temp.left.y = y;
                edge_temp.left.x = x;
            }
            x = 0;
            cell_val=gridmap_local.getPos(x,y);
            while(cell_val >gray_threshold && x < x_max)
            {
                cell_val=gridmap_local.getPos(x,y);
                x+= grid_map_resolution;
            }
            if(x ==x_max)
            {
                edge_temp.right.y = y;
                edge_temp.right.x = 0;
            }else
            {
                edge_temp.right.y = y;
                edge_temp.right.x = x;
            }
            road_edge.push_back(edge_temp);
        }
        std::vector<road_middle_type>  road_middle;
        road_middle_type road_middle_temp;

        for(int i=0;i<road_edge.size();i++)
        {
            road_middle_temp.middle.x=(road_edge[i].left.x + road_edge[i].right.x)/2;
            road_middle_temp.middle.y=(road_edge[i].left.y + road_edge[i].right.y)/2;
            road_middle.push_back(road_middle_temp);
        }
        float xx=0;
        float yy=0;
        float xy=0;
        float sum_x;
        float sum_y=0;
        for(int i=0;i<road_middle.size();i++)
        {
            xx += road_middle[i].middle.y *road_middle[i].middle.y;
            yy += road_middle[i].middle.x *road_middle[i].middle.x;
            xy += road_middle[i].middle.x *road_middle[i].middle.y;
            sum_x +=road_middle[i].middle.y;
            sum_y +=road_middle[i].middle.x;

        }
        float k,b;//y=kx+b
        int n= road_middle.size();
        k = (n * xy - sum_x * sum_y) / (n * xx - sum_x*sum_x);
        b = (xx*sum_y - sum_x * xy) / (n * xx - sum_x*sum_x);
        float distance_temp =10;//distance between car and virtual scanner
        float vertical_distance = k*distance_temp + b;
//        ROS_INFO("vertical distance is %f",vertical_distance);
//        senser_pose.setFromValues(vertical_distance,horizontal_distance,0,0,0,0);
        senser_pose.setFromValues(vertical_distance,horizontal_distance,0,0,0,0);
        set_senser_position(senser_pose);
        x++;
    }
    else
    {
        senser_pose.setFromValues(0,horizontal_distance,0,0,0,0);
        set_senser_position(senser_pose);
    }
#endif
    /******************/
//    gridmap_local.getCell(0,0);
}
void virtual_scan_type ::get_line_state()
{
    is_line_disappear.is_right_line_disappear = false;
    is_line_disappear.is_left_line_disappear = false;
    int size = car_based_admisbale_space_filtered.size();
    float start_sum=0.0;
    float start_average = 0.0;

    float end_sum=0.0;
    float end_average = 0.0;
    int start_end_count = 0;
    for(int i=0;i<size;i++)
    {
        if(car_based_admisbale_space_filtered[i].arch_radius > 4)
        {
            start_sum += car_based_admisbale_space_filtered[i].start;
            end_sum += car_based_admisbale_space_filtered[i].end;
            start_end_count++;
        }
    }
    if(start_end_count == 0)
    {
        is_line_disappear.is_right_line_disappear = true;
        is_line_disappear.is_left_line_disappear = true;
    }
    else
    {
        start_average = (float)(start_sum / (float)start_end_count);
        end_average = (float) (end_sum / (float)start_end_count);
        //接近180度则认为丢失
        if(start_average < start_angle + 20)
        {
            is_line_disappear.is_right_line_disappear = true;
        }
        //接近360度则认为丢失
        if(end_average >end_angle - 20)
        {
            is_line_disappear.is_left_line_disappear = true;
        }
    }
    math_characteristic.start_average = start_average;
    math_characteristic.end_average = end_average;
    ROS_INFO("start_ave %f, end_ave%f",start_average,end_average);
}
void virtual_scan_type:: math_characteristic_process()
{
    int i=0;
    int size = admisbale_space.size();
    float sum=0.0;

    float variance_temp=0.0;
    float middle_avrage  = 0.0;

    for(i = 0;i<size;i++)
    {
        sum += admisbale_space[i].middle;
    }
   middle_avrage = sum/size;

   if(middle_avrage > start_angle || middle_avrage<end_angle)//防止饱和
   {
       variance_temp =middle_angle;
   }
   math_characteristic.average_angle = middle_avrage;
    for(i=0;i<size;i++)
    {
        variance_temp +=(((float)admisbale_space[i].middle - middle_avrage) *((float)admisbale_space[i].middle - middle_avrage)) ;
    }
    math_characteristic.variance = variance_temp/size;
    float average_tem= fabs(math_characteristic.average_angle -middle_angle);
//    printf("average %f, variance %f !\n",average_tem,math_characteristic.variance );
}

bool virtual_scan_type::is_intesection()
{
    char inspector_temp=false;
//    math_characteristic_process();
    float average_tem= fabs(math_characteristic.average_angle -middle_angle);
    float end_sub_start = fabs(math_characteristic.end_average - math_characteristic.start_average);
    //调试信息

    if(math_characteristic.variance > variance_threshold  || is_line_disappear.is_right_line_disappear ||is_line_disappear.is_left_line_disappear  )//add a filter
    {
        inspector_temp =true;
    }else
    {
        inspector_temp=false;
        //return false;
    }
//    return inspector_temp;
    for(int i = filter_length-1;i>0;i--)
    {
        is_intersection_raw[i] = is_intersection_raw[i-1];
    }
    is_intersection_raw[0] = inspector_temp;//更新fifo
    char temp_inspecter[filter_length];
    for(int i=0;i<filter_length;i++)
    {
        temp_inspecter[i] = is_intersection_raw[i];//复制到缓存
    }
    Bubble_sort_char(temp_inspecter,filter_length);//冒泡排序
     if(temp_inspecter[filter_length/2] == true)//取中值作为最终的结果
    {
        return true;
    }else
    {
        return false;
    }
}

void virtual_scan_type::get_virtual_scan_val(mrpt::obs::CObservation2DRangeScan &range_scan,float gray_scale_threshold)
{
    gridmap_local.laserScanSimulator(range_scan,curRobotPoseEst,gray_scale_threshold,scan_point_num,0,1,mrpt::utils::DEG2RAD(0));

}




void virtual_scan_type::set_robot_position(mrpt::poses::CPose2D robot_pos)
{
    curRobotPoseEst = robot_pos;
}


void virtual_scan_type::set_senser_position(mrpt::poses::CPose3D senser_pos)
{
    senser_pose = senser_pos;
    gridmap_virtual_scan.setSensorPose(senser_pose);
}

std::vector<admissable_space_type> virtual_scan_type::get_passable_area(std::vector <float> &scan_result,float min_radius,float max_radius,float delat_radius)
{
    admissable_space_type   admisbale_space_one;
    std::vector<admissable_space_type>  admisbale_space_sequence;
    int num=scan_point_num; // the num of scan result
    int start_point = half_scan_num;
    float radius = min_radius;
    int  i;
    while(radius < max_radius)//半径由小到大
    {
        for(i = start_angle ;i<=end_angle;i++)//搜寻范围为车前的180°范围
        {
            if(scan_result[i] > radius)//
            {
                //
                if(i == start_angle)
                {
                    admisbale_space_one.start = i;
                    continue;
                }
                if(scan_result[i-1] < radius )//当前距离大于阈值，前一个距离小于阈值标示着可行域的起始位置
                {
                    admisbale_space_one.start = i;
                }
                if(i == end_angle || scan_result[i+1] < radius)//当前距离大于阈值，下一个距离小于阈值标示着可行域的结束位置
                {
                    admisbale_space_one.end = i;
                    admisbale_space_one.middle = (admisbale_space_one.end + admisbale_space_one.start)/2;
                    admisbale_space_one.arch_radius = radius;

                    //计算可行域的弧长
                    float radius_temp = 0;
                    if(scan_result[i +1] < scan_result[i -1])
                    {
                        radius_temp = scan_result[i +1];
                    }
                    else
                    {
                        radius_temp = scan_result[i -1];
                    }
                    float space_dimention = get_arch_dimension(( radius_temp),(float)(admisbale_space_one.end-admisbale_space_one.start) /(float)360 * 2*Pi);
                    admisbale_space_one.dimension = space_dimention;
                    //可行域弧长大于车宽并且可行域角度大于8°时计入 最终的可行域
                    if(space_dimention > Car_width && (admisbale_space_one.end - admisbale_space_one.start)>8)
                    {
                        admisbale_space_sequence.push_back(admisbale_space_one);//add space
                    }
                }
            }
        }
        radius +=delat_radius;
    }
    return admisbale_space_sequence;
}
float get_arch_dimension(float radius,float rad)
{
    float dimention = 2.0*radius*sin(rad/2.0);
    return dimention;
}


//使用路宽来进行可行域的滤波
void virtual_scan_type:: get_road_width()
{
    if(is_intesection() == false)
    {
        road_edge_type edge_temp;
    }
    else
    {
        return;
    }
}
