#include "display_data.h"
#include"virtual_scan_type.h"
#define Map_Size_max    50
#include"math.h"
#include"pcl_pointcloud.h"
display_data_type::display_data_type()
{
//    win3D = mrpt::gui::CDisplayWindow3D::Create();
    win3D.setWindowTitle("CyberTiggo_View");
    win3D.resize(600, 600);
    win3D.setCameraAzimuthDeg(270);//方向沿y轴由负方向向正方向看
    win3D.setCameraElevationDeg(60);//俯角20°
    win3D.setCameraPointingToPoint(0, 10, 0);//指向(0,10,0)点
    win3D.setCameraZoom(100);

//    win2D.axis(0,scan_point_num,0,Max_range + 10,false);
//    win2D = mrpt::gui::CDisplayWindowPlots::Create();

    groundPlane = mrpt::opengl::CGridPlaneXY::Create(//网格固定
         -300,
         300,
         -300,
         300,
        0,5); //地平线网格间隔
    groundPlane->setColor(0.0f, 0.0f, 0.0f);
    groundPlane->setLineWidth(0.5);
    virtual_scan_arrow = mrpt::opengl::CCylinder::Create(0.2,0.2,0.5);
    virtual_scan_arrow->setColor(0.5,0.5,0.5);
    axis = mrpt::opengl::CAxis::Create(0,0,0,2,4,2,0.25,3,false);
    axis->setColor(mrpt::utils::TColorf(0,0,1,1));
    suvModel = mrpt::opengl::CAssimpModel::Create();
    suvModel->loadScene("/home/hubing/ros-project/hubing_ws/src/my_pcl_demo/xtrail.3ds");//suv model without detail
    suvModel->setScale(0.004);//OriginSize=6000,TargetSize=42gelin
    suvModel->setColor(0.5, 0.44, 0.6);//0.5zuihou
    suvModel->setPose(mrpt::poses::CPose3D(0, 0, 0.95, Pi, 0, HalfPi));

    obj_scan = mrpt::opengl::CPlanarLaserScan::Create();
    obj_gridMap = mrpt::opengl::CSetOfObjects::Create();
    admissable_space_display = mrpt::opengl::CSetOfLines::Create();

}
void display_data_type::insert_axis()
{
    scene->insert(axis);
}

void display_data_type::insert_grid()
{
    scene->insert(groundPlane);
}

void display_data_type::insert_suvModel()
{
    scene->insert(suvModel);
}
void display_data_type::insert_scan_center_arrow()
{
    virtual_scan_arrow->setPose(mrpt::poses::CPose3D(virtual_scan.senser_pose.x(),virtual_scan.senser_pose.y(),0,0,0,0));
    scene->insert(virtual_scan_arrow);
}

void display_data_type::insert_gridmap_local()
{
    obj_gridMap = mrpt::opengl::CSetOfObjects::Create();
    virtual_scan.gridmap_local.getAs3DObject(obj_gridMap);
    obj_gridMap->setLocation(0, 0, 0);
    scene->insert(obj_gridMap);
}

void display_data_type::insert_scan_result()
{
    obj_scan = mrpt::opengl::CPlanarLaserScan::Create();
    obj_scan->enableSurface(1);
    obj_scan->setPointsWidth(3);
    obj_scan->enableLine(false);
    obj_scan->setColor(1, 0, 0);
    obj_scan->setScan(virtual_scan.gridmap_virtual_scan);
    obj_scan->setLocation(mrpt::math::TPoint3D(virtual_scan.curRobotPoseEst.x(),virtual_scan.curRobotPoseEst.y(),0.05));
    scene->insert(obj_scan);
}

void display_data_type::insert_all_obj()
{
    //    scene = mrpt::opengl::COpenGLScene::Create();
    //    clear_scene();

    insert_gridmap_local();
//    insert_scan_result();
    insert_grid();
//    insert_suvModel();
    insert_scan_center_arrow();
//
}
void display_data_type::refresh_3Ddisplay()
{
    mrpt::opengl::COpenGLScenePtr &ptrScene = win3D.get3DSceneAndLock();
    ptrScene = scene;
    win3D.unlockAccess3DScene();
    win3D.forceRepaint();
}
void display_data_type::refresh_2Ddisplay()
{
//    win2D.plot(virtual_scan.virtual_scan_result,"b-");
}
void display_data_type::clear_scene()
{

}
