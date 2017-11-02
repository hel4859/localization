#ifndef DISPLAY_DATA_H
#define DISPLAY_DATA_H

#include"ros/ros.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <mrpt/gui/CDisplayWindow3D.h>
#include <mrpt/maps/CSimpleMap.h>
#include <mrpt/maps/COccupancyGridMap2D.h>
#include <mrpt/maps/CMultiMetricMap.h>
#include<mrpt/maps/CColouredPointsMap.h>
#include <mrpt/math/utils.h>

#include <mrpt/maps/COctoMap.h>
#include"mrpt/gui.h"

#include"mrpt/opengl.h"
#include"mrpt/opengl/CPlanarLaserScan.h"
#include"mrpt/opengl/CCylinder.h"
#include"opencv2/opencv.hpp"

#include"mrpt/system/os.h"
//using namespace mrpt;

#include"virtual_scan_type.h"
class display_data_type
{
//    mrpt::gui::CDisplayWindowPlotsPtr win2D;


    mrpt::opengl::CGridPlaneXYPtr groundPlane;
    mrpt::opengl::CCylinderPtr virtual_scan_arrow;
    mrpt::opengl::CAssimpModelPtr suvModel;
    mrpt::opengl::CSetOfObjectsPtr obj_gridMap;
    mrpt::opengl::CPlanarLaserScanPtr obj_scan;
    mrpt::opengl::CSetOfLinesPtr    admissable_space_display;
    mrpt::opengl::CAxisPtr  axis;
    cv::Mat task_img;
public:
    mrpt::opengl::COpenGLViewportPtr view_mini;
    mrpt::opengl::COpenGLScenePtr	scene;
    display_data_type();
    virtual_scan_type   virtual_scan;
    mrpt::gui::CDisplayWindow3D win3D;
//    mrpt::gui::CDisplayWindow3D   win3D_imge;
    void insert_grid();
    void insert_gridmap_local();
    void insert_axis();
    void insert_suvModel();
    void insert_scan_center_arrow();
    void insert_scan_result();
    void insert_admissable_space();
    void insert_admissable_space(std::vector<admissable_space_type> space,mrpt::poses::CPose3D sensor);
    void insert_admissable_space(std::vector<admissable_space_type> space,mrpt::utils::TColorf color,mrpt::poses::CPose3D sensor);
    void insert_all_obj();
    void refresh_3Ddisplay();
    void refresh_2Ddisplay();
    void clear_scene();
};

#endif // DISPLAY_DATA_H
