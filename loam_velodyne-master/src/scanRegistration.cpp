// Copyright 2013, Ji Zhang, Carnegie Mellon University
// Further contributions copyright (c) 2016, Southwest Research Institute
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from this
//    software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// This is an implementation of the algorithm described in the following paper:
//   J. Zhang and S. Singh. LOAM: Lidar Odometry and Mapping in Real-time.
//     Robotics: Science and Systems Conference (RSS). Berkeley, CA, July 2014.

#include <cmath>
#include <vector>

#include <loam_velodyne/common.h>
#include <opencv/cv.h>
#include <nav_msgs/Odometry.h>
#include <opencv/cv.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf2_ros/buffer_interface.h>
#include <iostream>
using std::sin;
using std::cos;
using std::atan2;
using namespace std;
const double scanPeriod = 0.1;//vlp16 10hz
std::string target_frame_;

const int systemDelay = 20;
int systemInitCount = 0;
bool systemInited = false;

const int N_SCANS = 16;

float cloudCurvature[40000];
int cloudSortInd[40000];
int cloudNeighborPicked[40000];
int cloudLabel[40000];

int imuPointerFront = 0;
int imuPointerLast = -1;
const int imuQueLength = 200;

float imuRollStart = 0, imuPitchStart = 0, imuYawStart = 0;
float imuRollCur = 0, imuPitchCur = 0, imuYawCur = 0;

float imuVeloXStart = 0, imuVeloYStart = 0, imuVeloZStart = 0;
float imuShiftXStart = 0, imuShiftYStart = 0, imuShiftZStart = 0;

float imuVeloXCur = 0, imuVeloYCur = 0, imuVeloZCur = 0;
float imuShiftXCur = 0, imuShiftYCur = 0, imuShiftZCur = 0;

float imuShiftFromStartXCur = 0, imuShiftFromStartYCur = 0, imuShiftFromStartZCur = 0;
float imuVeloFromStartXCur = 0, imuVeloFromStartYCur = 0, imuVeloFromStartZCur = 0;

double imuTime[imuQueLength] = {0};
float imuRoll[imuQueLength] = {0};
float imuPitch[imuQueLength] = {0};
float imuYaw[imuQueLength] = {0};

float imuAccX[imuQueLength] = {0};
float imuAccY[imuQueLength] = {0};
float imuAccZ[imuQueLength] = {0};

float imuVeloX[imuQueLength] = {0};
float imuVeloY[imuQueLength] = {0};
float imuVeloZ[imuQueLength] = {0};

float imuShiftX[imuQueLength] = {0};
float imuShiftY[imuQueLength] = {0};
float imuShiftZ[imuQueLength] = {0};

ros::Publisher pubLaserCloud;
ros::Publisher pubCornerPointsSharp;
ros::Publisher pubCornerPointsLessSharp;
ros::Publisher pubSurfPointsFlat;
ros::Publisher pubSurfPointsLessFlat;
ros::Publisher pubImuTrans;

void ShiftToStartIMU(float pointTime)
{

  imuShiftFromStartXCur = imuShiftXCur - imuShiftXStart - imuVeloXStart * pointTime;
  imuShiftFromStartYCur = imuShiftYCur - imuShiftYStart - imuVeloYStart * pointTime;
  imuShiftFromStartZCur = imuShiftZCur - imuShiftZStart - imuVeloZStart * pointTime;

  float x1 = cos(imuYawStart) * imuShiftFromStartXCur - sin(imuYawStart) * imuShiftFromStartZCur;
  float y1 = imuShiftFromStartYCur;
  float z1 = sin(imuYawStart) * imuShiftFromStartXCur + cos(imuYawStart) * imuShiftFromStartZCur;

  float x2 = x1;
  float y2 = cos(imuPitchStart) * y1 + sin(imuPitchStart) * z1;
  float z2 = -sin(imuPitchStart) * y1 + cos(imuPitchStart) * z1;

  imuShiftFromStartXCur = cos(imuRollStart) * x2 + sin(imuRollStart) * y2;
  imuShiftFromStartYCur = -sin(imuRollStart) * x2 + cos(imuRollStart) * y2;
  imuShiftFromStartZCur = z2;
}

void VeloToStartIMU()
{
  imuVeloFromStartXCur = imuVeloXCur - imuVeloXStart;
  imuVeloFromStartYCur = imuVeloYCur - imuVeloYStart;
  imuVeloFromStartZCur = imuVeloZCur - imuVeloZStart;

  float x1 = cos(imuYawStart) * imuVeloFromStartXCur - sin(imuYawStart) * imuVeloFromStartZCur;
  float y1 = imuVeloFromStartYCur;
  float z1 = sin(imuYawStart) * imuVeloFromStartXCur + cos(imuYawStart) * imuVeloFromStartZCur;

  float x2 = x1;
  float y2 = cos(imuPitchStart) * y1 + sin(imuPitchStart) * z1;
  float z2 = -sin(imuPitchStart) * y1 + cos(imuPitchStart) * z1;

  imuVeloFromStartXCur = cos(imuRollStart) * x2 + sin(imuRollStart) * y2;
  imuVeloFromStartYCur = -sin(imuRollStart) * x2 + cos(imuRollStart) * y2;
  imuVeloFromStartZCur = z2;
}

void TransformToStartIMU(PointType *p)
{
  float x1 = cos(imuRollCur) * p->x - sin(imuRollCur) * p->y;
  float y1 = sin(imuRollCur) * p->x + cos(imuRollCur) * p->y;
  float z1 = p->z;

  float x2 = x1;
  float y2 = cos(imuPitchCur) * y1 - sin(imuPitchCur) * z1;
  float z2 = sin(imuPitchCur) * y1 + cos(imuPitchCur) * z1;

  float x3 = cos(imuYawCur) * x2 + sin(imuYawCur) * z2;
  float y3 = y2;
  float z3 = -sin(imuYawCur) * x2 + cos(imuYawCur) * z2;

  float x4 = cos(imuYawStart) * x3 - sin(imuYawStart) * z3;
  float y4 = y3;
  float z4 = sin(imuYawStart) * x3 + cos(imuYawStart) * z3;

  float x5 = x4;
  float y5 = cos(imuPitchStart) * y4 + sin(imuPitchStart) * z4;
  float z5 = -sin(imuPitchStart) * y4 + cos(imuPitchStart) * z4;

  p->x = cos(imuRollStart) * x5 + sin(imuRollStart) * y5 + imuShiftFromStartXCur;
  p->y = -sin(imuRollStart) * x5 + cos(imuRollStart) * y5 + imuShiftFromStartYCur;
  p->z = z5 + imuShiftFromStartZCur;
}

void AccumulateIMUShift()
{
  float roll = imuRoll[imuPointerLast];
  float pitch = imuPitch[imuPointerLast];
  float yaw = imuYaw[imuPointerLast];
  float accX = imuAccX[imuPointerLast];
  float accY = imuAccY[imuPointerLast];
  float accZ = imuAccZ[imuPointerLast];

  float x1 = cos(roll) * accX - sin(roll) * accY;
  float y1 = sin(roll) * accX + cos(roll) * accY;
  float z1 = accZ;

  float x2 = x1;
  float y2 = cos(pitch) * y1 - sin(pitch) * z1;
  float z2 = sin(pitch) * y1 + cos(pitch) * z1;

  accX = cos(yaw) * x2 + sin(yaw) * z2;
  accY = y2;
  accZ = -sin(yaw) * x2 + cos(yaw) * z2;
  //  std::cout<<"accX:"<<accX<<" ";
    //std::cout<<"accY:"<<accY<<" ";
    //std::cout<<"accZ:"<<accZ<<std::endl;

  int imuPointerBack = (imuPointerLast + imuQueLength - 1) % imuQueLength;
  double timeDiff = imuTime[imuPointerLast] - imuTime[imuPointerBack];
  if (timeDiff < scanPeriod) {

    imuShiftX[imuPointerLast] = imuShiftX[imuPointerBack] + imuVeloX[imuPointerBack] * timeDiff 
                              + accX * timeDiff * timeDiff / 2;
    imuShiftY[imuPointerLast] = imuShiftY[imuPointerBack] + imuVeloY[imuPointerBack] * timeDiff 
                              + accY * timeDiff * timeDiff / 2;
    imuShiftZ[imuPointerLast] = imuShiftZ[imuPointerBack] + imuVeloZ[imuPointerBack] * timeDiff 
                              + accZ * timeDiff * timeDiff / 2;

    imuVeloX[imuPointerLast] = imuVeloX[imuPointerBack] + accX * timeDiff;
    imuVeloY[imuPointerLast] = imuVeloY[imuPointerBack] + accY * timeDiff;
    imuVeloZ[imuPointerLast] = imuVeloZ[imuPointerBack] + accZ * timeDiff;
  }
}

void laserCloudHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg)
{
  if (!systemInited) {
    systemInitCount++;
    if (systemInitCount >= systemDelay) {
      systemInited = true;
    }
    return;
  }

  std::vector<int> scanStartInd(N_SCANS, 0);//16
  std::vector<int> scanEndInd(N_SCANS, 0);
  
  double timeScanCur = laserCloudMsg->header.stamp.toSec();
  pcl::PointCloud<pcl::PointXYZ> laserCloudIn;
  pcl::fromROSMsg(*laserCloudMsg, laserCloudIn);
  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(laserCloudIn, laserCloudIn, indices);
  int cloudSize = laserCloudIn.points.size();//一帧点云
   // cout<<cloudSize<<endl;
  float startOri = -atan2(laserCloudIn.points[0].y, laserCloudIn.points[0].x);
  float endOri = -atan2(laserCloudIn.points[cloudSize - 1].y,
                        laserCloudIn.points[cloudSize - 1].x) + 2 * M_PI;

  if (endOri - startOri > 3 * M_PI) {//保证起末角度为pi
    endOri -= 2 * M_PI;
  } else if (endOri - startOri < M_PI) {
    endOri += 2 * M_PI;
  }
  bool halfPassed = false;
  int count = cloudSize;
  PointType point;
    std::vector<pcl::PointCloud<PointType> > laserCloudScans(N_SCANS);
    //cout<<"first_point:"<<laserCloudIn.points[0].x<<","<<laserCloudIn.points[0].y<<","<<laserCloudIn.points[0].z<<endl;
    //cout<<"end_point:"<<laserCloudIn.points[cloudSize-1].x<<","<<laserCloudIn.points[cloudSize-1].y<<","<<laserCloudIn.points[cloudSize-1].z<<endl;

    for (int i = 0; i < cloudSize; i++) {
    point.x = laserCloudIn.points[i].y;
    point.y = laserCloudIn.points[i].z;
    point.z = laserCloudIn.points[i].x;

    float angle = atan(point.y / sqrt(point.x * point.x + point.z * point.z)) * 180 / M_PI;//z到x,y
    int scanID;
      //std::cout<<"angle:"<<angle<<std::endl;

    int roundedAngle = int(angle + (angle<0.0?-0.5:+0.5));
      //std::cout<<"roud:"<<roundedAngle<<std::endl;
    if (roundedAngle > 0){
      scanID = roundedAngle;
    }
    else {
      scanID = roundedAngle + (N_SCANS - 1);//保证在0-15
    }
    if (scanID > (N_SCANS - 1) || scanID < 0 ){
      count--;
      continue;
    }

    float ori = -atan2(point.x, point.z);//二维平面的点
    if (!halfPassed) {
      if (ori < startOri - M_PI / 2) {
        ori += 2 * M_PI;
      } else if (ori > startOri + M_PI * 3 / 2) {
        ori -= 2 * M_PI;//ori位于-0.5pi到1.5pi之间
      }

      if (ori - startOri > M_PI) {
        halfPassed = true;
      }
    } else {
      ori += 2 * M_PI;

      if (ori < endOri - M_PI * 3 / 2) {
        ori += 2 * M_PI;
      } else if (ori > endOri + M_PI / 2) {
        ori -= 2 * M_PI;
      } 
    }

    float relTime = (ori - startOri) / (endOri - startOri);
    point.intensity = scanID + scanPeriod * relTime;//eg: 13.0898
       // cout<<point.intensity<<endl;

    if (imuPointerLast >= 0) {//默认值是-1，无imu数据输入
      float pointTime = relTime * scanPeriod;
      while (imuPointerFront != imuPointerLast) {
        if (timeScanCur + pointTime < imuTime[imuPointerFront]) {
          break;
        }
        imuPointerFront = (imuPointerFront + 1) % imuQueLength;
      }

      if (timeScanCur + pointTime > imuTime[imuPointerFront]) {
        imuRollCur = imuRoll[imuPointerFront];
        imuPitchCur = imuPitch[imuPointerFront];
        imuYawCur = imuYaw[imuPointerFront];

        imuVeloXCur = imuVeloX[imuPointerFront];
        imuVeloYCur = imuVeloY[imuPointerFront];
        imuVeloZCur = imuVeloZ[imuPointerFront];

        imuShiftXCur = imuShiftX[imuPointerFront];
        imuShiftYCur = imuShiftY[imuPointerFront];
        imuShiftZCur = imuShiftZ[imuPointerFront];
      } else {
        int imuPointerBack = (imuPointerFront + imuQueLength - 1) % imuQueLength;
        float ratioFront = (timeScanCur + pointTime - imuTime[imuPointerBack]) 
                         / (imuTime[imuPointerFront] - imuTime[imuPointerBack]);
        float ratioBack = (imuTime[imuPointerFront] - timeScanCur - pointTime) 
                        / (imuTime[imuPointerFront] - imuTime[imuPointerBack]);

        imuRollCur = imuRoll[imuPointerFront] * ratioFront + imuRoll[imuPointerBack] * ratioBack;
        imuPitchCur = imuPitch[imuPointerFront] * ratioFront + imuPitch[imuPointerBack] * ratioBack;
        if (imuYaw[imuPointerFront] - imuYaw[imuPointerBack] > M_PI) {
          imuYawCur = imuYaw[imuPointerFront] * ratioFront + (imuYaw[imuPointerBack] + 2 * M_PI) * ratioBack;
        } else if (imuYaw[imuPointerFront] - imuYaw[imuPointerBack] < -M_PI) {
          imuYawCur = imuYaw[imuPointerFront] * ratioFront + (imuYaw[imuPointerBack] - 2 * M_PI) * ratioBack;
        } else {
          imuYawCur = imuYaw[imuPointerFront] * ratioFront + imuYaw[imuPointerBack] * ratioBack;
        }

        imuVeloXCur = imuVeloX[imuPointerFront] * ratioFront + imuVeloX[imuPointerBack] * ratioBack;
        imuVeloYCur = imuVeloY[imuPointerFront] * ratioFront + imuVeloY[imuPointerBack] * ratioBack;
        imuVeloZCur = imuVeloZ[imuPointerFront] * ratioFront + imuVeloZ[imuPointerBack] * ratioBack;

        imuShiftXCur = imuShiftX[imuPointerFront] * ratioFront + imuShiftX[imuPointerBack] * ratioBack;
        imuShiftYCur = imuShiftY[imuPointerFront] * ratioFront + imuShiftY[imuPointerBack] * ratioBack;
        imuShiftZCur = imuShiftZ[imuPointerFront] * ratioFront + imuShiftZ[imuPointerBack] * ratioBack;
      }
      if (i == 0) {
        imuRollStart = imuRollCur;
        imuPitchStart = imuPitchCur;
        imuYawStart = imuYawCur;

        imuVeloXStart = imuVeloXCur;
        imuVeloYStart = imuVeloYCur;
        imuVeloZStart = imuVeloZCur;

        imuShiftXStart = imuShiftXCur;
        imuShiftYStart = imuShiftYCur;
        imuShiftZStart = imuShiftZCur;
      } else {
        ShiftToStartIMU(pointTime);
        VeloToStartIMU();
        TransformToStartIMU(&point);
      }
    }
    laserCloudScans[scanID].push_back(point);//将一帧点云分成16根线的点云
  }
  cloudSize = count;

  pcl::PointCloud<PointType>::Ptr laserCloud(new pcl::PointCloud<PointType>());
  for (int i = 0; i < N_SCANS; i++) {
    *laserCloud += laserCloudScans[i];//一帧点云
  }
  int scanCount = -1;
  for (int i = 5; i < cloudSize - 5; i++) {//取所有点左右相邻的各5个点求曲率//可以通过调节点的数目让曲率计算更加准确
    float diffX = laserCloud->points[i - 5].x + laserCloud->points[i - 4].x //原来为y
                + laserCloud->points[i - 3].x + laserCloud->points[i - 2].x
                + laserCloud->points[i - 1].x - 10 * laserCloud->points[i].x
                + laserCloud->points[i + 1].x + laserCloud->points[i + 2].x
                + laserCloud->points[i + 3].x + laserCloud->points[i + 4].x
                + laserCloud->points[i + 5].x;
    float diffY = laserCloud->points[i - 5].y + laserCloud->points[i - 4].y //z
                + laserCloud->points[i - 3].y + laserCloud->points[i - 2].y
                + laserCloud->points[i - 1].y - 10 * laserCloud->points[i].y
                + laserCloud->points[i + 1].y + laserCloud->points[i + 2].y
                + laserCloud->points[i + 3].y + laserCloud->points[i + 4].y
                + laserCloud->points[i + 5].y;
    float diffZ = laserCloud->points[i - 5].z + laserCloud->points[i - 4].z //x
                + laserCloud->points[i - 3].z + laserCloud->points[i - 2].z
                + laserCloud->points[i - 1].z - 10 * laserCloud->points[i].z
                + laserCloud->points[i + 1].z + laserCloud->points[i + 2].z
                + laserCloud->points[i + 3].z + laserCloud->points[i + 4].z
                + laserCloud->points[i + 5].z;
    cloudCurvature[i] = diffX * diffX + diffY * diffY + diffZ * diffZ;//！！点云曲率
    cloudSortInd[i] = i;
    cloudNeighborPicked[i] = 0;
    cloudLabel[i] = 0;

    if (int(laserCloud->points[i].intensity) != scanCount) {
      scanCount = int(laserCloud->points[i].intensity);

      if (scanCount > 0 && scanCount < N_SCANS) {
        scanStartInd[scanCount] = i + 5;
        scanEndInd[scanCount - 1] = i - 5;
        //  cout<<"scanStartInd"<<scanCount<<":"<<scanStartInd[scanCount]<<endl;间隔为300
          //cout<<"scanEndInd"<<scanCount-1<<":"<<scanEndInd[scanCount - 1]<<endl;

      }
    }
  }
  scanStartInd[0] = 5;
  scanEndInd.back() = cloudSize - 5;

  for (int i = 5; i < cloudSize - 6; i++) {
    float diffX = laserCloud->points[i + 1].x - laserCloud->points[i].x;
    float diffY = laserCloud->points[i + 1].y - laserCloud->points[i].y;
    float diffZ = laserCloud->points[i + 1].z - laserCloud->points[i].z;
    float diff = diffX * diffX + diffY * diffY + diffZ * diffZ;

    if (diff > 0.1) {

      float depth1 = sqrt(laserCloud->points[i].x * laserCloud->points[i].x +
                     laserCloud->points[i].y * laserCloud->points[i].y +
                     laserCloud->points[i].z * laserCloud->points[i].z);//X(k,i)

      float depth2 = sqrt(laserCloud->points[i + 1].x * laserCloud->points[i + 1].x +
                     laserCloud->points[i + 1].y * laserCloud->points[i + 1].y +
                     laserCloud->points[i + 1].z * laserCloud->points[i + 1].z);

      if (depth1 > depth2) {
        diffX = laserCloud->points[i + 1].x - laserCloud->points[i].x * depth2 / depth1;
        diffY = laserCloud->points[i + 1].y - laserCloud->points[i].y * depth2 / depth1;
        diffZ = laserCloud->points[i + 1].z - laserCloud->points[i].z * depth2 / depth1;

        if (sqrt(diffX * diffX + diffY * diffY + diffZ * diffZ) / depth2 < 0.05) {//0.1
          cloudNeighborPicked[i - 5] = 1;//周围点，在后面不考虑作为特征点
          cloudNeighborPicked[i - 4] = 1;
          cloudNeighborPicked[i - 3] = 1;
          cloudNeighborPicked[i - 2] = 1;
          cloudNeighborPicked[i - 1] = 1;
          cloudNeighborPicked[i] = 1;
        }
      } else {
        diffX = laserCloud->points[i + 1].x * depth1 / depth2 - laserCloud->points[i].x;
        diffY = laserCloud->points[i + 1].y * depth1 / depth2 - laserCloud->points[i].y;
        diffZ = laserCloud->points[i + 1].z * depth1 / depth2 - laserCloud->points[i].z;

        if (sqrt(diffX * diffX + diffY * diffY + diffZ * diffZ) / depth1 < 0.05) {//0.1
          cloudNeighborPicked[i + 1] = 1;
          cloudNeighborPicked[i + 2] = 1;
          cloudNeighborPicked[i + 3] = 1;
          cloudNeighborPicked[i + 4] = 1;
          cloudNeighborPicked[i + 5] = 1;
          cloudNeighborPicked[i + 6] = 1;
        }
      }
    }

    float diffX2 = laserCloud->points[i].x - laserCloud->points[i - 1].x;
    float diffY2 = laserCloud->points[i].y - laserCloud->points[i - 1].y;
    float diffZ2 = laserCloud->points[i].z - laserCloud->points[i - 1].z;
    float diff2 = diffX2 * diffX2 + diffY2 * diffY2 + diffZ2 * diffZ2;

    float dis = laserCloud->points[i].x * laserCloud->points[i].x
              + laserCloud->points[i].y * laserCloud->points[i].y
              + laserCloud->points[i].z * laserCloud->points[i].z;

    if (diff > 0.0002 * dis && diff2 > 0.0002 * dis) {
      cloudNeighborPicked[i] = 1;
    }
  }


  pcl::PointCloud<PointType> cornerPointsSharp;
  pcl::PointCloud<PointType> cornerPointsLessSharp;
  pcl::PointCloud<PointType> surfPointsFlat;
  pcl::PointCloud<PointType> surfPointsLessFlat;

  for (int i = 0; i < N_SCANS; i++) {//将每一线划分为等间距的6段分别处理
    pcl::PointCloud<PointType>::Ptr surfPointsLessFlatScan(new pcl::PointCloud<PointType>);
    for (int j = 0; j < 6; j++) {
      int sp = (scanStartInd[i] * (6 - j)  + scanEndInd[i] * j) / 6;//分段，每一段的起点
      int ep = (scanStartInd[i] * (5 - j)  + scanEndInd[i] * (j + 1)) / 6 - 1;//分段，每一段的终点

      for (int k = sp + 1; k <= ep; k++) {//在每一段升序排列
        for (int l = k; l >= sp + 1; l--) {
          if (cloudCurvature[cloudSortInd[l]] < cloudCurvature[cloudSortInd[l - 1]]) {
            int temp = cloudSortInd[l - 1];
            cloudSortInd[l - 1] = cloudSortInd[l];
            cloudSortInd[l] = temp;
          }
        }
      }

      int largestPickedNum = 0;
      for (int k = ep; k >= sp; k--) {//主要用于处理曲率最大的20个位置点，其他的在后续处理
        int ind = cloudSortInd[k];//索引值并未实际值
        if (cloudNeighborPicked[ind] == 0 &&
            cloudCurvature[ind] > 0.1) {//作为和周围点的判别，周围点不作为关键点

          largestPickedNum++;
          if (largestPickedNum <= 2) {
            cloudLabel[ind] = 2;
            cornerPointsSharp.push_back(laserCloud->points[ind]);
            cornerPointsLessSharp.push_back(laserCloud->points[ind]);
          } else if (largestPickedNum <= 20) {
            cloudLabel[ind] = 1;//感觉是判断曲率大小赋不同权值
            cornerPointsLessSharp.push_back(laserCloud->points[ind]);
          } else {
            break;//结束for循环
          }

          cloudNeighborPicked[ind] = 1;//把曲率最大的20也赋值为1和曲率小的等价，不再考虑为特征点
          for (int l = 1; l <= 5; l++) {//将周围大的五个点曲率在0.05以下的置为1，不再考虑为特征点
            float diffX = laserCloud->points[ind + l].x
                        - laserCloud->points[ind + l - 1].x;
            float diffY = laserCloud->points[ind + l].y
                        - laserCloud->points[ind + l - 1].y;
            float diffZ = laserCloud->points[ind + l].z
                        - laserCloud->points[ind + l - 1].z;
            if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05) {
              break;
            }

            cloudNeighborPicked[ind + l] = 1;//将周围小的五个点曲率在0.05以下的置为1，不再考虑为特征点
          }
          for (int l = -1; l >= -5; l--) {
            float diffX = laserCloud->points[ind + l].x
                        - laserCloud->points[ind + l + 1].x;
            float diffY = laserCloud->points[ind + l].y
                        - laserCloud->points[ind + l + 1].y;
            float diffZ = laserCloud->points[ind + l].z
                        - laserCloud->points[ind + l + 1].z;
            if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05) {
              break;
            }

            cloudNeighborPicked[ind + l] = 1;
          }
        }
      }

      int smallestPickedNum = 0;
      for (int k = sp; k <= ep; k++) {
        int ind = cloudSortInd[k];
        if (cloudNeighborPicked[ind] == 0 &&
            cloudCurvature[ind] < 0.1) {

          cloudLabel[ind] = -1;//和1，2对比
          surfPointsFlat.push_back(laserCloud->points[ind]);
            //cout<<laserCloud->points[ind]<<endl;

          smallestPickedNum++;
          if (smallestPickedNum >= 4) {
            break;
          }

          cloudNeighborPicked[ind] = 1;
          for (int l = 1; l <= 5; l++) {
            float diffX = laserCloud->points[ind + l].x
                        - laserCloud->points[ind + l - 1].x;
            float diffY = laserCloud->points[ind + l].y
                        - laserCloud->points[ind + l - 1].y;
            float diffZ = laserCloud->points[ind + l].z
                        - laserCloud->points[ind + l - 1].z;
            if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05) {
              break;
            }

            cloudNeighborPicked[ind + l] = 1;
          }
          for (int l = -1; l >= -5; l--) {
            float diffX = laserCloud->points[ind + l].x
                        - laserCloud->points[ind + l + 1].x;
            float diffY = laserCloud->points[ind + l].y
                        - laserCloud->points[ind + l + 1].y;
            float diffZ = laserCloud->points[ind + l].z
                        - laserCloud->points[ind + l + 1].z;
            if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05) {
              break;
            }

            cloudNeighborPicked[ind + l] = 1;
          }
        }
      }

      for (int k = sp; k <= ep; k++) {
        if (cloudLabel[k] <= 0) {

          surfPointsLessFlatScan->push_back(laserCloud->points[k]);
        }
      }
    }

    pcl::PointCloud<PointType> surfPointsLessFlatScanDS;
    pcl::VoxelGrid<PointType> downSizeFilter;
    downSizeFilter.setInputCloud(surfPointsLessFlatScan);
    downSizeFilter.setLeafSize(0.2, 0.2, 0.2);//三维网格滤波，网格尺度0.2m×0.2m×0.2m
    downSizeFilter.filter(surfPointsLessFlatScanDS);

    surfPointsLessFlat += surfPointsLessFlatScanDS;//平面上的点
  }

  sensor_msgs::PointCloud2 laserCloudOutMsg;//发布原始点云，发布到velodyne_cloud_2上
  pcl::toROSMsg(*laserCloud, laserCloudOutMsg);
  laserCloudOutMsg.header.stamp = laserCloudMsg->header.stamp;
  laserCloudOutMsg.header.frame_id = "/camera";
  pubLaserCloud.publish(laserCloudOutMsg);

  sensor_msgs::PointCloud2 cornerPointsSharpMsg;//发布最关键边角点，每帧点云的每根线分成6部分，每部分都有两个
  pcl::toROSMsg(cornerPointsSharp, cornerPointsSharpMsg);
   // std::cout<<"corner:"<<cornerPointsSharp.size()<<std::endl;//输出关键点的数目
  cornerPointsSharpMsg.header.stamp = laserCloudMsg->header.stamp;
  cornerPointsSharpMsg.header.frame_id = "/camera";
  pubCornerPointsSharp.publish(cornerPointsSharpMsg);//发布出来是laser_cloud_sharp

  sensor_msgs::PointCloud2 cornerPointsLessSharpMsg;//发布关键边角点，每帧点云的每根线分成6部分，每部分都有20个
  pcl::toROSMsg(cornerPointsLessSharp, cornerPointsLessSharpMsg);
   // std::cout<<"corner_more:"<<cornerPointsLessSharp.size()<<std::endl;
  cornerPointsLessSharpMsg.header.stamp = laserCloudMsg->header.stamp;
  cornerPointsLessSharpMsg.header.frame_id = "/camera";
  pubCornerPointsLessSharp.publish(cornerPointsLessSharpMsg);

  sensor_msgs::PointCloud2 surfPointsFlat2;//平面点,比较平的laser_cloud_flat
  pcl::toROSMsg(surfPointsFlat, surfPointsFlat2);
  surfPointsFlat2.header.stamp = laserCloudMsg->header.stamp;
  surfPointsFlat2.header.frame_id = "/camera";
  pubSurfPointsFlat.publish(surfPointsFlat2);
   // std::cout<<"surf:"<<surfPointsFlat.size()<<std::endl;
  sensor_msgs::PointCloud2 surfPointsLessFlat2;//次平面点，发布laser_cloud_less_flat
  pcl::toROSMsg(surfPointsLessFlat, surfPointsLessFlat2);
  surfPointsLessFlat2.header.stamp = laserCloudMsg->header.stamp;
  surfPointsLessFlat2.header.frame_id = "/camera";
  pubSurfPointsLessFlat.publish(surfPointsLessFlat2);
    //std::cout<<"surf_more:"<<surfPointsLessFlat.size()<<std::endl;
  pcl::PointCloud<pcl::PointXYZ> imuTrans(4, 1);
  imuTrans.points[0].x = imuPitchStart;
  imuTrans.points[0].y = imuYawStart;
  imuTrans.points[0].z = imuRollStart;

  imuTrans.points[1].x = imuPitchCur;
  imuTrans.points[1].y = imuYawCur;
  imuTrans.points[1].z = imuRollCur;

  imuTrans.points[2].x = imuShiftFromStartXCur;
  imuTrans.points[2].y = imuShiftFromStartYCur;
  imuTrans.points[2].z = imuShiftFromStartZCur;

  imuTrans.points[3].x = imuVeloFromStartXCur;
  imuTrans.points[3].y = imuVeloFromStartYCur;
  imuTrans.points[3].z = imuVeloFromStartZCur;

  sensor_msgs::PointCloud2 imuTransMsg;//通过点云数据形式发布出来的。
  pcl::toROSMsg(imuTrans, imuTransMsg);
  imuTransMsg.header.stamp = laserCloudMsg->header.stamp;
  imuTransMsg.header.frame_id = "/camera";
  pubImuTrans.publish(imuTransMsg);
}

void imuHandler(const sensor_msgs::Imu::ConstPtr& imuIn)
{
    pcl::PointCloud<PointType>::Ptr laserCloudSurfLast(new pcl::PointCloud<PointType>());

    /**
     * 改变imu的坐标
     */
/*

    try
    {
        static tf::TransformListener tf_listener1;
        tf::StampedTransform transform1;
        tf_listener1.waitForTransform("/velodyne_front", "/imu_link", ros::Time(0), ros::Duration(1.0));
        tf_listener1.lookupTransform("/velodyne_front", "/imu_link", ros::Time(0), transform1);
        //姿态变换矩阵4x4,用于将每一帧点云按照机器人的位姿进行坐标转换

*/
        double roll, pitch, yaw;
        //double roll1, pitch1, yaw1;
        //Eigen::Vector3f temp;
     //   double  v_x,v_y,v_z;
        tf::Quaternion orientation;
        // tf::Quaternion transformImu;
        //transformImu.Quaternion.x=0.0;
        tf::quaternionMsgToTF(imuIn->orientation, orientation);
    tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw);
      //  orientation=transform1.getRotation()*orientation;
      //  ((tf::Matrix3x3(transform1.getRotation()))*(tf::Matrix3x3(orientation))).getRPY(roll, pitch, yaw);
       /* v_x=
                tf::Matrix3x3(transform1.getRotation())[0][0]* imuIn->linear_acceleration.x+
        tf::Matrix3x3(transform1.getRotation())[0][1]* imuIn->linear_acceleration.y+
        tf::Matrix3x3(transform1.getRotation())[0][2]* imuIn->linear_acceleration.z;
        v_y=
                tf::Matrix3x3(transform1.getRotation())[1][0]* imuIn->linear_acceleration.x+
                tf::Matrix3x3(transform1.getRotation())[1][1]* imuIn->linear_acceleration.y+
                tf::Matrix3x3(transform1.getRotation())[1][2]* imuIn->linear_acceleration.z;
        v_z=
                tf::Matrix3x3(transform1.getRotation())[2][0]* imuIn->linear_acceleration.x+
                tf::Matrix3x3(transform1.getRotation())[2][1]* imuIn->linear_acceleration.y+
                tf::Matrix3x3(transform1.getRotation())[2][2]* imuIn->linear_acceleration.z;

*/
      //  float accX = v_y - sin(roll) * cos(pitch) * 9.81;
     //   float accY = v_z - cos(roll) * cos(pitch) * 9.81;
      //  float accZ = v_x + sin(pitch) * 9.81;

        //imuIn->linear_acceleration= vectorChange*imuIn->linear_acceleration;
       float accX = imuIn->linear_acceleration.y - sin(roll) * cos(pitch) * 9.81;
        float accY = imuIn->linear_acceleration.z - cos(roll) * cos(pitch) * 9.81;
       float accZ = imuIn->linear_acceleration.x + sin(pitch) * 9.81;

        imuPointerLast = (imuPointerLast + 1) % imuQueLength;
       // std::cout<<"yaw:"<<yaw<<" ";
        //std::cout<<"pitch:"<<pitch<<" ";
      //  std::cout<<"roll:"<<roll<<std::endl;
        imuTime[imuPointerLast] = imuIn->header.stamp.toSec();
        imuRoll[imuPointerLast] = roll;
        imuPitch[imuPointerLast] = pitch;
        imuYaw[imuPointerLast] = yaw;
        imuAccX[imuPointerLast] = accX;
        imuAccY[imuPointerLast] = accY;
        imuAccZ[imuPointerLast] = accZ;


        AccumulateIMUShift();

 /*}
    catch (tf::TransformException ex)
    {
        ROS_INFO("%s", ex.what());
        ros::Duration(0.01).sleep();
    }*/

   // boost::shared_ptr<tf2_ros::Buffer> tf2_;
   // tf2_->transform(*imuIn, imu_out, target_frame_);



}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "scanRegistration");
  ros::NodeHandle nh;
    nh.param<std::string>("target_frame", target_frame_, "/imu");
  ros::Subscriber subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2> 
                                  ("/velodyne_points", 2, laserCloudHandler);//点云输入
  ros::Subscriber subImu = nh.subscribe<sensor_msgs::Imu> ("/imu/data", 50, imuHandler);

  pubLaserCloud = nh.advertise<sensor_msgs::PointCloud2>
                                 ("/velodyne_cloud_2", 2);

  pubCornerPointsSharp = nh.advertise<sensor_msgs::PointCloud2>
                                        ("/laser_cloud_sharp", 2);

  pubCornerPointsLessSharp = nh.advertise<sensor_msgs::PointCloud2>
                                            ("/laser_cloud_less_sharp", 2);

  pubSurfPointsFlat = nh.advertise<sensor_msgs::PointCloud2>
                                       ("/laser_cloud_flat", 2);

  pubSurfPointsLessFlat = nh.advertise<sensor_msgs::PointCloud2>
                                           ("/laser_cloud_less_flat", 2);

  pubImuTrans = nh.advertise<sensor_msgs::PointCloud2> ("/imu_trans", 5);

  ros::spin();

  return 0;
}


