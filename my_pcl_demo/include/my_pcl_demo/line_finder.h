#ifndef LINE_FINDER_H
#define LINE_FINDER_H
#include"opencv2/opencv.hpp"
#include"stdio.h"
using namespace cv;
#define PI  3.1415926
class line_finder
{
    // 直线对应的点参数向量
    std::vector<cv::Vec4i> lines;
    //步长
    double deltaRho;
    double deltaTheta;
    // 判断是直线的最小投票数
    int minVote;
    // 判断是直线的最小长度
    double minLength;
    // 同一条直线上点之间的距离容忍度
    double maxGap;
public:
//    line_finder();

    //初始化
          line_finder();
          // 设置步长
          void setAccResolution(double dRho, double dTheta) ;
          // 设置最小投票数
          void setMinVote(int minv);
          // 设置最小线段长度和线段间距容忍度
          void setLineLengthAndGap(double length, double gap) ;

          //寻找线段
          std::vector<cv::Vec4i> findLines(cv::Mat& binary) ;

          // 画线段
          void drawDetectedLines(cv::Mat &image, cv::Scalar color=cv::Scalar(0));
          void Line_filter(float xielv);

};

#endif // LINE_FINDER_H
