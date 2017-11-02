#include "line_finder.h"

line_finder::line_finder()
{
    deltaRho = 1;
    deltaTheta = PI/180;

   minVote = 10;
   minLength = 0;
   maxGap = 0;
}

void line_finder::setAccResolution(double dRho, double dTheta)
{
        deltaRho= dRho;
        deltaTheta= dTheta;
}

// 设置最小投票数
void line_finder::setMinVote(int minv)
{
        minVote= minv;
}
// 设置最小线段长度和线段间距容忍度
void line_finder::setLineLengthAndGap(double length, double gap)
{
        minLength= length;
        maxGap= gap;
}

std::vector<cv::Vec4i> line_finder::findLines(cv::Mat& binary)
{
        lines.clear();
        std::vector<cv::Vec4i>(lines).swap(lines);
        cv::Mat temp = binary;
        cv::HoughLinesP(temp,lines, deltaRho, deltaTheta, minVote,minLength, maxGap);
        return lines;
}

// 画线段
void line_finder::drawDetectedLines(cv::Mat &image, cv::Scalar color)
{
std::vector<cv::Vec4i>::const_iterator it2=lines.begin();
while (it2!=lines.end())
{
        cv::Point pt1((*it2)[0],(*it2)[1]);
        cv::Point pt2((*it2)[2],(*it2)[3]);

        cv::line( image, pt1, pt2, color);
        ++it2;
      }
}
void line_finder::Line_filter(float xielv)
{
    std::vector<cv::Vec4i>::iterator    Iter;
    std::vector<cv::Vec4i>::iterator    Iter_end;

    int x0;
    int y0 ;
    int x1 ;
    int y1 ;
    float x0_f=0.0;
    float y0_f=0.0;
    float x1_f=0.0;
    float y1_f=0.0;
    float xielv_temp=0.0;
    Iter = lines.begin();
    Iter_end = lines.end();
    int size = lines.size();
    for(int i=0; i<size;)
    {
        Iter = lines.begin() + i;
        x0 =(*Iter)[0];
        y0 = (*Iter)[1];
        x1 = (*Iter)[2];
        y1 = (*Iter)[3];
        x0_f = x0;
        y0_f = y0;
        x1_f = x1;
        y1_f = y1;
        if( (x0-x1) ==0)
        {
            i++;
            continue;
        }
      else
      {
            xielv_temp =(y1_f - y0_f)/(x1_f-x0_f);
           if( fabs(xielv_temp)< xielv)
           {
             Iter = lines.erase(Iter);//Iter为删除元素的下一个元素的迭代器
           }
      //即第一次这段语句后Iter 会是20，大家可以通过debug调试出来查看下数值
      }
     std::vector<cv::Vec4i>(lines).swap(lines);
     size = lines.size();
      if(Iter == lines.end()) //要控制迭代器不能超过整个容器
      {
            break;
      }
    }
}
