#include "my_math.h"

//my_math::my_math()
//{

//}

void Bubble_sort_int ( int r[], int n){
    int low = 0;
    int high= n -1; //设置变量的初始值
    int tmp,j;
    while (low < high) {
        for (j= low; j< high; ++j) //正向冒泡,找到最大者
            if (r[j]> r[j+1]) {
                tmp = r[j]; r[j]=r[j+1];r[j+1]=tmp;
            }
        --high;                 //修改high值, 前移一位
        for ( j=high; j>low; --j) //反向冒泡,找到最小者
            if (r[j]<r[j-1]) {
                tmp = r[j]; r[j]=r[j-1];r[j-1]=tmp;
            }
        ++low;                  //修改low值,后移一位
    }
}

void Bubble_sort_char( char r[], int n){
    int low = 0;
    int high= n -1; //设置变量的初始值
    int tmp,j;
    while (low < high) {
        for (j= low; j< high; ++j) //正向冒泡,找到最大者
            if (r[j]> r[j+1]) {
                tmp = r[j]; r[j]=r[j+1];r[j+1]=tmp;
            }
        --high;                 //修改high值, 前移一位
        for ( j=high; j>low; --j) //反向冒泡,找到最小者
            if (r[j]<r[j-1]) {
                tmp = r[j]; r[j]=r[j-1];r[j-1]=tmp;
            }
        ++low;                  //修改low值,后移一位
    }
}
