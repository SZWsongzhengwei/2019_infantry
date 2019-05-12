#ifndef MYKALMAN_H
#define MYKALMAN_H
/*三维匀速直线卡尔曼滤波，状态向量包含每个轴的位移，速度，测量向量为坐标点*/


#include<opencv.hpp>
#include<iostream>

//噪声参数
#define processNoise  1e-5
#define measurementNoise  1e-1

using namespace cv;
using namespace std;

void MyInit(int DP, int MP);//根据观测状态和测量参数的状态初始化参量

void MyCorrect(double dt, Mat measurement);//更新
//输入的点与上个输入点的时间差值
//观测矩阵

Point3f MyPredict(double dt);//预测
//预测dt2时间后的坐标位置


#endif // MYKALMAN_H
