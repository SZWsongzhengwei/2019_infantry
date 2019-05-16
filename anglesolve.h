#ifndef ANGLESOLVE_H
#define ANGLESOLVE_H
#define LIGHT_LENGTH 140;//装甲板灯条的实际物理高度
#define CAMERA_HEIGHT 100;//相机的光轴距离炮管的高度
#define FIX_X 6;

#include<opencv2/opencv.hpp>
#include<iostream>

using namespace cv;
using namespace std;

Point2f angle_solve(vector<Point2f> amor_4points, float &yaw, float &pitch);
/*@brief 计算云台需要移动的角度
*@param target_point Point2f类型 目标位置的像素坐标
*@param amor_point Point2f类型 实际位置的像素坐标
*@yaw double类型 存放计算得到的yaw轴角度
*@pitch double类型 存放计算得到的pitch轴角度
*/

double distance_solve(double pixel_distance);
/*@brief 粗略计算装甲板距离相机的距离
*@param amor_4points vector<Point2f>类型 装甲板在图像中的四个角的像素坐标，从左下角顺时针
*@return double类型 返回距离，单位为mm
*/

Point2f target_solve(double distance);
/*@brief 计算目标位置的像素坐标
*@param distance double类型 装甲板距离相机的距离，单位为mm
*@return void
*/

double point_distance(Point2f p1, Point2f p2);
/*@brief计算两个像素点之间的距离
 * @param p1 Point类型 第一个点
 * @param p2 Point类型 第二个点
 * @return double 返回两个点之间的距离
*/
#endif // ANGLESOLVE_H
