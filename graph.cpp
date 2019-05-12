#include "graph.h"

void graph(Mat src, float yaw, float pitch, int &num)
{
    if(num > 1199)
    {
        num = 25;
    }

    if(num == 25)
    {
        flash_graph(src);
    }

    Point2f target_yaw;
    target_yaw.x = num;
    target_yaw.y = 360 - yaw * 4;
    circle(src, target_yaw, 1, Scalar(255, 0, 0), -1, CV_AA, 0);

    Point2f target_pitch;
    target_pitch.x = num;
    target_pitch.y = 360 - pitch * 4;
    circle(src, target_pitch, 1, Scalar(0, 255, 0), -1, CV_AA, 0);

    num +=2;



}

void flash_graph(Mat src)
{
    src = Scalar(255, 255, 255);

    //XY
    line(src, Point(0, 360), Point(1199, 360), Scalar(0, 0, 0), 1);
    line(src, Point(25, 0), Point(25, 699), Scalar(0, 0, 0), 1);


}
