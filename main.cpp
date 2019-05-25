#include"ArmorFind.h"
#include "ArmorSet.h"
#include "anglesolve.h"
#include<RMVideoCapture.h>
#include <chrono>
#include "serial.h"
#include "graph.h"
#include "time.h"
#include <iostream>

using namespace std;
int main()
{
    namedWindow("frame");
    //创建线程并且启动线程
    pthread_mutex_init(&mutex, NULL);
    pthread_t serial_tid;
    int err = pthread_create(&serial_tid, NULL, thread_serial, NULL);
    if(err!=0)
    {
      cout<<"failed"<<endl;
    }

    float yaw, pitch;//解算角度，为相对值
    int time_num;//储存图像处理的时间，单位为5ms
    Point2f center_point;//储存目标中心点

    //用于在没有启动的时候发送，保证稳定
    float angle_gain_yaw = 0.0001;
    float angle_gain_pitch = -0.001;
    float angle_gain = 0;

    AST::Mode = 1;  //Mode =0;在红方时打开自瞄       Mode =1;在蓝方时打开自瞄
    AST::armormode = 1; //mode =1;小装甲板：步兵，工程         mode=11;大装甲板：英雄，哨兵
    char temp_status = 0x01;//初始状态为关闭状态
    char temp_color = 0x02;//初始阵营信息为无

    //初始化摄像头
    RMVideoCapture cap("/dev/video0",3);
    cap.setVideoFormat(640,480,1);
    cap.setVideoFPS(120);
    cap.setExposureTime(0,16);
    cap.startStream();
    cap.info();

    while (1)
    {
        const int64 start = getTickCount();
        Mat frame;
        cap >> frame;
        //获取阵营和状态值
        temp_color = get_color();
        temp_status = get_status();
        if(temp_color == 0x01 || temp_color == 0x02)//判断阵营
        {
            if(temp_color == 0x01)
            {
                AST::Mode = 0;
                cout<<"red"<<endl;
            }
            else
            {
                AST::Mode = 1;
                cout<<"blue"<<endl;
            }
            if(temp_status == 0x01 || temp_status == 0x02 )//判断状态
            {
                if(temp_status == 0x01)
                {
                    AST::armormode = 1;
                }
                else
                {
                    AST::armormode = 11;
                }

                Mat img_hsv, img_gray;
                Armorfind armor;
                vector<Point2f>  points;

                bool label = 0;
                points =armor.Armorfinds(frame, img_hsv, img_gray,label);
                double deal_time = (getTickCount()-start)/getTickFrequency();
                time_num = (int) (deal_time/0.005);
                time_num += ANGLE_BUFF_BASE;
                double duration =1/deal_time;
                //cout<<time_num<<endl;
                cout<<"FPS: "<<duration<<endl;

                if (!points.empty())
                {
                    center_point = angle_solve(points, yaw, pitch,AST::armormode);
                    updata_angle(yaw, pitch, time_num);
                }
            }
            else
            {
                time_num = 0;
                updata_angle(angle_gain_yaw, angle_gain_pitch, time_num);
                cout<<"没有状态信息"<<endl;
                imshow("frame", frame);
                sleep(0.01);
            }
        }
        else
        {
            //关闭状态时发送伪0数
            time_num = 0;
            updata_angle(angle_gain_yaw, angle_gain_pitch, time_num);
            cout<<"没有阵营信息"<<endl;
            imshow("frame", frame);
        }

        //保证每次发送的值都不同，才会使串口发送
        angle_gain = angle_gain_pitch;
        angle_gain_pitch = angle_gain_yaw;
        angle_gain_yaw = angle_gain;

        if((char)waitKey(1)=='q')
       {
            cap.closeStream();
            pthread_mutex_lock(&mutex);
            serial_over = 0;
            pthread_mutex_unlock(&mutex);//等待另一个线程结束
            break;
       }

    }
    return 0;
}
