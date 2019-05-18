#include"ArmorFind.h"
#include "ArmorSet.h"
#include "MyKalman.h"
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
    //graph put
    //int num = 25;
    //Mat M_graph(750, 1200, CV_8UC3, Scalar(255, 255, 255));

    pthread_mutex_init(&mutex, NULL);
    pthread_t serial_tid;
    sleep(2);

    int err = pthread_create(&serial_tid, NULL, thread_serial, NULL);
    if(err!=0)
    {
      cout<<"failed"<<endl;
    }

    float yaw, pitch;
    Point2f center_point;
    AST::Mode = 1;
    AST::capmode = 1;

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
        Mat img_hsv, img_gray;
        Armorfind armor;
        cap >> frame;
        vector<Point2f>  points;
        bool label=0;
        points =armor.Armorfinds(frame, img_hsv, img_gray,label);
        cout<<"种类"<<label<<endl;
        //double FPS;
        if (!points.empty())
        {
            center_point = angle_solve(points, yaw, pitch);
            updata_angle(yaw, pitch);

            //cout << "recive:  yaw0: " << yaw0 << " pitch0: " << pitch0 << endl;
            //cout << "solve:  yaw: " << yaw << " pitch: " << pitch << endl;
            //cout << "send:  yaw1: " << yaw1 << " pitch1: " << pitch1 << endl;
            //cout << "pre:  yaw_pre: " << yaw_pre<< " pitch_pre: " << pitch_pre << endl;
            //cout << "state:  yaw_state: " << yaw_state<< " pitch_state: " << pitch_state << endl;



        }
        //cout<<exp<<endl;
        //imshow("graph", M_graph);
        double duration =1/((getTickCount()-start)/getTickFrequency());
        cout<<duration<<endl;
        if((char)waitKey(5)=='q')
       {
            cap.closeStream();
            break;
        }
    }
    return 0;

}
