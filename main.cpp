#include"ArmorFind.h"
#include "ArmorSet.h"
#include "MyKalman.h"
#include "anglesolve.h"
#include<RMVideoCapture.h>
#include <chrono>
#include "serial.h"
#include "graph.h"
#include "time.h"
int main()
{
    //graph put
    int num = 25;
    Mat M_graph(750, 1200, CV_8UC3, Scalar(255, 255, 255));

    pthread_mutex_init(&mutex, NULL);
    pthread_t serial_tid;
    sleep(2);

    int err = pthread_create(&serial_tid, NULL, thread_serial, NULL);
    if(err!=0)
    {
      cout<<"failed"<<endl;
    }

    double  exp=8 ;

    float yaw, pitch;
    float yaw0, pitch0;
    float yaw1, pitch1;
    Point2f center_point;
    AST::Mode = 1;
    AST::capmode = 1;

    RMVideoCapture cap("/dev/video0",3);
    cap.setVideoFormat(640,480,1);
    cap.setVideoFPS(120);
    cap.setExposureTime(0,16);
    cap.startStream();
    cap.info();
/*
    VideoCapture cap("/dev/video0");

    cap.set(CAP_PROP_EXPOSURE,1);
*/
    while (1)
    {
        //const int64 start = getTickCount();
        Mat frame;
        Mat img_hsv, img_gray;
        Armorfind armor;
        cap >> frame;


     //   imshow("123",frame);
      exp+=1;

        vector<Point2f>  points;
        points =armor.Armorfinds(frame, img_hsv, img_gray);
        //double FPS;
        if (!points.empty())
        {
            get_angle(yaw0, pitch0);
            center_point = angle_solve(points, yaw, pitch);
            circle(frame, center_point, 2, Scalar(255, 0, 0), -1);
            yaw1 = yaw0+yaw;
            pitch1 = pitch0+pitch;
            updata_angle(yaw1, pitch1);

            cout << "recive:  yaw0: " << yaw0 << " pitch0: " << pitch0 << endl;
            cout << "solve:  yaw: " << yaw << " pitch: " << pitch << endl;
            cout << "send:  yaw1: " << yaw1 << " pitch1: " << pitch1 << endl;



            graph(M_graph, pitch0, pitch1, num);

        }
        //cout<<exp<<endl;
        imshow("graph", M_graph);
        //double duration =1/((getTickCount()-start)/getTickFrequency());
        //cout<<duration<<endl;
        if((char)waitKey(10)=='q')
        {
            //cap.closeStream();
            break;
        }
    }
    return 0;

}
