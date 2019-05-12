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

    //初始化卡尔漫滤波
    RNG rng;
    const int stateNum=4;
    const int measureNum=2;
    KalmanFilter KF(stateNum, measureNum, 0);
    KF.transitionMatrix = (Mat_<float>(4,4) << 1,0,1,0,0,1,0,1,0,0,1,0,0,0,0,1);
    setIdentity(KF.measurementMatrix);
    setIdentity(KF.processNoiseCov, Scalar::all(1e-5));
    setIdentity(KF.measurementNoiseCov, Scalar::all(1e-1));
    setIdentity(KF.errorCovPost, Scalar::all(1));
    rng.fill(KF.statePost, RNG::UNIFORM,-10,10);
    Mat measurement = Mat::zeros(measureNum, 1, CV_32F);

    float yaw, pitch;
    float yaw0, pitch0;
    float yaw1, pitch1;
    float yaw_pre, pitch_pre;
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
        const int64 start = getTickCount();
        Mat frame;
        Mat img_hsv, img_gray;
        Armorfind armor;
        cap >> frame;
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

            Mat prediction = KF.predict();
            yaw_pre = prediction.at<float>(0);
            pitch_pre = prediction.at<float>(1);
            updata_angle(yaw_pre, pitch_pre);
            measurement.at<float>(0) = yaw1;
            measurement.at<float>(1) = pitch1;
            KF.correct(measurement);
            cout << "recive:  yaw0: " << yaw0 << " pitch0: " << pitch0 << endl;
            cout << "solve:  yaw: " << yaw << " pitch: " << pitch << endl;
            cout << "send:  yaw1: " << yaw1 << " pitch1: " << pitch1 << endl;
            cout << "predict:  yaw_pre: " << yaw_pre << " pitch_pre: " << pitch_pre << endl;

            graph(M_graph, yaw0, yaw_pre, num);

        }
        //cout<<exp<<endl;
        imshow("graph", M_graph);
        double duration =(getTickCount()-start)/getTickFrequency();
        cout<<duration<<endl;
        if((char)waitKey(10)=='q')
        {
            //cap.closeStream();
            break;
        }
    }
    return 0;

}
