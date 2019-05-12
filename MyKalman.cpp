
#include"MyKalman.h"
#include<opencv.hpp>
using namespace cv;
using namespace std;

//这些矩阵在预测过程中不断变化
Mat statePre;           //!< predicted state (x'(k)): x(k)=A*x(k-1)+B*u(k) 预测状态向量a×1
Mat statePost;          //!< corrected state (x(k)): x(k)=x'(k)+K(k)*(z(k)-H*x'(k)) 估计状态向量a×1
Mat transitionMatrix;   //!< state transition matrix (A) 状态转移矩阵A，需人为赋值，全程不变（可以含变量）a×a
Mat measurementMatrix;  //!< measurement matrix (H) 测量矩阵H，大小由函数根据状态向量和测量向量自动算出，需要人为赋值 全程不变 b×a
Mat processNoiseCov;    //!< process noise covariance matrix (Q) 系统噪声协方差矩阵，全程不变 a×a
Mat measurementNoiseCov;//!< measurement noise covariance matrix(R) 测量噪声协方差矩阵，全程不变 b×b
Mat errorCovPre;        //!< priori error estimate covariance matrix (P'(k)):P'(k)=A*P(k-1)*At + Q) 先验协方差矩阵
Mat gain;               //!< Kalman gain matrix (K(k)): K(k)=P'(k)*Ht*inv(H*P'(k)*Ht+R) 增益矩阵
Mat errorCovPost;       //!< posteriori error estimate covariance matrix (P(k)): P(k)=(I-K(k)*H)*P'(k) 后验协方差矩阵
Mat transitionMatrix_predict;//预测用

Mat temp1;
Mat temp2;
Mat temp3;
Mat temp4;
Mat temp5;#include<opencv.hpp>
using namespace cv;#include<opencv.hpp>
using namespace cv;
using namespace std;

//这些矩阵在预测过程中不断变化
Mat statePre;           //!< predicted state (x'(k)): x(k)=A*x(k-1)+B*u(k) 预测状态向量a×1
Mat statePost;          //!< corrected state (x(k)): x(k)=x'(k)+K(k)*(z(k)-H*x'(k)) 估计状态向量a×1
Mat transitionMatrix;   //!< state transition matrix (A) 状态转移矩阵A，需人为赋值，全程不变（可以含变量）a×a
Mat measurementMatrix;  //!< measurement matrix (H) 测量矩阵H，大小由函数根据状态向量和测量向量自动算出，需要人为赋值 全程不变 b×a
Mat processNoiseCov;    //!< process noise covariance matrix (Q) 系统噪声协方差矩阵，全程不变 a×a
Mat measurementNoiseCov;//!< measurement noise covariance matrix(R) 测量噪声协方差矩阵，全程不变 b×b
Mat errorCovPre;        //!< priori error estimate covariance matrix (P'(k)):P'(k)=A*P(k-1)*At + Q) 先验协方差矩阵
Mat gain;               //!< Kalman gain matrix (K(k)): K(k)=P'(k)*Ht*inv(H*P'(k)*Ht+R) 增益矩阵
Mat errorCovPost;       //!< posteriori error estimate covariance matrix (P(k)): P(k)=(I-K(k)*H)*P'(k) 后验协方差矩阵
Mat transitionMatrix_predict;//预测用

using namespace std;

//这些矩阵在预测过程中不断变化
Mat statePre;           //!< predicted state (x'(k)): x(k)=A*x(k-1)+B*u(k) 预测状态向量a×1
Mat statePost;          //!< corrected state (x(k)): x(k)=x'(k)+K(k)*(z(k)-H*x'(k)) 估计状态向量a×1
Mat transitionMatrix;   //!< state transition matrix (A) 状态转移矩阵A，需人为赋值，全程不变（可以含变量）a×a
Mat measurementMatrix;  //!< measurement matrix (H) 测量矩阵H，大小由函数根据状态向量和测量向量自动算出，需要人为赋值 全程不变 b×a
Mat processNoiseCov;    //!< process noise covariance matrix (Q) 系统噪声协方差矩阵，全程不变 a×a
Mat measurementNoiseCov;//!< measurement noise covariance matrix(R) 测量噪声协方差矩阵，全程不变 b×b
Mat errorCovPre;        //!< priori error estimate covariance matrix (P'(k)):P'(k)=A*P(k-1)*At + Q) 先验协方差矩阵
Mat gain;               //!< Kalman gain matrix (K(k)): K(k)=P'(k)*Ht*inv(H*P'(k)*Ht+R) 增益矩阵
Mat errorCovPost;       //!< posteriori error estimate covariance matrix (P(k)): P(k)=(I-K(k)*H)*P'(k) 后验协方差矩阵
Mat transitionMatrix_predict;//预测用


void MyInit(int DP, int MP)//参数初始化，只在预测的刚开始调用一次
{
    //规定矩阵大小
    statePre = Mat::zeros(DP, 1, CV_32F);
    statePost = Mat::zeros(DP, 1, CV_32F);
    transitionMatrix = Mat::eye(DP, DP, CV_32F);

    transitionMatrix_predict = Mat::eye(DP, DP, CV_32F);//预测用

    processNoiseCov = Mat::eye(DP, DP, CV_32F);
    measurementMatrix = Mat::zeros(MP, DP, CV_32F);
    measurementNoiseCov = Mat::eye(MP, MP, CV_32F);

    errorCovPre = Mat::zeros(DP, DP, CV_32F);
    errorCovPost = Mat::zeros(DP, DP, CV_32F);
    gain = Mat::zeros(DP, MP, CV_32F);

    temp1.create(DP, DP, CV_32F);
    temp2.create(MP, DP, CV_32F);
    temp3.create(MP, MP, CV_32F);
    temp4.create(MP, DP, CV_32F);
    temp5.create(MP, 1, CV_32F);


    //状态转换矩阵初始化
    setIdentity(measurementMatrix);//设置单元矩阵 对角线为1（默认）
    //过程噪声矩阵初始化
    setIdentity(processNoiseCov, Scalar::all(processNoise));//单元矩阵，对角线为1e-5
    //测量噪声初始化
    setIdentity(measurementNoiseCov, Scalar::all(measurementNoise));
    //先验方差初始化，因为最后会迭代到固定值，随便设置即可
    setIdentity(errorCovPost, Scalar::all(1));
}

void MyCorrect(double dt, Mat measurement)//更新状态
{
    //更新状态转换矩阵（运动模型改变时，随之改变）
    float* data1 = transitionMatrix.ptr<float>(0);
    data1[3] = dt;
    data1 = transitionMatrix.ptr<float>(1);
    data1[4] = dt;
    data1 = transitionMatrix.ptr<float>(2);
    data1[5] = dt;

    //仿照opencv自带卡尔曼滤波函数
    // update the state: x'(k) = A*x(k)
    statePre = transitionMatrix * statePost;

    // update error covariance matrices: temp1 = A*P(k)
    temp1 = transitionMatrix * errorCovPost;

    // P'(k) = temp1*At + Q
    gemm(temp1, transitionMatrix, 1, processNoiseCov, 1, errorCovPre, GEMM_2_T);

    // handle the case when there will be measurement before the next predict.
    statePre.copyTo(statePost);
    errorCovPre.copyTo(errorCovPost);

    // temp2 = H*P'(k)
    temp2 = measurementMatrix * errorCovPre;

    // temp3 = temp2*Ht + R
    gemm(temp2, measurementMatrix, 1, measurementNoiseCov, 1, temp3, GEMM_2_T);

    // temp4 = inv(temp3)*temp2 = Kt(k)
    solve(temp3, temp2, temp4, DECOMP_SVD);

    // K(k)
    gain = temp4.t();

    // temp5 = z(k) - H*x'(k)
    temp5 = measurement - measurementMatrix * statePre;

    // x(k) = x'(k) + K(k)*temp5
    statePost = statePre + gain * temp5;

    // P(k) = P'(k) - K(k)*temp2
    errorCovPost = errorCovPre - gain * temp2;
}

Point3f MyPredict(double dt)
{
    //更新预测所用预测矩阵，不同于状态转换矩阵（运动模型改变时，随之改变）
    float* data2 = transitionMatrix.ptr<float>(0);
    data2[3] = dt;
    data2 = transitionMatrix.ptr<float>(1);
    data2[4] = dt;
    data2 = transitionMatrix.ptr<float>(2);
    data2[5] = dt;


    Mat XYZ_M;//预测dt2时间后的状态向量

    XYZ_M = transitionMatrix * statePost;

    Point3f XYZ_temp;//储存预测位置

    XYZ_temp = Point3f(XYZ_M.at<float>(0), XYZ_M.at<float>(1), XYZ_M.at<float>(2));

    return XYZ_temp;
}

