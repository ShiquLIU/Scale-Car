#include "opencv2/core.hpp"
#include <opencv2/core/utility.hpp>
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/cudaimgproc.hpp"
#include <iostream>
 
#include "ros/ros.h"
#include "um_acc/CarState_L.h"
#include <sstream>
#include <vector>
 
using namespace cv;
using namespace cv::cuda;
using namespace std;
 
/*Matx33f cameraMatrix(6.1357013269810250e+02, 0., 3.1950000000000000e+02,
    0., 6.1357013269810250e+02, 2.3950000000000000e+02,
    0., 0., 1.);
 
static const double arr[] = {6.2893659093366083e-02, -1.7290676377819170e-01,
    0., 0., -1.9864913372558086e-02};
vector<double> distCoeffs (arr, arr + sizeof(arr) / sizeof(arr[0]) );*/
 
int main(int argc, char **argv)
{
    VideoCapture cap(0); // open the default camera
    if(!cap.isOpened())  // check if we succeeded
        return -1;
 
    ros::init(argc, argv, "publisher");
    ros::NodeHandle n;
    ros::Publisher lane_pub = n.advertise<um_acc::CarState_L>("lane_data", 1000);
    ros::Rate loop_rate(10);
    um_acc::CarState_L msg;
         
    double phi = 0, y = 0;
    double sum_phi = 0, theta_n = 0;
    double lane_edge_1 = -1, lane_edge_2 = -1, x_mid = 0;
    double lane_edge_1_count = 0, lane_edge_2_count = 0;
    double lane_width_thresh = 50, screen_mid = 240;
    double car_center = 340, car_length = 11.5;  //car_length in inches
 
    double t_curr_s = 0, t_curr_n, t_prev_s, t_prev_n = 0, delta_t = 0;
    double y_prev = 0, phi_prev = 0;
 
    //calculate undistort matrix
    //Mat map1, map2;
    //Size imageSize(640, 480);
    //initUndistortRectifyMap(cameraMatrix, distCoeffs, Mat(),
            //getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, imageSize, 1, imageSize, 0),
            //imageSize, CV_16SC2, map1, map2);
 
    //create canny edge detector object
    Ptr<cuda::CannyEdgeDetector> canny = cuda::createCannyEdgeDetector(60, 180, 3, false);
 
    //create hough detector object
    Ptr<cuda::HoughLinesDetector> hough =
    cuda::createHoughLinesDetector(1.0f, (float) (CV_PI / 180.0f), 50, 5);
 
    for(;;)
    {
        Mat src;
        cap >> src; // get a new frame from camera
    t_curr_s = ros::Time::now().sec;
    t_curr_n = ros::Time::now().nsec;   // get time of camera measurement from ros
 
    //remap(src, src, map1, map2, INTER_LINEAR);
     
        cuda::GpuMat d_src(src);
        cuda::GpuMat d_lines, d_dst;
        cuda::cvtColor(d_src, d_dst, CV_BGR2GRAY, 0, cuda::Stream::Null());
        //gpu::bilateralFilter(d_dst, d_dst, -1, 50, 7);
 
        Mat cdst(d_dst);
 
        vector<int> Mid_X, Mid_Y;
        for(int i = 0 ; i < cdst.rows ; i ++)
        {
            vector<int> black;
            for(int j = 0 ; j < cdst.cols ; j ++)
            {
                char pixel = cdst.at<char>(i,j);
                if ( pixel < 80 )
                {
                    black.push_back(j);
                }
            }
            if( black.empty() == 0) // nonempty
            {
                int avg = accumulate( black.begin(), black.end(), 0.0) / black.size();
                Mid_X.push_back(avg);
                Mid_Y.push_back(i);
            }
             
            black.clear();
        }
         
        for(int i = 0 ; i < Mid_X.size() ; i ++)
        {
             cdst.at<char>(Mid_Y[i], Mid_X[i]) = 255;
        }
         
        //cout << Mid_X.size() << " " << Mid_Y.size() << endl;
/*
        Mat Xmat(Mid_X.size(), 2, CV_32FC1);
        Mat XmatT(2, Mid_X.size(), CV_32FC1);
        Mat Ymat(Mid_Y.size(), 1, CV_32FC1);
        Mat Tmp22(2, 2, CV_32FC1);
        Mat a(2, 1, CV_32FC1);
 
        for(int i = 0 ; i < Mid_X.size() ; i ++)
        {
            XmatT.at<float>(0, i) = 1;
            XmatT.at<float>(1, i) = Mid_X[i];
 
            Xmat.at<float>(i, 0) = 1;
            Xmat.at<float>(i, 1) = Mid_X[i];
 
            Ymat.at<float>(i, 0) = Mid_Y[i];
        }
        Tmp22 = XmatT*Xmat;
        Tmp22 = Tmp22.inv();
        a = Tmp22*XmatT*Ymat;
 
        for(int i = 0 ; i < cdst.rows ; i ++)
        {
            float Tmpx =  (i - a.at<float>(0, 0)) / a.at<float>(0, 1);
            if( ( Tmpx >=0)  && ( Tmpx < 480) )
            {
                cdst.at<char>(i, Tmpx) = 255;
            }
        }
 
*/
 
 
        // x = a0 + a1*y + a2 * y^2
        Mat Ymat(Mid_X.size(), 3, CV_32FC1);
        Mat YmatT(3, Mid_X.size(), CV_32FC1);
        Mat Xmat(Mid_Y.size(), 1, CV_32FC1);
        Mat Tmp33(3, 3, CV_32FC1);
        Mat a(3, 1, CV_32FC1);
 
        for(int i = 0 ; i < Mid_X.size() ; i ++)
        {
            YmatT.at<float>(0, i) = 1;
            YmatT.at<float>(1, i) = Mid_Y[i];
            YmatT.at<float>(2, i) = Mid_Y[i]*Mid_Y[i];
 
            Ymat.at<float>(i, 0) = 1;
            Ymat.at<float>(i, 1) = Mid_Y[i];
            Ymat.at<float>(i, 2) = Mid_Y[i]*Mid_Y[i];
 
            Xmat.at<float>(i, 0) = Mid_X[i];
        }
        Tmp33 = YmatT*Ymat;
        Tmp33 = Tmp33.inv();
 
        a = Tmp33*YmatT*Xmat;
 
cout << a << endl;
 
        for(int i = 0 ; i < cdst.rows ; i ++)
        {
            int Tmpx =  a.at<float>(0, 0) + a.at<float>(0, 1)*i + a.at<float>(0, 2)*i*i;
            if( ( Tmpx >=0)  && ( Tmpx < 480) )
            {
                cdst.at<char>(i, Tmpx) = 255;
            }
        }
 
        imshow("GRAY", cdst);
 
        Mid_X.clear();
        Mid_Y.clear();
        Ymat.release();
        YmatT.release();
        Xmat.release();
 
        y = a.at<float>(0, 0) + a.at<float>(0, 1)*240 + a.at<float>(0, 2)*240*240;
        cout << "y = " << y << endl;
 
        float k = a.at<float>(0, 1) + 2*a.at<float>(0, 2)*240;
        cout << "k = " << k << " atan(k) = " << atan(k) << endl;
 
        phi = -atan(k);
 
 
        //imshow("source", src);
        //imshow("canny edge detector", cdst_1);
        //imshow("hough transform", cdst);
 
        //transformation to get lane displacement
        msg.phi = phi;
        msg.y = (y-car_center)*(8.5/640) - car_length*tan(msg.phi);
 
cout << msg.y << " " << msg.phi << endl;
 
        delta_t = (t_curr_s - t_prev_s) + (t_curr_n - t_prev_n)/1000000000;
        msg.r = (msg.phi-phi_prev)/delta_t;
        msg.v = (msg.y - y_prev)/delta_t;
        t_prev_s = t_curr_s;
        t_prev_n = t_curr_n;
        y_prev = msg.y;
        phi_prev = msg.phi;
        lane_pub.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
 
        if(waitKey(30) >= 0) break;
    }
    // the camera will be deinitialized automatically in VideoCapture destructor
    return 0;
}
