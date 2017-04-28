#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/gpu/gpu.hpp>
#include <iostream>

#include "ros/ros.h"
#include "um_acc/CarState_L.h"
#include <sstream>
#include <vector>

using namespace cv;
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

    for(;;)
    {
        Mat src;
        cap >> src; // get a new frame from camera
	t_curr_s = ros::Time::now().sec;
	t_curr_n = ros::Time::now().nsec;	// get time of camera measurement from ros

	//remap(src, src, map1, map2, INTER_LINEAR);
    
		gpu::GpuMat d_src(src);
		gpu::GpuMat d_lines, d_dst;
		gpu::cvtColor(d_src, d_dst, CV_BGR2GRAY, 0, gpu::Stream::Null());
		//gpu::bilateralFilter(d_dst, d_dst, -1, 50, 7);
		gpu::Canny(d_dst, d_dst, 30, 90, 3); // adjust thresholds  // - 200 50 3
		Mat cdst(d_dst);
		
		vector<Vec2f> lines;
		gpu::HoughLinesBuf d_buf;
		gpu::HoughLines(d_dst, d_lines, d_buf, 1.0f, (float) (CV_PI/180), 125, false, 4096);
									
		gpu::HoughLinesDownload(d_lines, lines, noArray());

		for( size_t i = 0; i < lines.size(); i++ )
		{
			float rho = lines[i][0], theta = lines[i][1];
			Point pt1, pt2;
			double a = cos(theta), b = sin(theta);
			double x0 = a*rho, y0 = b*rho;			
			pt1.x = cvRound(x0 + 1000*(-b));
			pt1.y = cvRound(y0 + 1000*(a));
			pt2.x = cvRound(x0 - 1000*(-b));
			pt2.y = cvRound(y0 - 1000*(a));
			line( cdst, pt1, pt2, Scalar(255,255,255), 3, CV_AA);
			
			x_mid = pt2.x + (pt1.x - pt2.x) * (screen_mid - pt2.y) / 
				(pt1.y - pt2.y);  //interpolate to find intersection of
								  //middle of screen and lane (x coord)
			
			if(lane_edge_1 == -1)
			{
				lane_edge_1 = (lane_edge_1 * lane_edge_1_count + x_mid)
							/(lane_edge_1_count + 1);  //moving average
				lane_edge_1_count += 1;
			}
			else if(abs(x_mid - lane_edge_1) > lane_width_thresh)
			{
				lane_edge_2 = (lane_edge_2 * lane_edge_2_count + x_mid)
							/(lane_edge_2_count + 1);  //moving average
				lane_edge_2_count += 1;
			}
			else
			{
				lane_edge_1 = (lane_edge_1 * lane_edge_1_count + x_mid)
							/(lane_edge_1_count + 1);  //moving average
				lane_edge_1_count += 1;
			}
			
			theta_n = (CV_PI*0.5 ) - theta;
			if(theta_n < 0) theta_n += CV_PI;
			
			sum_phi += theta_n;
		}
		
		if(lines.size() != 0)
		{
			phi = sum_phi/((double)lines.size());
			y = (lane_edge_1 + lane_edge_2)/2;
		}
		else 
		{
			phi = -1;
			y = -1;
		}
		
		sum_phi = 0; lane_edge_1 = -1; lane_edge_2 = -1;
		lane_edge_1_count = 0; lane_edge_2_count = 0; x_mid = 0;

		imshow("source", src);
		imshow("detected lines", cdst);

		//transformation to get lane displacement
		msg.phi = -(phi - CV_PI/2);
		msg.y = (y-car_center)*(8.5/640) - car_length*tan(msg.phi);
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
