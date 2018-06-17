#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/utility.hpp"

#include <stdio.h>
#include<iostream>

using namespace cv;
using namespace std;

bool selectObject;
Rect selection;
Point origin;
Mat xyz;

std::string intrinsic_filename = "/home/jucic/study/macaca/src/3d_reconstructure/src/intrinsics.yml";
std::string extrinsic_filename = "/home/jucic/study/macaca/src/3d_reconstructure/src/extrinsics.yml";

int SADWindowSize=5, numberOfDisparities=256;//15,32//better:5,256//
float scale=1;

Ptr<StereoBM> bm = StereoBM::create(16, 9);
Ptr<StereoSGBM> sgbm = StereoSGBM::create(0, 16, 3);

Mat img1, img2;

Rect roi1, roi2;
Mat Q;

cv_bridge::CvImagePtr cv_ptr_;
cv::Mat img1_raw,img2_raw;
image_transport::Subscriber image_sub_;
image_transport::Publisher image_pub_;

void Reconstructure_Init()
{
}
void ImageCallback_left(const sensor_msgs::ImageConstPtr& msg)
{
    try
    {
        img1_raw=cv_bridge::toCvCopy(msg,"bgr8")->image;
        cv::imshow("left_scene",img1_raw);
        cv::waitKey(1);
    }
    catch(cv_bridge::Exception& e)
    {
        ROS_ERROR("couldn't convert fron '%s' to 'bgr8'.",msg->encoding.c_str());
    }
}
void ImageCallback_right(const sensor_msgs::ImageConstPtr& msg)
{
    try
    {
        img2_raw=cv_bridge::toCvCopy(msg,"bgr8")->image;
        cv::imshow("right_scene",img2_raw);
        cv::waitKey(1);
    }
    catch(cv_bridge::Exception& e)
    {
        ROS_ERROR("couldn't convert fron '%s' to 'bgr8'.",msg->encoding.c_str());
    }
}

static void onMouse(int event, int x, int y, int, void*)
{
	if (selectObject)
	{
		selection.x = MIN(x, origin.x);
		selection.y = MIN(y, origin.y);
		selection.width = std::abs(x - origin.x);
		selection.height = std::abs(y - origin.y);
	}

	switch (event)
	{
	case EVENT_LBUTTONDOWN:   //鼠标左按钮按下的事件
		origin = Point(x, y);
		selection = Rect(x, y, 0, 0);
		selectObject = true;

		cout << origin << "in world coordinate is: " << xyz.at<Vec3f>(origin) << endl;
		break;
	case EVENT_LBUTTONUP:    //鼠标左按钮释放的事件
		selectObject = false;
		if (selection.width > 0 && selection.height > 0)
			break;
	}
}
