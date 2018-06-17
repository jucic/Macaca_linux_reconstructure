#include "../include/3d_reconstructure/reconstructure.h"

int main(int argc,char** argv)
{
    ros::init(argc,argv,"reconstructor");
    ros::NodeHandle nh;
    image_transport::ImageTransport it_left(nh);
    image_transport::ImageTransport it_right(nh);
    image_transport::Subscriber sub_left=it_left.subscribe("scene/left/image_color",1,ImageCallback_left);
    image_transport::Subscriber sub_right=it_right.subscribe("scene/right/image_color",1,ImageCallback_right);

	FileStorage fs(intrinsic_filename, FileStorage::READ);
	if (!fs.isOpened())
	{
		printf("Failed to open file %s\n", intrinsic_filename.c_str());
		return -1;
	}

	Mat M1, D1, M2, D2;
	fs["M1"] >> M1;
	fs["D1"] >> D1;
	fs["M2"] >> M2;
	fs["D2"] >> D2;

	M1 *= scale;
	M2 *= scale;

	fs.open(extrinsic_filename, FileStorage::READ);
	if (!fs.isOpened())
	{
		printf("Failed to open file %s\n", extrinsic_filename.c_str());
		return -1;
	}

	Mat R, T, R1, P1, R2, P2;
	fs["R"] >> R;
	fs["T"] >> T;

	bm->setPreFilterCap(31);//31
	bm->setBlockSize(SADWindowSize > 0 ? SADWindowSize : 9);
	bm->setMinDisparity(0);
	bm->setTextureThreshold(10);//10
	bm->setUniquenessRatio(15);//15
	bm->setSpeckleWindowSize(100);
	bm->setSpeckleRange(32);
	bm->setDisp12MaxDiff(1);

	Mat disp, disp8;

	sgbm->setPreFilterCap(63);
	int sgbmWinSize = SADWindowSize > 0 ? SADWindowSize : 3;
	sgbm->setBlockSize(sgbmWinSize);


	sgbm->setMinDisparity(0);
	sgbm->setNumDisparities(numberOfDisparities);
	sgbm->setUniquenessRatio(10);
	sgbm->setSpeckleWindowSize(100);
	sgbm->setSpeckleRange(32);
	sgbm->setDisp12MaxDiff(1);
	sgbm->setMode(StereoSGBM::MODE_SGBM);

	namedWindow("disparity", 0);
	setMouseCallback("disparity", onMouse, 0);

	while(nh.ok())
    {
		ros::spinOnce();
		if(img1_raw.empty()||img2_raw.empty()) continue;

		cv::Mat img1,img2;
	    cv:: cvtColor(img1_raw,img1,cv::COLOR_BGR2GRAY);
	    cv:: cvtColor(img2_raw,img2,cv::COLOR_BGR2GRAY);
	    //cv::imshow("left_grey",img1);
		//cv::waitKey(1);

		Size img_size = img1.size();

		stereoRectify(M1, D1, M2, D2, img_size, R, T, R1, R2, P1, P2, Q, CALIB_ZERO_DISPARITY, -1, img_size, &roi1, &roi2);

		Mat map11, map12, map21, map22;
		initUndistortRectifyMap(M1, D1, R1, P1, img_size, CV_16SC2, map11, map12);
		initUndistortRectifyMap(M2, D2, R2, P2, img_size, CV_16SC2, map21, map22);

		Mat img1r, img2r;
		remap(img1, img1r, map11, map12, INTER_LINEAR);
		remap(img2, img2r, map21, map22, INTER_LINEAR);

		img1 = img1r;
		img2 = img2r;

		imshow("remapleft", img1);
		imshow("remapright", img2);

		numberOfDisparities = numberOfDisparities > 0 ? numberOfDisparities : ((img_size.width / 8) + 15) & -16;

		bm->setROI1(roi1);
		bm->setROI2(roi2);
		bm->setNumDisparities(numberOfDisparities);

		int cn = img1.channels();

		sgbm->setP1(8 * cn*sgbmWinSize*sgbmWinSize);
		sgbm->setP2(32 * cn*sgbmWinSize*sgbmWinSize);

		//copyMakeBorder(img1, img1, 0, 0, numberOfDisparities, 0, IPL_BORDER_REPLICATE);
		//copyMakeBorder(img2, img2, 0, 0, numberOfDisparities, 0, IPL_BORDER_REPLICATE);

		bm->compute(img1, img2, disp);
		//sgbm->compute(img1, img2, disp);

		//disp = disp.colRange(numberOfDisparities, img1.cols);

		disp.convertTo(disp8, CV_8U, 255 / (numberOfDisparities*16.));

		reprojectImageTo3D(disp, xyz, Q, true);
		//xyz = xyz * 16;
		//Mat vdispRGB = disp8;
		//cvtColor(disp8, vdispRGB, COLOR_GRAY2BGR);

		imshow("disparity", disp8);

		//wait for 40 milliseconds
		int c = waitKey(1);

		//exit the loop if user press "Esc" key  (ASCII value of "Esc" is 27)
		if (27 == char(c)) break;

    }
    return 0;
}
