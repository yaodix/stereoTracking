// objectDetection.cpp : 建立双目相机坐标系与舵机坐标系映射关系，并标定舵机工作频率和运动角度。


#include <opencv.hpp>
#include <iostream>
#include <vector>
using namespace cv;
using namespace std;

Point2f center(Mat &img, double threshValueOfCircle);

int stereoImgCap()
{
	VideoCapture cam1;
	VideoCapture cam2;
	Mat frame1,frame2;
	cam1.open(1);
	cam2.open(0);
	while (1)
	{
		cam1 >> frame1;
		cam2 >> frame2;
		cout << center(frame1, 0.75) << "	";
		cout << center(frame2, 0.75) << endl;

		imshow("frame1", frame1);
		imshow("frame2", frame2);

		if (waitKey(100) == 27)
		{
			return 0;
		}

	}
	return 0;
}


// 圆形目标提取
Point2f center(Mat &img, double threshValueOfCircle)
{
	Mat gray,threImg;
	Mat morTmp1, morTmp2;
	Point2f center;
	cvtColor(img, gray, CV_BGR2GRAY);
	threshold(gray, threImg, 20, 255, CV_THRESH_BINARY_INV);
	Mat mor1 = getStructuringElement(MORPH_RECT, Size(5, 5));
	Mat mor2 = getStructuringElement(MORPH_RECT, Size(13, 13));

	morphologyEx(threImg, morTmp1, MORPH_DILATE, mor1);
	morphologyEx(morTmp1, morTmp2, MORPH_ERODE, mor2);

	//morphologyEx(tmpH2, tmpH3, MORPH_ERODE, mor1);
	//morphologyEx(tmpH3, morphOut1,MORPH_DILATE , mor1);
	vector<Vec4i> v4i;
	double area, len;
	vector<vector<Point>> contours, vecCircleRes;
	findContours(morTmp2, contours, v4i, CV_RETR_CCOMP, CV_CHAIN_APPROX_NONE);
	for (int i = 0; i < contours.size(); i++)
	{
		area = contourArea(contours[i]);
		len = arcLength(contours[i], true);
		double ratio = (4 * CV_PI*area) / (len*len);
		if (ratio>threshValueOfCircle&& area > 1000)
		{
			vecCircleRes.push_back(contours[i]);
		}
	}
	Moments mome;
	if (vecCircleRes.size() > 0)
	{
		mome = moments(vecCircleRes[0]);
		 center=Point2f(mome.m10 / mome.m00, mome.m01 / mome.m00);
		circle(img, center, 3, Scalar(0, 0, 255));
		drawContours(img, vecCircleRes, -1, Scalar(0, 0, 255));
	}

	return center;
}

void transformPosToServoFrameAndSend(Vec3d camPos);
int main()
{
	Vec3d  pos1(0, 0, 64),pos2(0,1,0),pos3(-10,-100,100);
	transformPosToServoFrameAndSend(pos1);
	//transformPosToServoFrameAndSend(pos2);
	//transformPosToServoFrameAndSend(pos3);

	return 0;
}
void transformPosToServoFrameAndSend(Vec3d camPos)
{
	Mat objPosMat = Mat_<float>(4, 1);
	Mat camPosMat = (Mat_ < float>(4, 1) << camPos[0], camPos[1], camPos[2], 1);


	Mat rotateX_90 = (Mat_<double>(3, 3) << 1, 0, 0,
		0, 0, 1,
		0, -1, 0);

	Mat rotateZ_90 = (Mat_<double>(3, 3) << 0, 1, 0,
		-1, 0, 0,
		0, 0, 1);
	Mat rotation = rotateZ_90*rotateX_90;
	Mat camFrame2ServoFrame = (Mat_<float>(4, 4) <<
		rotation.at<double>(0, 0), rotation.at<double>(0, 1), rotation.at<double>(0, 2), 15,
		rotation.at<double>(1, 0), rotation.at<double>(1, 1), rotation.at<double>(1, 2), -90,
		rotation.at<double>(2, 0), rotation.at<double>(2, 1), rotation.at<double>(2, 2), 135,
		0, 0, 0, 1);

	objPosMat = camFrame2ServoFrame*camPosMat;
	//发送指令给云台
	int laserHeight = 50;
	int horAngle, verAngle;
	int x = objPosMat.at<float>(0, 0);
	int y = objPosMat.at<float>(1, 0);
	int z = objPosMat.at<float>(2, 0);
	horAngle = (atan2(y, x) / CV_PI) * 180;
	verAngle = (atan2(z - laserHeight, sqrtf(x*x + y*y)) / CV_PI) * 180;

	//角度转换为舵机旋转频率
	int horHZ = 700 + 4.55* horAngle;//水平方向舵机，每度频率变换5个占空比，角度方向为负
	int verHz = 700 - 4.55*verAngle;



	cout << objPosMat << endl;
}