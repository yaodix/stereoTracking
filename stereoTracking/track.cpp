//track.cpp :ǰ�����ܵĻ��ܣ�ʵ�ֶ�Ŀ��Ĺ�������


#include <opencv.hpp>
#include <iostream>
#include <math.h>
#include <tchar.h>
#include <windows.h>
#include "Serial.h"


#define  PI 3.1415936
using namespace cv;
using namespace std;
Vec3d calXYZ(Point2d PL, Point2d PR);
void calDis(Point2d PL1, Point2d PR1, Point2d PL2, Point2d PR2);
Point2f getCircleCenterInImg(Mat &img, double threshValueOfRoundness, int threshValueOfArea);
void showCanvas(Mat &rectifyImgL, Mat &rectifyImgR);
void readParams();
Point transformPosToServoFrame(Vec3d camPos);//point�а���ˮƽ����ֱ����������ռ�ձ�

	VideoCapture camL;
	VideoCapture camR;
	Mat imgL, imgR;

	//˫Ŀ��������
	Size imageSize(640, 480);
	Mat cameraMatrixL, cameraMatrixR;
	Mat distCoeffL, distCoeffR;
	Mat Rl, Rr;
	Mat Pl, Pr;
	Mat mapLx, mapLy;
	Mat mapRx, mapRy;
	Mat Q;
	Rect validROIL, validROIR;                              //ͼ��У��֮�󣬻��ͼ����вü��������validROI����ָ�ü�֮�������  

	Mat rectifyImageL, rectifyImageR;
	CSerial serial;

int main()
{
	bool isContinue=true;
	LONG    lLastError = ERROR_SUCCESS;
	lLastError = serial.Open(_T("COM4"), 0, 0, false);
	if (lLastError != ERROR_SUCCESS)
	{
		std::cout << "open false" << endl;
		return 1;
	}
	// Setup the serial port (9600,N81) using hardware handshaking
	lLastError = serial.Setup(CSerial::EBaud9600, CSerial::EData8, CSerial::EParNone, CSerial::EStop1);
	if (lLastError != ERROR_SUCCESS)
	{
		std::cout << "Setup false" << endl;
		return 1;
	}

	camL.open(0);
	camR.open(1);
	//��ȡ˫Ŀ�궨����
	readParams();

	while (isContinue)
	{
		camL.grab();
		camR.grab();

		camL.retrieve(imgL);
		camR.retrieve(imgR);

		//	imshow("imgL", imgL);
		//	imshow("imgR", imgR);
		//�������
		if (mapLx.empty())
		{
			initUndistortRectifyMap(cameraMatrixL, distCoeffL, Rl, Pl, imageSize, CV_32FC1, mapLx, mapLy);
			initUndistortRectifyMap(cameraMatrixR, distCoeffR, Rr, Pr, imageSize, CV_32FC1, mapRx, mapRy);
		}

		remap(imgL, rectifyImageL, mapLx, mapLy, INTER_LINEAR);
		remap(imgR, rectifyImageR, mapRx, mapRy, INTER_LINEAR);


		//Ѱ��Ŀ�꣬Բ��
		Point2f centerPointL = getCircleCenterInImg(rectifyImageL, 0.85, 1000);
		Point2f centerPointR = getCircleCenterInImg(rectifyImageR, 0.85, 1000);
		showCanvas(rectifyImageL, rectifyImageR);
		//imshow("rectifyImageL", rectifyImageL);
		//imshow("rectifyImageR", rectifyImageR);

		//�����������ϵ������
		Vec3d camPos = calXYZ(centerPointL, centerPointR);

		Point  rotateAngleHZ = transformPosToServoFrame(Mat(camPos));
		cout << "rotation = " << rotateAngleHZ << endl;
		if (rotateAngleHZ.x <400 || rotateAngleHZ.y<400)
		{

		} 
		else
		{
			char val[10];
			sprintf_s(val, "(%d,%d)", rotateAngleHZ.x, rotateAngleHZ.y);
			lLastError = serial.Write(val);
			if (lLastError != ERROR_SUCCESS)
			{
				std::cout << "Write false" << endl;
			}

		}
		if (waitKey(50) == 27)
		{
			isContinue = false;
		}
	}
	
	serial.Close();
	return 0;
}

Vec3d calXYZ(Point2d PL, Point2d PR)
{
	double cx = -Q.at<double>(0,3);
	double cy = -Q.at<double>(1,3);
	double f = Q.at<double>(2,3);
	double Tx = 1/Q.at<double>(3,2);
	double d = PL.x - PR.x;
	Vec3d real;
	real[0] = (PL.x - cx)*Tx / d;
	real[1] = (PL.y - cy)*Tx / d;
	real[2] = f*Tx / d;
	return real;

}
void calDis(Point2d PL1, Point2d PR1, Point2d PL2, Point2d PR2)
{
	Vec3d p1, p2;
	p1 = calXYZ(PL1, PR1);
	p2 = calXYZ(PL2, PR2);
	std::cout << "p1 " << p1 << endl;
	std::cout << "p2  " << p2 << endl;

	double dis = sqrtf(pow(p1[0] - p2[0], 2) + pow(p1[1] - p2[1], 2) + pow(p1[2] - p2[2], 2));

	std::cout << dis << endl;
}

Point2f getCircleCenterInImg(Mat &img, double threshValueOfRoundness,int threshValueOfArea)
{
	Mat gray, threImg;
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
		if (ratio>threshValueOfRoundness&& area > threshValueOfArea &&area<20000)
		{
			vecCircleRes.push_back(contours[i]);
		}
	}
	Moments mome;
	if (vecCircleRes.size() > 0)
	{
		mome = moments(vecCircleRes[0]);
		center = Point2f(mome.m10 / mome.m00, mome.m01 / mome.m00);
		circle(img, center, 3, Scalar(0, 0, 255));
		drawContours(img, vecCircleRes, -1, Scalar(0, 0, 255));
	}

	return center;
}

void showCanvas(Mat &rectifyImgL,Mat &rectifyImgR)
{
	Mat canvas;
	double sf;
	int w, h;
	sf = 640. / MAX(imageSize.width, imageSize.height);
	w = cvRound(imageSize.width * sf);
	h = cvRound(imageSize.height * sf);
	canvas.create(h, w * 2, CV_8UC3);

	/*��ͼ�񻭵�������*/
	Mat canvasPart = canvas(Rect(w * 0, 0, w, h));                                //�õ�������һ����  
	resize(rectifyImgL, canvasPart, canvasPart.size(), 0, 0, INTER_AREA);     //��ͼ�����ŵ���canvasPartһ����С  
	Rect vroiL(cvRound(validROIL.x*sf), cvRound(validROIL.y*sf),                //��ñ���ȡ������    
		cvRound(validROIL.width*sf), cvRound(validROIL.height*sf));
	rectangle(canvasPart, vroiL, Scalar(0, 0, 255), 3, 8);                      //����һ������  

	std::cout << "Painted ImageL" << endl;

	/*��ͼ�񻭵�������*/
	canvasPart = canvas(Rect(w, 0, w, h));                                      //��û�������һ����  
	resize(rectifyImgR, canvasPart, canvasPart.size(), 0, 0, INTER_LINEAR);
	Rect vroiR(cvRound(validROIR.x * sf), cvRound(validROIR.y*sf),
		cvRound(validROIR.width * sf), cvRound(validROIR.height * sf));
	rectangle(canvasPart, vroiR, Scalar(0, 255, 0), 3, 8);

	std::cout << "Painted ImageR" << endl;

	/*���϶�Ӧ������*/
	//for (int i = 0; i < canvas.rows; i += 50)
		//line(canvas, Point(0, i), Point(canvas.cols, i), Scalar(0, 255, 0), 1, 8);
	imshow("rectified", canvas);
}
void readParams()
{
	FileStorage fs;
	bool ret = fs.open("intrinsics.yml", FileStorage::READ);
	if (ret)
	{
		fs["cameraMatrixL"] >> cameraMatrixL;
		fs["cameraMatrixR"] >> cameraMatrixR;
		fs["distCoeffL"] >> distCoeffL;
		fs["distCoeffR"] >> distCoeffR;
		std::cout << "read intrinsics sucess" << endl;
	}
	else
	{
		std::cout << "read intrinsics error" << endl;
	}
	fs.release();

	ret = fs.open("extrinsics.yml", FileStorage::READ);
	if (ret)
	{
		fs["Rl"] >> Rl;
		fs["Rr"] >> Rr;
		fs["Pl"] >> Pl;
		fs["Pr"] >> Pr;
		fs["Q"] >> Q;
		fs["validROIL"] >> validROIL;
		fs["validROIL"] >> validROIR;
		std::cout << "read extrinsics sucess" << endl;
	}
	else
	{
		std::cout << "read extrinsics error" << endl;
	}
}
Point transformPosToServoFrame(Vec3d camPos)
{
	static int horHZ, verHZ;
	static int lastHorHZ, lastVerHZ;
	Mat objPolastVerHZsMat = Mat_<float>(4, 1);
	Mat camPosMat = (Mat_ < float>(4, 1) << camPos[0], camPos[1], camPos[2], 1);

	//������̨����ϵX˳ʱ����ת90��
	Mat rotateX_90 = (Mat_<double>(3, 3) << 1, 0, 0,
		0, 0, 1,
		0, -1, 0);
	//������̨����ϵZ˳ʱ����ת90��
	Mat rotateZ_90 = (Mat_<double>(3, 3) << 0, 1, 0,
		-1, 0, 0,
		0, 0, 1);
	Mat rotation = rotateZ_90*rotateX_90;
	Mat camFrame2ServoFrame = (Mat_<float>(4, 4) <<
		rotation.at<double>(0, 0), rotation.at<double>(0, 1), rotation.at<double>(0, 2), 15,
		rotation.at<double>(1, 0), rotation.at<double>(1, 1), rotation.at<double>(1, 2), -90,
		rotation.at<double>(2, 0), rotation.at<double>(2, 1), rotation.at<double>(2, 2), 135,
		0, 0, 0, 1);

	Mat  objPosMat = camFrame2ServoFrame*camPosMat;
	//����ָ�����̨
	int laserHeight = 20;
	int horAngle, verAngle;
	int x = objPosMat.at<float>(0, 0);//float ��
	int y = objPosMat.at<float>(1, 0);
	int z = objPosMat.at<float>(2, 0);
	horAngle = (atan2(y, x) / CV_PI) * 180;
	verAngle = (atan2(z - laserHeight, sqrtf(x*x + y*y)) / CV_PI) * 180;
	 lastHorHZ = horHZ;
	 lastVerHZ = verHZ;
	//�Ƕ�ת��Ϊ�����תƵ��
	 horHZ = 700 + 4.8* horAngle;//ˮƽ��������ÿ��Ƶ�ʱ任5��ռ�ձȣ��Ƕȷ���Ϊ��
	 verHZ = 700 - 4.55*verAngle;
	std::cout << objPosMat << endl;
		return Point(horHZ, verHZ);

}