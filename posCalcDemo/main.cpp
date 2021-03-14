// main.cpp : 手动选取图像坐标，验证三维坐标计算结果

#include <opencv.hpp>
#include<iostream>
using namespace std;
using namespace cv;

class Test
{
public:
	Test(){}
	~Test(){}
	void show()
	{

	}
};
Vec3d calXYZ(Point2d PL, Point2d PR)
{
	//rms =0.21
	//double Tx = 168.024657;
	//double f = 2191.2144;
	//double cx = 703.05;
	//double cy = 417.68;

	//rms = 0.321
	double Tx = 501.46545;
	double f = 2223.144325;
	double cx = 1356.0392;
	double cy = 992.970741;
	double d = PL.x - PR.x;
	Vec3d real;
	real[0] = (PL.x-cx)*Tx / d;
	real[1] = (PL.y - cy)*Tx / d;
	real[2] = f*Tx / d;
	return real;
	
}
void calDis(Point2d PL1, Point2d PR1,Point2d PL2,Point2d PR2)
{
	Vec3d p1, p2;
	p1 = calXYZ(PL1,PR1);
	p2 = calXYZ(PL2, PR2);
	cout << "first  point pos  = " << p1 << endl;
	cout << "second point pos  = " << p2 << endl;

	double dis = sqrtf(pow(p1[0] - p2[0], 2) + pow(p1[1] - p2[1], 2) + pow(p1[2] - p2[2], 2));

	cout << "				real distance = "<<dis << "mm"<<endl;
}
int main()
{
	Mat threImg;
	int scale = 1;
	int delta = 0;
	int ddepth = CV_16S;
	int c;
	Mat imgL = imread("F:\\testPic\\双目测试\\imgL.bmp");
	Mat imgR = imread("F:\\testPic\\双目测试\\imgR.bmp");

	calDis(Point2d(1532, 453), Point2d(252, 453), Point2d(1513,698 ), Point2d(240 ,698));
	calDis(Point2d(1532, 453), Point2d(252, 453), Point2d(1495, 942), Point2d(228, 942));
	calDis(Point2d(1532, 453), Point2d(252, 453), Point2d(1476, 1183), Point2d(216, 1183));
	calDis(Point2d(1532, 453), Point2d(252, 453), Point2d(1458, 1422), Point2d(203, 1422));
	calDis(Point2d(1532, 453), Point2d(252, 453), Point2d(1440, 1660), Point2d(191, 1659));
	// calDis(Point2d(1532, 453), Point2d(252, 453), Point2d(2206, 525), Point2d(994, 526));


	waitKey(0);
}