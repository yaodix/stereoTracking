// monocalib.cpp : 单目相机标定过程示例
//

//只支持opencv3.0及之后的版本
//#include "stdafx.h"

#include <opencv.hpp>
#include <iostream>
#include <vector>

using namespace std;
using namespace cv;

const int imageWidth = 640;								//摄像头的分辨率
const int imageHeight = 480;
const int boardWidth = 11;								//横向的角点数目
const int boardHeight = 7;								//纵向的角点数据
const int boardCornerNum = boardWidth * boardHeight;		//总的角点数据
const int frameNumber =13;								//相机标定时需要采用的图像帧数
const int squareSize = 20;								//标定板黑白格子的大小 单位mm
const Size boardSize = Size(boardWidth, boardHeight);	//
	
Mat intrinsic;											//相机内参数
Mat distortion_coeff;									//相机畸变参数
vector<Mat> rvecs;									    //旋转向量
vector<Mat> tvecs;										//平移向量
vector<vector<Point2f>> corners;						//各个图像找到的角点的集合 和objRealPoint 一一对应
vector<vector<Point3f>> objRealPoint;					//各副图像的角点的实际物理坐标集合


vector<Point2f> corner;									//某一副图像找到的角点

Mat rgbImage, grayImage;

/*计算标定板上模块的实际物理坐标*/
void calRealPoint(vector<vector<Point3f>>& obj, int boardwidth,int boardheight, int imgNumber, int squaresize)
{
//	Mat imgpoint(boardheight, boardwidth, CV_32FC3,Scalar(0,0,0));
	vector<Point3f> imgpoint;
	for (int rowIndex = 0; rowIndex < boardheight; rowIndex++)	//boardheight=6;
	{
		for (int colIndex = 0; colIndex < boardwidth; colIndex++) //boardWidth = 9;		
		{
		//	imgpoint.at<Vec3f>(rowIndex, colIndex) = Vec3f(rowIndex * squaresize, colIndex*squaresize, 0);
			imgpoint.push_back(Point3f(rowIndex * squaresize, colIndex * squaresize, 0));
		}
	}
	for (int imgIndex = 0; imgIndex < imgNumber; imgIndex++)
	{
		obj.push_back(imgpoint);
	}
}

/*设置相机的初始参数 也可以不估计*/
void guessCameraParam(void )
{
	/*分配内存*/
	intrinsic.create(3, 3, CV_64FC1);
	distortion_coeff.create(5, 1, CV_64FC1);

	/*
	fx 0 cx
	0 fy cy
	0 0  1
	*/
	intrinsic.at<double>(0,0) = 1164.33;   //fx		
	intrinsic.at<double>(0, 2) = 320;   //cx
	intrinsic.at<double>(1, 1) = 1162.14;   //fy
	intrinsic.at<double>(1, 2) = 240;   //cy

	intrinsic.at<double>(0, 1) = 0;
	intrinsic.at<double>(1, 0) = 0;
	intrinsic.at<double>(2, 0) = 0;
	intrinsic.at<double>(2, 1) = 0;
	intrinsic.at<double>(2, 2) = 1;

	/*
	k1 k2 p1 p2 p3
	*/
	distortion_coeff.at<double>(0, 0) = -0.193740;  //k1
	distortion_coeff.at<double>(1, 0) = -0.378588;  //k2
	distortion_coeff.at<double>(2, 0) = 0.028980;   //p1
	distortion_coeff.at<double>(3, 0) = 0.008136;   //p2
	distortion_coeff.at<double>(4, 0) = 0;		  //p3
}

void outputCameraParam(void )
{
	/*保存数据*/
	//cvSave("cameraMatrix.xml", &intrinsic);
	//cvSave("cameraDistoration.xml", &distortion_coeff);
	//cvSave("rotatoVector.xml", &rvecs);
	//cvSave("translationVector.xml", &tvecs);
	/*输出数据*/
	cout << "fx :" << intrinsic.at<double>(0, 0) << endl << "fy :" << intrinsic.at<double>(1, 1) << endl;
	cout << "cx :" << intrinsic.at<double>(0, 2) << endl << "cy :" << intrinsic.at<double>(1, 2) << endl;

	cout << "k1 :" << distortion_coeff.at<double>(0, 0) << endl;
	cout << "k2 :" << distortion_coeff.at<double>(1, 0) << endl;
	cout << "p1 :" << distortion_coeff.at<double>(2, 0) << endl;
	cout << "p2 :" << distortion_coeff.at<double>(3, 0) << endl;
	cout << "p3 :" << distortion_coeff.at<double>(4, 0) << endl;
}

//计算重投影误差
double computeReprojectionErrors(
	const vector<vector<Point3f> >& objectPoints,
	const vector<vector<Point2f> >& imagePoints,
	const vector<Mat>& rvecs, const vector<Mat>& tvecs,
	const Mat& cameraMatrix, const Mat& distCoeffs,
	vector<float>& perViewErrors)
{
	vector<Point2f> imagePoints2;
	imagePoints2.reserve(imagePoints[0].size());
	int i, totalPoints = 0;
	double totalErr = 0, err;
	perViewErrors.resize(objectPoints.size());

	for (i = 0; i < (int)objectPoints.size(); i++)
	{
		projectPoints(Mat(objectPoints[i]), rvecs[i], tvecs[i],
			cameraMatrix, distCoeffs, imagePoints2);
		err = norm(Mat(imagePoints[i]), Mat(imagePoints2), CV_L2);
		int n = (int)objectPoints[i].size();
		perViewErrors[i] = (float)std::sqrt(err * err / n);
		totalErr += err * err;
		totalPoints += n;
	}
	//cout<<double( sqrt(totalErr / totalPoints))<<endl;
	return std::sqrt(totalErr / totalPoints);
}


int  main(int argc, char *arg[] )
{
	Mat img;
	int goodFrameCount = 0;
	int num = 0;
	namedWindow("chessboard");
	cout << "按Q退出 ..." << endl;
	while (goodFrameCount < frameNumber)
	{
		char filename[100],filename2[100];
		sprintf_s(filename, "F:\\testPic\\双目目标\\right%02d.jpg", goodFrameCount + 1);
		//	cout << filename << endl;
		rgbImage = imread(filename);
		cvtColor(rgbImage, grayImage, CV_BGR2GRAY);
		imshow("Camera", grayImage);
		bool isFind = findChessboardCorners(grayImage, boardSize, corner);
		if (isFind == true)	//所有角点都被找到 说明这幅图像是可行的
		{
			/*
			Size(5,5) 搜索窗口的一半大小
			Size(-1,-1) 死区的一半尺寸
			TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 20, 0.1)迭代终止条件
			*/
			cornerSubPix(grayImage, corner, Size(5, 5), Size(-1, -1), TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 20, 0.1));
			drawChessboardCorners(rgbImage, boardSize, corner, isFind);
			imshow("chessboard", rgbImage);
			corners.push_back(corner);
			goodFrameCount++;
			cout << "The image is good" << endl;
		}
		else
		{
			cout << "The image is bad please try again" << endl;
		}
		//	cout << "Press any key to continue..." << endl;
		//	waitKey(0);

		if (waitKey(10) == 'q')
		{
			break;
		}
		//	imshow("chessboard", rgbImage);
	}

	/*
	图像采集完毕 接下来开始摄像头的校正
	calibrateCamera()
	输入参数 objectPoints  角点的实际物理坐标
			 imagePoints   角点的图像坐标
			 imageSize	   图像的大小
	输出参数
			 cameraMatrix  相机的内参矩阵
			 distCoeffs	   相机的畸变参数
			 rvecs		   旋转矢量(外参数)
			 tvecs		   平移矢量(外参数）
	*/

	/*设置实际初始参数 根据calibrateCamera来 如果flag = 0 也可以不进行设置*/
	guessCameraParam();
	cout << "guess successful" << endl;
	/*计算实际的校正点的三维坐标*/
	calRealPoint(objRealPoint, boardWidth, boardHeight, frameNumber, squareSize);
	cout << "cal real successful" << endl;
	/*标定摄像头*/
	//calibrateCamera(objRealPoint, corners, Size(imageWidth, imageHeight), intrinsic, distortion_coeff, rvecs, tvecs, CV_CALIB_FIX_PRINCIPAL_POINT);
	double rms = calibrateCamera(objRealPoint, corners, Size(imageWidth, imageHeight), intrinsic, distortion_coeff, rvecs, tvecs);

	cout << "calibration successful" << endl;
	/*保存并输出参数*/
	outputCameraParam();
	cout << "out successful" << endl;


	/*显示畸变校正效果*/
	Mat inImage = imread("F:\\testPic\\双目追踪\\right04.jpg");
	imshow("原图", inImage);
	Mat cImage;
	Mat newIntrinsic;

	undistort(inImage, cImage, intrinsic, distortion_coeff);
	imshow("矫正后图", cImage);

	vector<float> reprojErrs;
	reprojErrs.resize(objRealPoint.size());
	double totalAvgErr = 0;

	totalAvgErr = computeReprojectionErrors(objRealPoint, corners,
		rvecs, tvecs, intrinsic, distortion_coeff, reprojErrs);

	cout << "重投影误差：" << totalAvgErr << endl;



	//vector<Point2f>  image_points2;             /****   保存重新计算得到的投影点    ****/

	//vector<Point3f> tempPointSet = objRealPoint[11];
	///****    通过得到的摄像机内外参数，对空间的三维点进行重新投影计算，得到新的投影点     ****/
	//Mat r3 = (Mat_<double>(1, 3) << 0, 0, 0);
	//Mat t3 = (Mat_<double>(1, 3) << 0, 0, 0);

	//projectPoints(tempPointSet, r3, tvecs[11], intrinsic, distortion_coeff, image_points2);

	//Mat pointsImg(imageHeight, imageWidth, CV_8U, Scalar(0));
	//for (int size = 0; size < image_points2.size(); ++size)
	//{
	//	circle(pointsImg, image_points2[size], 2, Scalar(255), 2, 8);
	//	cout << image_points2[size] << endl;
	//}
	//imshow("pointImage", pointsImg);
	waitKey(0);
	//system("pause");
	return 0;
}



