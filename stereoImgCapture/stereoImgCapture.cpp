// stereoImgCapture.cpp : 图像对采集



#include <opencv.hpp>
#include <iostream>

using namespace cv;
using namespace std;
Mat imgL, imgR;
int imgCnt = 0;
char Lpath[200];
char Rpath[200];
void saveImg(int i, void*)
{
	cout << i << endl;
	sprintf_s(Lpath,"F:\\testPic\\双目目标\\left%02d.jpg", i);
	sprintf_s(Rpath, "F:\\testPic\\双目目标\\right%02d.jpg", i);
	imwrite(Lpath, imgL);
	imwrite(Rpath, imgR);
}


int main()
{
	VideoCapture camL;
	VideoCapture camR;
	bool isContinue=true;
	int zero = 1;
	namedWindow("imgL");
	camL.open(0);
	camR.open(1);

	while (isContinue)
	{
		camL.grab();
		camR.grab();

		camL.retrieve(imgL);
		camR.retrieve(imgR);
		createTrackbar("save", "imgL", &zero, 15, saveImg);
		imshow("imgL", imgL);
		imshow("imgR", imgR);
		if (waitKey(100) == 27)
		{
			isContinue = false;
		}
	}
	return 0;
}