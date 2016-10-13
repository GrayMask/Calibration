#include "Camera.h";
#include "opencv2/opencv.hpp";
#include <iostream>;
using namespace std;
using namespace cv;

int Camera::takePic()
{
	VideoCapture cap0(0);//デバイスのオープン
	VideoCapture cap1(1);

	if (!cap0.isOpened() || !cap1.isOpened())//カメラデバイスが正常にオープンしたか確認．
	{
		//読み込みに失敗したときの処理
		cout << "カメラ読み込みに失敗した" << endl;
		return;
	}

	while (1)//無限ループ
	{
		Mat frame0;
		Mat frame1;
		cap0 >> frame0; // get a new frame from camera
		cap1 >> frame1;

		imshow("window0", frame0);//画像を表示．
		imshow("window1", frame1);//画像を表示．

		int key = waitKey(1);
		if (key == 113)//qボタンが押されたとき
		{
			break;//whileループから抜ける．
		}
		else if (key == 115)//sが押されたとき
		{
			imwrite("img0.png", frame0);
			imwrite("img1.png", frame1);
		}
	}
	destroyAllWindows();
}