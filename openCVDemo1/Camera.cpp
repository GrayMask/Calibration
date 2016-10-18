#include "Camera.h";
#include "Const.h";
#include "opencv2/opencv.hpp";
#include <iostream>;
using namespace std;
using namespace cv;

int Camera::takePic()
{
	VideoCapture *cap = new VideoCapture[cameraNum];//デバイスのオープン
	Mat *frame = new Mat[cameraNum];
	char **buf = new char*[cameraNum];
	for (int i = 0; i < cameraNum; i++) {
		cap[i] = VideoCapture(i);
		if (!cap[i].isOpened())//カメラデバイスが正常にオープンしたか確認．
		{
			//読み込みに失敗したときの処理
			cout << "カメラ" << i << "読み込みに失敗した" << endl;
			return 0;
		}
		buf[i] = new char[windowNameLength];
		sprintf(buf[i], windowName, i);
		cout << buf[i] << endl;
	}
	int count = 0;

	while (1)//無限ループ
	{
		for (int i = 0; i < cameraNum; i++) {
			cap[i] >> frame[i]; // get a new frame from camera
			imshow(buf[i], frame[i]);//画像を表示．
		}

		int key = waitKey(1);
		if (key == 113)//qボタンが押されたとき
		{
			break;//whileループから抜ける．
		}
		else if (key == 115)//sが押されたとき
		{
			for (int i = 0; i < cameraNum; i++) {
				char *savedImgName = new char[calibImgNameLength];
				sprintf(savedImgName, calibImgName, count, i);
				imwrite(savedImgName, frame[i]);
			}
			count++;
		}
	}
	destroyAllWindows();
	return count;
}