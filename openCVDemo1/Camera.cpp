#include "Camera.h";
#include "Const.h";
#include "opencv2/opencv.hpp";
#include <iostream>;
#include <afx.h>;
using namespace std;
using namespace cv;

int Camera::takePic()
{
	VideoCapture cap[cameraNum];//�f�o�C�X�̃I�[�v��\

	for (int i = 0; i < cameraNum; i++) {
		cap[i] = VideoCapture(i);
		if (!cap[i].isOpened())//�J�����f�o�C�X������ɃI�[�v���������m�F�D
		{
			//�ǂݍ��݂Ɏ��s�����Ƃ��̏���
			cout << "�J����" << i << "�ǂݍ��݂Ɏ��s����" << endl;
			return;
		}
	}
	int count = 0;
	while (1)//�������[�v
	{
		Mat frame[cameraNum];
		for (int i = 0; i < cameraNum; i++) {
			cap[i] >> frame[0]; // get a new frame from camera
			imshow("window" + i, frame[i]);//�摜��\���D
		}

		int key = waitKey(1);
		if (key == 113)//q�{�^���������ꂽ�Ƃ�
		{
			break;//while���[�v���甲����D
		}
		else if (key == 115)//s�������ꂽ�Ƃ�
		{
			for (int i = 0; i < cameraNum; i++) {
				char * savedImgName;
				sprintf(savedImgName, calibImgName, count, i);
				imwrite(savedImgName, frame[i]);
			}
			count++;
		}
	}
	destroyAllWindows();
}