#include "Camera.h";
#include "opencv2/opencv.hpp";
#include <iostream>;
using namespace std;
using namespace cv;

int Camera::takePic()
{
	VideoCapture cap0(0);//�f�o�C�X�̃I�[�v��
	VideoCapture cap1(1);

	if (!cap0.isOpened() || !cap1.isOpened())//�J�����f�o�C�X������ɃI�[�v���������m�F�D
	{
		//�ǂݍ��݂Ɏ��s�����Ƃ��̏���
		cout << "�J�����ǂݍ��݂Ɏ��s����" << endl;
		return;
	}

	while (1)//�������[�v
	{
		Mat frame0;
		Mat frame1;
		cap0 >> frame0; // get a new frame from camera
		cap1 >> frame1;

		imshow("window0", frame0);//�摜��\���D
		imshow("window1", frame1);//�摜��\���D

		int key = waitKey(1);
		if (key == 113)//q�{�^���������ꂽ�Ƃ�
		{
			break;//while���[�v���甲����D
		}
		else if (key == 115)//s�������ꂽ�Ƃ�
		{
			imwrite("img0.png", frame0);
			imwrite("img1.png", frame1);
		}
	}
	destroyAllWindows();
}