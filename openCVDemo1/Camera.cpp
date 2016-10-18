#include "Camera.h";
#include "Const.h";
#include "opencv2/opencv.hpp";
#include <iostream>;
using namespace std;
using namespace cv;

int Camera::takePic()
{
	VideoCapture *cap = new VideoCapture[cameraNum];//�f�o�C�X�̃I�[�v��
	Mat *frame = new Mat[cameraNum];
	char **buf = new char*[cameraNum];
	for (int i = 0; i < cameraNum; i++) {
		cap[i] = VideoCapture(i);
		if (!cap[i].isOpened())//�J�����f�o�C�X������ɃI�[�v���������m�F�D
		{
			//�ǂݍ��݂Ɏ��s�����Ƃ��̏���
			cout << "�J����" << i << "�ǂݍ��݂Ɏ��s����" << endl;
			return 0;
		}
		buf[i] = new char[windowNameLength];
		sprintf(buf[i], windowName, i);
		cout << buf[i] << endl;
	}
	int count = 0;

	while (1)//�������[�v
	{
		for (int i = 0; i < cameraNum; i++) {
			cap[i] >> frame[i]; // get a new frame from camera
			imshow(buf[i], frame[i]);//�摜��\���D
		}

		int key = waitKey(1);
		if (key == 113)//q�{�^���������ꂽ�Ƃ�
		{
			break;//while���[�v���甲����D
		}
		else if (key == 115)//s�������ꂽ�Ƃ�
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