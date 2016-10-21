#include "Camera.h"
#include "Calib.h"

int main(int argh, char* argv[])
{
	int imgNum = Camera::takePic();
	//Calib::Calibrate(imgNum);
}