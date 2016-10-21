#include "Calib.h";
#include "Const.h";
#include "opencv2/opencv.hpp";
#include <iostream>;
using namespace std;
using namespace cv;

void Calib::Calibrate()
{
	int displayCorners = 1;
	int showUndistorted = 1;
	bool isVerticalStereo = false;//OpenCV can handle left-right
								  //or up-down camera arrangements
	const int maxScale = 1;

	//FILE* f = fopen(imageList, "rt");
	int i, j, lr, nframes, n = patRow * patCol, N = 0;
	vector<string> imageNames[2];
	vector<CvPoint3D32f> objectPoints;
	vector<CvPoint2D32f> points[2];
	vector<int> npoints;
	vector<uchar> active[2];
	vector<CvPoint2D32f> temp(n);
	CvSize imageSize = { 0,0 };
	// ARRAY AND VECTOR STORAGE:
	double M1[3][3], M2[3][3], D1[5], D2[5];
	double R[3][3], T[3], E[3][3], F[3][3];
	CvMat _M1 = cvMat(3, 3, CV_64F, M1);
	CvMat _M2 = cvMat(3, 3, CV_64F, M2);
	CvMat _D1 = cvMat(1, 5, CV_64F, D1);
	CvMat _D2 = cvMat(1, 5, CV_64F, D2);
	CvMat _R = cvMat(3, 3, CV_64F, R);
	CvMat _T = cvMat(3, 1, CV_64F, T);
	CvMat _E = cvMat(3, 3, CV_64F, E);
	CvMat _F = cvMat(3, 3, CV_64F, F);
	if (displayCorners)
		cvNamedWindow("corners", 1);
	// READ IN THE LIST OF CHESSBOARDS:
	//if (!f)
	//{
	//	fprintf(stderr, "can not open file %s\n", imageList);
	//	return;
	//}
	for (i = 0;; i++)
	{
		int count = 0, result = 0;
		lr = i % 2;
		vector<CvPoint2D32f>& pts = points[lr];

		char buf[32];
		int imagePairNum = i / 2;
		sprintf(buf, calibImgName, imagePairNum, lr);
		IplImage* img;
		if ((img = cvLoadImage(buf, 0)) == NULL) {
			//fprintf(stderr, "cannot load image file : %s\n", buf);
			break;
		}

		imageSize = cvGetSize(img);
		imageNames[lr].push_back(buf);
		//FIND CHESSBOARDS AND CORNERS THEREIN:
		for (int s = 1; s <= maxScale; s++)
		{
			IplImage* timg = img;
			if (s > 1)
			{
				timg = cvCreateImage(cvSize(img->width*s, img->height*s),
					img->depth, img->nChannels);
				cvResize(img, timg, CV_INTER_CUBIC);
			}
			result = cvFindChessboardCorners(timg, cvSize(patRow, patCol),
				&temp[0], &count,
				CV_CALIB_CB_ADAPTIVE_THRESH |
				CV_CALIB_CB_NORMALIZE_IMAGE);
			if (timg != img)
				cvReleaseImage(&timg);
			if (result || s == maxScale)
				for (j = 0; j < count; j++)
				{
					temp[j].x /= s;
					temp[j].y /= s;
				}
			if (result)
				break;
		}
		if (displayCorners)
		{
			printf("%s\n", buf);
			IplImage* cimg = cvCreateImage(imageSize, 8, 3);
			cvCvtColor(img, cimg, CV_GRAY2BGR);
			cvDrawChessboardCorners(cimg, cvSize(patRow, patCol), &temp[0],
				count, result);
			cvShowImage("corners", cimg);
			if (cvWaitKey(0) == 113) //Allow Q to next picture
				cvDestroyWindow("corners");
			cvReleaseImage(&cimg);
			if (cvWaitKey(0) == 27) //Allow ESC to quit
				exit(-1);
		}
		else
			putchar('.');
		N = pts.size();
		pts.resize(N + n, cvPoint2D32f(0, 0));
		active[lr].push_back((uchar)result);
		//assert( result != 0 );
		if (result)
		{
			//Calibration will suffer without subpixel interpolation
			cvFindCornerSubPix(img, &temp[0], count,
				cvSize(11, 11), cvSize(-1, -1),
				cvTermCriteria(CV_TERMCRIT_ITER + CV_TERMCRIT_EPS,
					30, 0.01));
			copy(temp.begin(), temp.end(), pts.begin() + N);
		}
		cvReleaseImage(&img);
	}
	//fclose(f);
	printf("\n");
	// HARVEST CHESSBOARD 3D OBJECT POINT LIST:
	nframes = active[0].size();//Number of good chessboads found

	N = nframes*n;
	objectPoints.resize(N);
	for (i = 0; i < patRow; i++)
		for (j = 0; j <patCol; j++)
			objectPoints[i*patRow + j] =
			cvPoint3D32f(i*squareSize, j*squareSize, 0);
	for (i = 1; i < nframes; i++)
		copy(objectPoints.begin(), objectPoints.begin() + n,
			objectPoints.begin() + i*n);
	npoints.resize(nframes, n);
	CvMat _objectPoints = cvMat(1, N, CV_32FC3, &objectPoints[0]);
	CvMat _imagePoints1 = cvMat(1, N, CV_32FC2, &points[0][0]);
	CvMat _imagePoints2 = cvMat(1, N, CV_32FC2, &points[1][0]);
	CvMat _npoints = cvMat(1, npoints.size(), CV_32S, &npoints[0]);
	cvSetIdentity(&_M1);
	cvSetIdentity(&_M2);
	cvZero(&_D1);
	cvZero(&_D2);

	// CALIBRATE THE STEREO CAMERAS
	printf("Running stereo calibration ...");
	fflush(stdout);
	cvStereoCalibrate(&_objectPoints, &_imagePoints1,
		&_imagePoints2, &_npoints,
		&_M1, &_D1, &_M2, &_D2,
		imageSize, &_R, &_T, &_E, &_F, CV_CALIB_FIX_ASPECT_RATIO +
		CV_CALIB_ZERO_TANGENT_DIST +
		CV_CALIB_SAME_FOCAL_LENGTH,
		cvTermCriteria(CV_TERMCRIT_ITER +
			CV_TERMCRIT_EPS, 100, 1e-5)
	);
	printf(" done\n");

	// XMLƒtƒ@ƒCƒ‹‚Ö‚Ì‘‚«o‚µ
	CvFileStorage *fs;
	fs = cvOpenFileStorage("camera.xml", 0, CV_STORAGE_WRITE);
	cvWrite(fs, "intrinsic-camera1", &_M1);
	cvWrite(fs, "intrinsic-camera2", &_M2);
	cvWrite(fs, "distortion-camera1", &_D1);
	cvWrite(fs, "distortion-camera2", &_D1);
	cvWrite(fs, "rotation", &_R);
	cvWrite(fs, "translation", &_T);
	cvReleaseFileStorage(&fs);

}