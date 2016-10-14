#include "Calib.h";
#include "Const.h";
#include "opencv2/opencv.hpp";
#include <iostream>;
using namespace std;
using namespace cv;

//#define IMAGE_NUM  (25)         /* 画像数 */
#define PAT_ROW    (7)          /* パターンの行数 */
#define PAT_COL    (10)         /* パターンの列数 */
#define PAT_SIZE   (PAT_ROW*PAT_COL)
//#define ALL_POINTS (IMAGE_NUM*PAT_SIZE)
#define CHESS_SIZE (24.0)       /* パターン1マスの1辺サイズ[mm] */

void Calib::Calibrate(const int image_pair_num)
{
	int i, j, k;
	int corner_count, found;
	int image_num = image_pair_num * 2;
	int *p_count = new int[image_num];
	int all_points = image_num * PAT_SIZE;
	IplImage **src_img = new IplImage*[image_num];
	CvSize pattern_size = cvSize(PAT_COL, PAT_ROW);
	CvPoint3D32f *objects = new CvPoint3D32f[all_points];
	CvPoint2D32f *corners = (CvPoint2D32f *)cvAlloc(sizeof(CvPoint2D32f) * all_points);
	CvMat object_points;
	CvMat image_points;
	CvMat point_counts;
	CvMat *intrinsic = cvCreateMat(3, 3, CV_32FC1);
	CvMat *rotation = cvCreateMat(1, 3, CV_32FC1);
	CvMat *translation = cvCreateMat(1, 3, CV_32FC1);
	CvMat *distortion = cvCreateMat(1, 4, CV_32FC1);

	// (1)キャリブレーション画像の読み込み
	for (i = 0; i < image_num; i++) {
		char buf[32];
		sprintf(buf, calibImgName, i);
		if ((src_img[i] = cvLoadImage(buf, CV_LOAD_IMAGE_COLOR)) == NULL) {
			fprintf(stderr, "cannot load image file : %s\n", buf);
		}
	}

	// (2)3次元空間座標の設定
	for (i = 0; i < image_num; i++) {
		for (j = 0; j < PAT_ROW; j++) {
			for (k = 0; k < PAT_COL; k++) {
				objects[i * PAT_SIZE + j * PAT_COL + k].x = j * CHESS_SIZE;
				objects[i * PAT_SIZE + j * PAT_COL + k].y = k * CHESS_SIZE;
				objects[i * PAT_SIZE + j * PAT_COL + k].z = 0.0;
			}
		}
	}
	cvInitMatHeader(&object_points, all_points, 3, CV_32FC1, objects);

	// (3)チェスボード（キャリブレーションパターン）のコーナー検出
	int found_num = 0;
	cvNamedWindow("Calibration", CV_WINDOW_AUTOSIZE);
	for (i = 0; i < image_num; i++) {
		found = cvFindChessboardCorners(src_img[i], pattern_size, &corners[i * PAT_SIZE], &corner_count);
		fprintf(stderr, "%02d...", i);
		if (found) {
			fprintf(stderr, "ok\n");
			found_num++;
		}
		else {
			fprintf(stderr, "fail\n");
		}
		// (4)コーナー位置をサブピクセル精度に修正，描画
		IplImage *src_gray = cvCreateImage(cvGetSize(src_img[i]), IPL_DEPTH_8U, 1);
		cvCvtColor(src_img[i], src_gray, CV_BGR2GRAY);
		cvFindCornerSubPix(src_gray, &corners[i * PAT_SIZE], corner_count,
			cvSize(3, 3), cvSize(-1, -1), cvTermCriteria(CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 20, 0.03));
		cvDrawChessboardCorners(src_img[i], pattern_size, &corners[i * PAT_SIZE], corner_count, found);
		p_count[i] = corner_count;
		cvShowImage("Calibration", src_img[i]);
		cvWaitKey(0);
	}
	cvDestroyWindow("Calibration");

	if (found_num != image_num)
		//return -1;
		return;
	cvInitMatHeader(&image_points, all_points, 1, CV_32FC2, corners);
	cvInitMatHeader(&point_counts, image_num, 1, CV_32SC1, p_count);

	double rms = cvStereoCalibrate(&object_points, camera1ImagePoints, camera2ImagePoints,
		cameraMatrix[0], distortionCoefficients[0],
		cameraMatrix[1], distortionCoefficients[1],
		imageSize, rotationMatrix, translation, essentialMatrix, fundamentalMatrix,
		TermCriteria(CV_TERMCRIT_ITER + CV_TERMCRIT_EPS, 100, 1e-5),
		CV_CALIB_FIX_ASPECT_RATIO +
		CV_CALIB_ZERO_TANGENT_DIST +
		CV_CALIB_SAME_FOCAL_LENGTH +
		CV_CALIB_RATIONAL_MODEL +
		CV_CALIB_FIX_K3 + CV_CALIB_FIX_K4 + CV_CALIB_FIX_K5);

	//// (5)内部パラメータ，歪み係数の推定
	//cvCalibrateCamera2(&object_points, &image_points, &point_counts, cvSize(src_img[0]->width, src_img[0]->height), intrinsic, distortion);

	//// (6)外部パラメータの推定
	//CvMat sub_image_points, sub_object_points;
	//int base = 0;
	//cvGetRows(&image_points, &sub_image_points, base * PAT_SIZE, (base + 1) * PAT_SIZE);
	//cvGetRows(&object_points, &sub_object_points, base * PAT_SIZE, (base + 1) * PAT_SIZE);
	//cvFindExtrinsicCameraParams2(&sub_object_points, &sub_image_points, intrinsic, distortion, rotation, translation);

	// (7)XMLファイルへの書き出し
	CvFileStorage *fs;
	fs = cvOpenFileStorage("camera.xml", 0, CV_STORAGE_WRITE);
	cvWrite(fs, "intrinsic", intrinsic);
	cvWrite(fs, "rotation", rotation);
	cvWrite(fs, "translation", translation); 
	cvWrite(fs, "distortion", distortion);
	cvReleaseFileStorage(&fs);

	for (i = 0; i < image_num; i++) {
		cvReleaseImage(&src_img[i]);
	}
}