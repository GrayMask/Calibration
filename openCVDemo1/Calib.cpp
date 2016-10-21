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
	int image_num = image_pair_num * cameraNum;
	int *p_count = new int[image_num];
	int all_points = image_num * PAT_SIZE;
	IplImage ***src_img = new IplImage**[cameraNum];
	for (i = 0; i < image_pair_num; i++) {
		src_img[i] = new IplImage*[image_pair_num];
	}
	CvSize pattern_size = cvSize(PAT_COL, PAT_ROW);
	CvPoint3D32f *objects = new CvPoint3D32f[all_points];
	CvPoint2D32f **corners = (CvPoint2D32f **)cvAlloc(sizeof(CvPoint2D32f) * cameraNum);
	for (i = 0; i < cameraNum; i++) {
		corners[i] = (CvPoint2D32f *)cvAlloc(sizeof(CvPoint2D32f) * all_points);
	}
	CvMat object_points;
	CvMat **image_points = new CvMat*[cameraNum];
	CvMat point_counts;
	CvMat *intrinsic = new CvMat[cameraNum];
	CvMat *rotation = cvCreateMat(1, 3, CV_32FC1);
	CvMat *translation = cvCreateMat(1, 3, CV_32FC1);
	CvMat *distortion = new CvMat[cameraNum];
	CvMat *essentialMatrix;
	CvMat *fundamentalMatrix;

	// (1)キャリブレーション画像の読み込み
	for (i = 0; i < cameraNum; i++) {
		for (j = 0; j < image_pair_num; j++) {
			char buf[32];
			sprintf(buf, calibImgName, j, i);
			if ((src_img[i][j] = cvLoadImage(buf, CV_LOAD_IMAGE_COLOR)) == NULL) {
				fprintf(stderr, "cannot load image file : %s\n", buf);
			}
		}
		
	}

	// (2)3次元空間座標の設定
	for (i = 0; i < image_pair_num; i++) {
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
	for (i = 0; i < cameraNum; i++) {
		for (j = 0; j < image_pair_num; j++) {
			found = cvFindChessboardCorners(src_img[i][j], pattern_size, &corners[i][j * PAT_SIZE], &corner_count);
			fprintf(stderr, "%02d...", i);
			if (found) {
				fprintf(stderr, "ok\n");
				found_num++;
			}
			else {
				fprintf(stderr, "fail\n");
			}
			// (4)コーナー位置をサブピクセル精度に修正，描画
			IplImage *src_gray = cvCreateImage(cvGetSize(src_img[i][j]), IPL_DEPTH_8U, 1);
			cvCvtColor(src_img[i][j], src_gray, CV_BGR2GRAY);
			cvFindCornerSubPix(src_gray, &corners[i][j * PAT_SIZE], corner_count,
				cvSize(3, 3), cvSize(-1, -1), cvTermCriteria(CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 20, 0.03));
			cvDrawChessboardCorners(src_img[i][j], pattern_size, &corners[i][j * PAT_SIZE], corner_count, found);
			p_count[i] = corner_count;
			cvShowImage("Calibration", src_img[i][j]);
			cvWaitKey(0);
		}
	}
	cvDestroyWindow("Calibration");

	if (found_num != image_num)
		//return -1;
		return;
	for (i = 0; i < cameraNum; i++) {
		cvInitMatHeader(image_points[i], all_points, 1, CV_32FC2, corners[i]);
	}
	cvInitMatHeader(&point_counts, image_num, 1, CV_32SC1, p_count);

	double rms = cvStereoCalibrate(&object_points, image_points[0], image_points[1], &point_counts,
		&intrinsic[0], &distortion[0],
		&intrinsic[1], &distortion[1],
		cvSize(src_img[0][0]->width, src_img[0][0]->height), rotation, translation, essentialMatrix, fundamentalMatrix);

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

	for (i = 0; i < cameraNum; i++) {
		for (j = 0; j < image_pair_num; j++) {
		cvReleaseImage(&src_img[i][j]);
	}
}