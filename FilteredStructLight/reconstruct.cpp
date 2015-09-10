#include "reconstruct.h"



Reconstruct3D::Reconstruct3D(int no_of_cams) 
	: no_of_cams_(no_of_cams)
{

}


Reconstruct3D::~Reconstruct3D()
{
}

void Reconstruct3D::stereo_calibrate(CameraImgMap& camera_img_map, int left_cam, int right_cam, Size boardSize, bool useCalibrated) {

	if (camera_img_map.size() < 2) {
		std::cout << "Only " << camera_img_map.size() << " available. Need at least 2..." << std::endl;
		return;
	}

	assert(left_cam < no_of_cams_);
	assert(right_cam < no_of_cams_);
	assert(camera_img_map.size() == no_of_cams_);



	cv::Size imageSize;
	bool displayCorners = false;//true;
	const int maxScale = 2;
	const float squareSize = 11.f;  // Set this to your actual square size
	// ARRAY AND VECTOR STORAGE:

	vector<vector<Point2f> > imagePoints[2];
	vector<vector<Point3f> > objectPoints;

	int i, j, k, nimages = (int)camera_img_map[0].size() / 2;

	imagePoints[0].resize(nimages);
	imagePoints[1].resize(nimages);
	//vector<string> goodImageList;

	for (i = j = 0; i < nimages; i++)
	{
		for (k = 0; k < 2; k++)
		{
			int image_index = (k == 0) ? left_cam : right_cam;
			Mat& img = camera_img_map[image_index][i];

			if (img.empty())
				break;
			if (imageSize == Size())
				imageSize = img.size();
			else if (img.size() != imageSize)
			{
				cout << "The image has the size different from the first image size. Skipping the pair\n";
				break;
			}
			bool found = false;
			vector<Point2f>& corners = imagePoints[k][j];
			for (int scale = 1; scale <= maxScale; scale++)
			{
				Mat timg;
				if (scale == 1)
					timg = img;
				else
					resize(img, timg, Size(), scale, scale);
				found = findChessboardCorners(timg, boardSize, corners,
					CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_NORMALIZE_IMAGE);
				if (found)
				{
					if (scale > 1)
					{
						Mat cornersMat(corners);
						cornersMat *= 1. / scale;
					}
					break;
				}
			}
			if (displayCorners)
			{
				//cout << filename << endl;
				Mat cimg, cimg1;
				cvtColor(img, cimg, COLOR_GRAY2BGR);
				drawChessboardCorners(cimg, boardSize, corners, found);
				double sf = 640. / MAX(img.rows, img.cols);
				resize(cimg, cimg1, Size(), sf, sf);
				imshow("corners", cimg1);
				char c = (char)waitKey(500);
				if (c == 27 || c == 'q' || c == 'Q') //Allow ESC to quit
					exit(-1);
			}
			else
				putchar('.');
			if (!found)
				break;
			cornerSubPix(img, corners, Size(11, 11), Size(-1, -1),
				TermCriteria(CV_TERMCRIT_ITER + CV_TERMCRIT_EPS,
				30, 0.01));
		}
		if (k == 2)
		{
			//goodImageList.push_back(camera_img_map[left_cam][i]);
			//goodImageList.push_back(camera_img_map[right_cam][i]);
			j++;
		}
	}
	cout << j << " pairs have been successfully detected.\n";
	nimages = j;
	if (nimages < 2)
	{
		cout << "Error: too little pairs to run the calibration\n";
		return;
	}

	imagePoints[0].resize(nimages);
	imagePoints[1].resize(nimages);
	objectPoints.resize(nimages);

	for (i = 0; i < nimages; i++)
	{
		for (j = 0; j < boardSize.height; j++)
			for (k = 0; k < boardSize.width; k++)
				objectPoints[i].push_back(Point3f(j*squareSize, k*squareSize, 0));
	}

	cout << "Running stereo calibration ...\n";

	//cameraMatrix[0] = Mat::eye(3, 3, CV_64F);
	//cameraMatrix[1] = Mat::eye(3, 3, CV_64F);

	//double rms = stereoCalibrate(objectPoints, imagePoints[0], imagePoints[1],
	//	cameraMatrix[0], distCoeffs[0],
	//	cameraMatrix[1], distCoeffs[1],
	//	imageSize, R, T, E, F,
	//	TermCriteria(CV_TERMCRIT_ITER + CV_TERMCRIT_EPS, 100, 1e-5),
	//	CV_CALIB_FIX_ASPECT_RATIO +
	//	CV_CALIB_ZERO_TANGENT_DIST +
	//	//CV_CALIB_SAME_FOCAL_LENGTH +
	//	CV_CALIB_RATIONAL_MODEL +
	//	CV_CALIB_FIX_K3 + CV_CALIB_FIX_K4 + CV_CALIB_FIX_K5);
	//cout << "done with RMS error=" << rms << endl;

	//// CALIBRATION QUALITY CHECK
	//// because the output fundamental matrix implicitly
	//// includes all the output information,
	//// we can check the quality of calibration using the
	//// epipolar geometry constraint: m2^t*F*m1=0
	//double err = 0;
	//int npoints = 0;
	//vector<Vec3f> lines[2];
	//for (i = 0; i < nimages; i++)
	//{
	//	int npt = (int)imagePoints[0][i].size();
	//	Mat imgpt[2];
	//	for (k = 0; k < 2; k++)
	//	{
	//		imgpt[k] = Mat(imagePoints[k][i]);
	//		undistortPoints(imgpt[k], imgpt[k], cameraMatrix[k], distCoeffs[k], Mat(), cameraMatrix[k]);
	//		computeCorrespondEpilines(imgpt[k], k + 1, F, lines[k]);
	//	}
	//	for (j = 0; j < npt; j++)
	//	{
	//		double errij = fabs(imagePoints[0][i][j].x*lines[1][j][0] +
	//			imagePoints[0][i][j].y*lines[1][j][1] + lines[1][j][2]) +
	//			fabs(imagePoints[1][i][j].x*lines[0][j][0] +
	//			imagePoints[1][i][j].y*lines[0][j][1] + lines[0][j][2]);
	//		err += errij;
	//	}
	//	npoints += npt;
	//}
	//cout << "average reprojection err = " << err / npoints << endl;

	//// save intrinsic parameters
	//FileStorage fs("intrinsics.yml", CV_STORAGE_WRITE);
	//if (fs.isOpened())
	//{
	//	fs << "M1" << cameraMatrix[0] << "D1" << distCoeffs[0] <<
	//		"M2" << cameraMatrix[1] << "D2" << distCoeffs[1];
	//	fs.release();
	//}
	//else {
	//	cout << "Error: can not save the intrinsic parameters\n";
	//}



	//fs.open("extrinsics.yml", CV_STORAGE_WRITE);
	//if (fs.isOpened())
	//{
	//	fs << "R" << R << "T" << T << "imageSize" << imageSize;
	//	fs.release();
	//}
	//else {
	//	cout << "Error: can not save the intrinsic parameters\n";
	//}
}
