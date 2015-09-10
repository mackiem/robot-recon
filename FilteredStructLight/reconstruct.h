#pragma once
#include "opencv2/opencv.hpp"
#include <QtCore>
#include "FlyCapture2.h"
#include <unordered_map>

using namespace FlyCapture2;
using namespace std;
using namespace cv;

typedef std::unordered_map<int, std::vector<cv::Mat>> CameraImgMap;

class Reconstruct3D : public QObject
{
	Q_OBJECT;

private:
	int no_of_cams_;

	struct ImageSet {
		int cam_no;
		std::vector<cv::Mat> images;
	};

	CameraImgMap camera_img_map_;

public:
	
	void collect_calibration_imgs();
	void stereo_calibrate(CameraImgMap& camera_img_map, int left_cam, int right_cam, Size boardSize, bool useCalibrated);
	Reconstruct3D(int no_of_cams);
	~Reconstruct3D();


};

