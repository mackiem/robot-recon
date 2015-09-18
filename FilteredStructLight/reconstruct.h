#pragma once
#include "opencv2/opencv.hpp"
#include <QtCore>
#include "FlyCapture2.h"
#include <unordered_map>

using namespace FlyCapture2;
using namespace std;
using namespace cv;

typedef std::unordered_map<int, std::vector<cv::Mat>> CameraImgMap;
typedef std::vector<std::pair<int, int>> CameraPairs;
typedef std::unordered_map<CameraPairs, cv::Mat> CameraPairMatrix;

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
	void clear_camera_img_map();

	void run_calibration(std::vector<std::pair<int, int>> camera_pairs);

	void stereo_calibrate(CameraImgMap& camera_img_map, int left_cam, int right_cam, Size boardSize, bool useCalibrated);
	
	Reconstruct3D(int no_of_cams, QObject* parent);
	~Reconstruct3D();

public slots:
	void collect_images(FlyCapture2::Image img, int cam_no);
};

