#pragma once
#include "opencv2/opencv.hpp"
#include <QtCore>
#include "FlyCapture2.h"

using namespace FlyCapture2;

class Reconstruct3D : public QObject
{
	Q_OBJECT;

private:
	int no_of_cams_;

	struct ImageSet {
		int cam_no;
		std::vector<cv::Mat> images;
	};

public:
	
	void collect_calibration_imgs();
	Reconstruct3D(int no_of_cams);
	~Reconstruct3D();


};

