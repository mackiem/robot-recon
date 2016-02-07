#include "reconstruct.h"
#include <fstream>
#include <sstream>
#include <iostream>
#include <algorithm>
#include <omp.h>
#include <chrono>
#include <cmath>
#include "gl_core_3_3.h"
#include "gaussfit.h"
#include <QtWidgets/QMessageBox>
#include <iomanip>
#include <opencv2/video/background_segm.hpp>
#include <opencv2/video/background_segm.hpp>
#include "smoothopt.h"
#include <cmath>

#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/surface/mls.h>
#include <pcl/surface/poisson.h>
#include <pcl/io/vtk_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/bilateral.h>



typedef std::chrono::high_resolution_clock Clock;
typedef std::chrono::milliseconds milliseconds;

std::string Reconstruct3D::calib_dirname_ = "calibration";
std::string Reconstruct3D::camera_subdir_prefix_ = "camera_";
std::string Reconstruct3D::recon_dirname_ = "reconstruction";

Reconstruct3D::Reconstruct3D(int no_of_cams, QObject* parent) 
	: no_of_cams_(no_of_cams), QObject(parent), started_capture_(false)
{
	last_updated_ = Clock::now();
	board_size_ = cv::Size(12, 12);
}


Reconstruct3D::~Reconstruct3D()
{
}

void Reconstruct3D::collect_images(const FlyCapture2::Image& img, int cam_no)
{
	std::chrono::high_resolution_clock::time_point now = Clock::now();
	auto duration = std::chrono::duration_cast<std::chrono::milliseconds>( now - last_updated_ ).count();
	
	if (cam_no == 0 && duration > 5000)
	{
		started_capture_ = true;
	}

	if (started_capture_) {
		cv::Mat test(img.GetRows(), img.GetCols(), CV_8UC1, img.GetData(), img.GetStride());
		cv::Mat test2;
		//cv::cvtColor(test, colored, CV_BayerBG2BGR);
		cv::cvtColor(test, test2, CV_BayerBG2GRAY);
		camera_img_map_[cam_no].push_back(test2.clone());
		std::cout << "Image captured from " << cam_no << " captured..."<< std::endl;
	}

	if ((cam_no == no_of_cams_ - 1) && started_capture_)
	{
		std::cout << std::endl << std::endl;
		last_updated_ = Clock::now();
		started_capture_ = false;
	}
}

void Reconstruct3D::collect_images_without_delay(const FlyCapture2::Image& img, int cam_no)
{
	cv::Mat test(img.GetRows(), img.GetCols(), CV_8UC1, img.GetData(), img.GetStride());
	cv::Mat colored;
	cv::Mat test2 = test.clone();
	//cv::cvtColor(test, colored, CV_BayerBG2BGR);

	//cv::imshow("bayer 2 gray", test2);
	//cv::imshow("bayer 2 color", colored
	//cv::imshow("bayer ", test2);

	//std::cout << img.GetTimeStamp().cycleSeconds << std::endl;
	//std::cout << img.GetTimeStamp().cycleOffset<< std::endl;
	//std::cout << img.GetTimeStamp().cycleCount<< std::endl;
	//std::cout << img.GetTimeStamp().microSeconds << std::endl <<std::endl;

	camera_img_map_[cam_no].push_back(test2);
}

void Reconstruct3D::compute_correspondence(FlyCapture2::Image img, int cam_no)
{
}

void Reconstruct3D::clear_camera_img_map()
{
	camera_img_map_.clear();
}

void Reconstruct3D::calibrate(std::vector<std::pair<int, int>> camera_pairs)
{
	// first save the images for each camera
	QDir calibration_dir(calib_dirname_.c_str());
	if (!calibration_dir.exists()) {
		calibration_dir.mkdir(".");
	}

	for (auto itr = camera_img_map_.begin(); itr != camera_img_map_.end(); ++itr) {
		std::string camera_pair_subdirname =  camera_subdir_prefix_ + std::to_string(itr->first);
		std::string camera_pair_dir_path = calib_dirname_ + std::string("/") + camera_pair_subdirname;
		QDir camera_pair_dir(camera_pair_dir_path.c_str());
		if (camera_pair_dir.exists()) {
			// clear out all the files
			camera_pair_dir.setNameFilters(QStringList() << "*.*");
			camera_pair_dir.setFilter(QDir::Files);
			foreach(QString camera_pair_dir_file, camera_pair_dir.entryList()) {
				camera_pair_dir.remove(camera_pair_dir_file);
			}
		} else {
			camera_pair_dir.mkdir(".");
		}
		
		auto& images = itr->second;
		for (auto i = 0u; i < images.size(); ++i) {
			std::string filename = "camera_" + std::to_string(i) + std::string(".png");
			QString filepath = camera_pair_dir.filePath(filename.c_str());
			cv::imwrite(filepath.toStdString(), images[i]);
		}
	}
		
	// now calibrate each pair
	for (auto& camera_pair : camera_pairs)
	{
		stereo_calibrate(camera_img_map_, camera_pair.first, camera_pair.second, board_size_, true); 
	}
	
	//// test for now
	//for (auto i = 0u; i < camera_img_map_.size();++i)
	//{
	//	camera_img_map_[i].resize(20);
	//}
	//stereo_calibrate(camera_img_map_, camera_pairs[0].first, camera_pairs[0].second, cv::Size(9, 6), true); 
	//stereo_calibrate(camera_img_map_, camera_pairs[0].first, camera_pairs[0].second, cv::Size(12, 12), true); 
	
}

void Reconstruct3D::recalibrate(std::vector<std::pair<int, int>> camera_pairs)
{
	/*for (auto& camera_pair_ : camera_pairs)
	{
		stereo_calibrate(camera_img_map_, camera_pair_.first, camera_pair_.second, cv::Size(9, 6), true); 
	}*/
	
	// test for now
	//const int left_cam = camera_pairs[0].first;
	//const int right_cam = camera_pairs[0].second;

	//for (auto i = 0u; i < 20;++i)
	//{
	//	

	//		std::string left_img_name = "left0";  
	//		std::string right_img_name = "right0";

	//		cv::Mat left_img = imread(left_img_name + std::to_string(i+1) + std::string(".jpg"), 0);
	//		cv::Mat right_img = imread(right_img_name + std::to_string(i+1) + std::string(".jpg"), 0);
	//		if (left_img.data && right_img.data)
	//		{
	//			camera_img_map_[left_cam].push_back(left_img);
	//			camera_img_map_[right_cam].push_back(right_img);
	//		}
	//		
	//}
	//stereo_calibrate(camera_img_map_, camera_pairs[0].first, camera_pairs[0].second, cv::Size(12, 12), true, false); 
	
	// first load the images for each camera
	QDir calibration_dir(calib_dirname_.c_str());
	if (!calibration_dir.exists()) {
		//QMessageBox::warning(QMessageBox::Warning, "Calibration Error!", calibration_dir.dirName() + QString(" does not exist!"), QMessageBox::Ok);
		std::cout << "Calibration directory : " << calib_dirname_ << " does not exist!" << std::endl;
		return;
	}

	for (auto i = 0u; i < no_of_cams_; ++i) {
		// TODO change to serial num, maybe
		std::string camera_pair_subdirname =  camera_subdir_prefix_ + std::to_string(i);
		std::string camera_pair_dir_path = calib_dirname_ + std::string("/") + camera_pair_subdirname;
		QDir camera_pair_dir(camera_pair_dir_path.c_str());
		if (!camera_pair_dir.exists()) {
			std::cout << "images for camera " << i << " does not exist.";
			continue;
		}
		QDirIterator it(camera_pair_dir.absolutePath(), QStringList() << "*.png", QDir::Files);
		while (it.hasNext()) {
			cv::Mat img = cv::imread(it.next().toStdString(), 0);
			camera_img_map_[i].push_back(img);
		}
	}

	// now calibrate each pair
	for (auto& camera_pair : camera_pairs)
	{
		stereo_calibrate(camera_img_map_, camera_pair.first, camera_pair.second, board_size_, true); 
	}

}

void Reconstruct3D::run_reconstruction(std::vector<std::pair<int, int>> camera_pairs, int no_of_images)
{
	// first save the images for each camera
	QDir reconstruction_dir(recon_dirname_.c_str());
	if (!reconstruction_dir.exists()) {
		reconstruction_dir.mkdir(".");
	}

	for (auto itr = camera_img_map_.begin(); itr != camera_img_map_.end(); ++itr) {
		std::string camera_pair_subdirname =  camera_subdir_prefix_ + std::to_string(itr->first);
		std::string camera_pair_dir_path = recon_dirname_ + std::string("/") + camera_pair_subdirname;
		QDir camera_pair_dir(camera_pair_dir_path.c_str());
		if (camera_pair_dir.exists()) {
			// clear out all the files
			camera_pair_dir.setNameFilters(QStringList() << "*.*");
			camera_pair_dir.setFilter(QDir::Files);
			foreach(QString camera_pair_dir_file, camera_pair_dir.entryList()) {
				camera_pair_dir.remove(camera_pair_dir_file);
			}
		} else {
			camera_pair_dir.mkdir(".");
		}
		
		auto& images = itr->second;
		for (auto i = 0u; i < images.size(); ++i) {
			std::ostringstream ss;
			ss << std::setw(5) << std::setfill('0') << i ;
			std::string s2(ss.str());
			std::string filename = "camera_" + s2 + std::string(".png");
			QString filepath = camera_pair_dir.filePath(filename.c_str());
			cv::Mat converted_img;
			cv::cvtColor(images[i], converted_img, CV_BayerBG2GRAY);
			cv::imwrite(filepath.toStdString(), converted_img);
			images[i] = converted_img;
			std::cout << filepath.toStdString() << std::endl;
		}
	}

	reconstruct(camera_pairs, no_of_images);

}



void Reconstruct3D::stereo_calibrate(CameraImgMap& camera_img_map, int left_cam, int right_cam, cv::Size boardSize, bool useCalibrated, bool write_images) {

	if (camera_img_map.size() < 2) {
		std::cout << "Only " << camera_img_map.size() << " available. Need at least 2..." << std::endl;
		return;
	}

	if (left_cam >= no_of_cams_ || (right_cam >= no_of_cams_)) {
		std::cout << left_cam << " or " << right_cam << " greater than no of cams : " << no_of_cams_ << std::endl;
		return;
	}
//	assert(camera_img_map.size() == no_of_cams_);

	
	bool displayCorners = true;//true;
	const int maxScale = 2;
	const float squareSize = 38.1f;  // Set this to your actual square size
	//const float squareSize = 29.1f;  // Set this to your actual square size
	
	// ARRAY AND VECTOR STORAGE:
	vector<vector<cv::Point2f> > imagePoints[2];
	vector<vector<cv::Point3f> > objectPoints;

	int i, j, k, nimages = (int)camera_img_map[left_cam].size();

	imagePoints[0].resize(nimages);
	imagePoints[1].resize(nimages);
	//vector<string> goodImageList;
	int good_images_found = 0;
	for (i = j = 0; i < nimages; i++)
	{
		for (k = 0; k < 2; k++)
		{
			int image_index = (k == 0) ? left_cam : right_cam;
			cv::Mat& img = camera_img_map[image_index][i];
			
			std::string img_name = (k == 0) ? "left0" : "right0";
			img_name += std::to_string(i+1);

			if (img.empty())
				break;
			if (imageSize == cv::Size())
				imageSize = img.size();
			else if (img.size() != imageSize)
			{
				std::cout << "The image has the size different from the first image size. Skipping the pair\n";
				break;
			}
			bool found = false;
			vector<cv::Point2f>& corners = imagePoints[k][j];
			for (int scale = 1; scale <= maxScale; scale++)
			{
				cv::Mat timg;
				if (scale == 1)
					timg = img;
				else
					resize(img, timg, cv::Size(), scale, scale);
				found = findChessboardCorners(timg, boardSize, corners,
					CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_NORMALIZE_IMAGE
					);
				if (found)
				{
					if (scale > 1)
					{
						cv::Mat cornersMat(corners);
						cornersMat *= 1. / scale;
					}
					break;
				}
			}
			
			
				
			if (!found)
				break;
			cv::cornerSubPix(img, corners, cv::Size(11, 11), cv::Size(-1, -1),
			                 cv::TermCriteria(CV_TERMCRIT_ITER + CV_TERMCRIT_EPS,
				30, 0.01));
			if (displayCorners)
			{
				//cout << filename << endl;
				cv::Mat cimg, cimg1;
				cvtColor(img, cimg, cv::COLOR_GRAY2BGR);
				drawChessboardCorners(cimg, boardSize, corners, found);
				//double sf = 640. / MAX(img.rows, img.cols);
				//resize(cimg, cimg1, Size(), sf, sf);
				cv::imshow(img_name, cimg);
				if (write_images)
				{
					imwrite(img_name  + std::string(".png"), cimg);
				}
				char c = (char)cv::waitKey(500);
				if (c == 27 || c == 'q' || c == 'Q') //Allow ESC to quit
					exit(-1);
			}
			else
				putchar('.');
			good_images_found++;

			//for (auto x = 0; x < corners.size(); ++x) {
			//	corners[x].y = imageSize.height - corners[x].y;
			//	corners[x].x = imageSize.width - corners[x].x;
			//}
			
		}
		if (k == 2)
		{
			j++;	
		}
		
		/*if (good_images_found > 20)
		{
			break;
		}*/
		
	}
	std::cout << j << " pairs have been successfully detected.\n";
	
	nimages = j;
	//nimages = (j < 10) ? j : 10;
	//cout << " only using 10 of them... \n";
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
		//for (j = boardSize.height - 1; j >= 0; --j)
		for (j = 0; j < boardSize.height; j++)
			for (k = 0; k < boardSize.width; k++)
				//for (k = boardSize.width - 1; k >= 0; --k)
				objectPoints[i].push_back(cv::Point3f(j*squareSize, k*squareSize, 0));
	}

	cout << "Running stereo calibration ...\n";



	cameraMatrix[0] = cv::Mat::eye(3, 3, CV_64F);
	cameraMatrix[1] = cv::Mat::eye(3, 3, CV_64F);

	cameraMatrix[0] = initCameraMatrix2D(objectPoints, imagePoints[0], imageSize, 1);
	cameraMatrix[1] = initCameraMatrix2D(objectPoints, imagePoints[1], imageSize, 1);

	std::cout << "intrinsic mat 1 : " << cameraMatrix[0] << std::endl;
	std::cout << "intrinsic mat 2 : " << cameraMatrix[1] << std::endl;

	double rms = stereoCalibrate(objectPoints, imagePoints[0], imagePoints[1],
		cameraMatrix[0], distCoeffs[0],
		cameraMatrix[1], distCoeffs[1],
		imageSize, R, T, E, F,
		cv::TermCriteria(CV_TERMCRIT_ITER + CV_TERMCRIT_EPS, 555, 1e-5),
		CV_CALIB_USE_INTRINSIC_GUESS +
		//CV_CALIB_FIX_ASPECT_RATIO +
		CV_CALIB_ZERO_TANGENT_DIST +
		//CV_CALIB_SAME_FOCAL_LENGTH +
		CV_CALIB_RATIONAL_MODEL +
		CV_CALIB_FIX_K3 + CV_CALIB_FIX_K4 + 
		CV_CALIB_FIX_K5);
	std::cout << "done with RMS error=" << rms << " px" << std::endl;

	// CALIBRATION QUALITY CHECK
	// because the output fundamental matrix implicitly
	// includes all the output information,
	// we can check the quality of calibration using the
	// epipolar geometry constraint: m2^t*F*m1=0
	double err = 0;
	int npoints = 0;
	vector<cv::Vec3f> lines[2];
	for (i = 0; i < nimages; i++)
	{
		int npt = (int)imagePoints[0][i].size();
		cv::Mat imgpt[2];
		for (k = 0; k < 2; k++)
		{
			imgpt[k] = cv::Mat(imagePoints[k][i]);
			undistortPoints(imgpt[k], imgpt[k], cameraMatrix[k], distCoeffs[k], cv::Mat(), cameraMatrix[k]);
			computeCorrespondEpilines(imgpt[k], k + 1, F, lines[k]);
		}
		for (j = 0; j < npt; j++)
		{
			double errij = fabs(imagePoints[0][i][j].x*lines[1][j][0] +
				imagePoints[0][i][j].y*lines[1][j][1] + lines[1][j][2]) +
				fabs(imagePoints[1][i][j].x*lines[0][j][0] +
				imagePoints[1][i][j].y*lines[0][j][1] + lines[0][j][2]);
			err += errij;
		}
		npoints += npt;
	}
	std::cout << "average reprojection err = " << err / npoints << " px" << std::endl;



	// save intrinsic parameters
	cv::FileStorage fs(generate_intrinsics_filename(left_cam, right_cam), CV_STORAGE_WRITE);
	if (fs.isOpened())
	{
		fs << "M1" << cameraMatrix[0] << "D1" << distCoeffs[0] <<
			"M2" << cameraMatrix[1] << "D2" << distCoeffs[1];
		fs.release();
	}
	else {
		cout << "Error: can not save the intrinsic parameters\n";
	}



	fs.open(generate_extrinsics_filename(left_cam, right_cam), CV_STORAGE_WRITE);
	if (fs.isOpened())
	{
		fs << "R" << R << "T" << T << "imageSize" << imageSize;
		fs.release();
	}
	else {
		cout << "Error: can not save the intrinsic parameters\n";
	}


	create_rectification_map();
	
	    // OpenCV can handle left-right
    // or up-down camera arrangements
    bool isVerticalStereo = fabs(P2.at<double>(1, 3)) > fabs(P2.at<double>(0, 3));

	cv::Mat rmap[2][2];
// IF BY CALIBRATED (BOUGUET'S METHOD)
    if( useCalibrated )
    {
        // we already computed everything
    }
// OR ELSE HARTLEY'S METHOD
    else
 // use intrinsic parameters of each camera, but
 // compute the rectification transformation directly
 // from the fundamental matrix
    {
        vector<cv::Point2f> allimgpt[2];
        for( k = 0; k < 2; k++ )
        {
            for( i = 0; i < nimages; i++ )
                std::copy(imagePoints[k][i].begin(), imagePoints[k][i].end(), back_inserter(allimgpt[k]));
        }
        F = cv::findFundamentalMat(cv::Mat(allimgpt[0]), cv::Mat(allimgpt[1]), cv::FM_8POINT, 0, 0);
        cv::Mat H1, H2;
        stereoRectifyUncalibrated(cv::Mat(allimgpt[0]), cv::Mat(allimgpt[1]), F, imageSize, H1, H2, 3);

        R1 = cameraMatrix[0].inv()*H1*cameraMatrix[0];
        R2 = cameraMatrix[1].inv()*H2*cameraMatrix[1];
        P1 = cameraMatrix[0];
        P2 = cameraMatrix[1];
    }

    //Precompute maps for cv::remap()
    initUndistortRectifyMap(cameraMatrix[0], distCoeffs[0], R1, P1, imageSize, CV_16SC2, rmap[0][0], rmap[0][1]);
    initUndistortRectifyMap(cameraMatrix[1], distCoeffs[1], R2, P2, imageSize, CV_16SC2, rmap[1][0], rmap[1][1]);

    cv::Mat canvas;
    double sf;
    int w, h;
    if( !isVerticalStereo )
    {
        sf = 600./MAX(imageSize.width, imageSize.height);
        w = cvRound(imageSize.width*sf);
        h = cvRound(imageSize.height*sf);
        canvas.create(h, w*2, CV_8UC3);
    }
    else
    {
        sf = 300./MAX(imageSize.width, imageSize.height);
        w = cvRound(imageSize.width*sf);
        h = cvRound(imageSize.height*sf);
        canvas.create(h*2, w, CV_8UC3);
    }

    for( i = 0; i < nimages; i++ )
    {
        for( k = 0; k < 2; k++ )
        {
			int image_index = (k == 0) ? left_cam : right_cam;
			cv::Mat img = camera_img_map[image_index][i];
             //= imread(goodImageList[i*2+k], 0, 
			cv::Mat rimg, cimg;
            remap(img, rimg, rmap[k][0], rmap[k][1], CV_INTER_LINEAR);
            cvtColor(rimg, cimg, cv::COLOR_GRAY2BGR);
            cv::Mat canvasPart = !isVerticalStereo ? canvas(cv::Rect(w*k, 0, w, h)) : canvas(cv::Rect(0, h*k, w, h));
            resize(cimg, canvasPart, canvasPart.size(), 0, 0, CV_INTER_AREA);
            if( useCalibrated )
            {
                cv::Rect vroi(cvRound(validRoi[k].x*sf), cvRound(validRoi[k].y*sf),
                          cvRound(validRoi[k].width*sf), cvRound(validRoi[k].height*sf));
                rectangle(canvasPart, vroi, cv::Scalar(0,0,255), 3, 8);
            }
        }

        if( !isVerticalStereo )
            for( j = 0; j < canvas.rows; j += 16 )
                line(canvas, cv::Point(0, j), cv::Point(canvas.cols, j), cv::Scalar(0, 255, 0), 1, 8);
        else
            for( j = 0; j < canvas.cols; j += 16 )
                line(canvas, cv::Point(j, 0), cv::Point(j, canvas.rows), cv::Scalar(0, 255, 0), 1, 8);
        imshow("rectified" + std::to_string(left_cam) + std::string("_") + std::to_string(right_cam), canvas);
		


        char c = (char)cv::waitKey();
        if( c == 27 || c == 'q' || c == 'Q' )
            break;
	}

	// project and write images
	//for (auto z = 0u; z < objectPoints.size(); ++z) {
	//	cv::Mat left_img = camera_img_map[left_cam][i];
	//	cv::Mat right_img = camera_img_map[right_cam][i];

	//	for (auto i = 0u; i < imagePoints[0].size(); ++i) {
	//		cv::cv::Mat points_vec(4, 1, CV_64F);
	//		for (auto n  = 0u; n < imagePoints[0][i].size(); ++n) {
	//			for (auto x = 0u; x < 3; ++x) {
	//				points_vec.at<double>(x, 0) = image_poin
	//			}
	//			
	//		}
	//	}

	//}
}



void Reconstruct3D::resize_img(cv::Mat& img, const unsigned int resize_width) const {
    //if (img.size().width > resize_width) {
        float aspect_ratio = (float)(img.size().width) / (float)(img.size().height);
        int operating_height = (int)((float)(resize_width) / aspect_ratio);
        cv::resize(img, img, cv::Size(resize_width, operating_height), cv::INTER_NEAREST);
    //}
}

//void Reconstruct3D::resize_img(cv::Mat& img) const {
//	resize_img(img, OPERATING_WIDTH);
//}
//
//void Reconstruct3D::resize_segmentation_img(cv::Mat& img) const {
//	resize_img(img, SEGMENT_OPERATING_WIDTH);
//}

std::string Reconstruct3D::generate_intrinsics_filename(int left_num, int right_num) {
	std::string filename = "intrinsics_" + std::to_string(left_num) + "_" + std::to_string(right_num) + ".yml";
	return filename;
}

std::string Reconstruct3D::generate_extrinsics_filename(int left_num, int right_num) {
	std::string filename = "extrinsics_" + std::to_string(left_num) + "_" + std::to_string(right_num) + ".yml";
	return filename;
}

void Reconstruct3D::load_calibration(int left_cam, int right_cam) {
    // save intrinsic parameters
	cv::FileStorage fs(generate_intrinsics_filename(left_cam, right_cam), CV_STORAGE_READ);
    if (fs.isOpened())
    {
        fs["M1"] >> cameraMatrix[0];
        fs["D1"] >> distCoeffs[0];
        fs["M2"] >> cameraMatrix[1];
        fs["D2"] >> distCoeffs[1];
        fs.release();
    }
    else {
        cout << "Error: can not load the intrinsic parameters\n";
        return;
    }



    fs.open(generate_extrinsics_filename(left_cam, right_cam), CV_STORAGE_READ);
    if (fs.isOpened())
    {
        fs["R"] >> R;
        fs["T"] >> T;
		fs["imageSize"] >> imageSize;
        fs.release();
    }
    else {
        cout << "Error: can not save the intrinsic parameters\n";
        return;
    }

    calibration_loaded_ = true;

}


void Reconstruct3D::create_rectification_map() {
    if (!calibration_loaded_) {
        std::cout << "Calibration not loaded! Please load file...";
        return;
    }

    stereoRectify(cameraMatrix[0], distCoeffs[0],
        cameraMatrix[1], distCoeffs[1],
        imageSize, R, T, R1, R2, P1, P2, Q,
        //CALIB_ZERO_DISPARITY, 1, imageSize, &validRoi[0], &validRoi[1]);
        0, 1, imageSize, &validRoi[0], &validRoi[1]);
    
    initUndistortRectifyMap(cameraMatrix[0], distCoeffs[0], R1, P1, imageSize, CV_16SC2, rmap[0][0], rmap[0][1]);
    initUndistortRectifyMap(cameraMatrix[1], distCoeffs[1], R2, P2, imageSize, CV_16SC2, rmap[1][0], rmap[1][1]);

	std::cout << "Proj 1" << P1<< std::endl;
	std::cout << "Proj 2" << P2<< std::endl;

}

void convert_2_left_camera_color(const cv::Mat& img, const cv::Mat& inter_camera_color_mtx, cv::Mat& output_img) {
	output_img = cv::Mat::zeros(cv::Size(img.cols, img.rows), img.type());
	for (auto i = 0u; i < img.rows; ++i) {
		for (auto j = 0u; j < img.cols; ++j) {
			 cv::Mat tmp = inter_camera_color_mtx * cv::Mat(cv::Vec3d(img.at<cv::Vec3b>(i, j)));
			 output_img.at<cv::Vec3b>(i, j) = cv::Vec3b(tmp);
		}
	}
}


void Reconstruct3D::pre_process_img(cv::Mat& input, cv::Mat& output, bool is_right) {
	cv::Mat thresholded;
	cv::threshold(input, thresholded, 50, 255, CV_THRESH_TOZERO);
	remap(thresholded, output, rmap[is_right][0], rmap[is_right][1], CV_INTER_CUBIC);
}

void Reconstruct3D::init_imgs(CameraImgMap& camera_img_map, int cam, bool is_right) {

	assert(cam < no_of_cams_);
	//assert(camera_img_map.size() == no_of_cams_);
	assert(camera_img_map.find(cam) != camera_img_map.end());

	auto& camera_imgs = camera_img_map[cam];
	std::string img_name = "remap_" + std::to_string(cam) + ".png";
	for (int i = 0; i < camera_imgs.size(); i++) {
		cv::Mat& img = camera_imgs[i];
		cv::Mat rImg;
		pre_process_img(img, rImg, is_right);
		camera_imgs[i] = rImg;

		imwrite(img_name, rImg);
		cv::cvtColor(rImg, rImg, CV_GRAY2BGR);
		cv::rectangle(rImg, validRoi[is_right], cv::Scalar(0, 0, 255));
		imshow(img_name, rImg);
	}
}


//This colors the segmentations
static void floodFillPostprocess(cv::Mat& img, const cv::Scalar& colorDiff = cv::Scalar::all(1))
{
    CV_Assert(!img.empty());
    cv::RNG rng = cv::theRNG();
    cv::Mat mask(img.rows + 2, img.cols + 2, CV_8UC1, cv::Scalar::all(0));
    for (int y = 0; y < img.rows; y++)
    {
        for (int x = 0; x < img.cols; x++)
        {
            if (mask.at<uchar>(y + 1, x + 1) == 0)
            {
                cv::Vec3b color = img.at<cv::Vec3b>(y, x);
                //cv::Scalar newVal( rng(256), rng(256), rng(256) );
                cv::Scalar newVal(color[0], color[1], color[2]);
                cv::floodFill(img, mask, cv::Point(x, y), newVal, 0, colorDiff, colorDiff);
            }
        }
    }
}


void Reconstruct3D::get_unique_edges(CameraImgMap& camera_img_map, int cam, std::vector<UniqueEdges> & unique_colors_all_images,
                                     bool is_right) {

	int color_threshold = 100;

  	auto& camera_imgs = camera_img_map[cam];
	unique_colors_all_images.resize(camera_imgs.size());

	for (int i = 0; i < camera_imgs.size(); i++) {    
		cv::Mat& img = camera_imgs[i];
		int l = 20;
		cv::Canny(img, img, l, l * 3, 3);
		std::string filename = (is_right) ? "edge_detected_image_right.png" : "edge_detected_image_left.png";
		cv::imwrite(filename, img);
		cv::Mat circle_img;
		cv::cvtColor(img, circle_img, CV_GRAY2BGR);
		UniqueEdges unique_edges;

		for (int row = 0; row < img.rows; ++row) {
			std::vector<int> unique_edges_in_row;
			int expanded_row = 0;
			int expanded_col = 0;
            cv::Vec3i prev_pre_color(0);
            cv::Vec3i prev_post_color(0);
            int unique_colors_no = 0;
			
			for (int col = 0; col < img.cols; ++col) {
				if (col - 1 >= 0) {
					unsigned char pre_color = img.at<unsigned char>(row, col - 1);
					unsigned char post_color = img.at<unsigned char>(row, col);
					
					if ((pre_color - post_color) > color_threshold) {
						unique_edges_in_row.push_back(col - 1);

						// draw a circle at the point
						//int w = img.cols;

						//int thickness = 0;
						//int lineType = 8;

						//cv::circle(circle_img,
						//	cv::Point(col, row),
						//	w / (32.0 * 4.0),
						//	cv::Scalar(0, 255, 0),
						//	thickness,
						//	lineType);
						circle_img.at<cv::Vec3b>(row, col - 1) = cv::Vec3b(0, 255, 0);
					}
				}	
			}
            unique_edges[row] = unique_edges_in_row;
			//if (unique_edges_in_row.size() > 0) {
			//	std::cout << "Unique edges in row " << row << " - " <<unique_edges_in_row.size() << std::endl;
			//}
		}

        unique_colors_all_images[i] = unique_edges;

		imshow(std::string("unique edges_") + std::to_string(is_right), circle_img);
		imwrite(std::string("unique edges_") + std::to_string(is_right) + ".png", circle_img);
    }
}


void Reconstruct3D::compute_correlation(IPts& img_pts1, IPts& img_pts2,
		CameraImgMap& camera_img_map, std::vector<std::pair<int, int>> camera_pairs) {


    std::vector<cv::Mat> imgs_left;
    std::vector<cv::Mat> imgs_right;
    init_imgs(camera_img_map, camera_pairs[0].first, false);
    init_imgs(camera_img_map, camera_pairs[0].second, true);


    std::unordered_map<int, std::unordered_map< int, std::vector<UniqueColorPair> > > unique_colors_left;
	std::vector<UniqueEdges> unique_edges_left;
	std::vector<UniqueEdges> unique_edges_right;
	
    get_unique_edges(camera_img_map, camera_pairs[0].first, unique_edges_left, false);
    get_unique_edges(camera_img_map, camera_pairs[0].second, unique_edges_right, true);


    std::vector < std::vector<int> > correlations;

    img_pts1.clear();
    img_pts2.clear();

    pick_correlated_points(unique_edges_left, unique_edges_right, img_pts1, img_pts2, 
		camera_img_map, camera_pairs[0].first, camera_pairs[0].second);


    write_file("recon.cp", img_pts1, img_pts2);
    //read_input_file("recon.cp", img_pts1, img_pts2);
}

void Reconstruct3D::compute_correlation_using_gaussian(IPts& img_pts1, IPts& img_pts2, 
													   CameraImgMap& camera_img_map, std::vector<std::pair<int, int>> camera_pairs,
													   Intensities& left_intensities, Intensities& right_intensities) {

    init_imgs(camera_img_map, camera_pairs[0].first, false);
    init_imgs(camera_img_map, camera_pairs[0].second, true);

	typedef std::unordered_map<int, int> GaussMidPoints;

	correpond_with_gaussians(camera_img_map, camera_pairs[0].first, camera_pairs[0].second, img_pts1, img_pts2, left_intensities, right_intensities);
    write_file("recon.cp", img_pts1, img_pts2);
}

void Reconstruct3D::write_file(const std::string& file_name, const IPts& img_pts1, const IPts& img_pts2) {
    std::ofstream file;
    file.open(file_name);
    for (int img = 0; img < img_pts1.size(); ++img) {
		for (auto i = 0u; i < img_pts1[img].size(); ++i) {
			file << img_pts1[img][i][0] << "\t";
			file << img_pts1[img][i][1] << "\t";
			file << img_pts2[img][i][0] << "\t";
			file << img_pts2[img][i][1] << "\n";
		}
    }
    file.close();
}

void Reconstruct3D::read_file(const std::string& file_name, IPts& img_pts1, IPts& img_pts2) {
    std::ifstream file;
    file.open(file_name);
	
	std::string line;
	std::vector<int> points;
	while (std::getline(file, line)) {
		std::stringstream stringstream(line);

		std::string single_coordinate;
		while (std::getline(stringstream, single_coordinate, '\t')) {
			points.push_back(std::stoi(single_coordinate));
		}
	}

	for (auto i = 0u; i < points.size(); i += 4) {
		img_pts1[i][0] = points[4 * i];
		img_pts1[i][1] = points[4 * i + 1];
		img_pts2[i][0] = points[4 * i + 2];
		img_pts2[i][1] = points[4 * i + 3];
	}

    file.close();
}
    
void Reconstruct3D::pick_correlated_points(std::vector<UniqueEdges>& unique_colors_left,
    std::vector<UniqueEdges>& unique_colors_right,
    IPts& img_pts1, IPts& img_pts2, CameraImgMap& camera_img_map, int left_cam, int right_cam) {


		bool one_point = false;
		for (auto j = 0u; j < unique_colors_left.size(); ++j)
		{
			auto& unique_colors_per_image_left = unique_colors_left[j];
			auto& unique_colors_per_image_right = unique_colors_right[j];

			assert( unique_colors_per_image_left.size() == unique_colors_per_image_right.size());

			cv::Mat& left_img = camera_img_map[left_cam][j];
			cv::Mat& right_img = camera_img_map[right_cam][j];

			cv::Mat left_correspond_img;
			cv::cvtColor(left_img, left_correspond_img, CV_GRAY2BGR);

			cv::Mat right_correspond_img;
			cv::cvtColor(right_img, right_correspond_img, CV_GRAY2BGR);

			for (auto ucliter = unique_colors_per_image_left.begin(); ucliter != unique_colors_per_image_left.end(); ++ucliter)
			{
				auto& row_edges_left = *ucliter;
				int row_no = row_edges_left.first;

				auto ucr_iter = unique_colors_per_image_right.find(row_no);
				if (ucr_iter == unique_colors_per_image_right.end())
				{
					continue;
				}

				auto& row_edges_right = *ucr_iter;

				auto& row_edges_vec_right = row_edges_right.second;
				auto& row_edges_vec_left = row_edges_left.second;

				if (row_edges_vec_right.size() != row_edges_vec_left.size()
					|| row_edges_vec_right.size() != 2)
				{
					continue;	
				}

				for (auto i = 0u; i < row_edges_vec_left.size(); ++i) {
					//img_pts1.push_back(cv::Vec2f(row_no, row_edges_vec_left[i]));
					//img_pts2.push_back(cv::Vec2f(row_no, row_edges_vec_right[i]));
					int left_col = row_edges_vec_left[i];
					int right_col = row_edges_vec_right[i];
					//img_pts1.push_back(cv::Vec2f(left_col, row_no));
					//img_pts2.push_back(cv::Vec2f(right_col, row_no));
					//img_pts1.push_back(cv::Vec2f(row_no, left_col));
					//img_pts2.push_back(cv::Vec2f(row_no, right_col));

					// draw a circle at the point
					//int w = left_img.cols;

					//int thickness = 0;
					//int lineType = 8;
					//cv::circle(correspond_img,
					//	cv::Point(col, row_no),
					//	w / (32.0 * 4.0),
					//	cv::Scalar(0, 255, 0),
					//	thickness,
					//	lineType);
					cv::Vec3b color = (i == 0) ? cv::Vec3b(0, 255, 0) : cv::Vec3b(0, 0, 255);

					left_correspond_img.at<cv::Vec3b>(row_no, left_col) = color;
					right_correspond_img.at<cv::Vec3b>(row_no, right_col) = color;

					//if (img_pts1.size() > 3) {
					//	one_point = true;
					//	break;
					//}
				}
				if (one_point) {
					break;
				}
			}
			imshow("Left Correspondence Image", left_correspond_img);
			imwrite("left_correspondence.png", left_correspond_img);
			imshow("Right Correspondence Image", left_correspond_img);
			imwrite("right_correspondence.png", right_correspond_img);
		}

}

void Reconstruct3D::fit_gaussians(CameraImgMap& camera_img_map, int cam_no, std::unordered_map<int, std::pair<int, int>> mid_points) {
	auto& imgs = camera_img_map[cam_no];

	for (auto i = 0u; i < imgs.size(); ++i) {
		cv::Mat& img = imgs[i];
		
		cv::Mat row_sum;
		cv::reduce(img, row_sum, 1, CV_REDUCE_SUM, CV_32S);

		for (auto j = 0u; j < row_sum.rows; ++j) {
			if (row_sum.at<float>(0, j) >= 10) {
				
			} else {
				
			}
		}
	}

}

void Reconstruct3D::convert(const WPts& world_pts, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
	for (auto img = 0u; img < world_pts.size(); ++img) {
		for (auto i = 0u; i < world_pts[img].size(); ++i) {
			pcl::PointXYZ pt;
			pt.x = world_pts[img][i][0];
			pt.y = world_pts[img][i][1];
			pt.z = world_pts[img][i][2];
			cloud->push_back(pt);
		}
	}
}

void Reconstruct3D::convert(const WPts& world_pts, const Intensities& intensities, pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud) {
	for (auto img = 0u; img < world_pts.size(); ++img) {
		for (auto i = 0u; i < world_pts[img].size(); ++i) {
			pcl::PointXYZI pt;
			pt.x = world_pts[img][i][0];
			pt.y = world_pts[img][i][1];
			pt.z = world_pts[img][i][2];
			pt.intensity = intensities[img][i];

			cloud->push_back(pt);
		}
	}
}

void Reconstruct3D::convert(const pcl::PointCloud<pcl::PointNormal>& cloud, WPts& world_pts) {
		world_pts.clear();
		world_pts.resize(1);

		for (auto i = 0u; i < cloud.size(); ++i) {
			pcl::PointNormal pt = (cloud)[i];
			cv::Vec3d vec_pt(pt.x, pt.y, pt.z);
			world_pts[0].push_back(vec_pt);
		}
}

void Reconstruct3D::convert(const pcl::PointCloud<pcl::PointXYZI>& cloud, WPts& world_pts) {
		world_pts.clear();
		world_pts.resize(1);

		for (auto i = 0u; i < cloud.size(); ++i) {
			pcl::PointXYZI pt = (cloud)[i];
			cv::Vec3d vec_pt(pt.x, pt.y, pt.z);
			world_pts[0].push_back(vec_pt);
		}
}

void Reconstruct3D::remesh_with_smoothing(WPts& world_pts) {
	// Load input file into a PointCloud<T> with an appropriate type
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ> ());


  // Load bun0.pcd -- should be available with the PCL archive in test 
//  pcl::io::loadPCDFile ("bun0.pcd", *cloud);
  convert(world_pts, cloud);

  // Create a KD-Tree
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);

  // Output has the PointNormal type in order to store the normals calculated by MLS
  pcl::PointCloud<pcl::PointNormal> mls_points;

  // Init object (second point type is for the normals, even if unused)
  pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;
 
  mls.setComputeNormals (true);

  // Set parameters
  mls.setInputCloud (cloud);
  mls.setPolynomialFit (true);
  mls.setSearchMethod (tree);
//  mls.setSearchRadius (0.03);
  mls.setSearchRadius (10);

  // Reconstruct
  mls.process (mls_points);


  convert(mls_points, world_pts);
  // Save output
//  pcl::io::savePCDFile ("bun0-mls.pcd", mls_points);
	
}

void Reconstruct3D::bilateral_smooth(WPts& world_pts, const Intensities& left_intensities) {
		
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI> ());
	convert(world_pts, left_intensities, cloud);


	pcl::search::KdTree<pcl::PointXYZI>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZI>);

//	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZI> ());
	pcl::PointCloud<pcl::PointXYZI> cloud_filtered;

	pcl::BilateralFilter<pcl::PointXYZI> fbFilter; 
	fbFilter.setInputCloud(cloud); 
	fbFilter.setHalfSize(1.0);
	fbFilter.setStdDev(0.2);
//	fbFilter.setSearchMethod(tree);

//	fbFilter.setHalfSize
//	fbFilter.setSigmaR(15.0f); 
//	fbFilter.setSigmaS(0.2f); 
	fbFilter.applyFilter(cloud_filtered); 

	convert(cloud_filtered, world_pts);
}



bool Reconstruct3D::non_consecutive_points_exists(unsigned img, unsigned row, const cv::Mat& left_non_zero_points, const cv::Mat& right_non_zero_points) {
	bool found_anomaly = false;
	cv::Vec2i prev_row;
	for (auto i = 0u; i < left_non_zero_points.rows; ++i) {
		cv::Vec2i curr_row = left_non_zero_points.at<cv::Vec2i>(i, 0);
		if (i > 0) {
			if (curr_row[0] - prev_row[0] > 1) {
				found_anomaly = true;
				break;
			}
		}
		prev_row = curr_row;
	}
							
	if (found_anomaly) {
#ifdef DEBUG
		std::cout << "Found anomaly in left img : " << img << " row : " << row << std::endl;
#endif
		return true;
	}

	//							cv::Vec2i prev_row;
	for (auto i = 0u; i < right_non_zero_points.rows; ++i) {
		cv::Vec2i curr_row = right_non_zero_points.at<cv::Vec2i>(i, 0);
		if (i > 0) {
			if (curr_row[0] - prev_row[0] > 1) {
				found_anomaly = true;
				break;
			}
		}
		prev_row = curr_row;
	}

	if (found_anomaly) {
#ifdef DEBUG
		std::cout << "Found anomaly in right img : " << img << " row : " << row << std::endl;
#endif
		return true;
	}
	return false;
}

bool Reconstruct3D::anomaly_exists_in_vertical_points(std::vector<cv::Point2d>& points) {
	if (points.size() < 3) {
		return true;
	}

	for (auto i = 1; i < points.size() - 1; ++i) {
		cv::Point2d top_point = points[i -  1];
		cv::Point2d middle_point = points[i];
		cv::Point2d bottom_point = points[i + 1];

		int pixel_tolerance = 1;

		if ((std::abs(top_point.x - middle_point.x) > pixel_tolerance)
			&& (std::abs(bottom_point.x - middle_point.x) > pixel_tolerance)) {
				return true;
		}
	}
	return false;
}

void estimate_plane(const cv::Vec3d& p, const cv::Vec3d& q, const cv::Vec3d& r, 
					cv::Vec3d& n, double d) {

						cv::Vec3d pq = q - p;
						cv::Vec3d qr = r - q;
						n = pq.cross(qr);
						d = n.dot(p);
	
}

void Reconstruct3D::correpond_with_gaussians(CameraImgMap& camera_img_map, int left_cam_no, int right_cam_no, 
											 IPts& img_pts1, IPts& img_pts2, Intensities& left_intensities, Intensities& right_intensities) {

	auto& left_imgs = camera_img_map[left_cam_no];
	auto& right_imgs = camera_img_map[right_cam_no];


	assert(left_imgs.size() == right_imgs.size());

	int threshold = 50;
	img_pts1.resize(left_imgs.size());
	img_pts2.resize(right_imgs.size());

	left_intensities.resize(left_imgs.size());
	right_intensities.resize(right_imgs.size());

	for (auto img = 0u; img < left_imgs.size(); ++img) {
		cv::Mat& left_img = left_imgs[img];
		cv::Mat left_row_sum;
		cv::reduce(left_img, left_row_sum, 1, CV_REDUCE_SUM, CV_32S);

		cv::Mat& right_img = right_imgs[img];
		cv::Mat right_row_sum;
		cv::reduce(right_img, right_row_sum, 1, CV_REDUCE_SUM, CV_32S);

		// debug data
		cv::Mat left_corr_img;
		cv::cvtColor(left_img, left_corr_img, CV_GRAY2BGR);

		cv::Mat right_corr_img;
		cv::cvtColor(right_img, right_corr_img, CV_GRAY2BGR);

		const int image_width = left_img.cols;
		const int width_threshold_percentage = 10;
		const int std_dev_threhsold = image_width * 0.01 * 1 * 0.5;

		// noise reduction

		std::vector<cv::Mat> non_zero_rows;
		// for top, current, bottom
		non_zero_rows.resize(3); 

		std::vector<cv::Point2d> temp_left_points;
		std::vector<cv::Point2d> temp_right_points;

		for (auto row = 1u; row < left_row_sum.rows - 1; ++row) {
			if ((left_row_sum.at<int>(row, 0) >= threshold)
				&& (right_row_sum.at<int>(row, 0) >= threshold)) {

					double left_mid_point;
					cv::Mat left_non_zero_points;
					cv::findNonZero(left_img.row(row), left_non_zero_points);

					double right_mid_point;
					cv::Mat right_non_zero_points;
					cv::findNonZero(right_img.row(row), right_non_zero_points);

					// let's do some basic noise filtering
					bool found_anomaly = false;
					if (left_non_zero_points.rows > 0
						&& right_non_zero_points.rows > 0) {
						if (non_consecutive_points_exists(img, row, left_non_zero_points, right_non_zero_points)) {
							continue;
						}

					} else {
						// no point going further, skip this row
						continue;
					}


#ifdef DEBUG
					for (auto i = 0; i < left_non_zero_points.rows; ++i) {
						cv::Vec2i non_zero_point = left_non_zero_points.at<cv::Vec2i>(i, 0);
						std::cout << "row : " << row << " (" << row << ", " << non_zero_point[0] << ") : " << static_cast<int>(left_img.at<unsigned char>(row, non_zero_point[0])) << std::endl;
					}
#endif


					//cv::Mat left_mid_point_estimate_mat;
					//cv::reduce(cv::Mat(left_non_zero_points), left_mid_point_estimate_mat, 0, CV_REDUCE_SUM, CV_32S);
					//cv::Vec2i left_mid_point_estimate = left_mid_point_estimate_mat.at<cv::Vec2i>(0, 0);

//					cv::Vec2d left_avg;
//					for (auto i = 0u; i < left_non_zero_points.rows; ++i) {
//						left_avg += left_non_zero_points.at<cv::Vec2i>(i, 0);
//					}
//					left_avg /= static_cast<double>(left_non_zero_points.rows);

					cv::Scalar left_mean;
					cv::Scalar left_std_dev;
					cv::meanStdDev(left_non_zero_points, left_mean, left_std_dev);

					//left_mid_point = left_avg[0];



					//cv::Mat right_mid_point_estimate_mat;
					//cv::reduce(cv::Mat(right_points), right_mid_point_estimate_mat, 0, CV_REDUCE_AVG, CV_32F);
					//cv::Vec2i right_mid_point_estimate = right_mid_point_estimate_mat.at<cv::Vec2i>(0, 0);

//					cv::Vec2d right_avg;
//					for (auto i = 0u; i < right_non_zero_points.rows; ++i) {
//						right_avg += right_non_zero_points.at<cv::Vec2i>(i, 0);
//					}
//					right_avg /= static_cast<double>(right_non_zero_points.rows);

					cv::Scalar right_mean;
					cv::Scalar right_std_dev;
					cv::meanStdDev(right_non_zero_points, right_mean, right_std_dev);

					if (left_std_dev[0] > std_dev_threhsold) {
						std::cout << "Intensity points spread too far in left image. Std dev : " << left_std_dev[0] << "Skipping row : " << row << std::endl;
						continue;
					} else if (right_std_dev[0] > std_dev_threhsold) {
						std::cout << "Intensity points spread too far in right image. Std dev : " << right_std_dev[0] << "Skipping row : " << row << std::endl;
						continue;
					}

//					if (img > 43 && img < 50) {
//						std::cout << img << ", " << row << " : " << std::endl;
//						std::cout << "left mean : " << left_mean << std::endl;
//						std::cout << "right mean : " << right_mean << std::endl;
//						std::cout << "left std deviation : " << left_std_dev << std::endl;
//						std::cout << "right std deviation : " << right_std_dev << std::endl;
//					}

					// fit gaussians only after checking for weird standard deviations, if points are all over, some error condition has occured

					fit_gauss(left_img, row, left_non_zero_points, left_mean[0], left_mid_point);

					fit_gauss(right_img, row, right_non_zero_points, right_mean[0], right_mid_point);
					//right_mid_point = right_avg[0];


					//Rect rect(50, 50, 270, 270);
					cv::Rect left_rect(validRoi[0]);
					cv::Rect right_rect(validRoi[1]);

					if (left_rect.contains(cv::Point2d(left_mid_point, row))
						&& (right_rect.contains(cv::Point2d(right_mid_point, row)))) {

							// insert temporarily
//							temp_left_points[row] = cv::Point2d(left_mid_point, row);
//							temp_right_points[row] = cv::Point2d(right_mid_point, row);

							temp_left_points.push_back(cv::Point2d(left_mid_point, row));
							temp_right_points.push_back(cv::Point2d(right_mid_point, row));

					}
					//img_pts1.push_back(cv::Point2f(row, left_mid_point));
					//img_pts2.push_back(cv::Point2f(row, right_mid_point));
					//img_pts1.push_back(cv::Point2f(left_img.cols - left_mid_point, left_img.rows-row));
					//img_pts2.push_back(cv::Point2f(right_img.cols - right_mid_point, right_img.rows - row));
					//img_pts1.push_back(cv::Point2f(row, left_mid_point));
					//img_pts2.push_back(cv::Point2f(row, right_mid_point));

			}
		}

		// filter noise

//		for (auto itr = temp_left_points.begin(); itr != temp_left_points.end(); ++itr) {

		double threshold_percentage = 10;
		int threshold_no = left_img.rows * threshold_percentage * 0.01;

		// skip points all together, if less than threshold
		if (temp_left_points.size() > threshold_no) {
		for (auto i = 1; i < temp_left_points.size() - 1; ++i) {
//			int row = itr->first;
//
//			if (temp_right_points.find(row) == temp_right_points.end()) {
//				throw std::runtime_error("Same row does not exist for left and right points. Img : " + std::to_string(img)
//					+ " row : " + std::to_string(row));
//			}

			// skip points if there are not enough points in image

			cv::Point2d left_point = temp_left_points[i];
			double left_mid_point = left_point.x;
			int left_row = left_point.y;


			cv::Point2d right_point = temp_right_points[i];
			double right_mid_point = right_point.x;
			int right_row = right_point.y;

			assert(left_row == right_row);
			int row = left_row;

			// filter noise
			std::vector<cv::Point2d> left_vertical_points;

			cv::Point2d top_left_point = temp_left_points[i - 1];
			cv::Point2d bottom_left_point = temp_left_points[i + 1];

			left_vertical_points.push_back(top_left_point);
			left_vertical_points.push_back(left_point);
			left_vertical_points.push_back(bottom_left_point);

			bool anomaly_in_points = false;

			anomaly_in_points = anomaly_exists_in_vertical_points(left_vertical_points);

			if (anomaly_in_points) {
				// skip points
#ifdef DEBUG
				std::cout << "Found vertical anomaly in left img : " << img << " row : " << row << std::endl;
#endif
				continue;
			}


			std::vector<cv::Point2d> right_vertical_points;

			cv::Point2d top_right_point = temp_right_points[i - 1];
			cv::Point2d bottom_right_point = temp_right_points[i + 1];

			right_vertical_points.push_back(top_right_point);
			right_vertical_points.push_back(right_point);
			right_vertical_points.push_back(bottom_right_point);

			anomaly_in_points = anomaly_exists_in_vertical_points(right_vertical_points);

			if (anomaly_in_points) {
				// skip points
#ifdef DEBUG
				std::cout << "Found vertical anomaly in right img : " << img << " row : " << row << std::endl;
#endif
				continue;
			}


			img_pts1[img].push_back(cv::Point2d(left_mid_point, row));
			img_pts2[img].push_back(cv::Point2d(right_mid_point, row));

			double left_intensity = left_img.at<unsigned char>(row, static_cast<int>(left_mid_point + 0.5));
			double right_intensity = right_img.at<unsigned char>(row, static_cast<int>(right_mid_point + 0.5));

			left_intensities[img].push_back(left_intensity);
			right_intensities[img].push_back(right_intensity);

			left_corr_img.at<cv::Vec3b>(row, left_mid_point) = cv::Vec3b(0, 0, 255);
			right_corr_img.at<cv::Vec3b>(row, right_mid_point) = cv::Vec3b(0, 0, 255);
		}
		}

		cv::imshow("left_gauss_fit", left_corr_img);
		cv::imshow("right_gauss_fit", right_corr_img);
		cv::imwrite("left_gauss_fit.png", left_corr_img);
		cv::imwrite("right_gauss_fit.png", right_corr_img);
	}
}

void Reconstruct3D::triangulate_pts(const WPts& world_pnts, WPt& triangles, 
	IPt& texture_coords, cv::Mat& remapped_img) {


	const cv::Mat proj = P1;
	//double origin[] = { 0.0, 0.0, 0.0 };
	//cv::Mat some_pnt(3, 1, CV_32F, origin);

	//double z1[] = { 0.0, 0.0, 1.0 };
	//cv::Mat pt_on_plane(3, 1, CV_32F, z1);
	//cv::Mat normal = some_pnt - pt_on_plane;

	WPt world_pt_single_array;
	for (auto img = 0u; img < world_pnts.size(); ++img) {
		for (auto i = 0u; i < world_pnts[img].size(); ++i) {
			world_pt_single_array.push_back(world_pnts[img][i]);
		}
	}

	std::vector<cv::Point2f> projected_pts;
	for (auto i = 0u; i < world_pt_single_array.size(); ++i) {
			// project point
			cv::Mat world_pnt(4, 1, CV_64F);
			for (int x = 0; x < 3; ++x) {
				world_pnt.at<double>(x, 0) = (double)world_pt_single_array[i][x];
			}
			world_pnt.at<double>(3, 0) = 1.0;
			cv::Mat projected_pt = proj * world_pnt;
			double z = projected_pt.at<double>(2, 0);
			projected_pts.push_back(cv::Point2f(projected_pt.at<double>(0, 0) / z, projected_pt.at<double>(1, 0) / z));
	}

	assert(world_pt_single_array.size() == projected_pts.size());


	auto cmpx = [&] (const cv::Point2f& left, const cv::Point2f& right) -> bool {
		return (left.x < right.x);
	};

	auto cmpy = [&] (const cv::Point2f& left, const cv::Point2f& right) -> bool {
		return (left.y < right.y);
	};

	if (world_pt_single_array.size() == 0) {
		std::cout << "No points to triangulate" << std::endl;
		return;
	}

	// construct rect
	float maxX = (*std::max_element(projected_pts.begin(), projected_pts.end(), cmpx)).x;
	float minX = (*std::min_element(projected_pts.begin(), projected_pts.end(), cmpx)).x;
	float maxY = (*std::max_element(projected_pts.begin(), projected_pts.end(), cmpy)).y;
	float minY = (*std::min_element(projected_pts.begin(), projected_pts.end(), cmpy)).y;

	//cv::Rect rect(minX - 1, minY - 1, maxX + std::abs(minX), maxY + std::abs(minY));
	cv::Rect rect(minX - 1, minY - 1, (maxX) + std::abs(minX), (maxY) + std::abs(minY));

	cv::Subdiv2D subdiv(rect);
	std::cout << std::endl;
	for (int i = 0u; i < projected_pts.size(); ++i) {
		//std::cout << projected_pts[i].x << ", " << projected_pts[i].y << std::endl;
		subdiv.insert(projected_pts[i]);
	}

	std::vector<cv::Vec6f> triangle_list;
	subdiv.getTriangleList(triangle_list);


	const float t = 1e-4;
	for (auto i = 0u; i < triangle_list.size(); ++i) {
		auto vec6 = triangle_list[i];
		cv::Point2f triangle_pts[3];
		std::vector<int> triangle_index;
		for (int x = 0; x < 3; ++x) {
			for (int k = 0; k < projected_pts.size(); ++k) {
				if (projected_pts[k].x + t > vec6[2 * x]
					&& projected_pts[k].x - t < vec6[2 * x]
					&& projected_pts[k].y + t > vec6[(2 * x) + 1]
					&& projected_pts[k].y - t < vec6[(2 * x) + 1]) {
					triangle_index.push_back(k);
					break;
				}
			}
		}

		if (triangle_index.size() == 3) {
			WPt triangle;
			std::vector<cv::Point2f> image_coords;
			for (auto k = 0u; k < triangle_index.size(); ++k) {
				triangles.push_back(world_pt_single_array[triangle_index[k]]);
				triangle.push_back(world_pt_single_array[triangle_index[k]]);
				image_coords.push_back(projected_pts[triangle_index[k]]);
			}
			for (int i = 0; i < 3; ++i) {
				texture_coords.push_back(cv::Vec2f((float)(image_coords[i].x) / (float)(remapped_img.cols),
					(float)(image_coords[i].y) / (float)(remapped_img.rows)));
			}
		}
	}

	//assert(3 * triangle_list.size() == triangles.size());


}

void Reconstruct3D::gen_texture(GLuint& texture_id, cv::Mat& remapped_img_for_texture) const {

	glGenTextures(1, &texture_id);
	glBindTexture(GL_TEXTURE_2D, texture_id);
	////use fast 4-byte alignment (default anyway) if possible
	//glPixelStorei(GL_UNPACK_ALIGNMENT, (image.step & 3) ? 1 : 4);

	////set length of one complete row in data (doesn't need to equal image.cols)
	//glPixelStorei(GL_UNPACK_ROW_LENGTH, image.step / image.elemSize());

	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

	// Set texture clamping method
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);


	glTexImage2D(GL_TEXTURE_2D,     // Type of texture
		0,
		GL_RGB,
		remapped_img_for_texture.cols,
		remapped_img_for_texture.rows,
		0,
		GL_BGR,
		GL_UNSIGNED_BYTE,
		remapped_img_for_texture.ptr());


	glBindTexture(GL_TEXTURE_2D, 0);

}

void Reconstruct3D::alter_img_for_projection(const cv::Mat& img, cv::Mat& remapped_img, bool is_right) const {
	cv::Mat img_copy = img.clone();
	//resize_img(img_copy);
	cv::remap(img_copy, remapped_img, rmap[is_right][0], rmap[is_right][1], CV_INTER_LINEAR);
}

void Reconstruct3D::smooth_points(WPts& world_pts, const Intensities& left_intensities, const Intensities& right_intensities) {

//	for (auto img = 0u; img < world_pts.size(); ++img) {
//		optimize_smoothness(world_pts[img], left_intensities[img], right_intensities[img]);
//	}

	// simple laplacian

	for (auto iter = 0u; iter < 1000000; ++iter ) {
		for (auto img = 0u; img < world_pts.size(); ++img) {
			//		optimize_smoothness(world_pts[img], left_intensities[img], right_intensities[img]);
			double lambda = 0.1;
			if (world_pts[img].size() >= 3) {
				for (auto i = 1; i < world_pts[img].size() - 1; ++i) {
					double old_z = world_pts[img][i][2];
					double top_z = world_pts[img][i-1][2];
					double bottom_z = world_pts[img][i+1][2];
					double length_top_old_z = std::abs(top_z - old_z);
					double length_bottom_old_z = std::abs(bottom_z - old_z);

					if (length_top_old_z < 1e-6 || length_bottom_old_z < 1e-6) {
						// skip this point, nothing to smooth
						continue;
					}

					double weights_top = 1.0 / length_top_old_z;
					double weights_bottom = 1.0 / length_bottom_old_z;
					double total_weights = weights_top + weights_bottom;
					double laplacian = ((weights_top * top_z + weights_bottom * bottom_z) / total_weights) - old_z;
					double new_z = old_z + lambda * (laplacian);
					if (new_z > 1e5 || new_z < -1e5) {
						std::cout << new_z << std::endl;
					}
					world_pts[img][i][2] = new_z;
				}
			}
		}
	}
#ifdef DEBUG
		for (auto img = 0u; img < world_pts.size(); ++img) {
			double z_diff_avg = 0.0;
			if (world_pts[img].size() >= 3) {
				for (auto i = 1; i < world_pts[img].size() - 1; ++i) {
					double old_z = world_pts[img][i][2];
					double top_z = world_pts[img][i-1][2];
					double bottom_z = world_pts[img][i+1][2];
					z_diff_avg += top_z - old_z; 
				}
			}
			std::cout << "img # : " << img << " z difference : " << z_diff_avg / world_pts[img].size() << std::endl;
		}
#endif
}

cv::Vec3d Reconstruct3D::calculate_3D_point(const cv::Vec2d& left_image_point, const cv::Vec2d& right_image_point, 
		const cv::Mat& proj1, const cv::Mat& proj2) const {
			cv::Mat D(4, 3, CV_64F);
			cv::Mat b(4, 1, CV_64F);

			// fill rows from point 1
			for (auto x = 0u; x < 2; ++x) {
				cv::Mat row_D(1, 3, CV_64F);
				cv::Mat row_B(1, 1, CV_64F);
				fill_row(proj1, left_image_point[x], row_D, row_B, x);
				row_D.row(0).copyTo(D.row(x));
				row_B.row(0).copyTo(b.row(x));
			}
			for (auto x = 0u; x < 2; ++x) {
				cv::Mat row_D(1, 3, CV_64F);
				cv::Mat row_B(1, 1, CV_64F);
				fill_row(proj2, right_image_point[x], row_D, row_B, x);
				row_D.row(0).copyTo(D.row(x + 2));
				row_B.row(0).copyTo(b.row(x + 2));
			}
			cv::Mat XYZ(3, 1, CV_64F);
			cv::solve(D, b, XYZ, cv::DECOMP_SVD);
			//cv::Mat D_inv = D.inv(DECOMP_SVD);
			//cv::Mat XYZ = D_inv * b;

			//cv::Mat b_test = D * XYZ;
			//cv::Mat results = b - b_test;
			//std::cout << "error: " << results << std::endl;
			//std::cout << "XYZ : " << XYZ << std::endl;
			//std::cout << "D Matrix : " << D << std::endl;
			//std::cout << "b Matrix : " << b << std::endl;
			//std::cout << "Img Points 1 : " << img_pts1[i].x << ", " << img_pts1[i].y << std::endl;
			//std::cout << "Img Points 2 : " << img_pts2[i].x << ", " << img_pts2[i].y << std::endl;
			cv::Vec3d point_3D(XYZ.at<double>(0, 0), XYZ.at<double>(1, 0), XYZ.at<double>(2, 0));

			return point_3D;
}


void  Reconstruct3D::correct_img_coordinates(IPts& img_pts1, IPts& img_pts2) {
	// read file
	cv::Mat proj1 = P1;
	cv::Mat proj2 = P2;

	for (auto img = 0u; img < img_pts1.size(); ++img) {
		for (auto i = 0u; i < img_pts1[img].size(); ++i) {
			optimize_image_coordinates(img_pts1[img][i], img_pts2[img][i], this, proj1, proj2);
		}
	}
}

void Reconstruct3D::recon_obj(const IPts& img_pts1, const IPts& img_pts2, WPts& world_pts) {
	assert(img_pts1.size() == img_pts2.size());

	// read file
	cv::Mat proj1 = P1;
	cv::Mat proj2 = P2;

	world_pts.clear();

	//std::cout << "Projection Matrix 1: " << std::endl;
	//std::cout << proj1 << std::endl;
	//std::cout << "Projection Matrix 2: " << std::endl;
	//std::cout << proj2 << std::endl;

	//for (auto img = 0u; img < img_pts1.size(); ++img) {
	//	WPt world_pts_per_img;
	//	cv::Mat img_pt_mat1(2, img_pts1[img].size(), CV_64F);
	//	cv::Mat img_pt_mat2(2, img_pts1[img].size(), CV_64F);

	//	for (auto i = 0u; i < img_pts1[img].size(); ++i) {
	//		for (auto x = 0u; x < 2; ++x) {
	//			img_pt_mat1.at<double>(x, i) = img_pts1[img][i][x];
	//		}
	//	}

	//	for (auto i = 0u; i < img_pts2[img].size(); ++i) {
	//		for (auto x = 0u; x < 2; ++x) {
	//			img_pt_mat2.at<double>(x, i) = img_pts2[img][i][x];
	//		}
	//	}

	//	cv::Mat pts4D(4, img_pts1[img].size(), CV_64F);

	//	cv::triangulatePoints(proj1, proj2, img_pt_mat1, img_pt_mat2, pts4D);

	//	for (auto i = 0u; i < img_pts2[img].size(); ++i) {
	//		world_pts_per_img.push_back(cv::Vec3f(pts4D.at<double>(0, i) / pts4D.at<double>(3, i),
	//			pts4D.at<double>(1, i) / pts4D.at<double>(3, i),
	//			pts4D.at<double>(2, i) / pts4D.at<double>(3, i)));
	//	}
	//	world_pts.push_back(world_pts_per_img);
	//}

	for (auto img = 0u; img < img_pts1.size(); ++img) {
		WPt world_pt;
		for (auto i = 0u; i < img_pts1[img].size(); ++i) {
			world_pt.push_back(calculate_3D_point(img_pts1[img][i], img_pts2[img][i], proj1, proj2));
		}
		world_pts.push_back(world_pt);
	}

	//emit finished_reconstruction_with_triangles(world_pts);
}

cv::Point2d Reconstruct3D::project_point(const cv::Vec3d& world_pt, const cv::Mat& projection_matrix) const {
	cv::Mat world_pnt_mtx(4, 1, CV_64F);
	for (int x = 0; x < 3; ++x) {
		world_pnt_mtx.at<double>(x, 0) = (double)world_pt[x];
	}
	world_pnt_mtx.at<double>(3, 0) = 1.0;

	cv::Mat projected_matrix = projection_matrix * world_pnt_mtx;
	double z = projected_matrix.at<double>(2, 0);

	cv::Point2d projected_point(projected_matrix.at<double>(0, 0) / z, projected_matrix.at<double>(1, 0) / z);

	return projected_point;
}

cv::RNG rng(255);

void Reconstruct3D::project_points_on_to_img(WPts& world_pts, WPts& world_point_colors_all,
                                             cv::Mat& left_img, cv::Mat& right_img, IPts& img_pts1, IPts& img_pts2) {

	cv::Mat proj1 = P1;
	cv::Mat proj2 = P2;

	cv::Mat left_projected_img = left_img.clone();
	cv::cvtColor(left_projected_img, left_projected_img, CV_GRAY2BGR);

	cv::Mat right_projected_img = right_img.clone();
	cv::cvtColor(right_projected_img, right_projected_img, CV_GRAY2BGR);


	world_point_colors_all.clear();

	for (auto img = 0u; img < world_pts.size(); ++img) {
		img_pts1[img].clear();
		img_pts2[img].clear();

		WPt world_pt_colors_per_img;
		for (auto i = 0u; i < world_pts[img].size(); ++i) {
			cv::Vec3b common_color(rng.next(), rng.next(), rng.next());
			// project point

			cv::Point2d left_projected_point = project_point(world_pts[img][i], P1);

			// unique color for each stripe for better recognition, common color 
			if (i < 5 && img == 0) {
				std::cout  << std::setprecision(15) << " left projected points : "<< left_projected_point.x << ", " << left_projected_point.y << std::endl;
			}

			cv::Vec3d world_pt_color(0, 0, 0);

			bool left_point_exists = false;
			if ((left_projected_point.x >= 0 && left_projected_point.x < left_projected_img.cols)
				&& ((left_projected_point.y >= 0 && left_projected_point.y < left_projected_img.rows))) {
					left_projected_img.at<cv::Vec3b>(left_projected_point.y, left_projected_point.x) = common_color;
					unsigned char red = left_img.at<unsigned char>(left_projected_point.y, left_projected_point.x);
					float grayscale_float = red;

					world_pt_color = cv::Vec3d(grayscale_float, grayscale_float, grayscale_float);
					//world_pt_color = cv::Vec3d(common_color[2], common_color[1], common_color[0]);
					world_pt_color /= 255.f;
					left_point_exists = true;
			} 
			world_pt_colors_per_img.push_back(world_pt_color);

			cv::Point2d right_projected_point = project_point(world_pts[img][i], P2);

			if ((right_projected_point.x >= 0 && right_projected_point.x < right_projected_img.cols)
				&& ((right_projected_point.y >= 0 && right_projected_point.y < right_projected_img.rows))) {
					right_projected_img.at<cv::Vec3b>(right_projected_point.y, right_projected_point.x) = common_color;
					//left_projected_img.at<cv::Vec3b>(right_projected_point.y, right_projected_point.x) = common_color;
					if (left_point_exists) {
						img_pts1[img].push_back(cv::Vec2d(left_projected_point.x, left_projected_point.y));
						img_pts2[img].push_back(cv::Vec2d(right_projected_point.x, right_projected_point.y));
					}
			}

			if (i < 5 && img == 0) {
				std::cout  << " right projected points : "<< right_projected_point.x << ", " << right_projected_point.y << std::endl;
			}
		}
		world_point_colors_all.push_back(world_pt_colors_per_img);
	}

	cv::imshow("left projected image", left_projected_img);
	cv::imwrite("left_projected_image.png", left_projected_img);
	cv::imshow("right projected image", right_projected_img);
	cv::imwrite("right_projected_image.png", right_projected_img);
	
}

void Reconstruct3D::reconstruct(CameraPairs& camera_pairs, int no_of_images) {

	try {

		int min = (no_of_images < 0) ? INT_MAX : no_of_images;
		//int min = INT_MAX;
		for (auto i = 0u; i < camera_img_map_.size();++i)
		{
			min = std::min(static_cast<int>(camera_img_map_[i].size()), min);
		}

		for (auto i = 0u; i < camera_img_map_.size();++i)
		{
			camera_img_map_[i].resize(min);
		}


		load_calibration(camera_pairs[0].first, camera_pairs[0].second);
		create_rectification_map();

		IPts img_pts1;
		IPts img_pts2;
		WPts world_pts;
		WPt triangles;
		WPts world_point_colors;
		IPt texture_coordinates;

		Intensities left_intensities;
		Intensities right_intensities;

		compute_correlation_using_gaussian(img_pts1, img_pts2, camera_img_map_, camera_pairs,
			left_intensities, right_intensities);


		cv::Mat left_img = camera_img_map_[camera_pairs[0].first][0];
		cv::Mat right_img = camera_img_map_[camera_pairs[0].second][0];


		cv::Mat left_texture_img = cv::imread("left_texture.png", 0);
		if (!left_texture_img.data) {
			left_texture_img = left_img;
		} else {
			// remap it
			pre_process_img(left_texture_img, left_texture_img, false);
		}

		cv::Mat right_texture_img = cv::imread("right_texture.png", 0);
		if (!right_texture_img.data) {
			right_texture_img = right_img;
		} else {
			// remap it
			pre_process_img(right_texture_img, right_texture_img, true);
		}

//		for (auto i = 0; i < 15; ++i) {
//			std::cout << "original points : " << img_pts1[0][i][0] << ", " << img_pts1[0][i][1] << std::endl;
//		}
//		std::cout << std::endl;
//
//		for (auto i = 0; i < 15; ++i) {
//			std::cout << "original points : " << img_pts2[0][i][0] << ", " << img_pts2[0][i][1] << std::endl;
//		}
//		std::cout << std::endl;

	
		//correct_img_coordinates(img_pts1, img_pts2);

//		for (auto i = 0; i < 15; ++i) {
//			std::cout << "corrected points : " << img_pts1[0][i][0] << ", " << img_pts1[0][i][1] << std::endl;
//		}
//		std::cout << std::endl;
//
//		for (auto i = 0; i < 15; ++i) {
//			std::cout << "corrected points : " << img_pts2[0][i][0] << ", " << img_pts2[0][i][1] << std::endl;
//		}
//		std::cout << std::endl;

		recon_obj(img_pts1, img_pts2, world_pts);
		project_points_on_to_img(world_pts, world_point_colors, left_texture_img, right_texture_img, img_pts1, img_pts2);

		triangulate_pts(world_pts, triangles, texture_coordinates, left_img);
//		for (auto i = 0; i < 15; ++i) {
//		smooth_points(world_pts, left_intensities, right_intensities);
		remesh_with_smoothing(world_pts);
//		bilateral_smooth(world_pts, left_intensities);
		
//		}

//		project_points_on_to_img(world_pts, world_point_colors, left_texture_img, right_texture_img, img_pts1, img_pts2);
//
//		recon_obj(img_pts1, img_pts2, world_pts);

	

		emit finished_reconstruction_with_triangles(world_pts, world_point_colors, triangles, texture_coordinates, left_texture_img);
		//emit finished_reconstruction_with_triangles(triangles, texture_coordinates, texture_img);
	
	} catch (std::exception &e)
	{
		std::cout << e.what() << std::endl;
	}
}

void Reconstruct3D::re_reconstruct(CameraPairs& camera_pairs, int no_of_images) {

		//std::string left_remapped_img_name = "left_img.png";
		//std::string right_remapped_img_name = "right_img.png";

		//cv::Mat left_img = cv::imread(left_remapped_img_name, 0);
		//cv::Mat right_img = cv::imread(right_remapped_img_name, 0);

		//if (!left_img.data 
		//	|| !right_img.data) {
		//	// no image file
		//	std::cout << "Remapped images do not exist - " << left_remapped_img_name
		//		<< " " << right_remapped_img_name << std::endl;
		//	return;
		//}

		//camera_img_map_[camera_pairs[0].first].push_back(left_img);
		//camera_img_map_[camera_pairs[0].second].push_back(right_img);


	// first load the images for each camera
	QDir calibration_dir(recon_dirname_.c_str());
	if (!calibration_dir.exists()) {
		//QMessageBox::warning(QMessageBox::Warning, "Calibration Error!", calibration_dir.dirName() + QString(" does not exist!"), QMessageBox::Ok);
		std::cout << "Calibration directory : " << recon_dirname_ << " does not exist!" << std::endl;
		return;
	}

	for (auto i = 0u; i < no_of_cams_; ++i) {
		// TODO change to serial num, maybe
		std::string camera_pair_subdirname =  camera_subdir_prefix_ + std::to_string(i);
		std::string camera_pair_dir_path = recon_dirname_ + std::string("/") + camera_pair_subdirname;
		QDir camera_pair_dir(camera_pair_dir_path.c_str());
		if (!camera_pair_dir.exists()) {
			std::cout << "images for camera " << i << " does not exist.";
			continue;
		}
		QDirIterator it(camera_pair_dir.absolutePath(), QStringList() << "*.png", QDir::Files);
		int z = 0;
		while (it.hasNext()) {
			std::string filename = it.next().toStdString();
			if (z++ < 4) {
				std::cout << filename << std::endl;
			}
			cv::Mat img = cv::imread(filename, 0);
			camera_img_map_[i].push_back(img);
		}
	}

	reconstruct(camera_pairs, no_of_images);
}

void Reconstruct3D::fill_row(const cv::Mat& P, double coord, cv::Mat& fill_matrix, cv::Mat& B, bool is_y) const {
	int row = is_y;
	for (int j = 0; j < 3; ++j) {
		fill_matrix.at<double>(0, j) = coord * P.at<double>(2, j) - P.at<double>(row, j);
	}
	B.at<double>(0, 0) = P.at<double>(row, 3) - coord * P.at<double>(2, 3);
}
