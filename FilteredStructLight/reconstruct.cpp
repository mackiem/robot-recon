#include "reconstruct.h"
#include <fstream>
#include <sstream>
#include <iostream>
#include <algorithm>
#include <omp.h>
#include <chrono>
#include <cmath>
#include "gl_core_3_3.h"


Reconstruct3D::Reconstruct3D(int no_of_cams, QObject* parent) 
	: no_of_cams_(no_of_cams), QObject(parent)
{

}


Reconstruct3D::~Reconstruct3D()
{
}

void Reconstruct3D::collect_images(FlyCapture2::Image img, int cam_no)
{
	cv::Mat test(img.GetRows(), img.GetCols(), CV_8UC1, img.GetData(), img.GetStride());
	camera_img_map_[cam_no].push_back(test.clone());
}

void Reconstruct3D::compute_correspondence(FlyCapture2::Image img, int cam_no)
{
}

void Reconstruct3D::clear_camera_img_map()
{
	camera_img_map_.clear();
}

void Reconstruct3D::run_calibration(std::vector<std::pair<int, int>> camera_pairs)
{
	for (auto& camera_pair_ : camera_pairs)
	{
		stereo_calibrate(camera_img_map_, camera_pair_.first, camera_pair_.second, cv::Size(9, 6), true); 
	}
}

void Reconstruct3D::run_reconstruction(std::vector<std::pair<int, int>> camera_pairs)
{
	IPts img_pts1;
	IPts img_pts2;
	WPts world_pts;

	compute_correlation(img_pts1, img_pts2, camera_img_map_, camera_pairs);

	recon_obj(img_pts1, img_pts2, world_pts);
}

void Reconstruct3D::stereo_calibrate(CameraImgMap& camera_img_map, int left_cam, int right_cam, Size boardSize, bool useCalibrated) {

	if (camera_img_map.size() < 2) {
		std::cout << "Only " << camera_img_map.size() << " available. Need at least 2..." << std::endl;
		return;
	}

	assert(left_cam < no_of_cams_);
	assert(right_cam < no_of_cams_);
	assert(camera_img_map.size() == no_of_cams_);

	
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



	cameraMatrix[0] = Mat::eye(3, 3, CV_64F);
	cameraMatrix[1] = Mat::eye(3, 3, CV_64F);

	double rms = stereoCalibrate(objectPoints, imagePoints[0], imagePoints[1],
		cameraMatrix[0], distCoeffs[0],
		cameraMatrix[1], distCoeffs[1],
		imageSize, R, T, E, F,
		TermCriteria(CV_TERMCRIT_ITER + CV_TERMCRIT_EPS, 100, 1e-5),
		CV_CALIB_FIX_ASPECT_RATIO +
		CV_CALIB_ZERO_TANGENT_DIST +
		//CV_CALIB_SAME_FOCAL_LENGTH +
		CV_CALIB_RATIONAL_MODEL +
		CV_CALIB_FIX_K3 + CV_CALIB_FIX_K4 + CV_CALIB_FIX_K5);
	cout << "done with RMS error=" << rms << endl;

	// CALIBRATION QUALITY CHECK
	// because the output fundamental matrix implicitly
	// includes all the output information,
	// we can check the quality of calibration using the
	// epipolar geometry constraint: m2^t*F*m1=0
	double err = 0;
	int npoints = 0;
	vector<Vec3f> lines[2];
	for (i = 0; i < nimages; i++)
	{
		int npt = (int)imagePoints[0][i].size();
		Mat imgpt[2];
		for (k = 0; k < 2; k++)
		{
			imgpt[k] = Mat(imagePoints[k][i]);
			undistortPoints(imgpt[k], imgpt[k], cameraMatrix[k], distCoeffs[k], Mat(), cameraMatrix[k]);
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
	cout << "average reprojection err = " << err / npoints << endl;

	// save intrinsic parameters
	FileStorage fs("intrinsics.yml", CV_STORAGE_WRITE);
	if (fs.isOpened())
	{
		fs << "M1" << cameraMatrix[0] << "D1" << distCoeffs[0] <<
			"M2" << cameraMatrix[1] << "D2" << distCoeffs[1];
		fs.release();
	}
	else {
		cout << "Error: can not save the intrinsic parameters\n";
	}



	fs.open("extrinsics.yml", CV_STORAGE_WRITE);
	if (fs.isOpened())
	{
		fs << "R" << R << "T" << T << "imageSize" << imageSize;
		fs.release();
	}
	else {
		cout << "Error: can not save the intrinsic parameters\n";
	}
}

void Reconstruct3D::reconstruct()
{
}


void Reconstruct3D::resize_img(cv::Mat& img, const unsigned int resize_width) const {
    //if (img.size().width > resize_width) {
        float aspect_ratio = (float)(img.size().width) / (float)(img.size().height);
        int operating_height = (int)((float)(resize_width) / aspect_ratio);
        cv::resize(img, img, cv::Size(resize_width, operating_height), INTER_NEAREST);
    //}
}

//void Reconstruct3D::resize_img(cv::Mat& img) const {
//	resize_img(img, OPERATING_WIDTH);
//}
//
//void Reconstruct3D::resize_segmentation_img(cv::Mat& img) const {
//	resize_img(img, SEGMENT_OPERATING_WIDTH);
//}


void Reconstruct3D::load_calibration() {
    // save intrinsic parameters
    FileStorage fs("intrinsics.yml", CV_STORAGE_READ);
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



    fs.open("extrinsics.yml", CV_STORAGE_READ);
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


void Reconstruct3D::init_imgs(CameraImgMap& camera_img_map, int cam, bool is_right) {

	assert(cam < no_of_cams_);
	assert(camera_img_map.size() == no_of_cams_);
	assert(camera_img_map.find(cam) != camera_img_map.end());

	auto& camera_imgs = camera_img_map[cam];

	for (int i = 0; i < camera_imgs.size(); i++) {
		cv::Mat& img = camera_imgs[i];
		cv::Mat thresholded;
		cv::threshold(img, thresholded, 100, 255, CV_THRESH_BINARY);
		cv::Mat rImg;
		remap(thresholded, rImg, rmap[is_right][0], rmap[is_right][1], CV_INTER_LINEAR);
		camera_imgs[i] = rImg;
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

	int color_threshold = 20;
	
	typedef std::chrono::high_resolution_clock Clock;
	typedef std::chrono::milliseconds milliseconds;

  	auto& camera_imgs = camera_img_map[cam];

	for (int i = 0; i < camera_imgs.size(); i++) {    
		cv::Mat& img = camera_imgs[i];
		cv::Mat circle_img = img.clone();
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
					

					if (cv::norm(pre_color, post_color) > color_threshold) {
						auto color_pair = std::make_pair(pre_color, post_color);
						unique_edges_in_row.push_back(col);

						// draw a circle at the point
						int w = img.cols;

						int thickness = 0;
						int lineType = 8;

						cv::circle(circle_img,
							cv::Point(col, row),
							w / (32.0 * 4.0),
							cv::Scalar(255, 255, 255),
							thickness,
							lineType);
					}
				}	
			}
            unique_edges[row] = unique_edges_in_row;
		}

        unique_colors_all_images[i] = unique_edges;
    }
}


void Reconstruct3D::compute_correlation(std::vector <cv::Vec2f>& img_pts1, std::vector <cv::Vec2f>& img_pts2,
		CameraImgMap& camera_img_map, std::vector<std::pair<int, int>> camera_pairs) {


    std::vector<cv::Mat> imgs_left;
    std::vector<cv::Mat> imgs_right;
    init_imgs(camera_img_map, camera_pairs[0].first, false);
    init_imgs(camera_img_map, camera_pairs[0].second, true);


    std::unordered_map<int, std::unordered_map< int, std::vector<UniqueColorPair> > > unique_colors_left;
	std::vector<UniqueEdges> unique_edges_left;
	std::vector<UniqueEdges> unique_edges_right;
	
    get_unique_edges(camera_img_map, camera_pairs[0].first, unique_edges_left, false);
    get_unique_edges(camera_img_map, camera_pairs[0].second, unique_edges_right, false);


    std::vector < std::vector<int> > correlations;

    img_pts1.clear();
    img_pts2.clear();

    compute_correlation_per_image(unique_edges_left, unique_edges_right, img_pts1, img_pts2);


    write_file("recon.cp", img_pts1, img_pts2);
    //read_input_file("recon.cp", img_pts1, img_pts2);
}

void Reconstruct3D::write_file(const std::string& file_name, const std::vector <cv::Vec2f>& img_pts1, const std::vector <cv::Vec2f>& img_pts2) {
    std::ofstream file;
    file.open(file_name);
    for (int i = 0; i < img_pts1.size(); ++i) {
        file << img_pts1[i][0] << "\t";
        file << img_pts1[i][1] << "\t";
        file << img_pts2[i][0] << "\t";
        file << img_pts2[i][1] << "\n";
    }
    file.close();
}

    
void Reconstruct3D::compute_correlation_per_image(std::vector<UniqueEdges>& unique_colors_left,
    std::vector<UniqueEdges>& unique_colors_right,
    std::vector <cv::Vec2f>& img_pts1, std::vector <cv::Vec2f>& img_pts2) {

		for (auto j = 0u; j < unique_colors_left.size(); ++j)
		{
			auto& unique_colors_per_image_left = unique_colors_left[j];
			auto& unique_colors_per_image_right = unique_colors_right[j];

			assert( unique_colors_per_image_left.size() == unique_colors_per_image_right.size());

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
					&& row_edges_vec_right.size() != 2)
				{
					continue;	
				}

				for (auto i = 0u; i < row_edges_vec_left.size(); ++i) {
					img_pts1.push_back(cv::Vec2f(row_no, row_edges_vec_left[i]));
					img_pts2.push_back(cv::Vec2f(row_no, row_edges_vec_right[i]));
				}

			}
		}

}
 

void Reconstruct3D::triangulate_pts(const WPts& world_pnts, WPts& triangles, 
	std::vector<cv::Vec2f>& texture_coords, cv::Mat& remapped_img) {


	const cv::Mat proj = P1;
	//double origin[] = { 0.0, 0.0, 0.0 };
	//cv::Mat some_pnt(3, 1, CV_32F, origin);

	//double z1[] = { 0.0, 0.0, 1.0 };
	//cv::Mat pt_on_plane(3, 1, CV_32F, z1);
	//cv::Mat normal = some_pnt - pt_on_plane;

	std::vector<cv::Point2f> projected_pts;
	for (int i = 0u; i < world_pnts.size(); ++i) {
		// project point
		cv::Mat world_pnt(4, 1, CV_64F);
		for (int x = 0; x < 3; ++x) {
			world_pnt.at<double>(x, 0) = (double)world_pnts[i][x];
		}
		world_pnt.at<double>(3, 0) = 1.0;
		cv::Mat projected_pt = proj * world_pnt;
		double z = projected_pt.at<double>(2, 0);
		projected_pts.push_back(cv::Point2f(projected_pt.at<double>(0, 0) / z, projected_pt.at<double>(1, 0) / z));
	}

	assert(world_pnts.size() == projected_pts.size());


	auto cmpx = [&] (const cv::Point2f& left, const cv::Point2f& right) -> bool {
		return (left.x < right.x);
	};

	auto cmpy = [&] (const cv::Point2f& left, const cv::Point2f& right) -> bool {
		return (left.y < right.y);
	};

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
		Point2f triangle_pts[3];
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
			WPts triangle;
			std::vector<cv::Point2f> image_coords;
			for (auto k = 0u; k < triangle_index.size(); ++k) {
				triangles.push_back(world_pnts[triangle_index[k]]);
				triangle.push_back(world_pnts[triangle_index[k]]);
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

void Reconstruct3D::recon_obj(const std::vector <cv::Vec2f>& img_pts1, const std::vector <cv::Vec2f>& img_pts2, std::vector<cv::Vec3f>& world_pts) const {
	assert(img_pts1.size() == img_pts2.size());

	// read file
	cv::Mat proj1 = P1;
	cv::Mat proj2 = P2;

	//std::cout << "Projection Matrix 1: " << std::endl;
	//std::cout << proj1 << std::endl;
	//std::cout << "Projection Matrix 2: " << std::endl;
	//std::cout << proj2 << std::endl;

	//cv::Mat img_pt_mat1(2, img_pts1.size(), CV_64F);
	//cv::Mat img_pt_mat2(2, img_pts1.size(), CV_64F);

	//for (auto i = 0u; i < img_pts1.size(); ++i) {
	//	for (auto x = 0u; x < 2; ++x) {
	//		img_pt_mat1.at<double>(x, i) = img_pts1[i][x];
	//	}
	//}

	//for (auto i = 0u; i < img_pts2.size(); ++i) {
	//	for (auto x = 0u; x < 2; ++x) {
	//		img_pt_mat2.at<double>(x, i) = img_pts2[i][x];
	//	}
	//}

	//cv::Mat pts4D(4, img_pts1.size(), CV_64F);

	//cv::triangulatePoints(proj1, proj2, img_pt_mat1, img_pt_mat2, pts4D);

	//for (auto i = 0u; i < img_pts2.size(); ++i) {
	//	world_pts.push_back(glm::dvec3(pts4D.at<double>(0, i) / pts4D.at<double>(3, i),
	//		pts4D.at<double>(1, i) / pts4D.at<double>(3, i),
	//		pts4D.at<double>(2, i) / pts4D.at<double>(3, i)));
	//}

	for (auto i = 0u; i < img_pts1.size(); ++i) {
		cv::Mat D(4, 3, CV_64F);
		cv::Mat b(4, 1, CV_64F);

		// fill rows from point 1
		for (auto x = 0u; x < 2; ++x) {
			cv::Mat row_D(1, 3, CV_64F);
			cv::Mat row_B(1, 1, CV_64F);
			fill_row(proj1, img_pts1[i][x], row_D, row_B, x);
			row_D.row(0).copyTo(D.row(x));
			row_B.row(0).copyTo(b.row(x));
		}
		for (auto x = 0u; x < 2; ++x) {
			cv::Mat row_D(1, 3, CV_64F);
			cv::Mat row_B(1, 1, CV_64F);
			fill_row(proj2, img_pts2[i][x], row_D, row_B, x);
			row_D.row(0).copyTo(D.row(x + 2));
			row_B.row(0).copyTo(b.row(x + 2));
		}
		cv::Mat XYZ(3, 1, CV_64F);
		cv::solve(D, b, XYZ, DECOMP_SVD);
		//cv::Mat D_inv = D.inv(DECOMP_SVD);
		//cv::Mat XYZ = D_inv * b;

		cv::Mat b_test = D * XYZ;
		cv::Mat results = b - b_test;
		//std::cout << "error: " << results << std::endl;
		//std::cout << "XYZ : " << XYZ << std::endl;
		//std::cout << "D Matrix : " << D << std::endl;
		//std::cout << "b Matrix : " << b << std::endl;
		//std::cout << "Img Points 1 : " << img_pts1[i].x << ", " << img_pts1[i].y << std::endl;
		//std::cout << "Img Points 2 : " << img_pts2[i].x << ", " << img_pts2[i].y << std::endl;
		world_pts.push_back(cv::Vec3f(XYZ.at<double>(0, 0), XYZ.at<double>(1, 0), XYZ.at<double>(2, 0)));
	}

}

void Reconstruct3D::fill_row(const cv::Mat& P, double coord, cv::Mat& fill_matrix, cv::Mat& B, bool is_y) const {
	int row = is_y;
	for (int j = 0; j < 3; ++j) {
		fill_matrix.at<double>(0, j) = coord * P.at<double>(2, j) - P.at<double>(row, j);
	}
	B.at<double>(0, 0) = P.at<double>(row, 3) - coord * P.at<double>(2, 3);
}
