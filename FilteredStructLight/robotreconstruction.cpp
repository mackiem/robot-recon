#include "robotreconstruction.h"
#include <unordered_map>
#include <fstream>
#include "planefit.h"
#include <random>

RobotReconstruction::RobotReconstruction(void) {
}


RobotReconstruction::~RobotReconstruction(void) {
}

void RobotReconstruction::calibrate_intrinsic_from_video(const std::string& video_filename) {
	auto frames = get_subset_of_video_frames(video_filename, 100);
    // the camera will be deinitialized automatically in VideoCapture destructor
	calibrate_intrinsic(frames, camera_matrix_, dist_coeffs_);

}

void RobotReconstruction::identify_line_from_video(const std::string& video_filename) {
	auto frame = get_first_video_frame(video_filename);
	find_line(frame);
}

void RobotReconstruction::calibrate_extrinsic_from_video(const std::string& video_filename) {
	load_calibration();
	auto frame = get_first_video_frame(video_filename);
	//auto frame = get_image(video_filename);
	cv::Mat rvec;
	cv::Mat tvec;
	std::vector<cv::Point2f> calib_2d_points;
	std::vector<cv::Point3f> calib_3d_points;
	calc_camera_pos(frame, rvec, tvec, calib_3d_points, calib_2d_points);

	std::cout << "Rotation : " << rvec << std::endl;
	std::cout << "Translation : " << tvec << std::endl;
}

cv::Mat RobotReconstruction::get_first_video_frame(const std::string& video_filename) {
	cv::VideoCapture cap(video_filename); // open the default camera

    if(!cap.isOpened()) { // check if we succeeded
        throw std::runtime_error("Unable to open video");
	}

	cap.set(CV_CAP_PROP_POS_FRAMES, 30);

	cv::Mat frame;
	cap >> frame; // get a new frame from camera

	return frame;
}

cv::Mat RobotReconstruction::get_image(const std::string& image_filename) {
	cv::Mat image = cv::imread(image_filename);
	return image;
}

std::vector<cv::Mat> RobotReconstruction::get_subset_of_video_frames(const std::string& video_filename, const int nth_frame) {
	if (nth_frame < 1) {
        throw std::runtime_error("invalid frame value : " + nth_frame);
	}

	cv::VideoCapture cap(video_filename); // open the default camera
    if(!cap.isOpened()) { // check if we succeeded
        throw std::runtime_error("Unable to open video");
	}

    cv::Mat edges;
	std::vector<cv::Mat> frames;
    
	int frame_count = 0;
    for(;;) {
        cv::Mat frame;
        cap >> frame; // get a new frame from camera
		if (frame.empty()) {
			break;
		}
		if (frame_count % nth_frame == 0) {
			//cv::cvtColor(frame, frame, CV_BGR2GRAY);
			frames.push_back(frame);
		}
		
		frame_count++;
    }

	return frames;
}


void RobotReconstruction::calc_camera_pos(const std::vector<cv::Mat>& frames, cv::Mat& rotation_mat, cv::Mat& translation_mat,
                                          std::vector<cv::Point3f>& calib_3d_points,
                                          std::vector<cv::Point2f> & calib_2d_points) {

	cv::Size boardSize(9, 6);
	//std::vector<std::vector<cv::Point3f>> objectPointsVector;
	//std::vector<std::vector<cv::Point2f>> imagePointsVector;
	float squareSize = 23.f;

	cv::Mat grayscale_frame;

	bool found;

	for (int i = 0; i < frames.size(); ++i) {
		const cv::Mat& frame = frames[i];
		cv::cvtColor(frame, grayscale_frame, CV_BGR2GRAY);

		cv::Size imageSize = cv::Size(grayscale_frame.cols, grayscale_frame.rows);


		found = cv::findChessboardCorners(grayscale_frame, boardSize, calib_2d_points,
			CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FAST_CHECK | CV_CALIB_CB_NORMALIZE_IMAGE);

		if (found) {
			cv::cornerSubPix(grayscale_frame, calib_2d_points, cv::Size(11, 11),
				cv::Size(-1, -1), cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));


			for (auto j = 0; j < boardSize.height; j++) {
				for (auto k = 0; k < boardSize.width; k++) {
					calib_3d_points.push_back(cv::Point3f(j*squareSize, k*squareSize, 0));
				}
			}
			break;
		}
		else {
			std::cout << "Cannot find chessboard corners in grayscale in frame : " << i << std::endl;
		}
	}

	cv::Mat cloned_frame = grayscale_frame.clone();
	cv::cvtColor(cloned_frame, cloned_frame, CV_GRAY2BGR);
	cv::drawChessboardCorners(cloned_frame, boardSize, calib_2d_points, found);
	std::string filename("pose_");
	filename.append(".png");
	cv::imwrite(filename, cloned_frame);

	//cv::imshow("image", cloned_frame);
	// display image

	emit display_image(cloned_frame);

	if (!found) {
		// no point in continuing
		return;
	}

	cv::solvePnP(calib_3d_points, calib_2d_points, camera_matrix_, dist_coeffs_,
		rotation_mat, translation_mat);

	// save intrinsic parameters
	cv::FileStorage fs("extrinsic.yml", CV_STORAGE_WRITE);
	if (fs.isOpened())
	{
		fs << "R" << rotation_mat << "T" << translation_mat;
		fs.release();
	}
	else {
		std::cout << "Error: can not save the intrinsic parameters\n";
	}
}

// get extrinsic R|t
// get line
// interpolate line in 3D
// optimize light position

std::vector<cv::Point> RobotReconstruction::find_line(const cv::Mat& frame) {
	cv::RNG rng(12345);
	cv::Mat yuv_frame;
	cv::cvtColor(frame, yuv_frame, CV_BGR2GRAY);

	cv::imwrite("before_edge_detection.png", frame);
	// throttle to red
	//cv::inRange(yuv_frame, cv::Scalar(), cv::Scalar(), yuv_frame);

	cv::Mat row_sum;
	cv::reduce(yuv_frame, row_sum, 1, CV_REDUCE_SUM, CV_32S);

	int row_threshold = 1000;

	// for each row
	// order is important
	std::map<int, int> edge_value;

	for (int y = 0; y < yuv_frame.rows; ++y) {
		if ((row_sum.at<int>(y, 0) >= row_threshold)) {
			double min_val, max_val;
			cv::Point min_loc, max_loc;
			cv::Mat row_unsigned = yuv_frame.row(y);
			cv::Mat row;
			row_unsigned.convertTo(row, CV_32F);

			cv::minMaxLoc(row, &min_val, &max_val, &min_loc, &max_loc);

			float avg_intensity = (min_val + max_val) / 2.f;

			cv::Mat delta_I(1, row.cols, CV_32F);
			delta_I = row - cv::Scalar(avg_intensity);

			cv::threshold(delta_I, delta_I, 0, 255, CV_THRESH_BINARY);

			for (int x = delta_I.cols - 1; x >= 0; --x) {
				if (delta_I.at<float>(0, x) >= 254) {
					edge_value[y] = x;
					break;
				}
			}

			delta_I.copyTo(yuv_frame.row(y));
		}
	}

	// create edges
	std::vector<std::vector<cv::Point>> edges;

	std::vector<cv::Point> rightmost_edge;
	const int edge_threshold = 50;

	//for (auto itr = edge_value.begin(); itr != edge_value.end(); ++itr) {
	//	auto entry = *itr;
	//	int row = entry.first;
	//	int extreme_right_edge_col = entry.second;

	//	if (rightmost_edge.size() == 0) {
	//		rightmost_edge.push_back(cv::Point(extreme_right_edge_col, row));
	//	}
	//	else {
	//		// look at last entry
	//		cv::Point last_point = rightmost_edge.front();

	//		int edge_difference = extreme_right_edge_col - last_point.x;

	//		if (edge_difference > edge_threshold) {
	//			// we are on the wrong edge, there's a new edge around
	//			rightmost_edge.clear();
	//			rightmost_edge.push_back(cv::Point(extreme_right_edge_col, row));
	//		} else if (std::abs(edge_difference) < edge_threshold) {
	//			// we only add points if it's within the threshold
	//			rightmost_edge.push_back(cv::Point(extreme_right_edge_col, row));
	//		}

	//	}



		//if (edges.size() == 0) {
		//	std::vector<cv::Point> edge;
		//	cv::Point initial_edge_point(extreme_right_edge_col, row);
		//	edge.push_back(initial_edge_point);
		//	edges.push_back(edge);
		//} else {
		//	// find if closer to any one edge, and insert
		//	for ()

		//	// if not, create new edge

		//}
	//}

	for (auto itr = edge_value.begin(); itr != edge_value.end(); ++itr) {
		auto entry = *itr;
		int row = entry.first;
		int extreme_right_edge_col = entry.second;
		rightmost_edge.push_back(cv::Point(extreme_right_edge_col, row));
	}

	cv::Mat drawing = frame.clone();
	cv::Vec3b color(255, 255, 0);
	for (auto& point : rightmost_edge) {
		drawing.at<cv::Vec3b>(point) = color;
	}

	//drawing = frame.clone();
	//std::vector<cv::Vec2f> lines;
	//cv::HoughLines(yuv_frame, lines, 1, CV_PI / 180, 100);

	//for (size_t i = 0; i < lines.size(); i++)
	//{
	//	float rho = lines[i][0];
	//	float theta = lines[i][1];
	//	double a = cos(theta), b = sin(theta);
	//	double x0 = a*rho, y0 = b*rho;
	//	cv::Point pt1(cvRound(x0 + 1000 * (-b)),
	//		cvRound(y0 + 1000 * (a)));
	//	cv::Point pt2(cvRound(x0 - 1000 * (-b)),
	//		cvRound(y0 - 1000 * (a)));
	//	cv::line(drawing, pt1, pt2, cv::Scalar(0, 0, 255), 3, 8);
	//}

	// find max


	// i max + i min / 2
	cv::imwrite("after_edge_detection.png", drawing);

	//// edge detect
	//float threshold = 100.f;
	//cv::Mat canny_output_frame;
	//cv::Canny(yuv_frame, canny_output_frame, threshold, 2 * threshold);

	//// get rightmost pixels
	//std::vector<std::vector<cv::Point> > contours;
	//std::vector<cv::Vec4i> hierarchy;

	///// Find contours
	//cv::findContours(canny_output_frame, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0));

	///// Draw contours
	//for (int i = 0; i< contours.size(); i++)
	//{
	//	cv::Scalar color = cv::Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
	//	drawContours(drawing, contours, i, color, 2, 8, hierarchy, 0, cv::Point());
	//}

	emit display_image(drawing);

	return rightmost_edge;
}

std::vector<cv::Point3f> RobotReconstruction::convert_to_camera_reference_frame(
	const std::vector<cv::Point3f>& points_3d, const cv::Mat& rotation_mat, const cv::Mat& translation_mat) {

	std::vector < cv::Point3f> converted_points;

	for (auto& point_3d : points_3d) {
		cv::Mat point_XYZ_1(3, 1, CV_64F);
		point_XYZ_1.at<double>(0, 0) = point_3d.x;
		point_XYZ_1.at<double>(1, 0) = point_3d.y;
		point_XYZ_1.at<double>(2, 0) = point_3d.z;
		
		cv::Mat R;
		cv::Rodrigues(rotation_mat, R);
		cv::Mat calib_3d_point_in_camera_reference = 
			 R * point_XYZ_1 + translation_mat;

		cv::Point3f converted_3d_point(calib_3d_point_in_camera_reference);
		converted_points.push_back(converted_3d_point);
	}

	return converted_points;
}


std::vector<cv::Point3f> RobotReconstruction::interpolate_edge(cv::Mat& rotation_mat, cv::Mat& translation_mat,
                                                               const std::vector<cv::Point3f>& checkerboard_3d_points,
                                                               const std::vector<cv::Point2f>& checkerboard_2d_points,
                                                               const std::vector<cv::Point>& edge_2d_points) {

	std::vector<cv::Point3f> edge_3d_points;

	// find interpolation
	for (int i = 0; i < edge_2d_points.size(); ++i) {
		cv::Point edge_point = edge_2d_points[i];

		// find closest 4 points

		double min_distance = DBL_MAX;
		int index = -1;

		for (int p = 0; p < checkerboard_2d_points.size(); ++p) {
			cv::Point cb_2d_point = checkerboard_2d_points[p];
			double distance = cv::norm(cv::Vec2i(cb_2d_point), cv::Vec2i(edge_point));
			if (distance < min_distance) {
				min_distance = distance;
				index = p;
			}
		}

		if (index == -1) {
			std::cout << "Error occurred when searching for minimum. Distance : " << min_distance 
				<< std::endl;
		}

		std::set<PointDistance> minimal_points;
		cv::Point minimum_point = checkerboard_2d_points[index];
		for (int p = 0; p < checkerboard_2d_points.size(); ++p) {
			cv::Point cb_2d_point = checkerboard_2d_points[p];
			double distance = cv::norm(cv::Vec2i(cb_2d_point), cv::Vec2i(minimum_point));
			PointDistance point_distance;
			point_distance.distance = distance;
			point_distance.point = cb_2d_point;
			point_distance.index = p;
			minimal_points.insert(point_distance);
		}

		// interpolate

		// ratio
		std::vector<PointDistance> selected_points(4);
		double ratio[4][2];
		int count = 0;

		for (auto itr = minimal_points.begin();
			(itr != minimal_points.end()) && count < 4;
			++itr, ++count) {
			PointDistance point_distance = *itr;
			selected_points[count] = point_distance;
		}

		ratio[0][0] = edge_point.x  - selected_points[0].point.x;
		ratio[0][1] = edge_point.y  - selected_points[0].point.y;

		ratio[1][0] =  selected_points[1].point.x - edge_point.x;
		ratio[1][1] = selected_points[1].point.y - edge_point.y;

		ratio[2][0] = edge_point.x  - selected_points[2].point.x;
		ratio[2][1] = edge_point.y  - selected_points[2].point.y;

		ratio[3][0] = selected_points[3].point.x - edge_point.x;
		ratio[3][1] = selected_points[3].point.y - edge_point.y;

		cv::Mat point_XYZ_0(3, 1, CV_64F);
		cv::Point3f checkerboard_point_0 =  checkerboard_3d_points[selected_points[0].index];
		point_XYZ_0.at<double>(0, 0) = checkerboard_point_0.x;
		point_XYZ_0.at<double>(1, 0) = checkerboard_point_0.y;
		point_XYZ_0.at<double>(2, 0) = checkerboard_point_0.z;

		cv::Mat point_XYZ_1(3, 1, CV_64F);
		cv::Point3f checkerboard_point_1 =  checkerboard_3d_points[selected_points[1].index];
		point_XYZ_1.at<double>(0, 0) = checkerboard_point_1.x;
		point_XYZ_1.at<double>(1, 0) = checkerboard_point_1.y;
		point_XYZ_1.at<double>(2, 0) = checkerboard_point_1.z;

		cv::Mat R;
		cv::Rodrigues(rotation_mat, R);

		cv::Mat rotated = R * point_XYZ_0;

		cv::Mat calib_3d_point_in_camera_reference_0 = 
			 rotated + translation_mat;

		cv::Mat calib_3d_point_in_camera_reference_1 = 
			 R * point_XYZ_1 + translation_mat;

		cv::Point3f calib_3d_point_0(calib_3d_point_in_camera_reference_0);
		cv::Point3f calib_3d_point_1(calib_3d_point_in_camera_reference_1);

		// interpolate
		float edge_3d_x = ratio[1][0] * calib_3d_point_0.x + ratio[0][0] * calib_3d_point_1.x;
		edge_3d_x /= (ratio[1][0] + ratio[0][0]);

		float edge_3d_y = ratio[1][1] * calib_3d_point_0.y + ratio[0][1] * calib_3d_point_1.y;
		edge_3d_y /= (ratio[1][1] + ratio[0][1]);

		edge_3d_points.push_back(cv::Point3f(edge_3d_x, edge_3d_y, 0));
	}
	return edge_3d_points;
}

void RobotReconstruction::calibrate_intrinsic(const std::vector<cv::Mat>& frames, cv::Mat& camera_matrix, cv::Mat& dist_coeffs) {

	cv::Size boardSize(9, 6);
	std::vector<std::vector<cv::Point3f>> objectPointsVector;
	std::vector<std::vector<cv::Point2f>> imagePointsVector;
	float squareSize = 11.f;

	cv::Size imageSize;
	int checkerboard_found_count = 0;
	for (auto i = 0; i < frames.size(); ++i) {
		cv::Mat grayscale_frame;
		cv::cvtColor(frames[i], grayscale_frame, CV_BGR2GRAY);
		imageSize = cv::Size(frames[i].cols, frames[i].rows);
		std::vector<cv::Point2f> point_buffer;
		bool found = cv::findChessboardCorners( grayscale_frame, boardSize, point_buffer,
			CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FAST_CHECK | CV_CALIB_CB_NORMALIZE_IMAGE);
		
		if (found) {
			cv::cornerSubPix( grayscale_frame, point_buffer, cv::Size(11,11),
				cv::Size(-1,-1), cv::TermCriteria( CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 30, 0.1 ));

			std::vector<cv::Point3f> objectPoints;

			for (auto j = 0; j < boardSize.height; j++) {
				for (auto k = 0; k < boardSize.width; k++) {
					objectPoints.push_back(cv::Point3f(j*squareSize, k*squareSize, 0));
				}
			}
			objectPointsVector.push_back(objectPoints);
			imagePointsVector.push_back(point_buffer);

			// stop if enough checkerboards found
			checkerboard_found_count++;
			if (checkerboard_found_count > 4) {
				break;
			}

		} else {
			std::cout << "Cannot find chessboard corners in frame : " << i << std::endl;
		}
		cv::Mat cloned_frame = grayscale_frame.clone();
		cv::cvtColor(cloned_frame, cloned_frame, CV_GRAY2BGR);
		cv::drawChessboardCorners(cloned_frame, boardSize, point_buffer, found);
		std::string filename("checkerboard_");
		filename.append(std::to_string(i)).append(".png");
		cv::imwrite(filename, cloned_frame);

		//cv::imshow("image", cloned_frame);
		// display image
		emit display_image(cloned_frame);
	}

	camera_matrix = cv::Mat::eye(3, 3, CV_64F);
    
	dist_coeffs = cv::Mat::zeros(8, 1, CV_64F);

	std::vector<cv::Mat> rvecs; 
	std::vector<cv::Mat> tvecs;

	if (imagePointsVector.size() > 0) {
		
		camera_matrix = cv::initCameraMatrix2D(objectPointsVector, imagePointsVector, imageSize);
		double rms = calibrateCamera(objectPointsVector, imagePointsVector, imageSize, camera_matrix,
                    dist_coeffs, rvecs, tvecs, CV_CALIB_USE_INTRINSIC_GUESS | CV_CALIB_FIX_K4|CV_CALIB_FIX_K5);
	 
		std::cout << "Camera Matrix : " << std::endl << camera_matrix << std::endl;
		std::cout << "Dist. Coeffs : " << std::endl << dist_coeffs << std::endl;

		// these measurements (mm) are from https://www.raspberrypi.org/documentation/hardware/camera.md

		float RPI_SENSOR_WIDTH = 3.76;
		float RPI_SENSOR_HEIGHT = 2.74;
		
		const float pixelSize = 1.4 / 1000.0;
		float fx = camera_matrix.at<double>(0, 0) * pixelSize; // RPI_SENSOR_WIDTH / imageSize.width;
		float fy = camera_matrix.at<double>(1, 1) * pixelSize; // RPI_SENSOR_HEIGHT / imageSize.height;

		std::cout << "focal length (x, y, actual) in mm: " << fx << " , " << fy << " , " <<  (fx + fy) / 2.f << std::endl;  

//		std::cout << "Rotation : " << std::endl << cameraMatrix << std::endl;

		std::cout << "RMS error reported by calibrateCamera in pixels: " << rms << std::endl;
		std::cout << "No of checkerboard images: " << checkerboard_found_count << std::endl;

		// save intrinsic parameters
		cv::FileStorage fs("intrinsic.yml", CV_STORAGE_WRITE);
		if (fs.isOpened())
		{
			fs << "M" << camera_matrix_ 
				<< "D" << dist_coeffs_;
			fs.release();
		}
		else {
			std::cout << "Error: can not save the intrinsic parameters\n";
		}

		double  fovx, fovy, focalLength, aspectRatio;
		cv::Point2d principalPoint;
		cv::calibrationMatrixValues(camera_matrix, imageSize, pixelSize * imageSize.width, pixelSize * imageSize.height,
			fovx, fovy, focalLength, principalPoint, aspectRatio);

		std::cout
			<< fovx << ", "
			<< fovy << ", "
			<< focalLength << ", "
			<< aspectRatio << ", "
			<< principalPoint << std::endl;

	} else {
		std::cout << "Not enough images to calibrate. No. of checkerboards detected: " << imagePointsVector.size() << std::endl;
	}

	
}

void RobotReconstruction::load_calibration() {

    // save intrinsic parameters
	cv::FileStorage fs("intrinsic.yml", CV_STORAGE_READ);
    if (fs.isOpened())
    {
        fs["M"] >> camera_matrix_;
        fs["D"] >> dist_coeffs_;
        fs.release();
    }
    else {
        std::cout << "Error: can not load the intrinsic parameters\n";
        return;
    }


    fs.open("extrinsic.yml", CV_STORAGE_READ);
    if (fs.isOpened())
    {
        fs["R"] >> rvecs_;
        fs["T"] >> tvecs_;
        fs.release();
    }
    else {
        std::cout << "Error: can not save the intrinsic parameters\n";
        return;
    }


}

std::vector<cv::Point3f> RobotReconstruction::calculate_stripe_3d_points_in_camera_space
	(const std::string& checkerboard_video_filename,
	 const std::string& stripe_video_filename, Plane& checkerboard_plane) {

	auto checkerboard_frames = get_subset_of_video_frames(checkerboard_video_filename, 100);
	//auto frame = get_image("calib-checkerboard.png");
	cv::Mat rvec;
	cv::Mat tvec;
	std::vector<cv::Point2f> calib_2d_points;
	std::vector<cv::Point3f> calib_3d_points;
	calc_camera_pos(checkerboard_frames, rvec, tvec, calib_3d_points, calib_2d_points);

	std::cout << "Rotation : " << rvec << std::endl;
	std::cout << "Translation : " << tvec << std::endl;

	auto line_stripe_frame = get_first_video_frame(stripe_video_filename);
	auto line_image_coordinates = find_line(line_stripe_frame);

	// construct plane
	std::vector<cv::Point3f> calib_3d_point_camera_origin =
		convert_to_camera_reference_frame(calib_3d_points, rvec, tvec);

	checkerboard_plane = construct_plane(calib_3d_point_camera_origin);

	// get 3d point
	std::vector<cv::Point3f> line_3d_points;
	for (auto& line_image_coordinate : line_image_coordinates) {
		try {
			Ray ray = construct_ray(camera_matrix_, line_image_coordinate);
			auto intersection_point = ray_plane_intersect(checkerboard_plane, ray);
			line_3d_points.push_back(intersection_point);
		}
		catch (RRException& ex) {
			std::cout << "Cannot intersect point : " << line_image_coordinate << std::endl;
		}
	}

	return line_3d_points;
}

void RobotReconstruction::visualize_3d_points(const std::vector<cv::Point3f>& line_3d_points,
	const Ray& ray,
	const std::string& video_filename, const std::string& output_image_filename) {

	auto line_stripe_frame = get_first_video_frame(video_filename);

	auto line_image_coordinates = find_line(line_stripe_frame);

	cv::Mat drawing = line_stripe_frame.clone();

	cv::Vec3b original_edge_color = cv::Vec3b(0, 255, 0);
	cv::Vec3b projected_edge_color = cv::Vec3b(0, 255, 255);

	cv::Rect rect(0, 0, drawing.cols, drawing.rows);
	for (auto& line_pt : line_image_coordinates) {
		//projected_pt = cv::Point2f(projected_pt.y, projected_pt.x);
		if (rect.contains(line_pt)) {
			drawing.at<cv::Vec3b>(line_pt) = original_edge_color;
		}
	}

	cv::Mat null_rotation_matrix = cv::Mat::eye(3, 3, CV_64F);
	cv::Mat null_rvec;
	cv::Rodrigues(null_rotation_matrix, null_rvec);
	cv::Vec3f null_tvec(0, 0, 0);

	std::vector<cv::Point2f> projected_image_points;
	cv::projectPoints(line_3d_points, null_rvec, null_tvec,
		camera_matrix_, dist_coeffs_, projected_image_points);

	for (auto& projected_pt : projected_image_points) {
		//projected_pt = cv::Point2f(projected_pt.y, projected_pt.x);
		if (rect.contains(projected_pt)) {
			drawing.at<cv::Vec3b>(projected_pt) = projected_edge_color;
		}
	}


	// project line

	std::vector<cv::Point3d> fitted_line;
	fitted_line.push_back(ray.a);
	fitted_line.push_back(ray.b);
	std::vector<cv::Point2d> projected_line_points;

	cv::projectPoints(fitted_line, null_rvec, null_tvec,
		camera_matrix_, dist_coeffs_, projected_line_points);

	cv::line(drawing, projected_line_points[0], projected_line_points[1], cv::Scalar(255.0, 0.0, 0.0), 1);

	cv::imwrite(output_image_filename, drawing);
	emit display_image(drawing);
}

void RobotReconstruction::save_calibrated_plane(const Plane& plane) {

	cv::FileStorage fs("plane.yml", CV_STORAGE_WRITE);
	if (fs.isOpened())
	{
		fs << "n" << plane.n << "d" << plane.d;
		fs.release();
	}
	else {
		std::cout << "Error: can not save the intrinsic parameters\n";
	}

}

void RobotReconstruction::load_calibrated_plane() {
	cv::FileStorage fs("plane.yml", CV_STORAGE_READ);
    if (fs.isOpened())
    {
        fs["n"] >> calibrated_plane_.n;
        fs["d"] >> calibrated_plane_.d;
        fs.release();
    }
    else {
        std::cout << "Error: can not load the calibrated plane\n";
        return;
    }

}

cv::Vec3f convert(cv::Vec3d& pt) {
	cv::Vec3f new_pt(static_cast<float>(pt[0]), static_cast<float>(pt[1]), static_cast<float>(pt[2]));
	return new_pt;
}

std::vector<cv::Vec3f> convert_vec(const std::vector<cv::Point3f>& point_vec) {
	std::vector<cv::Vec3f> vec_vec(point_vec.size());
	for (auto i = 0u; i < point_vec.size(); ++i) {
		vec_vec[i] = cv::Vec3f(point_vec[i].x, point_vec[i].y, point_vec[i].z);
	}
	return vec_vec;
}
	

void RobotReconstruction::calculate_light_position() {
	load_calibration();

	std::vector<std::string> checkerboard_filenames;
	std::vector<std::string> stripe_filenames;

	for (int i = 1; i < 6; ++i) {
		std::stringstream light_stripe_filename;
		light_stripe_filename << "light-stripe-calib-stripe_" << i << ".h264";
		stripe_filenames.push_back(light_stripe_filename.str());

		std::stringstream checkerboard_filename;
		checkerboard_filename << "light-stripe-calib-checkerboard_" << i << ".h264";
		checkerboard_filenames.push_back(checkerboard_filename.str());
	}

	std::vector<Ray> rays;
	std::vector<std::vector<cv::Point3f>> lines_3d_points;
	for (auto i = 0; i < checkerboard_filenames.size(); ++i) {
		Plane checkerboard_plane;
		auto line_3d_points = calculate_stripe_3d_points_in_camera_space(
			checkerboard_filenames[i],
			stripe_filenames[i], checkerboard_plane);

		lines_3d_points.push_back(line_3d_points);
		Ray ray;
		fit_line(line_3d_points, ray);

		std::stringstream projected_line_filename;
		projected_line_filename << "projected_line_" << (i + 1) << ".png";
		visualize_3d_points(line_3d_points, ray, stripe_filenames[i], projected_line_filename.str());


		rays.push_back(ray);

		// visualize 3d
		create_plane_with_points_and_lines(convert_vec(line_3d_points),
			convert(ray.a), convert(ray.b), convert(checkerboard_plane.n), checkerboard_plane.d);

	}


	Plane calibrated_plane;
	fit_plane(rays, calibrated_plane);

	std::cout << "Plane : " << " normal : " << calibrated_plane.n << std::endl;
	std::cout << "d : " << calibrated_plane.d << std::endl;

	
	for (auto i = 0; i < lines_3d_points.size(); ++i) {
		double error = calculate_error(calibrated_plane, lines_3d_points[i]);
		std::cout << "Error from plane fit  - line " << (i+1) << " : " << error << std::endl;
	}

	create_plane(calibrated_plane.n, calibrated_plane.d, cv::Vec4f(1.f, 0.f, 0.f, 0.6f));

	save_calibrated_plane(calibrated_plane);


	//auto interpolated_points = interpolate_edge(rvec, tvec, calib_3d_points, calib_2d_points, line_points);
	//std::cout << "Interpolated Points : " << std::endl;
	//std::ofstream file("edge_points.txt");
	//for (auto& interpolated_point : interpolated_points) {
	//	std::cout << interpolated_point << std::endl;
	//	file << interpolated_point << std::endl;
	//}

	//file.close();
}

Ray RobotReconstruction::construct_ray(const cv::Mat& camera_matrix, const cv::Point& image_coordinate) const {

	cv::Mat R = cv::Mat::eye(3, 3, CV_64F);
	Ray ray = construct_ray(camera_matrix_, image_coordinate, R, cv::Vec3d(0.f, 0.f, 0.f));

	return ray;
}

Ray RobotReconstruction::construct_ray(const cv::Mat& camera_matrix, const cv::Point& image_coordinate, 
	const cv::Mat& rotation, const cv::Vec3d& translation) const {

	const double pixelSize = 1.4 / 1000.0;
	const double sensor_width = 3.76;
	const double sensor_height = 2.74;
	//const double screen_width = 2592;
	//const double screen_height = 1944;
	const double screen_width = 1920;
	const double screen_height = 1080;

	// ******************
	// ******************
	double focal_length_px = camera_matrix.at<double>(0, 0);
	// ******************
	// ******************

	double u_0 = camera_matrix.at<double>(0, 2);
	double v_0 = camera_matrix.at<double>(1, 2);

	cv::Vec3d principal_point(u_0, v_0, focal_length_px);
	cv::Vec3d origin(0, 0, 0);

	cv::Vec3d principal_point_mm;
	principal_point_mm[0] = principal_point[0] * (sensor_width / screen_width);
	principal_point_mm[1] = principal_point[1] * (sensor_height / screen_height);
	principal_point_mm[2] = focal_length_px * sensor_width / screen_width;


	cv::Point2d image_coordinate_mm;
	image_coordinate_mm.x = image_coordinate.x * (sensor_width / screen_width);
	image_coordinate_mm.y = image_coordinate.y * (sensor_height / screen_height);

	cv::Vec3d transformed_principal_point = transform(principal_point_mm, rotation, translation);
	cv::Vec3d transformed_origin = transform(origin, rotation, translation);

	//cv::Point2d image_coordinate_mm(image_coordinate.x * pixelSize, image_coordinate.y * pixelSize);

	double x = image_coordinate_mm.x - transformed_principal_point[0];

	// ******************
	// ******************
	double y = image_coordinate_mm.y - transformed_principal_point[1];
	// ******************
	// ******************

	double transformed_focal_length = transformed_principal_point[2];

	Ray ray;
	ray.a = (transformed_origin);
	ray.b = cv::Vec3d(x, y, transformed_focal_length);

	return ray;
}

Plane RobotReconstruction::transform_plane(const Plane& plane, const cv::Mat& rotation, const cv::Vec3d& translation) const {
	double x = 0;
	double y = 0;
	
	// find point on plane
	cv::Vec3d calc_point(0, 0, 0);
	double z = 0.f;
	float c_component = plane.n[2];
	if (std::abs(c_component) > 1e-6) {
		z = (plane.d - plane.n.dot(calc_point)) / c_component;
	}

	cv::Vec3d point_on_plane(x, y, z);
	cv::Vec3d normalized_n = cv::normalize(plane.n);

	Plane transformed_plane;
	cv::Vec3d null_translation(0.0, 0.0, 0.0);
	transformed_plane.n = cv::normalize(transform(normalized_n, rotation, null_translation));
	cv::Vec3d transformed_point = transform(point_on_plane, rotation, translation);
	transformed_plane.d = transformed_plane.n.dot(transformed_point);
	return transformed_plane;
	
}

cv::Vec3d RobotReconstruction::transform(const cv::Vec3d& point, 
	const cv::Mat& rotation, const cv::Vec3d& translation) const {
	cv::Mat transformed_point = rotation * cv::Mat(point) + cv::Mat(translation);
	return cv::Vec3d(transformed_point);
}

cv::Point3f RobotReconstruction::ray_plane_intersect(const Plane& plane, const Ray& ray) const {
	cv::Vec3d v = ray.b - ray.a;
	v = cv::normalize(v);
	// t(n . v) - d = 0, so t = d / (n.v)
	double numerator = plane.d - (plane.n.dot(ray.a));
	double denominator = v.dot(plane.n);

	if (std::abs(denominator) < 1e-6) {
		// line is parallel to plain
		std::cout << "Line is parallel to plane" << std::endl;
		throw RRException();
	}

	double t = numerator / denominator;

	//if (t < 0) {
	//	// ray intersects on opposite direction
	//	std::cout << "ray interesects on opposite direction" << std::endl;
	//	throw RRException();
	//}

	cv::Point3f intersection_pt = ray.a + t * v;

	return intersection_pt;
}

Plane RobotReconstruction::construct_plane(const std::vector<cv::Point3f>& points) const {
	
	if (points.size() < 3) {
		throw RRException();
	}

	cv::Vec3f vec_1, vec_2, cross_vector;

	bool non_collinear_points_found = false;

	cv::Point3f point_on_plane;
	
	for (auto i = 0u; i < points.size(); ++i) {
		point_on_plane = points[i];
		vec_1 = points[i] - points[i+1];
		vec_2 = points[i+1] - points[i+2];

		cross_vector = vec_1.cross(vec_2);

		double norm = cv::norm(cross_vector);
		if (norm > 1e-2) {
			// non - colinear points
			non_collinear_points_found = true;
			break;
		}
	}

	Plane plane;
	plane.n = cv::normalize(cross_vector);
	plane.d = plane.n.dot(cv::Vec3f(point_on_plane));


	return plane;

}

double RobotReconstruction::point_to_plane_distance(const Plane& plane, const cv::Point3f & point) const {
	cv::Vec3d double_point(point.x, point.y, point.z);

	double n_length = cv::norm(plane.n);
	double distance = (plane.n.dot(double_point) - plane.d) / n_length;

	return distance;
}

double RobotReconstruction::calculate_error(const Plane& plane, 
	const std::vector<cv::Point3f> & lines_3d) const {
	double error = 0.0;
	if (lines_3d.size() > 0) {
		for (auto& pt : lines_3d) {
			error += std::pow(point_to_plane_distance(plane, pt), 2);
		}
		error /= lines_3d.size();
		error = std::sqrt(error);
	}
	return error;
}

void RobotReconstruction::reconstruct_from_video(const std::string& video_filename, int frame_no, 
	float velocity, cv::Vec3f direction) {

	load_calibration();
	load_calibrated_plane();

	std::vector<cv::Mat> video_frames = get_subset_of_video_frames(video_filename, frame_no);

	float fps = 24.f;

	std::random_device rd;
	std::mt19937 e2(rd());
	std::uniform_real_distribution<> dist(0.3, 1);


	for (auto i = 0u; i < video_frames.size(); ++i) {
		cv::Mat& frame = video_frames[i];

		std::vector<cv::Point> line_2d = find_line(frame);
		std::vector<cv::Vec3f> reconstructed_points;
		
		if (line_2d.size() > 0) {
			
			cv::Vec3f translation = i * (1.f / fps) * velocity * direction;
			cv::Mat R = cv::Mat::eye(3, 3, CV_64F);
			Plane transformed_plane = transform_plane(calibrated_plane_, R, translation);

			for (auto& line_pt : line_2d) {
				try {
					Ray ray = construct_ray(camera_matrix_, line_pt, R, translation);
					cv::Vec3f reconstructed_point = ray_plane_intersect(transformed_plane, ray);
					reconstructed_points.push_back(reconstructed_point);
				}
				catch (RRException& exception) {
					// ignore
				}
			}
			if (reconstructed_points.size() > 0) {
				cv::Vec4f common_color(dist(e2), dist(e2), dist(e2), 0.3f);
				emit create_plane(transformed_plane.n, transformed_plane.d, common_color);
				emit create_points(reconstructed_points, common_color);
				QCoreApplication::processEvents();
			}
		}
	}
}
