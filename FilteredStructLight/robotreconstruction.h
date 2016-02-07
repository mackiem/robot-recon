#pragma once

#include <string>
#include <QtCore>
#include "opencv2/opencv.hpp"
#include <set>
#include <stdexcept>

class RRException : public std::exception {
public:
	RRException() { std::exception(); };

};

struct PointDistance {
	cv::Point point;
	double distance;
	int index;
	bool operator<(const PointDistance& other) const {
		return (distance < other.distance);
	}
};

struct Ray {
	cv::Vec3d a;
	cv::Vec3d b;
};

struct Plane {
	// ax + by + cz = d
	cv::Vec3d n;
	double d;
};

class RobotReconstruction : public QObject
{
	Q_OBJECT
public:
	RobotReconstruction(void);
	~RobotReconstruction(void);
	void calibrate_intrinsic_from_video(const std::string& video_filename);
	void identify_line_from_video(const std::string& video_filename);
	void calibrate_extrinsic_from_video(const std::string& video_filename);
	cv::Mat get_first_video_frame(const std::string& video_filename);
	cv::Mat get_image(const std::string& video_filename);
	std::vector<cv::Mat> get_subset_of_video_frames(const std::string& video_filename, const int nth_frame);
	void calc_camera_pos(const std::vector<cv::Mat>& frames, cv::Mat& rotation_mat, cv::Mat& translation_mat, std::vector<cv::Point3f>& calib_3d_points, std::vector<cv::Point2f> & calib_2d_points);
	std::vector<cv::Point> find_line(const cv::Mat& frame);
	std::vector<cv::Point3f> convert_to_camera_reference_frame(const std::vector<cv::Point3f>& point_3f, const cv::Mat& rotation_mat, const cv::Mat& translation_mat);
	std::vector<cv::Point3f> interpolate_edge(cv::Mat& rotation_mat, cv::Mat& translation_mat,
	                                          const std::vector<cv::Point3f>& checkerboard_3d_points,
	                                          const std::vector<cv::Point2f>& checkerboard_2d_points,
	                                          const std::vector<cv::Point>& edge_points);

	void calibrate_intrinsic(const std::vector<cv::Mat>& frames, cv::Mat& camera_matrix, cv::Mat& dist_coeffs);
	void load_calibration();
	std::vector<cv::Point3f> calculate_stripe_3d_points_in_camera_space(const std::string& checkerboard_video_filename, const std::string& stripe_video_filename);
	void calculate_light_position();

	Ray construct_ray(const cv::Mat& camera_matrix, const cv::Point& image_coordinate) const;
	cv::Point3f ray_plane_intersect(const Plane& plane, const Ray& ray) const;
	Plane construct_plane(const std::vector<cv::Point3f>& points) const;


private:

	cv::Mat camera_matrix_;
	cv::Mat dist_coeffs_;

	cv::Mat rvecs_;
	cv::Mat tvecs_;

signals:
	void display_image(const cv::Mat mat);
};

