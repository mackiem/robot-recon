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
	std::vector<cv::Point> find_line(const cv::Mat& frame, cv::Mat& drawing);
	std::vector<cv::Point3f> convert_to_camera_reference_frame(const std::vector<cv::Point3f>& point_3f, const cv::Mat& rotation_mat, const cv::Mat& translation_mat);
	std::vector<cv::Point3f> interpolate_edge(cv::Mat& rotation_mat, cv::Mat& translation_mat,
	                                          const std::vector<cv::Point3f>& checkerboard_3d_points,
	                                          const std::vector<cv::Point2f>& checkerboard_2d_points,
	                                          const std::vector<cv::Point>& edge_points);

	void calibrate_intrinsic(const std::vector<cv::Mat>& frames, cv::Mat& camera_matrix, cv::Mat& dist_coeffs);
	void load_calibration();
	std::vector<cv::Point3f> calculate_stripe_3d_points_in_camera_space(const std::string& checkerboard_video_filename, const std::string& stripe_video_filename, Plane& checkerboard_plane);
	void visualize_3d_points(const std::vector<cv::Point3f>& line_3d_points, const Ray& ray, const std::string& video_filename, const std::string& output_image_filename);
	void save_calibrated_plane(const Plane& plane);
	void load_calibrated_plane();
	void calculate_light_position();

	Ray construct_ray(const cv::Mat& camera_matrix, const cv::Point& image_coordinate) const;
	Ray construct_ray(const cv::Mat& camera_matrix, const cv::Point& image_coordinate, 
		const cv::Mat& rotation, const cv::Vec3d& translation) const;

	Plane transform_plane(const Plane& plane, 
		const cv::Mat& rotation, const cv::Vec3d& translation) const;

	cv::Vec3d transform(const cv::Vec3d& point, const cv::Mat& rotation, const cv::Vec3d& translation) const;
	
	cv::Point3f ray_plane_intersect(const Plane& plane, const Ray& ray) const;
	Plane construct_plane(const std::vector<cv::Point3f>& points) const;

	double point_to_plane_distance(const Plane& plane, const cv::Point3f & point) const;

	double calculate_error(const Plane& plane, const std::vector<cv::Point3f> & lines_3d) const;


	// reconstruct from video
	void reconstruct_from_video(const std::string& video_filename, int frame_no, float velocity, cv::Vec3f direction);

	cv::Mat createRT(cv::Mat& R, cv::Vec3f& T);

private:

	cv::Mat camera_matrix_;
	cv::Mat dist_coeffs_;

	cv::Mat rvecs_;
	cv::Mat tvecs_;

	Plane calibrated_plane_;

signals:
	void display_image(const cv::Mat mat);
	void create_plane_with_points_and_lines(std::vector<cv::Vec3f> points_3d,
		cv::Vec3f line_a, cv::Vec3f line_b, cv::Vec3f normal, double d);
	void create_points(std::vector<cv::Vec3f> points_3d, cv::Vec4f point_color);
	void create_plane(cv::Vec3f normal, double d, cv::Vec4f plane_color);

	void start_reconstruction_sequence();
	void create_reconstruction_frame(std::vector<cv::Vec3f> points_3d,
		cv::Vec3f line_a, cv::Vec3f line_b, cv::Vec3f normal, double d, cv::Mat RT);
	void create_reconstruction_image_list(std::vector<std::string> image_list);
};

