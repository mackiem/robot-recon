#pragma once
#include <QtCore>
#include "FlyCapture2.h"
#include <unordered_map>
#include "cameradisplaywidget.h"
#include <chrono>
#include "fsl_common.h"
#include <opencv2/video/background_segm.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/core/core.hpp>

using namespace FlyCapture2;
using namespace std;
using namespace cv;

typedef std::unordered_map<int, std::vector<cv::Mat>> CameraImgMap;
typedef std::vector<std::pair<int, int>> CameraPairs;
typedef std::unordered_map<CameraPairs, cv::Mat> CameraPairMatrix;


typedef std::vector<cv::Vec6f> TriangleList;
typedef std::pair<int, std::pair<cv::Vec3i, cv::Vec3i> > UniqueColorPair;
typedef std::unordered_map<int, std::vector<int>> UniqueEdges;



class Reconstruct3D : public QObject
{
	Q_OBJECT


private:
	int no_of_cams_;
	bool calibration_loaded_;
	bool started_capture_;
	static std::string recon_dirname_;
	static std::string calib_dirname_;
	static std::string camera_subdir_prefix_;

	struct ImageSet {
		int cam_no;
		std::vector<cv::Mat> images;
	};

	CameraImgMap camera_img_map_;
	cv::Size board_size_;

	// temp stuff for demo
	cv::Mat cameraMatrix[2];
	cv::Mat distCoeffs[2];
	cv::Mat R, T, E, F;
	cv::Size imageSize;
	cv::Mat rmap[2][2]; 
	cv::Rect validRoi[2];
	cv::Mat R1, R2, P1, P2, Q;
	std::chrono::high_resolution_clock::time_point last_updated_;
public:
	void clear_camera_img_map();

	void calibrate(std::vector<std::pair<int, int>> camera_pairs);

	void run_reconstruction(std::vector<std::pair<int, int>> camera_pairs, int no_of_images);

	void stereo_calibrate(CameraImgMap& camera_img_map, int left_cam, int right_cam,
	                      Size boardSize, bool useCalibrated, bool write_images = true);

	void recalibrate(std::vector<std::pair<int, int>> camera_pairs);
	
	void alter_img_for_projection(const cv::Mat& img, cv::Mat& remapped_img, bool is_right) const;
	void fill_row(const cv::Mat& P, double coord, cv::Mat& fill_matrix, cv::Mat& B, bool is_y) const;

		void load_calibration(int left_cam, int right_cam);

	void create_rectification_map();

	void pick_correlated_points(std::vector<UniqueEdges>& unique_colors_left,
		std::vector<UniqueEdges>& unique_colors_right,
		IPts& img_pts1, IPts& img_pts, CameraImgMap& camera_img_map, int left_cam, int right_cam);

	void fit_gaussians(CameraImgMap& camera_img_map, int cam_no, std::unordered_map<int, std::pair<int, int>> mid_points);

	void correpond_with_gaussians(CameraImgMap& camera_img_map, int left_cam_no, int right_cam_no,
		IPts& img_pts1, IPts& img_pts2, Intensities& left_intensities, Intensities& right_intensities);

	void compute_correlation_using_gaussian(IPts& img_pts1, IPts& img_pts2,
		CameraImgMap& camera_img_map, std::vector<std::pair<int, int>> camera_pairs, 
		Intensities& left_intensities, Intensities& right_intensities);

	void get_unique_edges(CameraImgMap& camera_img_map, int cam, std::vector<UniqueEdges> & unique_colors_per_image, bool is_right);
	
	void compute_correlation(IPts& img_pts1, IPts& img_pts2,
		CameraImgMap& camera_img_map, std::vector<std::pair<int, int>> camera_pairs);

	void pre_process_img(cv::Mat& img, cv::Mat& rImg, bool is_right);
	void init_imgs(CameraImgMap& camera_img_map, int cam, bool is_right);
	

	void read_file(const std::string& file_name, IPts& img_pts1, IPts& img_pts2);
	void write_file(const std::string& file_name, const IPts& img_pts1, const IPts& img_pts2);

	void triangulate_pts(const WPts& pnts, WPt& triangles, 
		IPt& texture_coords, cv::Mat& remapped_img);
	void gen_texture(GLuint& texture_id, cv::Mat& remapped_img_for_texture) const;
	
	
	void resize_img(cv::Mat& img, const unsigned int resize_width) const;
	std::string generate_intrinsics_filename(int left_num, int right_num);
	std::string generate_extrinsics_filename(int left_num, int right_num);


	cv::Vec3d Reconstruct3D::calculate_3D_point(const cv::Vec2d& left_image_point, const cv::Vec2d& right_image_point, 
		const cv::Mat& proj1, const cv::Mat& proj2) const;
	void recon_obj(const IPts& img_pts1, const IPts& img_pts2, WPts& world_pts);

	void correct_img_coordinates(IPts& img_pts1, IPts& img_pts2);


	cv::Point2d Reconstruct3D::project_point(const cv::Vec3d& world_pt, const cv::Mat& projection_matrix) const;

	void project_points_on_to_img(WPts& world_pts, WPts& world_point_colors, cv::Mat& left_img, cv::Mat& right_img,
		IPts& img_pts1, IPts& img_pts2);
	void reconstruct(CameraPairs& camera_pairs, int no_of_images);
	//void gen_texture(GLuint& texture_id_, cv::Mat& remapped_img_for_texture) const;
	void re_reconstruct(CameraPairs& camera_pairs, int no_of_images);
	void filter_noise(WPts& world_pts, const Intensities& left_intensities, const Intensities& right_intensities);


	Reconstruct3D(int no_of_cams, QObject* parent);
	~Reconstruct3D();

public slots:
	void collect_images(const FlyCapture2::Image& img, int cam_no);
	void collect_images_without_delay(const FlyCapture2::Image& img, int cam_no);
	void compute_correspondence(FlyCapture2::Image img, int cam_no);

signals:
	void finished_reconstruction(WPts world_pts);
	void finished_reconstruction_with_triangles(WPts world_pts, WPts world_pt_colors, WPt triangles, IPt texture_coords, cv::Mat texture_img);
};

