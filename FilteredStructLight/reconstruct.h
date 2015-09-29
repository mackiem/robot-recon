#pragma once
#include <QtCore>
#include "FlyCapture2.h"
#include <unordered_map>
#include "cameradisplaywidget.h"
#include <chrono>
#include "fsl_common.h"

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
	Q_OBJECT;

private:
	int no_of_cams_;
	bool calibration_loaded_;
	bool started_capture_;

	struct ImageSet {
		int cam_no;
		std::vector<cv::Mat> images;
	};

	CameraImgMap camera_img_map_;

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

	void run_calibration(std::vector<std::pair<int, int>> camera_pairs);

	void run_reconstruction(std::vector<std::pair<int, int>> camera_pairs);

	void stereo_calibrate(CameraImgMap& camera_img_map, int left_cam, int right_cam,
	                      Size boardSize, bool useCalibrated, bool write_images = true);

	void recalibrate(std::vector<std::pair<int, int>> camera_pairs);
	
	void reconstruct();
	void alter_img_for_projection(const cv::Mat& img, cv::Mat& remapped_img, bool is_right) const;
	void fill_row(const cv::Mat& P, double coord, cv::Mat& fill_matrix, cv::Mat& B, bool is_y) const;

		void load_calibration();

	void create_rectification_map();

	void pick_correlated_points(std::vector<UniqueEdges>& unique_colors_left,
		std::vector<UniqueEdges>& unique_colors_right,
		std::vector <cv::Vec2f>& img_pts1, std::vector <cv::Vec2f>& img_pts, CameraImgMap& camera_img_map, int left_cam, int right_cam);

	void fit_gaussians(CameraImgMap& camera_img_map, int cam_no, std::unordered_map<int, std::pair<int, int>> mid_points);

	void correpond_with_gaussians(CameraImgMap& camera_img_map, int left_cam_no, int right_cam_no,
		std::vector <cv::Vec2f>& img_pts1, std::vector <cv::Vec2f>& img_pts2);

	void compute_correlation_using_gaussian(std::vector <cv::Vec2f>& img_pts1, std::vector <cv::Vec2f>& img_pts2,
		CameraImgMap& camera_img_map, std::vector<std::pair<int, int>> camera_pairs);

	void get_unique_edges(CameraImgMap& camera_img_map, int cam, std::vector<UniqueEdges> & unique_colors_per_image, bool is_right);
	
	void compute_correlation(std::vector <cv::Vec2f>& img_pts1, std::vector <cv::Vec2f>& img_pts2,
		CameraImgMap& camera_img_map, std::vector<std::pair<int, int>> camera_pairs);

	void init_imgs(CameraImgMap& camera_img_map, int cam, bool is_right);
	

	void read_file(const std::string& file_name, std::vector <cv::Vec2f>& img_pts1, std::vector <cv::Vec2f>& img_pts2);
	void write_file(const std::string& file_name, const std::vector <cv::Vec2f>& img_pts1, const std::vector <cv::Vec2f>& img_pts2);

	void triangulate_pts(const WPts& pnts, WPts& triangles, 
		std::vector<cv::Vec2f>& texture_coords, cv::Mat& remapped_img);
	void gen_texture(GLuint& texture_id, cv::Mat& remapped_img_for_texture) const;
	
	
	void resize_img(cv::Mat& img, const unsigned int resize_width) const;

	void recon_obj(const std::vector <cv::Vec2f>& img_pts1, const std::vector <cv::Vec2f>& img_pts2, std::vector<cv::Vec3f>& world_pts);

	//void gen_texture(GLuint& texture_id, cv::Mat& remapped_img_for_texture) const;


	Reconstruct3D(int no_of_cams, QObject* parent);
	~Reconstruct3D();

public slots:
	void collect_images(FlyCapture2::Image img, int cam_no);
	void collect_images_without_delay(FlyCapture2::Image img, int cam_no);
	void compute_correspondence(FlyCapture2::Image img, int cam_no);

signals:
	void finished_reconstruction(WPts world_pts);
};

