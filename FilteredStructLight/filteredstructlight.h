#ifndef FILTEREDSTRUCTLIGHT_H
#define FILTEREDSTRUCTLIGHT_H

#include "cameradisplaywidget.h"
#include <QtWidgets/QMainWindow>
#include <QtWidgets>
#include "camthread.h"
#include "reconstruct.h"
#include "robotreconstruction.h"

#define CAM_CALIB_PAIRS 6
#include "modelviewer.h"
#include "imageviewer.h"
#include "robotviewer.h"


class QArrayPushButton : public QPushButton {
	Q_OBJECT

protected:
	int id_;
	
public:
	QArrayPushButton(QString& text, int id, QWidget* parent = 0);
	int get_id() const;

private slots:
	void intercept_clicked();
signals:
	void clicked_with_id(int id);

};

class FilteredStructLight : public QMainWindow
{
	Q_OBJECT

public:
	FilteredStructLight(QWidget *parent = 0);
	~FilteredStructLight();
	void setupUi();

protected:
	void keyReleaseEvent(QKeyEvent* e);
	void onClose(QKeyEvent* e);

private:
	// labels
	static const char* RECONSTRUCTION_VIDEO_FILENAME_LABEL;
	static const char* CHECKERBOARD_VIDEO_FILENAME_LABEL;
	static const char* SCANLINE_VIDEO_FILENAME_LABEL;
	static const char* VELOCITY_LABEL;
	static const int MAX_VIDEO_NO;

	QString settings_filepath_;

	CamThread* cam_thread_;
	Reconstruct3D* reconstructor_;

	RobotReconstruction* robot_reconstruction_;

	QWidget* central_widget_;

	QWidget* left_panel_;
	QPushButton* view_cameras_;

	QWidget* right_panel_;
	GLWidget* opengl_widget_;

	QSlider* threshold_slider_;
	QCheckBox* threshold_toggle_checkbox_;

	QGroupBox* camera_pairs_group_;
	QSpinBox* camera_pair_[CAM_CALIB_PAIRS];

	QGroupBox* calibration_group_;
	QPushButton* start_calibration_video_;
	QPushButton* end_calibration_video_;

	QGroupBox* reconstruction_group_;
	QPushButton* load_camera_calibration_;
	QPushButton* start_reconstruction_video_;
	QPushButton* end_reconstruction_video_;
	QPushButton* recalibrate_button;
	QWidget* reconstruction_tab_;
	ModelViewer* model_viewer_;
	QWidget* camera_tab_;
	QPushButton* re_reconstruction_button;
	QWidget* camera_info_tab_;
	QLabel** camera_uuid_labels_;
	QGroupBox* camera_uuids_group_box_;
	QPushButton* reset_orientation_button_;
	QSpinBox* recon_no_of_images_spin_box_;

	QWidget* robot_calibration_tab_;
	ImageViewer* image_viewer_;
	QPushButton* calibrate_intrinsic_with_video_;
	QPushButton* calibrate_extrinsic_with_video_;
	QPushButton* find_line_extrinsic_with_video_;
	QPushButton* find_line_with_video_;
	QPushButton* interpolate_line_;
	QWidget* robot_viewer_tab_;
	RobotViewer* robot_viewer_;
	
	QLineEdit* velocity_line_edit_;
	QLineEdit* reconstruction_video_filename_;
	QPushButton* browse_button_;
	QLineEdit* nframe_line_edit_;
	QCheckBox* draw_planes_check_box_;
	QCheckBox* draw_points_check_box_;
	QCheckBox* draw_lines_check_box_;
	QCheckBox* draw_default_check_box_;
	QString selected_filename_;

	// frame by frame
	QLabel* large_image_preview_;
	QSpinBox* frame_selection_spin_box_;
	QCheckBox* view_frame_by_frame_;
	QCheckBox* display_large_frame_;
	QLabel* image_preview_;
	QWidget* large_preview_widget_;


	QLineEdit** calibration_video_filename_;
	QArrayPushButton** calibration_video_browse_;

	QLineEdit** scanline_video_filename_;
	QArrayPushButton** scanline_video_browse_;

	void load_settings();

	void shutdown_cam_thread();

	void create_camera_pairs(CameraPairs& pairs);

	void add_reconstruction_tab(CameraPairs& camera_pairs, QTabWidget* tab_widget);
	void add_reconstruction_options(QGroupBox* recon_options_group_box);
	void connect_line_edits_to_save_settings();
	void add_display_options(QGroupBox* display_options_group_box);
	void add_frame_analysis_options(QGroupBox* frame_analysis_group_box);
	void add_robot_viewer_tab(QTabWidget* tab_widget);
	void add_camera_info_tab(QTabWidget* tab_widget, std::vector<unsigned>& camera_uuids);

	void add_calibration_options(QGroupBox* calibration_group_box);

	void add_camera_calibration_tab(QTabWidget* tab_widget);
	void add_robot_calibration_tab(QTabWidget* tab_widget);

	std::vector<std::string> frame_filenames_;

private slots:
	void update_images(int frame_no);
	void save_settings();

public slots:
	void start_reconstruction_sequence();
	void handle_frame_filenames(std::vector<std::string> image_list);

};

#endif // FILTEREDSTRUCTLIGHT_H
