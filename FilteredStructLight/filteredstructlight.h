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
	QLineEdit* video_filename_;
	QPushButton* browse_button_;
	QLineEdit* nframe_line_edit_;
	QCheckBox* draw_planes_check_box_;
	QCheckBox* draw_points_check_box_;
	QCheckBox* draw_lines_check_box_;
	QCheckBox* draw_default_check_box_;
	void shutdown_cam_thread();
	void create_camera_pairs(CameraPairs& pairs);

	void add_reconstruction_tab(CameraPairs& camera_pairs, QTabWidget* tab_widget);
	void add_reconstruction_options(QGroupBox* recon_options_group_box);
	void add_display_options(QGroupBox* display_options_group_box);
	void add_robot_viewer_tab(QTabWidget* tab_widget);
	void add_camera_info_tab(QTabWidget* tab_widget, std::vector<unsigned>& camera_uuids);

	void add_camera_calibration_tab(QTabWidget* tab_widget);
	void add_robot_calibration_tab(QTabWidget* tab_widget);

};

#endif // FILTEREDSTRUCTLIGHT_H
