#ifndef FILTEREDSTRUCTLIGHT_H
#define FILTEREDSTRUCTLIGHT_H

#include "cameradisplaywidget.h"
#include <QtWidgets/QMainWindow>
#include <QtWidgets>
#include "camthread.h"
#include "reconstruct.h"

#define CAM_CALIB_PAIRS 6
#include "modelviewer.h"

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
	void shutdown_cam_thread();
	void create_camera_pairs(CameraPairs& pairs);

	void add_reconstruction_tab(CameraPairs& camera_pairs, QTabWidget* tab_widget);
	void add_camera_info_tab(QTabWidget* tab_widget, std::vector<unsigned>& camera_uuids);
};

#endif // FILTEREDSTRUCTLIGHT_H
