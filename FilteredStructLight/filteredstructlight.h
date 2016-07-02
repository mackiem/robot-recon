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
#include "swarmviewer.h"


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

class QArrayRadioButton : public QRadioButton {
	Q_OBJECT

protected:
	int id_;
	
public:
	QArrayRadioButton(QString& text, int id, QWidget* parent = 0);
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
	virtual ~FilteredStructLight();
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
	static const char* ROBOTS_NO_LABEL;
	static const char* EXPLORATION_CONSTANT_LABEL;
	static const char* SEPARATION_CONSTANT_LABEL;
	static const char* ALIGNMENT_CONSTANT_LABEL;
	static const char* CLUSTER_CONSTANT_LABEL;
	static const char* PERIMETER_CONSTANT_LABEL;


	static const char* EXPLORE_RANGE_MIN_LABEL;
	static const char* SEPARATION_RANGE_MIN_LABEL;
	static const char* ALIGNMENT_RANGE_MIN_LABEL;
	static const char* CLUSTER_RANGE_MIN_LABEL;
	static const char* PERIMETER_RANGE_MIN_LABEL;
	static const char* OBSTACLE_AVOIDANCE_NEAR_RANGE_MIN_LABEL;
	static const char* OBSTACLE_AVOIDANCE_NEAR_RANGE_MAX_LABEL;
	static const char* EXPLORE_RANGE_MAX_LABEL;
	static const char* SEPARATION_RANGE_MAX_LABEL;
	static const char* ALIGNMENT_RANGE_MAX_LABEL;
	static const char* CLUSTER_RANGE_MAX_LABEL;
	static const char* PERIMETER_RANGE_MAX_LABEL;
	static const char* OBSTACLE_AVOIDANCE_FAR_RANGE_MIN_LABEL;
	static const char* OBSTACLE_AVOIDANCE_FAR_RANGE_MAX_LABEL;
	static const char* GOTO_WORK_CONSTANT_LABEL;
	static const char* SEPARATION_DISTANCE_LABEL;
	static const char* FORMATION_LABEL;
	static const char* SENSOR_RANGE_LABEL;
	static const char* DISCOVERY_RANGE_LABEL;
	static const char* SHOW_FORCES_LABEL;
	static const char* COLLIDE_WITH_OTHER_ROBOTS_LABEL;
	static const char* MAGICK_LABEL;
	static const char* NEIGHBORHOOD_COUNT_LABEL;
	static const char* GRID_RESOLUTION_LABEL;
	static const char* GRID_LENGTH_LABEL;
	static const char* BUILDING_INTERIOR_SCALE_LABEL;
	static const char* BUILDING_OFFSET_X_LABEL;
	static const char* BUILDING_OFFSET_Y_LABEL;
	static const char* BUILDING_OFFSET_Z_LABEL;
	static const char* SHOW_BUILDING_LABEL;
	static const char* INTERIOR_MODEL_FILENAME;
	static const char* SWARM_CONFIG_FILENAME_LABEL;
	static const char* SQUARE_RADIUS;
	static const char* BOUNCE_FUNCTION_POWER;
	static const char* BOUNCE_FUNCTION_MULTIPLIER;
	static const int MAX_FORMATION_NO;
	static const int MAX_VIDEO_NO;

	QString recon_settings_filepath_;

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
	SwarmViewer* swarm_viewer_;
	
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

	// swarm settings
	QString swarm_settings_filepath_;
	QSpinBox* robots_spinbox_;
	QDoubleSpinBox* exploration_constant_;
	QDoubleSpinBox* separation_constant_;
	QDoubleSpinBox* cluster_constant_;
	QDoubleSpinBox* perimeter_constant_;
	QDoubleSpinBox* goto_work_constant_;
	QDoubleSpinBox* scale_spinbox_;
	QSpinBox* x_spin_box_;
	QSpinBox* y_spin_box_;
	QSpinBox* z_spin_box_;
	QCheckBox* show_interior_;
	QSpinBox* grid_resolution_spin_box_;
	QSpinBox* grid_length_spin_box_;
	QPushButton* swarm_reset_button_;
	QDoubleSpinBox* separation_distance_;
	QArrayRadioButton** formation_buttons_;

	QCheckBox* show_forces_;
	QLineEdit* model_filename_;
	QPushButton* model_filename_browse_;
	QLineEdit* swarm_config_filename_;
	QPushButton* swarm_config_filename_browse_;
	QPushButton* load_swarm_config_button_;
	QPushButton* save_swarm_config_button_;
	QDoubleSpinBox* alignment_constant_;

	QDoubleSpinBox* sensor_range_;

	QDoubleSpinBox* separation_range_min_;
	QDoubleSpinBox* separation_range_max_;
	QDoubleSpinBox* alignment_range_min_;
	QDoubleSpinBox* alignment_range_max_;
	QDoubleSpinBox* cluster_range_min_;
	QDoubleSpinBox* cluster_range_max_;
	QDoubleSpinBox* perimeter_range_min_;
	QDoubleSpinBox* perimeter_range_max_;
	QDoubleSpinBox* explore_range_min_;
	QDoubleSpinBox* explore_range_max_;
	QPushButton* swarm_pause_button_;
	QPushButton* swarm_step_button_;
	QPushButton* swarm_resume_button_;
	QLabel* time_step_count_label_;
	QSpinBox* discovery_range_;
	QPushButton* run_least_squared_optimization_button_;
	QPushButton* run_mcmc_optimization_button_;
	QDoubleSpinBox* magic_k_spin_box_;
	QCheckBox* should_render_check_box_;
	QCheckBox* slow_down_check_box_;
	QLabel* avg_simultaneous_sampling_label_;
	QPushButton* run_brute_force_optimization_button_;
	QSpinBox* neighborhood_count_;
	QDoubleSpinBox* obstacle_avoidance_near_range_min_;
	QDoubleSpinBox* obstacle_avoidance_near_range_max_;
	QDoubleSpinBox* obstacle_avoidance_far_range_min_;
	QDoubleSpinBox* obstacle_avoidance_far_range_max_;
	QCheckBox* collide_with_other_robots_;
	QDoubleSpinBox* square_radius_;
	QDoubleSpinBox* bounce_function_power_;
	QDoubleSpinBox* bounce_function_multiplier_;
	void load_recon_settings();

	void shutdown_cam_thread();

	void create_camera_pairs(CameraPairs& pairs);

	void add_reconstruction_tab(CameraPairs& camera_pairs, QTabWidget* tab_widget);
	void add_reconstruction_options(QGroupBox* recon_options_group_box);
	void connect_recon_line_edits_to_save_settings();
	void add_display_options(QGroupBox* display_options_group_box);
	void add_frame_analysis_options(QGroupBox* frame_analysis_group_box);
	void add_robot_viewer_tab(QTabWidget* tab_widget);
	void add_camera_info_tab(QTabWidget* tab_widget, std::vector<unsigned>& camera_uuids);
	void add_interior_options(QGroupBox* group_box);
	void add_robot_options(QGroupBox* group_box);
	void add_misc_options(QGroupBox* group_box);
	void add_grid_options(QGroupBox* group_box);
	void add_swarm_optimization_options(QGroupBox* group_box);
	void add_swarm_config_save_options(QGroupBox* group_box);
	void add_swarm_sim_flow_control_options(QGroupBox* group_box);
	void add_swarm_sim_tab(QTabWidget* tab_widget);
	void connect_widgets_to_swarm_save_settings();

	void add_calibration_options(QGroupBox* calibration_group_box);

	void add_camera_calibration_tab(QTabWidget* tab_widget);
	void add_robot_calibration_tab(QTabWidget* tab_widget);

	std::vector<std::string> frame_filenames_;
	void connect_load_filename_to_save_settings();

private slots:
	void update_images(int frame_no);
	void save_recon_settings();
	void save_swarm_settings(QString swarm_conf_filepath);
	void load_swarm_settings(QString swarm_conf_filepath);
	void load_swarm_config_settings();
	void save_swarm_config_settings();
public slots:
	void start_reconstruction_sequence();
	void handle_frame_filenames(std::vector<std::string> image_list);
	void update_time_step_count(int count);
	void update_sampling(double sampling);
};

#endif // FILTEREDSTRUCTLIGHT_H
