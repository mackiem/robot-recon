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
	static const int MAX_FORMATION_NO;
	static const int MAX_VIDEO_NO;

	static const char* SWARM_CONFIG_FILENAME_LABEL;
	static const char* OPT_CONFIG_FILENAME_LABEL;

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
	//QPushButton* run_least_squared_optimization_button_;
	QPushButton* run_mcmc_optimization_button_;
	QDoubleSpinBox* magic_k_spin_box_;
	QCheckBox* should_render_check_box_;
	QCheckBox* slow_down_check_box_;
	QLabel* avg_simultaneous_sampling_label_;
	//QPushButton* run_brute_force_optimization_button_;
	QSpinBox* neighborhood_count_;
	QDoubleSpinBox* obstacle_avoidance_near_range_min_;
	QDoubleSpinBox* obstacle_avoidance_near_range_max_;
	QDoubleSpinBox* obstacle_avoidance_far_range_min_;
	QDoubleSpinBox* obstacle_avoidance_far_range_max_;
	QCheckBox* collide_with_other_robots_;
	QDoubleSpinBox* square_radius_;
	QDoubleSpinBox* bounce_function_power_;
	QDoubleSpinBox* bounce_function_multiplier_;
	QLabel* coverage_label_;
	QLabel* occlusion_label_;


	QSpinBox* max_time_taken_;
	QSpinBox* no_of_clusters_;

	QDoubleSpinBox* death_percentage_;
	QSpinBox* death_time_taken_;

	// optimization
	QPushButton* add_swarm_configuration_button_;
	QListView* swarm_configs_for_optimization_list_;
	QSpinBox* no_of_threads_spin_box_;
	QSpinBox* no_of_iterations_spin_box_;
	QSpinBox* culling_nth_iteration_spin_box_;
	QLineEdit* optimization_config_filename_;
	QPushButton* optimization_config_filename_browse_;
	QPushButton* load_optimization_config_button_;
	QPushButton* save_optimization_config_button_;
	QStringListModel* swarm_configs_list_model_;
	QPushButton* remove_swarm_configuration_button_;
	QPushButton* batch_optimize_button_;
	QLabel* multi_sampling_label_;
	QLabel* clustering_label_;
	QLabel* opt_score_label_;

	QLineEdit* model_matrix_filename_;
	QPushButton* model_matrix_filename_browse_;
	QPushButton* create_model_matrix_;
	QLabel* time_step_count_score_label_;
	QDoubleSpinBox* time_step_count_score_textbox_;
	QLabel* avg_simultaneous_sampling_score_label_;
	QDoubleSpinBox* simultaneous_sampling_score_textbox_;
	QLabel* multi_sampling_score_label_;
	QDoubleSpinBox* multi_sampling_score_textbox_;
	QLabel* coverage_score_label_;
	QDoubleSpinBox* coverage_score_textbox_;
	QLabel* occlusion_score_label_;
	QDoubleSpinBox* occlusion_score_textbox_;
	QLabel* clustering_score_label_;
	QDoubleSpinBox* clustering_score_textbox_;
	QDoubleSpinBox* coverage_needed_;
	QCheckBox* trail_mode_;
	QCheckBox* display_local_map_mode_;
	QCheckBox* display_astar_path_mode_;
	QSpinBox* local_map_robot_id_spinbox_;
	QDoubleSpinBox* desired_sampling_;
	QCheckBox* record_video_mode_;
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

	void save_optimization_settings(const QString& filename);
	void set_opt_params_to_ui(const OptimizationParams& opt_params);
	void load_optimization_settings(const QString& filename);

private slots:
	void update_images(int frame_no);
	void save_recon_settings();
	SwarmParams get_swarm_params_from_ui();
	void set_swarm_params_to_ui(const SwarmParams& swarm_params);
	void save_swarm_settings(QString swarm_conf_filepath);
	void load_swarm_settings(QString swarm_conf_filepath);
	OptimizationParams get_opt_params_from_ui();
	void load_swarm_config_settings();
	void save_swarm_config_settings();
public slots:
	void start_reconstruction_sequence();
	void handle_frame_filenames(std::vector<std::string> image_list);
	void update_time_step_count(int count);
	void update_simul_sampling(double sampling);
	void update_sim_results(OptimizationResults results);
};

#endif // FILTEREDSTRUCTLIGHT_H
