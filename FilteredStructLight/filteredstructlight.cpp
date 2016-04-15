#include "filteredstructlight.h"
#include <QGLFormat>
#include "modelviewer.h"
#include "robotviewer.h"

const int FilteredStructLight::MAX_VIDEO_NO = 5;
const char* FilteredStructLight::RECONSTRUCTION_VIDEO_FILENAME_LABEL = "reconstruction_video_filename";
const char* FilteredStructLight::CHECKERBOARD_VIDEO_FILENAME_LABEL = "checkerboard_video_filename";
const char* FilteredStructLight::SCANLINE_VIDEO_FILENAME_LABEL = "scanline_video_filename";
const char* FilteredStructLight::VELOCITY_LABEL = "velocity";


const char* FilteredStructLight::ROBOTS_NO_LABEL = "robots_no";
const char* FilteredStructLight::EXPLORATION_CONSTANT_LABEL = "exploration_constant";
const char* FilteredStructLight::SEPERATION_CONSTANT_LABEL = "seperation_constant";
const char* FilteredStructLight::GOTO_WORK_CONSTANT_LABEL = "goto_work_constant";
const char* FilteredStructLight::SEPARATION_DISTANCE_LABEL = "separation_distance";
const char* FilteredStructLight::SHOW_FORCES_LABEL = "show_forces";

const char* FilteredStructLight::GRID_RESOLUTION_LABEL = "grid_resolution";
const char* FilteredStructLight::GRID_LENGTH_LABEL = "grid_length";
const char* FilteredStructLight::BUILDING_INTERIOR_SCALE_LABEL = "interior_scale";
const char* FilteredStructLight::BUILDING_OFFSET_X_LABEL = "interior_offset_x";
const char* FilteredStructLight::BUILDING_OFFSET_Y_LABEL = "interior_offset_y";
const char* FilteredStructLight::BUILDING_OFFSET_Z_LABEL = "interior_offset_z";
const char* FilteredStructLight::SHOW_BUILDING_LABEL = "show_interior";
const char* FilteredStructLight::INTERIOR_MODEL_FILENAME = "interior_model_filename";


FilteredStructLight::FilteredStructLight(QWidget *parent)
	: QMainWindow(parent)
{
	//ui.setupUi(this);
	//recon_settings_filepath_ = ":/FilteredStructLight/settings.ini";
	recon_settings_filepath_ = QDir::currentPath() + "/settings.ini";
	swarm_settings_filepath_ = QDir::currentPath() + "/swarm_settings.ini";
	setupUi();
}

FilteredStructLight::~FilteredStructLight()
{
}

void FilteredStructLight::load_recon_settings() {
	QSettings settings(recon_settings_filepath_, QSettings::IniFormat);
	
	reconstruction_video_filename_->setText(settings.value(RECONSTRUCTION_VIDEO_FILENAME_LABEL, "").toString());
	std::string calibration_checkerboard_video_filename = CHECKERBOARD_VIDEO_FILENAME_LABEL;
	std::string calibration_scanline_video_filename = SCANLINE_VIDEO_FILENAME_LABEL;

	for (int i = 0; i < MAX_VIDEO_NO; ++i) {
		std::stringstream checkerboard_ss;
		checkerboard_ss << calibration_checkerboard_video_filename << "_" << (i + 1);

		std::stringstream scanline_ss;
		scanline_ss << calibration_scanline_video_filename << "_" << (i + 1);
		calibration_video_filename_[i]->setText(settings.value(QString(checkerboard_ss.str().c_str()), "").toString());
		scanline_video_filename_[i]->setText(settings.value(QString(scanline_ss.str().c_str()), "").toString());
	}

	velocity_line_edit_->setText(settings.value(VELOCITY_LABEL, "").toString());
}

void FilteredStructLight::save_recon_settings() {
	QSettings settings(recon_settings_filepath_, QSettings::IniFormat);

	settings.setValue(RECONSTRUCTION_VIDEO_FILENAME_LABEL, reconstruction_video_filename_->text());

	for (int i = 0; i < MAX_VIDEO_NO; ++i) {
		std::stringstream checkerboard_ss;
		checkerboard_ss << CHECKERBOARD_VIDEO_FILENAME_LABEL << "_" << (i + 1);

		std::stringstream scanline_ss;
		scanline_ss << SCANLINE_VIDEO_FILENAME_LABEL << "_" << (i + 1);

		settings.setValue(QString(checkerboard_ss.str().c_str()), calibration_video_filename_[i]->text());
		settings.setValue(QString(scanline_ss.str().c_str()), scanline_video_filename_[i]->text());
	}

	settings.setValue(VELOCITY_LABEL, velocity_line_edit_->text());
}

void FilteredStructLight::save_swarm_settings() {
	QSettings settings(swarm_settings_filepath_, QSettings::IniFormat);

	settings.setValue(ROBOTS_NO_LABEL, robots_spinbox_->value());
	settings.setValue(EXPLORATION_CONSTANT_LABEL, exploration_constant_->value());
	settings.setValue(SEPERATION_CONSTANT_LABEL, separation_constant_->value());
	settings.setValue(GOTO_WORK_CONSTANT_LABEL, goto_work_constant_->value());

	settings.setValue(SEPARATION_DISTANCE_LABEL, separation_distance_->value());

	settings.setValue(GRID_RESOLUTION_LABEL, grid_resolution_spin_box_->value());
	settings.setValue(GRID_LENGTH_LABEL, grid_length_spin_box_->value());

	settings.setValue(BUILDING_INTERIOR_SCALE_LABEL, scale_spinbox_->value());
	settings.setValue(BUILDING_OFFSET_X_LABEL, x_spin_box_->value());
	settings.setValue(BUILDING_OFFSET_Y_LABEL, y_spin_box_->value());
	settings.setValue(BUILDING_OFFSET_Z_LABEL, z_spin_box_->value());
	settings.setValue(SHOW_BUILDING_LABEL, (show_interior_->isChecked()));
	settings.setValue(INTERIOR_MODEL_FILENAME, model_filename_->text());

	settings.setValue(SHOW_FORCES_LABEL, (show_forces_->isChecked()));
}


void FilteredStructLight::load_swarm_settings() {
	QSettings settings(swarm_settings_filepath_, QSettings::IniFormat);
	
	robots_spinbox_->setValue(settings.value(ROBOTS_NO_LABEL, "10").toInt());
	emit robots_spinbox_->valueChanged(robots_spinbox_->value());
	exploration_constant_->setValue(settings.value(EXPLORATION_CONSTANT_LABEL, "1").toDouble());
	separation_constant_->setValue(settings.value(SEPERATION_CONSTANT_LABEL, "1").toDouble());
	goto_work_constant_->setValue(settings.value(GOTO_WORK_CONSTANT_LABEL, "1").toDouble());
	separation_distance_->setValue(settings.value(SEPARATION_DISTANCE_LABEL, "100").toDouble());
	emit separation_distance_->valueChanged(separation_distance_->value());

	grid_resolution_spin_box_->setValue(settings.value(GRID_RESOLUTION_LABEL, "4096").toInt());
	grid_length_spin_box_->setValue(settings.value(GRID_LENGTH_LABEL, "20").toInt());
	emit grid_length_spin_box_->valueChanged(grid_length_spin_box_->value());

	model_filename_->setText(settings.value(INTERIOR_MODEL_FILENAME, "interior/house interior.obj").toString());
	emit model_filename_->textChanged(model_filename_->text());


	scale_spinbox_->setValue(settings.value(BUILDING_INTERIOR_SCALE_LABEL, "2").toDouble());
	x_spin_box_->setValue(settings.value(BUILDING_OFFSET_X_LABEL, "0").toInt());
	y_spin_box_->setValue(settings.value(BUILDING_OFFSET_Y_LABEL, "0").toInt());
	z_spin_box_->setValue(settings.value(BUILDING_OFFSET_Z_LABEL, "0").toInt());

	Qt::CheckState show_interior = settings.value(SHOW_BUILDING_LABEL, "1").toBool() ? Qt::CheckState::Checked : Qt::CheckState::Unchecked ;
	show_interior_->setCheckState(show_interior);
	emit show_interior_->stateChanged(show_interior);

	Qt::CheckState show_forces = settings.value(SHOW_FORCES_LABEL, "1").toBool() ? Qt::CheckState::Checked : Qt::CheckState::Unchecked ;
	show_forces_->setCheckState(show_forces);
	emit show_forces_->stateChanged(show_forces);

}

void FilteredStructLight::shutdown_cam_thread() {
	cam_thread_->shutdown();
	cam_thread_->wait();
}

void FilteredStructLight::create_camera_pairs(CameraPairs& pairs)
{
	for (int i = 0; i < CAM_CALIB_PAIRS / 2; ++i)
	{
		pairs.push_back(std::pair<int, int>(camera_pair_[2*i]->value(), 
		                                    camera_pair_[2*i + 1]->value()));
	}
}

void FilteredStructLight::add_reconstruction_tab(CameraPairs& camera_pairs, QTabWidget* tab_widget) {
	reconstruction_tab_ = new QWidget();
	QHBoxLayout* reconstruction_layout_ = new QHBoxLayout();
	// Specify an OpenGL 3.3 format using the Core profile.
	// That is, no old-school fixed pipeline functionality
	QGLFormat glFormat;
	glFormat.setVersion(3, 3);
	glFormat.setProfile(QGLFormat::CoreProfile); // Requires >=Qt-4.8.0
	glFormat.setSampleBuffers(true);
	//glFormat.setSwapInterval(1);

	QWidget* reconstruct_left_panel_ = new QWidget(reconstruction_tab_);
	QVBoxLayout* reconstruct_left_panel_layout = new QVBoxLayout();

	// camera pairs
	QGroupBox* camera_pairs_group_box = new QGroupBox("Camera Pair",reconstruction_tab_);
	QVBoxLayout* camera_pairs_box_layout = new QVBoxLayout();
	std::string camera_pair_prefix_string = "Camera Pair ";
	for (auto& camera_pair : camera_pairs) {
		std::string camera_pair_string = camera_pair_prefix_string + std::to_string(camera_pair.first) +
			std::string(" ") + std::to_string(camera_pair.second);
		QString camera_pair_qstring(camera_pair_string.c_str());
		QRadioButton* radio_button = new QRadioButton(camera_pair_qstring, camera_pairs_group_box);	
		camera_pairs_box_layout->addWidget(radio_button);
	}
	camera_pairs_group_box->setLayout(camera_pairs_box_layout);
	camera_pairs_group_box->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Minimum);


	// draw triangles or points
	QGroupBox* draw_method_group_box = new QGroupBox("Draw Method", reconstruction_tab_);
	QVBoxLayout* draw_method_box_layout = new QVBoxLayout();
	QRadioButton* draw_points_radio_button = new QRadioButton("Points", draw_method_group_box);	
	draw_method_box_layout->addWidget(draw_points_radio_button);
	QRadioButton* draw_triangles_radio_button = new QRadioButton("Triangles", draw_method_group_box);	
	draw_method_box_layout->addWidget(draw_triangles_radio_button);
	draw_method_group_box->setLayout(draw_method_box_layout);

	// use texture or solid colors
	QGroupBox* color_method_group_box = new QGroupBox("Color Method", reconstruction_tab_);
	QVBoxLayout* color_method_box_layout = new QVBoxLayout(color_method_group_box);
	QRadioButton* color_texture_radio_button = new QRadioButton("Texture", color_method_group_box);	
	color_method_box_layout->addWidget(color_texture_radio_button);
	QRadioButton* color_solid_color_radio_button = new QRadioButton("Solid Color", color_method_group_box);	
	color_method_box_layout->addWidget(color_solid_color_radio_button);
	color_method_group_box->setLayout(color_method_box_layout);

	QGroupBox* misc_controls_group_box = new QGroupBox("Misc. Controls", reconstruction_tab_);
	reset_orientation_button_ = new QPushButton("Reset Controls", misc_controls_group_box);
	QLabel* scale_label = new QLabel("Scale", misc_controls_group_box);
	QSlider* scale_slider = new QSlider(Qt::Orientation::Horizontal, misc_controls_group_box);
	scale_slider->setMaximum(100);
	scale_slider->setMinimum(1);
	scale_slider->setFocusPolicy(Qt::StrongFocus);
	scale_slider->setTickInterval(10);
	scale_slider->setSingleStep(1);
	scale_slider->setTickPosition(QSlider::TicksBothSides);

	QVBoxLayout* misc_control_box_layout = new QVBoxLayout(misc_controls_group_box);
	misc_control_box_layout->addWidget(reset_orientation_button_);
	misc_control_box_layout->addWidget(scale_label);
	misc_control_box_layout->addWidget(scale_slider);
	misc_controls_group_box->setLayout(misc_control_box_layout);


	reconstruct_left_panel_layout->addWidget(camera_pairs_group_box);
	reconstruct_left_panel_layout->addWidget(draw_method_group_box);
	reconstruct_left_panel_layout->addWidget(color_method_group_box);
	reconstruct_left_panel_layout->addWidget(misc_controls_group_box);

	reconstruct_left_panel_->setLayout(reconstruct_left_panel_layout);

	reconstruction_layout_->addWidget(reconstruct_left_panel_);

	model_viewer_ = new ModelViewer(glFormat, reconstruction_tab_);
	reconstruction_layout_->addWidget(model_viewer_);

	reconstruction_tab_->setLayout(reconstruction_layout_);

	tab_widget->addTab(reconstruction_tab_, "Reconstruction");

	connect(draw_points_radio_button, &QRadioButton::clicked, model_viewer_, &ModelViewer::draw_points);
	connect(draw_triangles_radio_button, &QRadioButton::clicked, model_viewer_, &ModelViewer::draw_triangles);
	connect(color_texture_radio_button, &QRadioButton::clicked, model_viewer_, &ModelViewer::draw_texture);
	connect(color_solid_color_radio_button, &QRadioButton::clicked, model_viewer_, &ModelViewer::draw_colors);
	connect(reset_orientation_button_, &QPushButton::clicked, model_viewer_, &ModelViewer::reset_view);
	connect(scale_slider, &QSlider::valueChanged, model_viewer_, &ModelViewer::set_scale);

	color_solid_color_radio_button->click();
	draw_points_radio_button->click();
	scale_slider->setValue(100);
}

void FilteredStructLight::add_reconstruction_options(QGroupBox* recon_options_group_box) {
	// reconstruction options start//

	QVBoxLayout* reconstruction_options_layout = new QVBoxLayout();

	QHBoxLayout* reconstruct_file_layout = new QHBoxLayout();
	QLabel* video_file_label = new QLabel("Video Filename:", recon_options_group_box);
	reconstruction_video_filename_ = new QLineEdit("", recon_options_group_box);
	browse_button_ = new QArrayPushButton(QString("..."), 0, recon_options_group_box);
	reconstruct_file_layout->addWidget(video_file_label);
	reconstruct_file_layout->addWidget(reconstruction_video_filename_);
	reconstruct_file_layout->addWidget(browse_button_);

	QHBoxLayout* velocity_layout = new QHBoxLayout();
	QLabel* velocity = new QLabel("Velocity", recon_options_group_box);
	velocity_line_edit_ = new QLineEdit("70", recon_options_group_box);
	velocity_line_edit_->setAlignment(Qt::AlignRight);
	velocity_layout->addWidget(velocity);
	velocity_layout->addWidget(velocity_line_edit_);

	QHBoxLayout* nth_frame_layout = new QHBoxLayout();
	QLabel* nframe_label = new QLabel("Nth Frame", recon_options_group_box);
	nframe_line_edit_ = new QLineEdit("1", recon_options_group_box);
	nframe_line_edit_->setAlignment(Qt::AlignRight);
	nth_frame_layout->addWidget(nframe_label);
	nth_frame_layout->addWidget(nframe_line_edit_);

	QPushButton* reconstruct_from_video_push_button = new QPushButton("Reconstruct from Video", recon_options_group_box);

	QPushButton* draw_points_push_button = new QPushButton("Clear Points", recon_options_group_box);	

	reconstruction_options_layout->addWidget(draw_points_push_button);
	reconstruction_options_layout->addLayout(reconstruct_file_layout);
	reconstruction_options_layout->addLayout(velocity_layout);
	reconstruction_options_layout->addLayout(nth_frame_layout);
	reconstruction_options_layout->addWidget(reconstruct_from_video_push_button);

	recon_options_group_box->setLayout(reconstruction_options_layout);
	recon_options_group_box->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Minimum);

	connect(draw_points_push_button, &QPushButton::clicked, this, 
		[&]()
	{
		robot_viewer_->clear_models();
	});

	
	connect(browse_button_, &QPushButton::clicked, this,
		[&]()
	{
		selected_filename_ = QFileDialog::getOpenFileName(this, QString("Open Video"), QDir::currentPath(),
			"Video Files(*.h264 *.avi)");
		reconstruction_video_filename_->setText(selected_filename_);
	});



	connect(reconstruct_from_video_push_button, &QPushButton::clicked,  this, 
		[&]()
	{
		if (reconstruction_video_filename_->text().compare(QString("")) == 0) {
			QMessageBox no_file_warning(QMessageBox::Icon::Warning, "No files selected!", "Please select a valid video file...",
				QMessageBox::Ok, this);
			no_file_warning.exec();
		} else {
			robot_reconstruction_->reconstruct_from_video(reconstruction_video_filename_->text().toStdString(),
				std::stoi(nframe_line_edit_->text().toStdString()),
				std::stof(velocity_line_edit_->text().toStdString()), cv::Vec3f(-1.f, 0.f, 0.f));
			
		}
	});

	
}

void FilteredStructLight::connect_recon_line_edits_to_save_settings() {
	connect(reconstruction_video_filename_, &QLineEdit::textChanged, this, &FilteredStructLight::save_recon_settings);
	connect(velocity_line_edit_, &QLineEdit::textChanged, this, &FilteredStructLight::save_recon_settings);

	for (int i = 0; i < (MAX_VIDEO_NO); ++i) {
		connect(calibration_video_filename_[i], &QLineEdit::textChanged, this, &FilteredStructLight::save_recon_settings);
		connect(scanline_video_filename_[i], &QLineEdit::textChanged, this, &FilteredStructLight::save_recon_settings);
	}
}

void FilteredStructLight::add_display_options(QGroupBox* display_options_group_box) {
	// checkboxes of options
	QVBoxLayout* display_options_layout = new QVBoxLayout();

	QHBoxLayout* draw_toggle_options_layout = new QHBoxLayout();
	QHBoxLayout* view_position_options_layout = new QHBoxLayout();

	//QLabel* draw_planes_label = new QLabel("Show Planes", display_options_group_box);
	draw_planes_check_box_ = new QCheckBox("Planes", display_options_group_box);
	draw_points_check_box_ = new QCheckBox("Points", display_options_group_box);
	draw_lines_check_box_ = new QCheckBox("Lines", display_options_group_box);
	draw_default_check_box_ = new QCheckBox("Camera Setup", display_options_group_box);

	draw_toggle_options_layout->addWidget(draw_planes_check_box_);
	draw_toggle_options_layout->addWidget(draw_points_check_box_);
	draw_toggle_options_layout->addWidget(draw_lines_check_box_);
	draw_toggle_options_layout->addWidget(draw_default_check_box_);

	QPushButton* reset_view = new QPushButton("Reset View", display_options_group_box);
	QPushButton* center_on_points_view = new QPushButton("Center on Pts.", display_options_group_box);

	view_position_options_layout->addWidget(reset_view);
	view_position_options_layout->addWidget(center_on_points_view);

	display_options_layout->addLayout(draw_toggle_options_layout);
	display_options_layout->addLayout(view_position_options_layout);

	display_options_group_box->setLayout(display_options_layout);

	connect(draw_points_check_box_, &QCheckBox::stateChanged, robot_viewer_, &RobotViewer::toggle_draw_points);
	connect(draw_planes_check_box_, &QCheckBox::stateChanged, robot_viewer_, &RobotViewer::toggle_draw_planes);
	connect(draw_lines_check_box_, &QCheckBox::stateChanged, robot_viewer_, &RobotViewer::toggle_draw_lines);
	connect(draw_default_check_box_, &QCheckBox::stateChanged, robot_viewer_, &RobotViewer::toggle_draw_default);

	connect(reset_view, &QPushButton::clicked, robot_viewer_, &RobotViewer::reset_view);
	connect(center_on_points_view, &QPushButton::clicked, robot_viewer_, &RobotViewer::center_view);

	draw_planes_check_box_->setChecked(true);
	draw_points_check_box_->setChecked(true);
	draw_lines_check_box_->setChecked(true);
	draw_default_check_box_->setChecked(true);
}


void FilteredStructLight::add_frame_analysis_options(QGroupBox* frame_analysis_group_box) {

	QVBoxLayout* frame_analysis_group_box_layout = new QVBoxLayout();

	QHBoxLayout* frame_selection_layout = new QHBoxLayout();
	QLabel* frame_no_label = new QLabel("Frame #");
	frame_selection_spin_box_ = new QSpinBox(frame_analysis_group_box);
	frame_selection_spin_box_->setMinimum(0);
	frame_selection_spin_box_->setMaximum(0);

	frame_selection_layout->addWidget(frame_no_label);
	frame_selection_layout->addWidget(frame_selection_spin_box_);

	view_frame_by_frame_ = new QCheckBox("Frame by Frame", frame_analysis_group_box);

	display_large_frame_ = new QCheckBox("Large Frame", frame_analysis_group_box);

	//view_all_frames_ = new QCheckBox("View All Frames", frame_analysis_group_box);

	image_preview_ = new QLabel();
	image_preview_->setFixedSize(320, 180);
	image_preview_->setBaseSize(320, 180);
	image_preview_->setScaledContents(true);
	image_preview_->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Fixed);

	frame_analysis_group_box_layout->addLayout(frame_selection_layout);
	frame_analysis_group_box_layout->addWidget(view_frame_by_frame_);
	frame_analysis_group_box_layout->addWidget(display_large_frame_);
	frame_analysis_group_box_layout->addWidget(image_preview_);



	frame_analysis_group_box->setLayout(frame_analysis_group_box_layout);

	connect(display_large_frame_, &QCheckBox::stateChanged, large_preview_widget_, &QWidget::setVisible);
	connect(view_frame_by_frame_, &QCheckBox::stateChanged, robot_viewer_, &RobotViewer::toggle_frame_by_frame);
	connect(frame_selection_spin_box_, static_cast<void (QSpinBox::*)(int)>(&QSpinBox::valueChanged),
		robot_viewer_, &RobotViewer::set_current_frame_to_draw);
	connect(frame_selection_spin_box_, static_cast<void (QSpinBox::*)(int)>(&QSpinBox::valueChanged),
		this, &FilteredStructLight::update_images);

	display_large_frame_->setChecked(false);
	emit display_large_frame_->stateChanged(false);
	view_frame_by_frame_->setChecked(true);
}

void FilteredStructLight::add_robot_viewer_tab(QTabWidget* tab_widget) {
	QHBoxLayout* robot_viewer_options_layout = new QHBoxLayout();

	robot_viewer_tab_ = new QWidget(tab_widget);

	// Specify an OpenGL 3.3 format using the Core profile.
	// That is, no old-school fixed pipeline functionality
	QGLFormat glFormat;
	glFormat.setVersion(3, 3);
	glFormat.setProfile(QGLFormat::CoreProfile); // Requires >=Qt-4.8.0
	glFormat.setSampleBuffers(true);
	//glFormat.setSwapInterval(1);
	robot_viewer_ = new RobotViewer(glFormat, central_widget_);
	QSizePolicy robot_viewer_size_policy;
	robot_viewer_size_policy.setHorizontalStretch(2);
	robot_viewer_size_policy.setHorizontalPolicy(QSizePolicy::Preferred);
	robot_viewer_size_policy.setVerticalPolicy(QSizePolicy::Preferred);
	robot_viewer_->setSizePolicy(robot_viewer_size_policy);

	large_preview_widget_ = new QWidget(robot_viewer_tab_);
	QVBoxLayout* large_preview_layout = new QVBoxLayout();

	large_image_preview_ = new QLabel(robot_viewer_tab_);
	large_image_preview_->setFixedSize(640, 360);
	large_image_preview_->setScaledContents(true);
	QSizePolicy size_policy;
	size_policy.setHorizontalStretch(1);
	size_policy.setHorizontalPolicy(QSizePolicy::Ignored);
	size_policy.setVerticalPolicy(QSizePolicy::Fixed);
	large_image_preview_->setSizePolicy(size_policy);

	QSpacerItem* spacer = new QSpacerItem(640, 360);

	large_preview_layout->addWidget(large_image_preview_);
	large_preview_layout->addSpacerItem(spacer);
	large_preview_widget_->setLayout(large_preview_layout);

	QScrollArea* left_panel_scroll_area = new QScrollArea(robot_viewer_tab_);
	left_panel_scroll_area->setWidgetResizable(true);

	QWidget* robot_viewer_left_panel_ = new QWidget(robot_viewer_tab_);
	robot_viewer_left_panel_->setSizePolicy(QSizePolicy::MinimumExpanding, QSizePolicy::MinimumExpanding);
	QVBoxLayout* robot_viewer_left_panel_layout = new QVBoxLayout();

	QGroupBox* recon_options_group_box = new QGroupBox("Recon. Options", robot_viewer_tab_);
	add_reconstruction_options(recon_options_group_box);

	QGroupBox* display_options_group_box = new QGroupBox("Display Options", robot_viewer_tab_);
	add_display_options(display_options_group_box);


	robot_viewer_left_panel_layout->addWidget(recon_options_group_box);
	robot_viewer_left_panel_layout->addWidget(display_options_group_box);


	robot_viewer_left_panel_->setLayout(robot_viewer_left_panel_layout);
	robot_viewer_left_panel_->setSizePolicy(QSizePolicy::Minimum, QSizePolicy::Minimum);

	//robot_viewer_layout->addWidget(robot_viewer_left_panel_);
	left_panel_scroll_area->setWidget(robot_viewer_left_panel_);
	robot_viewer_options_layout->addWidget(left_panel_scroll_area);


	robot_viewer_options_layout->addWidget(large_preview_widget_);


	QGroupBox* frame_analysis_group_box = new QGroupBox("Frame Options", robot_viewer_tab_);
	add_frame_analysis_options(frame_analysis_group_box);

	robot_viewer_left_panel_layout->addWidget(frame_analysis_group_box);

	QGroupBox* calibration_group_box = new QGroupBox("Calibration", robot_viewer_tab_);
	add_calibration_options(calibration_group_box);

	robot_viewer_left_panel_layout->addWidget(calibration_group_box);

	robot_viewer_options_layout->addWidget(robot_viewer_);

	robot_viewer_tab_->setLayout(robot_viewer_options_layout);

	tab_widget->addTab(robot_viewer_tab_, "Robot View");

}

void FilteredStructLight::add_camera_info_tab(QTabWidget* tab_widget, std::vector<unsigned>& camera_uuids) {
	camera_info_tab_ = new QWidget();
	QVBoxLayout* camera_info_tab_layout = new QVBoxLayout();

	camera_uuid_labels_ = new QLabel*[camera_uuids.size()];

	camera_uuids_group_box_ = new QGroupBox("Camera Serial Nos", camera_info_tab_);

	QHBoxLayout* camera_uuids_layout = new QHBoxLayout();

	for (auto i = 0u; i < camera_uuids.size(); ++i) {
		std::string label = "Camera Index " + std::to_string(i) + " : ";
		label.append(std::to_string(camera_uuids[i]));

		// init label and set text
		camera_uuid_labels_[i] = new QLabel(camera_info_tab_);
		camera_uuid_labels_[i]->setText(QString(label.c_str()));
		camera_uuids_layout->addWidget(camera_uuid_labels_[i]);
	}
	camera_uuids_group_box_->setLayout(camera_uuids_layout);
	camera_info_tab_layout->addWidget(camera_uuids_group_box_);
	camera_info_tab_->setLayout(camera_info_tab_layout);

	
	tab_widget->addTab(camera_info_tab_, "Camera Info");
}

void FilteredStructLight::add_interior_options(QGroupBox* group_box) {
	QVBoxLayout* group_box_layout = new QVBoxLayout();


	model_filename_ = new QLineEdit(group_box);
	model_filename_browse_ = new QPushButton("...", group_box);
	model_filename_browse_->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Preferred);
	model_filename_browse_->setMaximumSize(30, 30);

	QHBoxLayout* filename_layout = new QHBoxLayout();
	filename_layout->addWidget(model_filename_);
	filename_layout->addWidget(model_filename_browse_);

	connect(model_filename_browse_, &QPushButton::clicked, this, 
		[&] {
		QString selected_filename = QFileDialog::getOpenFileName(this, QString("Open Model"), QDir::currentPath(),
			"Video Files(*.obj)");
		model_filename_->setText(selected_filename);
	});


	connect(model_filename_, &QLineEdit::textChanged, swarm_viewer_, 
		&SwarmViewer::set_model_filename);

	QHBoxLayout* scale_layout = new QHBoxLayout();
	scale_spinbox_ = new QDoubleSpinBox(group_box);
	scale_spinbox_->setRange(0.0001, 50);
	//scale_spinbox_->setValue(10);
	QLabel* scale_label = new QLabel("Scale", group_box);
	scale_layout->addWidget(scale_label);
	scale_layout->addWidget(scale_spinbox_);

	QGroupBox* offset_group_box = new QGroupBox("Offset", group_box);
	QGridLayout* offset_layout = new QGridLayout();

	QLabel* x_offset = new QLabel("x", offset_group_box);
	x_spin_box_ = new QSpinBox(offset_group_box);
	x_spin_box_->setRange(0, 1000);
	x_spin_box_->setSingleStep(1);

	QLabel* y_offset = new QLabel("y", offset_group_box);
	y_spin_box_ = new QSpinBox(offset_group_box);
	y_spin_box_->setRange(0, 1000);
	y_spin_box_->setSingleStep(1);

	QLabel* z_offset = new QLabel("z", offset_group_box);
	z_spin_box_ = new QSpinBox(offset_group_box);
	z_spin_box_->setRange(0, 1000);
	z_spin_box_->setSingleStep(1);

	offset_layout->addWidget(x_offset, 0, 0);
	offset_layout->addWidget(x_spin_box_, 0, 1);
	offset_layout->addWidget(y_offset, 1, 0);
	offset_layout->addWidget(y_spin_box_, 1, 1);
	offset_layout->addWidget(z_offset, 2, 0);
	offset_layout->addWidget(z_spin_box_, 2, 1);

	offset_group_box->setLayout(offset_layout);

	show_interior_ = new QCheckBox("Show Interior", group_box);

	group_box_layout->addLayout(filename_layout);
	group_box_layout->addLayout(scale_layout);
	group_box_layout->addWidget(offset_group_box);
	group_box_layout->addWidget(show_interior_);

	group_box->setLayout(group_box_layout);

	connect(x_spin_box_, static_cast<void (QSpinBox::*)(int)>(&QSpinBox::valueChanged), this,
		[&](int value)
	{
		swarm_viewer_->set_interior_offset(glm::vec3(x_spin_box_->value(), y_spin_box_->value(), z_spin_box_->value()));
	}
	);

	connect(y_spin_box_, static_cast<void (QSpinBox::*)(int)>(&QSpinBox::valueChanged), this,
		[&](int value)
	{
		swarm_viewer_->set_interior_offset(glm::vec3(x_spin_box_->value(), y_spin_box_->value(), z_spin_box_->value()));
	}
	);

	connect(z_spin_box_, static_cast<void (QSpinBox::*)(int)>(&QSpinBox::valueChanged), this,
		[&](int value)
	{
		swarm_viewer_->set_interior_offset(glm::vec3(x_spin_box_->value(), y_spin_box_->value(), z_spin_box_->value()));
	}
	);

	connect(scale_spinbox_, static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), swarm_viewer_, &SwarmViewer::set_interior_scale);


	connect(show_interior_, &QCheckBox::stateChanged, swarm_viewer_, &SwarmViewer::set_show_interior);

}

void FilteredStructLight::add_robot_options(QGroupBox* group_box) {

	QVBoxLayout* group_box_layout = new QVBoxLayout();

	QHBoxLayout* robots_layout = new QHBoxLayout();

	robots_spinbox_ = new QSpinBox(group_box);
	robots_spinbox_->setRange(1, 2000);
	QLabel* robots_label = new QLabel("# of Robots");
	robots_layout->addWidget(robots_label);
	robots_layout->addWidget(robots_spinbox_);

	group_box_layout->addLayout(robots_layout);

	QGroupBox* constants_group_box = new QGroupBox("Constants");
	QGridLayout* constants_layout = new QGridLayout();


	double max_constant_value = 100.0;


	QLabel* exploration_label = new QLabel("Exploration", group_box);
	exploration_constant_ = new QDoubleSpinBox(group_box);
	exploration_constant_->setMinimum(0.0);
	exploration_constant_->setMaximum(max_constant_value);
	exploration_constant_->setSingleStep(0.1);

	constants_layout->addWidget(exploration_label, 0, 0);
	constants_layout->addWidget(exploration_constant_, 0, 1);


	QLabel* seperation_label = new QLabel("Seperation", group_box);
	separation_constant_ = new QDoubleSpinBox(group_box);
	separation_constant_->setMinimum(0.0);
	separation_constant_->setMaximum(max_constant_value);
	separation_constant_->setSingleStep(0.1);

	constants_layout->addWidget(seperation_label, 1, 0);
	constants_layout->addWidget(separation_constant_, 1, 1);


	QLabel* goto_work_label = new QLabel("Go to Work", group_box);
	goto_work_constant_ = new QDoubleSpinBox(group_box);
	goto_work_constant_->setMinimum(0.0);
	goto_work_constant_->setMaximum(max_constant_value);
	goto_work_constant_->setSingleStep(0.1);

	constants_layout->addWidget(goto_work_label, 2, 0);
	constants_layout->addWidget(goto_work_constant_, 2, 1);

	QLabel* separation_distance_label = new QLabel("Separation Distance", group_box);
	separation_distance_ = new QDoubleSpinBox(group_box);
	separation_distance_->setMinimum(0.0);
	separation_distance_->setMaximum(1000.0);
	separation_distance_->setSingleStep(0.1);

	constants_layout->addWidget(separation_distance_label, 3, 0);
	constants_layout->addWidget(separation_distance_, 3, 1);

	constants_group_box->setLayout(constants_layout);


	group_box_layout->addWidget(constants_group_box);

	show_forces_ = new QCheckBox("Show forces", group_box); 
	group_box_layout->addWidget(show_forces_);

	group_box->setLayout(group_box_layout);

	connect(robots_spinbox_, static_cast<void (QSpinBox::*)(int)>(&QSpinBox::valueChanged), swarm_viewer_, &SwarmViewer::set_no_of_robots);
	connect(exploration_constant_, static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), swarm_viewer_, &SwarmViewer::set_exploration_constant);
	connect(separation_constant_, static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), swarm_viewer_, &SwarmViewer::set_separation_constant);
	connect(goto_work_constant_, static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), swarm_viewer_, &SwarmViewer::set_goto_work_constant);
	connect(separation_distance_, static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), swarm_viewer_, &SwarmViewer::set_separation_distance);

	connect(show_forces_, &QCheckBox::stateChanged, swarm_viewer_, &SwarmViewer::set_show_forces);

}

void FilteredStructLight::add_grid_options(QGroupBox* group_box) {
	QVBoxLayout* group_box_layout = new QVBoxLayout();
	QGridLayout* grid_box_layout = new QGridLayout();

	QLabel* grid_resolution_per_side = new QLabel("Resolution (4^n)");
	grid_resolution_spin_box_ = new QSpinBox(group_box);
	grid_resolution_spin_box_->setRange(1, 60000);
	grid_resolution_spin_box_->setValue(16);

	grid_box_layout->addWidget(grid_resolution_per_side, 0, 0);
	grid_box_layout->addWidget(grid_resolution_spin_box_, 0, 1);

	QLabel* grid_length_per_side = new QLabel("Length");
	grid_length_spin_box_ = new QSpinBox(group_box);
	grid_length_spin_box_->setRange(1, 100);
	//grid_length_spin_box_->setValue(20);

	grid_box_layout->addWidget(grid_length_per_side, 1, 0);
	grid_box_layout->addWidget(grid_length_spin_box_, 1, 1);

	group_box_layout->addLayout(grid_box_layout);

	group_box->setLayout(group_box_layout);

	connect(grid_resolution_spin_box_, static_cast<void (QSpinBox::*)(int)>(&QSpinBox::valueChanged), swarm_viewer_, &SwarmViewer::set_grid_resolution);
	connect(grid_length_spin_box_, static_cast<void (QSpinBox::*)(int)>(&QSpinBox::valueChanged), swarm_viewer_, &SwarmViewer::set_grid_length);

}
	
void FilteredStructLight::add_reset_swarm_sim_options(QGroupBox* group_box) {
	QVBoxLayout* group_box_layout = new QVBoxLayout();
	swarm_reset_button_ = new QPushButton("Reset Sim.", group_box);
	group_box_layout->addWidget(swarm_reset_button_);
	group_box->setLayout(group_box_layout);
	connect(swarm_reset_button_, &QPushButton::clicked, swarm_viewer_, &SwarmViewer::reset_sim );
}

void FilteredStructLight::add_swarm_sim_tab(QTabWidget* tab_widget) {

	QWidget* swarm_tab = new QWidget(tab_widget);

	QHBoxLayout* swarm_viewer_layout = new QHBoxLayout();

	QVBoxLayout* swarm_left_panel_layout = new QVBoxLayout();

	// Specify an OpenGL 3.3 format using the Core profile.
	// That is, no old-school fixed pipeline functionality
	QGLFormat glFormat;
	glFormat.setVersion(3, 3);
	glFormat.setProfile(QGLFormat::CoreProfile); // Requires >=Qt-4.8.0
	glFormat.setSampleBuffers(true);
	//glFormat.setSwapInterval(1);
	swarm_viewer_ = new SwarmViewer(glFormat, swarm_tab);
	QSizePolicy robot_viewer_size_policy;
	robot_viewer_size_policy.setHorizontalStretch(2);
	robot_viewer_size_policy.setHorizontalPolicy(QSizePolicy::Preferred);
	robot_viewer_size_policy.setVerticalPolicy(QSizePolicy::Preferred);
	swarm_viewer_->setSizePolicy(robot_viewer_size_policy);

	QGroupBox* swarm_group_box = new QGroupBox("Reset", swarm_tab);
	add_reset_swarm_sim_options(swarm_group_box);

	// change room size
	QGroupBox* interior_group_box = new QGroupBox("Interior", swarm_tab);
	add_interior_options(interior_group_box);

	// no of robots
	QGroupBox* robots_group_box = new QGroupBox("Robots", swarm_tab);
	add_robot_options(robots_group_box);

	// grid box
	QGroupBox* grid_group_box = new QGroupBox("Grid", swarm_tab);
	add_grid_options(grid_group_box);

	swarm_left_panel_layout->addWidget(swarm_group_box);
	swarm_left_panel_layout->addWidget(interior_group_box);
	swarm_left_panel_layout->addWidget(robots_group_box);
	swarm_left_panel_layout->addWidget(grid_group_box);

	swarm_viewer_layout->addLayout(swarm_left_panel_layout);
	swarm_viewer_layout->addWidget(swarm_viewer_);

	swarm_tab->setLayout(swarm_viewer_layout);

	tab_widget->addTab(swarm_tab, "Swarm Sim");
}

void FilteredStructLight::connect_widgets_to_swarm_save_settings() {
	connect(grid_resolution_spin_box_, static_cast<void (QSpinBox::*)(int)>(&QSpinBox::valueChanged), 
		this,
		[&](int value)
	{
		save_swarm_settings();
	}
	);
	
	connect(grid_length_spin_box_, static_cast<void (QSpinBox::*)(int)>(&QSpinBox::valueChanged),
		this,
		[&](int value)
	{
		save_swarm_settings();
	}
	);

	connect(robots_spinbox_, static_cast<void (QSpinBox::*)(int)>(&QSpinBox::valueChanged), 
		this,
		[&](int value)
	{
		save_swarm_settings();
	}
	);

	connect(exploration_constant_, static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), 
		this,
		[&](double value)
	{
		save_swarm_settings();
	}
	);

	connect(separation_constant_, static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), 
		this,
		[&](double value)
	{
		save_swarm_settings();
	}
	);

	connect(goto_work_constant_, static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), 
		this,
		[&](double value)
	{
		save_swarm_settings();
	}
	);

	connect(separation_distance_, static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), 
		this,
		[&](double value)
	{
		save_swarm_settings();
	}
	);

	connect(x_spin_box_, static_cast<void (QSpinBox::*)(int)>(&QSpinBox::valueChanged), this,
		[&](int value)
	{
		save_swarm_settings();
	}
	);

	connect(y_spin_box_, static_cast<void (QSpinBox::*)(int)>(&QSpinBox::valueChanged), this,
		[&](int value)
	{
		save_swarm_settings();
	}
	);

	connect(z_spin_box_, static_cast<void (QSpinBox::*)(int)>(&QSpinBox::valueChanged), this,
		[&](int value)
	{
		save_swarm_settings();
	}
	);

	connect(scale_spinbox_, static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), this,
		[&](double value)
	{
		save_swarm_settings();
	}
	);

	connect(show_interior_, &QCheckBox::stateChanged, 
		this,
		[&](int value)
	{
		save_swarm_settings();
	}
	);

	connect(model_filename_, &QLineEdit::textChanged, 
		this,
		[&](const QString & text)
	{
		save_swarm_settings();
	}
	);
}

QArrayPushButton::QArrayPushButton(QString& text, int id, QWidget* parent) : id_(id) {
	QPushButton(text, parent);
	setText(text);
	setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Preferred);
	setMaximumSize(30, 30);
	connect(this, &QPushButton::clicked, this, &QArrayPushButton::intercept_clicked);
}

void QArrayPushButton::intercept_clicked() {
	emit clicked_with_id(id_);
}

int QArrayPushButton::get_id() const {
	return id_;
}

void FilteredStructLight::add_calibration_options(QGroupBox* calibration_group_box) {

	QGridLayout* calib_grid_layout = new QGridLayout();
	std::stringstream checkerboard_filename_ss;
	checkerboard_filename_ss << "Checkerboard";
	QLabel* checkerboard_video = new QLabel("CheckerBoard Videos", calibration_group_box);
	QLabel* scanline_video = new QLabel("Scanline Videos", calibration_group_box);
	
	calib_grid_layout->addWidget(checkerboard_video, 1, 2);
	calib_grid_layout->addWidget(scanline_video, 1, 4);

	QLabel** video_no_label = new QLabel*[MAX_VIDEO_NO];
	calibration_video_filename_ = new QLineEdit*[MAX_VIDEO_NO];
	calibration_video_browse_ = new QArrayPushButton*[MAX_VIDEO_NO];

	scanline_video_filename_ = new QLineEdit*[MAX_VIDEO_NO];
	scanline_video_browse_ = new QArrayPushButton*[MAX_VIDEO_NO];

	for (int i = 0; i < (MAX_VIDEO_NO); ++i) {
		std::stringstream video_name;
		video_name << "#" << (i + 1);
		video_no_label[i] = new QLabel(video_name.str().c_str(), calibration_group_box);

		QString browse_button_text("...");

		calibration_video_filename_[i] = new QLineEdit(calibration_group_box);
		calibration_video_browse_[i] = new QArrayPushButton(browse_button_text, i, calibration_group_box);
		calibration_video_browse_[i]->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Preferred);
		calibration_video_browse_[i]->setMaximumSize(30, 30);

		scanline_video_filename_[i] = new QLineEdit(calibration_group_box);
		scanline_video_browse_[i] = new QArrayPushButton(browse_button_text, i, calibration_group_box);
		scanline_video_browse_[i]->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Preferred);
		scanline_video_browse_[i]->setMaximumSize(30, 30);

		connect(calibration_video_browse_[i], &QArrayPushButton::clicked_with_id, this, 
			[&](int id) {
			QString selected_filename_ = QFileDialog::getOpenFileName(this, QString("Open Video"), QDir::currentPath(),
				"Video Files(*.h264 *.avi)");
			calibration_video_filename_[id]->setText(selected_filename_);
		});

		connect(scanline_video_browse_[i], &QArrayPushButton::clicked_with_id, this, 
			[&](int id) {
			QString selected_filename_ = QFileDialog::getOpenFileName(this, QString("Open Video"), QDir::currentPath(),
				"Video Files(*.h264 *.avi)");
			scanline_video_filename_[id]->setText(selected_filename_);
		});


		calib_grid_layout->addWidget(video_no_label[i], i + 2, 1);
		calib_grid_layout->addWidget(calibration_video_filename_[i], i + 2, 2);
		calib_grid_layout->addWidget(calibration_video_browse_[i], i + 2, 3);
		calib_grid_layout->addWidget(scanline_video_filename_[i], i + 2, 4);
		calib_grid_layout->addWidget(scanline_video_browse_[i], i + 2, 5);
	}

	QVBoxLayout* calib_main_layout = new QVBoxLayout();
	calib_main_layout->addLayout(calib_grid_layout);
	

	QPushButton* calibrate = new QPushButton("Calibrate");

	calib_main_layout->addWidget(calibrate);
	calibration_group_box->setLayout(calib_main_layout);

	connect(calibrate, &QPushButton::clicked, this, [&] {
		bool are_videos_selected = true;
		const int MIN_NO_VIDEOS_NEEDED = 2;
		for (int i = 0; i < MIN_NO_VIDEOS_NEEDED; ++i) {
			if (calibration_video_filename_[i]->text().compare("") == 0
				|| scanline_video_filename_[i]->text().compare("") == 0) {
				are_videos_selected = false;
				break;
			}
		}

		if (!are_videos_selected) {
			QMessageBox no_file_warning(QMessageBox::Icon::Warning, 
				"No files selected!", "Please select at least 2 calibration videos and 2 scanline videos...",
				QMessageBox::Ok, this);
			no_file_warning.exec();
		} else {
			// do the calibration
			std::vector<std::string> checkerboard_video_filenames;
			std::vector<std::string> scanline_video_filenames;
			
			for (int i = 0; i < MAX_VIDEO_NO; ++i) {
				std::string checkerboard_filename = calibration_video_filename_[i]->text().toStdString();
				std::string scanline_filename = scanline_video_filename_[i]->text().toStdString();

				if (checkerboard_filename.compare("") == 0) {
					// reached the end
					break;
				}
				checkerboard_video_filenames.push_back(checkerboard_filename);
				scanline_video_filenames.push_back(scanline_filename);
			}
			robot_reconstruction_->calculate_light_position(checkerboard_video_filenames, scanline_video_filenames);
		}

	});

}

void FilteredStructLight::add_robot_calibration_tab(QTabWidget* tab_widget) {
	robot_calibration_tab_ = new QWidget();

	QHBoxLayout* robot_calibration_tab_layout = new QHBoxLayout();
	QVBoxLayout* settings_layout = new QVBoxLayout();

	QWidget* settings_widget = new QWidget(robot_calibration_tab_);

	calibrate_intrinsic_with_video_ = new QPushButton("Calibrate Intrinsic Using Video");

	calibrate_extrinsic_with_video_ = new QPushButton("Calibrate Extrinsic Using Video");

	find_line_with_video_ = new QPushButton("Find Line Using Video");

	interpolate_line_ = new QPushButton("Calculate Light Position");

	settings_layout->addWidget(calibrate_intrinsic_with_video_);
	settings_layout->addWidget(calibrate_extrinsic_with_video_);
	settings_layout->addWidget(find_line_with_video_);
	settings_layout->addWidget(interpolate_line_);

	QVBoxLayout* image_layout = new QVBoxLayout();
	image_viewer_ = new ImageViewer(robot_calibration_tab_);

	robot_calibration_tab_->setLayout(robot_calibration_tab_layout);
	settings_widget->setLayout(settings_layout);

	robot_calibration_tab_layout->addWidget(settings_widget);
	robot_calibration_tab_layout->addWidget(image_viewer_);
	
	tab_widget->addTab(robot_calibration_tab_, "Robot Calibration");

	robot_reconstruction_ = new RobotReconstruction();

	connect(robot_reconstruction_, &RobotReconstruction::display_image, image_viewer_, &ImageViewer::display_image);

	connect(calibrate_intrinsic_with_video_, &QPushButton::clicked, this, 
		[&]()
	{
		robot_reconstruction_->calibrate_intrinsic_from_video("light-stripe-calib-checkerboard_1.h264");
	});

	connect(calibrate_extrinsic_with_video_, &QPushButton::clicked, this, 
		[&]()
	{
		//robot_reconstruction_->calibrate_extrinsic_from_video("light-stripe-calib-checkerboard.h264");
		robot_reconstruction_->calibrate_extrinsic_from_video("light-stripe-calib-checkerboard_1.h264");
	});

	connect(find_line_with_video_, &QPushButton::clicked, this, 
		[&]()
	{
		robot_reconstruction_->identify_line_from_video("light-stripe-calib-stripe_1.h264");
	});

	connect(interpolate_line_, &QPushButton::clicked, this, 
		[&]()
	{
		//robot_reconstruction_->calculate_light_position(,,);
	});

}

void FilteredStructLight::update_images(int frame_no) {
	if ((frame_no >= 0) && (frame_no < (frame_filenames_.size()))) {
		QPixmap pixmap(frame_filenames_[frame_no].c_str());
		image_preview_->setPixmap(pixmap);
		large_image_preview_->setPixmap(pixmap);
	}
}

void FilteredStructLight::start_reconstruction_sequence() {
	frame_filenames_.clear();
}

void FilteredStructLight::handle_frame_filenames(std::vector<std::string> image_list) {
	frame_filenames_ = image_list;
	frame_selection_spin_box_->setMinimum(0);
	int max_value = (image_list.size() > 0) ? image_list.size() - 1 : 0;
	frame_selection_spin_box_->setMaximum(max_value);
	frame_selection_spin_box_->setValue(0);
	emit frame_selection_spin_box_->valueChanged(0);
}

void FilteredStructLight::add_camera_calibration_tab(QTabWidget* tab_widget) {
	camera_tab_ = new QWidget();
	QHBoxLayout* camera_tab_layout = new QHBoxLayout();

	camera_tab_->setLayout(camera_tab_layout);


	left_panel_ = new QWidget(central_widget_);

	view_cameras_ = new QPushButton("View Cameras");
	QVBoxLayout* vbox_layout = new QVBoxLayout();
	vbox_layout->addWidget(view_cameras_);

	threshold_slider_ = new QSlider(Qt::Horizontal, left_panel_);
	threshold_slider_->setRange(0, 1000);
	threshold_slider_->setSliderPosition(50);

	vbox_layout->addWidget(threshold_slider_);

	threshold_toggle_checkbox_ = new QCheckBox(QString("Toggle Thresholding"), left_panel_);
	threshold_toggle_checkbox_->setChecked(false);

	vbox_layout->addWidget(threshold_toggle_checkbox_);

	left_panel_->setLayout(vbox_layout);
	left_panel_->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Preferred);

	camera_tab_layout->addWidget(left_panel_);


	right_panel_ = new QWidget(central_widget_);


	QVBoxLayout* vbox_layout2 = new QVBoxLayout();

	right_panel_->setLayout(vbox_layout2);
	right_panel_->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Preferred);

	camera_tab_layout->addWidget(right_panel_);
	

	// start cam thread
	cam_thread_ = new CamThread(this);

	auto serial_nos = cam_thread_->get_serial_nos();


	// Specify an OpenGL 3.3 format using the Core profile.
	// That is, no old-school fixed pipeline functionality
	QGLFormat glFormat;
	glFormat.setVersion(3, 3);
	glFormat.setProfile(QGLFormat::CoreProfile); // Requires >=Qt-4.8.0
	glFormat.setSampleBuffers(true);

	opengl_widget_ = new GLWidget(cam_thread_->get_no_of_cams(), glFormat, this);
	//opengl_widget_->setBaseSize(200, 200);

	vbox_layout2->addWidget(opengl_widget_);

	// camera pairs
	camera_pairs_group_ = new QGroupBox("Camera Calibration Pairs", left_panel_);
	QGridLayout* pair_grid_layout = new QGridLayout(left_panel_);
	
	for (int i = 0; i < CAM_CALIB_PAIRS; ++i)
	{
		camera_pair_[i] = new QSpinBox(left_panel_);
		camera_pair_[i]->setMaximum(CAM_CALIB_PAIRS);
		camera_pair_[i]->setMinimum(0);
		camera_pair_[i]->setValue(i);
		pair_grid_layout->addWidget(camera_pair_[i], i / 2, i % 2);
	}

	camera_pair_[0]->setValue(0);
	camera_pair_[1]->setValue(1);


	camera_pairs_group_->setLayout(pair_grid_layout);
	vbox_layout->addWidget(camera_pairs_group_);

	reconstructor_ = new Reconstruct3D(cam_thread_->get_no_of_cams(), this);


	// calibration
	calibration_group_ = new QGroupBox("Calibration", left_panel_);

	recalibrate_button = new QPushButton("Recalibrate", calibration_group_);
	recalibrate_button->setShortcut(QKeySequence(Qt::SHIFT + Qt::Key_R));

	start_calibration_video_ = new QPushButton("&Start Calibration", calibration_group_);
	start_calibration_video_->setShortcut(QKeySequence("s"));
	

	CameraPairs pairs;
	create_camera_pairs(pairs);
	tab_widget->addTab(camera_tab_, "Cameras");
	add_reconstruction_tab(pairs, tab_widget);
	add_camera_info_tab(tab_widget, serial_nos);
	connect(reconstructor_, &Reconstruct3D::finished_reconstruction_with_triangles, model_viewer_, &ModelViewer::update_model_with_triangles);

	connect(recalibrate_button, &QPushButton::clicked, this, 
		[&]()
	{
		reconstructor_->clear_camera_img_map();
		CameraPairs pairs;
		create_camera_pairs(pairs);
		reconstructor_->recalibrate(pairs);
	});

	connect(start_calibration_video_, &QPushButton::clicked, this, 
		[&]()
	{
		reconstructor_->clear_camera_img_map();
		connect(cam_thread_, &CamThread::image_ready, reconstructor_, &Reconstruct3D::collect_images);
	});

	end_calibration_video_ = new QPushButton("&End Calibration", calibration_group_);
	end_calibration_video_->setShortcut(QKeySequence("e"));
	connect(end_calibration_video_, &QPushButton::clicked, this, 
		[&]()
	{
		disconnect(cam_thread_, &CamThread::image_ready, reconstructor_, &Reconstruct3D::collect_images);
		CameraPairs pairs;
		create_camera_pairs(pairs);
		reconstructor_->calibrate(pairs);
	});

	QVBoxLayout* calibration_group_layout = new QVBoxLayout(left_panel_);
	calibration_group_layout->addWidget(recalibrate_button);
	calibration_group_layout->addWidget(start_calibration_video_);
	calibration_group_layout->addWidget(end_calibration_video_);

	start_calibration_video_->setEnabled(true);
	calibration_group_->setLayout(calibration_group_layout);

	vbox_layout->addWidget(calibration_group_);

	// reconstruction
	reconstruction_group_ = new QGroupBox("reconstruction", left_panel_);
	
	// re reconstruction
	re_reconstruction_button = new QPushButton("Re-reconstruct", reconstruction_group_);



	connect(re_reconstruction_button, &QPushButton::clicked, this, 
		[&]()
	{
		reconstructor_->clear_camera_img_map();
		CameraPairs pairs;
		create_camera_pairs(pairs);
		reconstructor_->re_reconstruct(pairs, recon_no_of_images_spin_box_->value());
	});

	load_camera_calibration_ = new QPushButton("Load calibration", reconstruction_group_);


	connect(load_camera_calibration_, &QPushButton::clicked, this, 
		[&]()
	{
		CameraPairs pairs;
		create_camera_pairs(pairs);
		reconstructor_->load_calibration(pairs[0].first, pairs[0].second);
	});

	start_reconstruction_video_ = new QPushButton("Start reconstruction", reconstruction_group_);
	start_reconstruction_video_->setShortcut(QKeySequence("r"));
	
	connect(start_reconstruction_video_, &QPushButton::clicked, this, 
		[&]()
	{
		reconstructor_->clear_camera_img_map();
		connect(cam_thread_, &CamThread::image_ready, reconstructor_, &Reconstruct3D::collect_images_without_delay);
		//connect(cam_thread_, &CamThread::image_ready, reconstructor_, &Reconstruct3D::collect_images);
	});

	end_reconstruction_video_ = new QPushButton("End reconstruction", reconstruction_group_);
	end_reconstruction_video_->setShortcut(QKeySequence("t"));
	connect(end_reconstruction_video_, &QPushButton::clicked, this, 
		[&]()
	{
		disconnect(cam_thread_, &CamThread::image_ready, reconstructor_, &Reconstruct3D::collect_images_without_delay);
		//disconnect(cam_thread_, &CamThread::image_ready, reconstructor_, &Reconstruct3D::collect_images);
		CameraPairs pairs;
		create_camera_pairs(pairs);
		reconstructor_->run_reconstruction(pairs, recon_no_of_images_spin_box_->value());
	});

	QLabel* no_of_images = new QLabel("No. of images to use (-1 = all)", reconstruction_group_);
	recon_no_of_images_spin_box_ = new QSpinBox(reconstruction_group_);
	recon_no_of_images_spin_box_->setMinimum(-1);
	recon_no_of_images_spin_box_->setMaximum(1000);
	recon_no_of_images_spin_box_->setValue(-1);

	QVBoxLayout* reconstruction_group_layout = new QVBoxLayout(left_panel_);
	reconstruction_group_layout->addWidget(no_of_images);
	reconstruction_group_layout->addWidget(recon_no_of_images_spin_box_);
	reconstruction_group_layout->addWidget(re_reconstruction_button);
	reconstruction_group_layout->addWidget(load_camera_calibration_);
	reconstruction_group_layout->addWidget(start_reconstruction_video_);
	reconstruction_group_layout->addWidget(end_reconstruction_video_);

	reconstruction_group_->setLayout(reconstruction_group_layout);

	vbox_layout->addWidget(reconstruction_group_);



	connect(cam_thread_, &CamThread::image_ready, opengl_widget_, &GLWidget::display_image);
	connect(threshold_slider_, &QSlider::valueChanged, opengl_widget_, &GLWidget::set_threshold);
	connect(threshold_toggle_checkbox_, &QCheckBox::stateChanged, opengl_widget_, &GLWidget::toggle_thresholding);

	cam_thread_->start();
}

void FilteredStructLight::setupUi() {
	central_widget_ = new QWidget(this);
	setCentralWidget(central_widget_);

	QHBoxLayout *main_layout = new QHBoxLayout();
	central_widget_->setLayout(main_layout);


	QTabWidget* tab_widget = new QTabWidget();
	tab_widget->setTabPosition(QTabWidget::West);
	
	main_layout->addWidget(tab_widget);


	QHBoxLayout* robot_viewer_layout = new QHBoxLayout();

	//main_layout->addWidget(robot_viewer_);

	add_swarm_sim_tab(tab_widget);
	add_robot_viewer_tab(tab_widget);
	add_robot_calibration_tab(tab_widget);

	connect(robot_reconstruction_, &RobotReconstruction::create_plane_with_points_and_lines,
		robot_viewer_, &RobotViewer::create_plane_with_points_and_lines);
	
	connect(robot_reconstruction_, &RobotReconstruction::create_points, robot_viewer_, &RobotViewer::create_points);
	connect(robot_reconstruction_, &RobotReconstruction::create_plane, robot_viewer_, &RobotViewer::create_plane);

	connect(robot_reconstruction_, &RobotReconstruction::start_reconstruction_sequence, robot_viewer_, 
		&RobotViewer::start_reconstruction_sequence);
	connect(robot_reconstruction_, &RobotReconstruction::create_reconstruction_frame, robot_viewer_, 
		&RobotViewer::create_reconstruction_frame);

	connect(robot_reconstruction_, &RobotReconstruction::end_reconstruction_sequence, robot_viewer_, 
		&RobotViewer::end_reconstruction_sequence);

	connect(robot_reconstruction_, &RobotReconstruction::create_calibration_frame, robot_viewer_, 
		&RobotViewer::create_calibration_frame);
	connect(robot_reconstruction_, &RobotReconstruction::create_final_calibration_frame, robot_viewer_, 
		&RobotViewer::create_final_calibration_frame);

	connect(robot_reconstruction_, &RobotReconstruction::start_reconstruction_sequence, this, 
		&FilteredStructLight::start_reconstruction_sequence);
	connect(robot_reconstruction_, &RobotReconstruction::create_reconstruction_image_list, this, 
		&FilteredStructLight::handle_frame_filenames);


	load_recon_settings();
	connect_recon_line_edits_to_save_settings();

	load_swarm_settings();
	connect_widgets_to_swarm_save_settings();


	central_widget_->adjustSize();
	showMaximized();
}

void FilteredStructLight::keyReleaseEvent(QKeyEvent* e) {
	switch (e->key()) {
	case Qt::Key_Escape:
		// Quit the application
		shutdown_cam_thread();
		QApplication::quit();
		break;
	default:
		// Propagate unhandled events
		QWidget::keyReleaseEvent(e);
		break;
	}
}

void FilteredStructLight::onClose(QKeyEvent* event) {
	 QMessageBox::StandardButton resBtn = QMessageBox::question( this, "FilteredStructLight",
                                                                tr("Are you sure?\n"),
                                                                QMessageBox::Cancel | QMessageBox::No | QMessageBox::Yes,
                                                                QMessageBox::Yes);
    if (resBtn != QMessageBox::Yes) {
        event->ignore();
    } else {
		// Quit the application
		shutdown_cam_thread();
        event->accept();
    }
}