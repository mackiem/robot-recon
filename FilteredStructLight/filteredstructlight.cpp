#include "filteredstructlight.h"
#include <QGLFormat>
#include "modelviewer.h"
#include "robotviewer.h"


FilteredStructLight::FilteredStructLight(QWidget *parent)
	: QMainWindow(parent)
{
	//ui.setupUi(this);
	setupUi();
}

FilteredStructLight::~FilteredStructLight()
{
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
	video_filename_ = new QLineEdit("", recon_options_group_box);
	browse_button_ = new QPushButton("Browse", recon_options_group_box);
	reconstruct_file_layout->addWidget(video_file_label);
	reconstruct_file_layout->addWidget(video_filename_);
	reconstruct_file_layout->addWidget(browse_button_);

	QHBoxLayout* velocity_layout = new QHBoxLayout();
	QLabel* velocity = new QLabel("Velocity", recon_options_group_box);
	velocity_line_edit_ = new QLineEdit("70", recon_options_group_box);
	velocity_line_edit_->setAlignment(Qt::AlignRight);
	velocity_layout->addWidget(velocity);
	velocity_layout->addWidget(velocity_line_edit_);

	QHBoxLayout* nth_frame_layout = new QHBoxLayout();
	QLabel* nframe_label = new QLabel("Nth Frame", recon_options_group_box);
	nframe_line_edit_ = new QLineEdit("100", recon_options_group_box);
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
		video_filename_->setText(selected_filename_);
	});



	connect(reconstruct_from_video_push_button, &QPushButton::clicked,  this, 
		[&]()
	{
		if (selected_filename_.compare(QString("")) == 0) {
			QMessageBox no_file_warning(QMessageBox::Icon::Warning, "No files selected!", "Please select a valid video file...",
				QMessageBox::Ok, this);
			no_file_warning.exec();
		} else {
			robot_reconstruction_->reconstruct_from_video(video_filename_->text().toStdString(),
				std::stoi(nframe_line_edit_->text().toStdString()),
				std::stof(velocity_line_edit_->text().toStdString()), cv::Vec3f(-1.f, 0.f, 0.f));
			
		}
	});
	
}

void FilteredStructLight::add_display_options(QGroupBox* display_options_group_box) {
	// checkboxes of options
	QVBoxLayout* display_options_layout = new QVBoxLayout();

	//QLabel* draw_planes_label = new QLabel("Show Planes", display_options_group_box);
	draw_planes_check_box_ = new QCheckBox("Planes", display_options_group_box);
	draw_points_check_box_ = new QCheckBox("Points", display_options_group_box);
	draw_lines_check_box_ = new QCheckBox("Lines", display_options_group_box);
	draw_default_check_box_ = new QCheckBox("Camera Setup", display_options_group_box);

	display_options_layout->addWidget(draw_planes_check_box_);
	display_options_layout->addWidget(draw_points_check_box_);
	display_options_layout->addWidget(draw_lines_check_box_);
	display_options_layout->addWidget(draw_default_check_box_);

	display_options_group_box->setLayout(display_options_layout);

	connect(draw_points_check_box_, &QCheckBox::stateChanged, robot_viewer_, &RobotViewer::toggle_draw_points);
	connect(draw_planes_check_box_, &QCheckBox::stateChanged, robot_viewer_, &RobotViewer::toggle_draw_planes);
	connect(draw_lines_check_box_, &QCheckBox::stateChanged, robot_viewer_, &RobotViewer::toggle_draw_lines);
	connect(draw_default_check_box_, &QCheckBox::stateChanged, robot_viewer_, &RobotViewer::toggle_draw_default);

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
	image_preview_->setBaseSize(320, 180);
	image_preview_->setScaledContents(true);
	image_preview_->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);

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
	robot_viewer_tab_ = new QWidget();

	QHBoxLayout* robot_viewer_layout = new QHBoxLayout();
	// Specify an OpenGL 3.3 format using the Core profile.
	// That is, no old-school fixed pipeline functionality
	QGLFormat glFormat;
	glFormat.setVersion(3, 3);
	glFormat.setProfile(QGLFormat::CoreProfile); // Requires >=Qt-4.8.0
	glFormat.setSampleBuffers(true);
	//glFormat.setSwapInterval(1);
	robot_viewer_ = new RobotViewer(glFormat, robot_viewer_tab_);

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

	QWidget* robot_viewer_left_panel_ = new QWidget(robot_viewer_tab_);
	QVBoxLayout* robot_viewer_left_panel_layout = new QVBoxLayout();

	QGroupBox* recon_options_group_box = new QGroupBox("Recon. Options", robot_viewer_tab_);
	add_reconstruction_options(recon_options_group_box);

	QGroupBox* display_options_group_box = new QGroupBox("Display Options", robot_viewer_tab_);
	add_display_options(display_options_group_box);


	robot_viewer_left_panel_layout->addWidget(recon_options_group_box);
	robot_viewer_left_panel_layout->addWidget(display_options_group_box);


	robot_viewer_left_panel_->setLayout(robot_viewer_left_panel_layout);
	robot_viewer_left_panel_->setSizePolicy(QSizePolicy::Minimum, QSizePolicy::Minimum);

	robot_viewer_layout->addWidget(robot_viewer_left_panel_);
	robot_viewer_layout->addWidget(large_preview_widget_);


	QGroupBox* frame_analysis_group_box = new QGroupBox("Frame Options", robot_viewer_tab_);
	add_frame_analysis_options(frame_analysis_group_box);

	robot_viewer_left_panel_layout->addWidget(frame_analysis_group_box);


	robot_viewer_layout->addWidget(robot_viewer_);
	QSizePolicy robot_viewer_size_policy;
	robot_viewer_size_policy.setHorizontalStretch(2);
	robot_viewer_size_policy.setHorizontalPolicy(QSizePolicy::Preferred);
	robot_viewer_size_policy.setVerticalPolicy(QSizePolicy::Preferred);
	robot_viewer_->setSizePolicy(robot_viewer_size_policy);

	robot_viewer_tab_->setLayout(robot_viewer_layout);

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
		robot_reconstruction_->calculate_light_position();
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
	
	main_layout->addWidget(tab_widget);
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

	connect(robot_reconstruction_, &RobotReconstruction::start_reconstruction_sequence, this, 
		&FilteredStructLight::start_reconstruction_sequence);
	connect(robot_reconstruction_, &RobotReconstruction::create_reconstruction_image_list, this, 
		&FilteredStructLight::handle_frame_filenames);



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