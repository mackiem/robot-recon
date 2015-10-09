#include "filteredstructlight.h"
#include <QGLFormat>
#include "modelviewer.h"

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
	QVBoxLayout* color_method_box_layout = new QVBoxLayout();
	QRadioButton* color_texture_radio_button = new QRadioButton("Texture", color_method_group_box);	
	color_method_box_layout->addWidget(color_texture_radio_button);
	QRadioButton* color_solid_color_radio_button = new QRadioButton("Solid Color", color_method_group_box);	
	color_method_box_layout->addWidget(color_solid_color_radio_button);
	color_method_group_box->setLayout(color_method_box_layout);

	reset_orientation_button_ = new QPushButton(reconstruct_left_panel_);

	reconstruct_left_panel_layout->addWidget(camera_pairs_group_box);
	reconstruct_left_panel_layout->addWidget(draw_method_group_box);
	reconstruct_left_panel_layout->addWidget(color_method_group_box);
	reconstruct_left_panel_layout->addWidget(reset_orientation_button_);

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

void FilteredStructLight::setupUi() {
	central_widget_ = new QWidget(this);
	setCentralWidget(central_widget_);

	QHBoxLayout *main_layout = new QHBoxLayout();
	central_widget_->setLayout(main_layout);


	QTabWidget* tab_widget = new QTabWidget();
	
	main_layout->addWidget(tab_widget);

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
		CameraPairs pairs;
		create_camera_pairs(pairs);
		reconstructor_->re_reconstruct(pairs);
	});

	load_camera_calibration_ = new QPushButton("Load calibration", reconstruction_group_);


	connect(load_camera_calibration_, &QPushButton::clicked, this, 
		[&]()
	{
		CameraPairs pairs;
		create_camera_pairs(pairs);
		reconstructor_->load_calibration(pairs[0].first, pairs[1].second);
	});

	start_reconstruction_video_ = new QPushButton("Start reconstruction", reconstruction_group_);
	start_reconstruction_video_->setShortcut(QKeySequence("r"));
	
	connect(start_reconstruction_video_, &QPushButton::clicked, this, 
		[&]()
	{
		reconstructor_->clear_camera_img_map();
		connect(cam_thread_, &CamThread::image_ready, reconstructor_, &Reconstruct3D::collect_images_without_delay);
	});

	end_reconstruction_video_ = new QPushButton("End reconstruction", reconstruction_group_);
	end_reconstruction_video_->setShortcut(QKeySequence("t"));
	connect(end_reconstruction_video_, &QPushButton::clicked, this, 
		[&]()
	{
		disconnect(cam_thread_, &CamThread::image_ready, reconstructor_, &Reconstruct3D::collect_images_without_delay);
		CameraPairs pairs;
		create_camera_pairs(pairs);
		reconstructor_->run_reconstruction(pairs);
	});

	QVBoxLayout* reconstruction_group_layout = new QVBoxLayout(left_panel_);
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
	central_widget_->adjustSize();

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