#include "filteredstructlight.h"
#include <QGLFormat>

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

void FilteredStructLight::setupUi() {
	central_widget_ = new QWidget(this);
	setCentralWidget(central_widget_);

	QHBoxLayout *main_layout = new QHBoxLayout();
	central_widget_->setLayout(main_layout);


	left_panel_ = new QWidget(central_widget_);

	view_cameras_ = new QPushButton("View Cameras");
	QVBoxLayout* vbox_layout = new QVBoxLayout();
	vbox_layout->addWidget(view_cameras_);

	threshold_slider_ = new QSlider(Qt::Horizontal, left_panel_);
	threshold_slider_->setRange(0, 1000);
	threshold_slider_->setSliderPosition(50);

	vbox_layout->addWidget(threshold_slider_);

	threshold_toggle_checkbox_ = new QCheckBox(QString("Toggle Thresholding"), left_panel_);
	threshold_toggle_checkbox_->setChecked(true);

	vbox_layout->addWidget(threshold_toggle_checkbox_);

	left_panel_->setLayout(vbox_layout);
	left_panel_->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Preferred);

	main_layout->addWidget(left_panel_);


	right_panel_ = new QWidget(central_widget_);


	QVBoxLayout* vbox_layout2 = new QVBoxLayout();

	right_panel_->setLayout(vbox_layout2);
	right_panel_->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Preferred);

	main_layout->addWidget(right_panel_);
	

	// start cam thread
	cam_thread_ = new CamThread(this);

	// Specify an OpenGL 3.3 format using the Core profile.
	// That is, no old-school fixed pipeline functionality
	QGLFormat glFormat;
	glFormat.setVersion(3, 3);
	glFormat.setProfile(QGLFormat::CoreProfile); // Requires >=Qt-4.8.0
	glFormat.setSampleBuffers(true);

	opengl_widget_ = new GLWidget(cam_thread_->get_no_of_cams(), glFormat, this);
	//opengl_widget_->setBaseSize(200, 200);

	vbox_layout2->addWidget(opengl_widget_);

	camera_pairs_group_ = new QGroupBox("Camera Calibration Pairs", left_panel_);

	QGridLayout* pair_grid_layout = new QGridLayout(left_panel_);
	
	for (int i = 0; i < CAM_CALIB_PAIRS; ++i)
	{
		camera_pair_[i] = new QSpinBox(left_panel_);
		camera_pair_[i]->setMaximum(CAM_CALIB_PAIRS);
		camera_pair_[i]->setMinimum(0);
		pair_grid_layout->addWidget(camera_pair_[i], i / 2, i % 2);
	}

	camera_pairs_group_->setLayout(pair_grid_layout);
	vbox_layout->addWidget(camera_pairs_group_);

	calibration_group_ = new QGroupBox("Calibration", left_panel_);
	

	start_calibration_video_ = new QPushButton("Start Calibration", calibration_group_);
	
	reconstructor_ = new Reconstruct3D(cam_thread_->get_no_of_cams(), this);

	connect(start_calibration_video_, &QPushButton::clicked, this, 
		[&]()
	{
		reconstructor_->clear_camera_img_map();
		connect(cam_thread_, &CamThread::image_ready, reconstructor_, &Reconstruct3D::collect_images);
	});

	end_calibration_video_ = new QPushButton("End Calibration", calibration_group_);
	connect(end_calibration_video_, &QPushButton::clicked, this, 
		[&]()
	{
		disconnect(cam_thread_, &CamThread::image_ready, reconstructor_, &Reconstruct3D::collect_images);
		CameraPairs pairs;
		for (int i = 0; i < CAM_CALIB_PAIRS / 2; ++i)
		{
			pairs.push_back(std::pair<int, int>(camera_pair_[2*i]->value(), 
				camera_pair_[2*i + 1]->value()));
		}
		reconstructor_->run_calibration(pairs);
	});

	QHBoxLayout* calib_group_layout = new QHBoxLayout(left_panel_);
	calib_group_layout->addWidget(start_calibration_video_);
	calib_group_layout->addWidget(end_calibration_video_);

	calibration_group_->setLayout(calib_group_layout);

	vbox_layout->addWidget(calibration_group_);

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