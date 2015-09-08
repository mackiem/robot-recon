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

void FilteredStructLight::setupUi() {
	central_widget_ = new QWidget(this);
	setCentralWidget(central_widget_);

	QHBoxLayout *main_layout = new QHBoxLayout();
	central_widget_->setLayout(main_layout);


	left_panel_ = new QWidget(central_widget_);

	view_cameras_ = new QPushButton("View Cameras");
	QVBoxLayout* vbox_layout = new QVBoxLayout();
	vbox_layout->addWidget(view_cameras_);

	left_panel_->setLayout(vbox_layout);
	left_panel_->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Preferred);

	main_layout->addWidget(left_panel_);


	right_panel_ = new QWidget(central_widget_);


	QVBoxLayout* vbox_layout2 = new QVBoxLayout();

	right_panel_->setLayout(vbox_layout2);
	right_panel_->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Preferred);

	main_layout->addWidget(right_panel_);
	
	central_widget_->adjustSize();

	// start cam thread
	CamThread* cam_thread = new CamThread(this);

	// Specify an OpenGL 3.3 format using the Core profile.
	// That is, no old-school fixed pipeline functionality
	QGLFormat glFormat;
	glFormat.setVersion(3, 3);
	glFormat.setProfile(QGLFormat::CoreProfile); // Requires >=Qt-4.8.0
	glFormat.setSampleBuffers(true);

	opengl_widget_ = new GLWidget(cam_thread->get_no_of_cams(), glFormat, this);
	opengl_widget_->setBaseSize(200, 200);

	vbox_layout2->addWidget(opengl_widget_);

	connect(cam_thread, &CamThread::image_ready, opengl_widget_, &GLWidget::display_image);

	cam_thread->start();




}


