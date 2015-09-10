#ifndef FILTEREDSTRUCTLIGHT_H
#define FILTEREDSTRUCTLIGHT_H

#include "cameradisplaywidget.h"
#include <QtWidgets/QMainWindow>
#include <QtWidgets>
#include "camthread.h"

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

	QWidget* central_widget_;

	QWidget* left_panel_;
	QPushButton* view_cameras_;

	QWidget* right_panel_;
	GLWidget* opengl_widget_;
	
	void shutdown_cam_thread();

};

#endif // FILTEREDSTRUCTLIGHT_H
