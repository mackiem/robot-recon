#ifndef FILTEREDSTRUCTLIGHT_H
#define FILTEREDSTRUCTLIGHT_H

#include "cameradisplaywidget.h"
#include <QtWidgets/QMainWindow>
#include "ui_filteredstructlight.h"
#include <QtWidgets>
#include "camthread.h"

class FilteredStructLight : public QMainWindow
{
	Q_OBJECT

public:
	FilteredStructLight(QWidget *parent = 0);
	~FilteredStructLight();
	void setupUi();

private:
	Ui::FilteredStructLightClass ui;
	QWidget* central_widget_;

	QWidget* left_panel_;
	QPushButton* view_cameras_;

	QWidget* right_panel_;
	GLWidget* opengl_widget_;
	

};

#endif // FILTEREDSTRUCTLIGHT_H
