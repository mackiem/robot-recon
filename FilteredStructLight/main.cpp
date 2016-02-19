#define QT_QML_DEBUG
#include "filteredstructlight.h"
#include "projectorwindow.h"
#include <QtWidgets/QApplication>
#include <QQuickView>


int main(int argc, char *argv[])
{
	QApplication a(argc, argv);
	qRegisterMetaType<FlyCapture2::Image>("FlyCapture2::Image");
	qRegisterMetaType<FlyCapture2::Error>("FlyCapture2::Error");
	qRegisterMetaType<cv::Vec3f>("cv::Vec3f");
	qRegisterMetaType<std::vector<cv::Vec3f>>("std::vector<cv::Vec3f>");
	qRegisterMetaType<cv::Mat>("cv::Mat");
	qRegisterMetaType<std::vector<std::string>>("std::vector<std::string>");

	FilteredStructLight w;
	w.show();

	//ProjectorWndow projector_window;
	//projector_window.show();

	QQuickView view; 
	view.setSource(QUrl("qrc:/FilteredStructLight/projector.qml"));
	view.showFullScreen();
	view.setPosition(QPoint(2960, 0));

	//QQuickView view_2;
	//view_2.setSource(QUrl("qrc:/FilteredStructLight/projector.qml"));
	//view_2.showFullScreen();
	//view_2.setPosition(QPoint(4250, 0));


	return a.exec();
}
