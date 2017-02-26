//#define QT_QML_DEBUG
#include "filteredstructlight.h"
#include "projectorwindow.h"
#include <QtWidgets/QApplication>
#include <QQuickView>
#define _CRTDBG_MAP_ALLOC
#include <stdlib.h>
#include <crtdbg.h>
#include <vld.h>

//#ifdef _DEBUG
//#ifndef DBG_NEW
//#define DBG_NEW new ( _NORMAL_BLOCK , __FILE__ , __LINE__ )
//#define new DBG_NEW
//#endif
//#endif  // _DEBUG

// Call _CrtDumpMemoryLeaks after main has returned and before program terminates.
//struct AtExit
//{
//	~AtExit() {
//		_CrtDumpMemoryLeaks();
//	}
//} doAtExit;
//
//void dump() {
//		_CrtDumpMemoryLeaks();
//}

void run_program(int argc, char *argv[]) {
	QApplication a(argc, argv);
	qRegisterMetaType<FlyCapture2::Image>("FlyCapture2::Image");
	qRegisterMetaType<FlyCapture2::Error>("FlyCapture2::Error");
	qRegisterMetaType<cv::Vec3f>("cv::Vec3f");
	qRegisterMetaType<std::vector<cv::Vec3f>>("std::vector<cv::Vec3f>");
	qRegisterMetaType<cv::Mat>("cv::Mat");
	qRegisterMetaType<std::vector<std::string>>("std::vector<std::string>");
	qRegisterMetaType<SwarmParams>("SwarmParams");
	qRegisterMetaType<OptimizationParams>("OptimizationParams");
	qRegisterMetaType<OptimizationResults>("OptimizationResults");
	qRegisterMetaType<MCMCParams>("MCMCParams");

	FilteredStructLight w;
	w.show();
	a.exec();
}

int main(int argc, char *argv[])
{
	//_CrtSetDbgFlag(_CRTDBG_ALLOC_MEM_DF | _CRTDBG_LEAK_CHECK_DF);

	//_CrtSetBreakAlloc(1144150);
	
	//_CrtSetBreakAlloc(1554835);
	run_program(argc, argv);

	//atexit(dump);
	//float *leak = new float();
	//printf("%f", *leak);

	//_CrtDumpMemoryLeaks();
}
