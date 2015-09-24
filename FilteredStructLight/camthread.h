#pragma once

#include <QThread>
#include "assert.h"
#include <QMutex>

#include <string>
#include <vector>
#include <sstream>
#include <iostream>
#include "FlyCapture2.h"

using namespace FlyCapture2;
using namespace std;

class CamThread : public QThread
{
	Q_OBJECT

private:

	bool is_shutting_down_;

	QMutex thread_lock;
	// Maximum cameras on the bus. (Maximum devices allowed on a 1394 bus is 64).
	const static int ciMaxCameras = 64;

	// Maximum size of expected (raw) image.
	const static int ciMaxImageSize = 2000 * 2000;

	// Directory to save data to.
	const static string csDestinationDirectory;

	// Number of grabbed images
	const static int numberOfImages = 20;

	// Buffers used for color-processing (BGR pixel format)
	//unsigned char g_srcBuffer[ciMaxImageSize];

	vector<string> filenames;
	FILE*	     arhFile[ciMaxCameras];
	size_t		 ardwBytesWritten[ciMaxCameras];
	Error		 error;

	BusManager	 busMgr;
	Image image;
	FC2Config BufferFrame;
	EmbeddedImageInfo EmbeddedInfo;

	unsigned int no_of_cams_;

	int iCountMissedIm;
	int iImageSize;
	int iFrameNumberPrev[ciMaxCameras];
	int iFrameNumberDelta[ciMaxCameras];

	ImageMetadata imFrameCount[ciMaxCameras];

	FILE* rawFile;

	int createFiles(FILE** arhFile, unsigned int g_uiNumCameras);
	 
	Camera** ppCameras;



public:
	int get_no_of_cams();
	int init();
	void run();
	void cleanup();
	void shutdown();
	CamThread(QObject* parent = NULL);
	~CamThread();

signals:
	void image_ready(FlyCapture2::Image img, int cam_no);

};

