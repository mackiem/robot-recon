#include "camthread.h"



const string CamThread::csDestinationDirectory = "";


CamThread::CamThread(QObject* parent)
{
	init();
}


CamThread::~CamThread()
{
	//cleanup();
}

void CamThread::cleanup() {
	for (unsigned int uiCamera = 0; uiCamera < no_of_cams_; uiCamera++)
	{
		ppCameras[uiCamera]->StopCapture();
		ppCameras[uiCamera]->Disconnect();
		delete ppCameras[uiCamera];
	}

	delete[] ppCameras;
}

int CamThread::createFiles(FILE** arhFile, unsigned int g_uiNumCameras)
{
	for (unsigned int uiCamera = 0; uiCamera < g_uiNumCameras; uiCamera++)
	{
		stringstream sstream;
		string tmpfilename;

		sstream << csDestinationDirectory << "camera" << uiCamera << ".tmp";
		sstream >> tmpfilename;
		filenames.push_back(tmpfilename);

		std::cout << "Creating " << tmpfilename << "..." << endl;

		// Create temporary files to do writing to
		arhFile[uiCamera] = fopen(tmpfilename.c_str(), "w+");
		if (arhFile[uiCamera] == NULL)
		{
			assert(false);
			return -1;
		}
	}
	return 0;
}


void PrintBuildInfo()
{
	FC2Version fc2Version;
	Utilities::GetLibraryVersion(&fc2Version);

	std::cout << "FlyCapture2 library version: "
		<< fc2Version.major << "." << fc2Version.minor << "."
		<< fc2Version.type << "." << fc2Version.build << endl << endl;

	std::cout << "Application build date: " << __DATE__ << ", " << __TIME__ << endl << endl;
}

void PrintCameraInfo(CameraInfo* pCamInfo)
{
	std::cout << "\n*** CAMERA INFORMATION ***\n"
		<< "Serial number - " << pCamInfo->serialNumber << endl
		<< "Camera model - " << pCamInfo->modelName << endl
		<< "Camera vendor - " << pCamInfo->vendorName << endl
		<< "Sensor - " << pCamInfo->sensorInfo << endl
		<< "Resolution - " << pCamInfo->sensorResolution << endl
		<< "Firmware version - " << pCamInfo->firmwareVersion << endl
		<< "Firmware build time - " << pCamInfo->firmwareBuildTime << endl << endl;
}

void PrintError(Error error)
{
	error.PrintErrorTrace();
}

int CamThread::get_no_of_cams() {
	return no_of_cams_;
}

int CamThread::init()
{

	iCountMissedIm = 0;
	iImageSize = 0;

	PrintBuildInfo();


	error = busMgr.GetNumOfCameras(&no_of_cams_);
	if (error != PGRERROR_OK)
	{
		PrintError(error);
		return -1;
	}

	std::cout << "Number of cameras detected: " << no_of_cams_ << endl << endl;

	if (no_of_cams_ < 1)
	{
		std::cout << "Insufficient number of cameras... press Enter to exit." << endl;
		cin.ignore();
		return -1;
	}

	// Create files to write to
	if (createFiles(arhFile, no_of_cams_) != 0)
	{
		std::cout << "There was error creating the files... press Enter to exit.";
		cin.ignore();
		return -1;
	}

	ppCameras = new Camera*[no_of_cams_];

	// Connect to all detected cameras and attempt to set them to
	// the same video mode and frame rate
	for (unsigned int uiCamera = 0; uiCamera < no_of_cams_; uiCamera++)
	{
		ppCameras[uiCamera] = new Camera();

		PGRGuid guid;
		error = busMgr.GetCameraFromIndex(uiCamera, &guid);
		if (error != PGRERROR_OK)
		{
			PrintError(error);
			return -1;
		}

		// Connect to a camera
		error = ppCameras[uiCamera]->Connect(&guid);
		if (error != PGRERROR_OK)
		{
			PrintError(error);
			return -1;
		}

		// Get the camera information
		CameraInfo camInfo;
		error = ppCameras[uiCamera]->GetCameraInfo(&camInfo);
		if (error != PGRERROR_OK)
		{
			PrintError(error);
			return -1;
		}

		PrintCameraInfo(&camInfo);

		// Get Format7 info
		Format7Info fmt7info;
		bool fmt7supported;
		error = ppCameras[uiCamera]->GetFormat7Info(&fmt7info, &fmt7supported);

		if (error != PGRERROR_OK) { 
			PrintError(error);
			return -1;
		}

		if (!fmt7supported) { 
			PrintError(error);
			return -1;
		}

		// Setup image format
		Format7ImageSettings fmt7settings;
		fmt7settings.mode = MODE_1;
		fmt7settings.offsetX = 156;
		fmt7settings.offsetY = 92;
		//fmt7settings.offsetX = 0;
		//fmt7settings.offsetY = 0;
		//fmt7settings.width = fmt7info.maxWidth;
		//fmt7settings.height = fmt7info.maxHeight;
		//fmt7settings.pixelFormat = PIXEL_FORMAT_RAW8;
		fmt7settings.width = 200;
		fmt7settings.height = 150;
		fmt7settings.pixelFormat = PIXEL_FORMAT_MONO8;

		// Validate format
		bool fmt7valid;
		Format7PacketInfo fmt7packetInfo;
		error = ppCameras[uiCamera]->ValidateFormat7Settings(&fmt7settings, &fmt7valid, &fmt7packetInfo);
		if (error != PGRERROR_OK)  
		{
			PrintError(error);
			return -1;
		}
		if (!fmt7valid) {
			PrintError(error);
			return -1;
		}
		// Apply image format
		error = ppCameras[uiCamera]->SetFormat7Configuration(&fmt7settings, fmt7packetInfo.maxBytesPerPacket);
		if (error != PGRERROR_OK) {
			PrintError(error);
			return -1;
		}

		// Set video mode and frame rate here!!!
		/*error = ppCameras[uiCamera]->SetVideoModeAndFrameRate(VIDEOMODE_1024x768Y8, FRAMERATE_1_875);
		if (error != PGRERROR_OK)
		{
			PrintError(error);
			std::cout << "Error starting cameras." << endl
				<< "This example requires cameras to be able to set to the same video mode and frame rate." << endl
				<< "If your cameras do not support the requested mode, please edit the source code and recompile the application." << endl
				<< "Press Enter to exit." << endl;

			cin.ignore();
			return -1;
		}*/
	}

	for (unsigned int uiCamera = 0; uiCamera < no_of_cams_; uiCamera++)
	{
		error = ppCameras[uiCamera]->GetConfiguration(&BufferFrame);
		if (error != PGRERROR_OK)
		{
			PrintError(error);
			return -1;
		}

		BufferFrame.numBuffers = 200;

		BufferFrame.grabMode = BUFFER_FRAMES;

		error = ppCameras[uiCamera]->SetConfiguration(&BufferFrame);
		if (error != PGRERROR_OK)
		{
			PrintError(error);
			return -1;
		}

		error = ppCameras[uiCamera]->GetEmbeddedImageInfo(&EmbeddedInfo);
		if (error != PGRERROR_OK)
		{
			PrintError(error);
			return -1;
		}

		if (EmbeddedInfo.timestamp.available == true)
		{
			EmbeddedInfo.timestamp.onOff = true;
		}
		else
		{
			std::cout << "Timestamp is not available!" << endl;
		}

		if (EmbeddedInfo.frameCounter.available == true)
		{
			EmbeddedInfo.frameCounter.onOff = true;
		}
		else
		{
			std::cout << "Framecounter is not avalable!" << endl;
		}

		error = ppCameras[uiCamera]->SetEmbeddedImageInfo(&EmbeddedInfo);
		if (error != PGRERROR_OK)
		{
			PrintError(error);
			return -1;
		}



		error = ppCameras[uiCamera]->StartCapture();
		if (error != PGRERROR_OK)
		{
			PrintError(error);
			std::cout << "Error starting to capture images." << endl
				<< "Press Enter to exit." << endl;
			cin.ignore();
			return -1;
		}
	}
	return 0;
}

void CamThread::shutdown() {
	is_shutting_down_ = true;
}

void CamThread::run() {
	std::cout << "Grabbing ...";

	// grabbing all images

	long long iImages = 0;
	is_shutting_down_ = false;
	while (true) {
		//for (int iImages = 0; iImages < k_numImages; iImages++)
			//{
				// grabbing for all cameras
				for (unsigned int uiCamera = 0; uiCamera < no_of_cams_; uiCamera++)
				{
					error = ppCameras[uiCamera]->RetrieveBuffer(&image);
					if (error != PGRERROR_OK)
					{
						PrintError(error);
						continue;
						//return -1;
					}

					emit image_ready(image, uiCamera);

					////Get the size of the buffer associated with the image, in bytes. Returns the size of the buffer in bytes.
					//iImageSize = image.GetDataSize();

					//// Write to the file
					//ardwBytesWritten[uiCamera] = fwrite(image.GetData(), 1, image.GetCols()*image.GetRows(), arhFile[uiCamera]);

					//// Ensure that the write was successful
					//if (ardwBytesWritten[uiCamera] != (unsigned)iImageSize)
					//{
					//	std::cout << "Error writing to file for camera " << uiCamera << " !" << endl;
					//	continue;
					//	//return -1;
					//}

					//imFrameCount[uiCamera] = image.GetMetadata();

					//// Keep track of the difference in image sequence numbers (iFrameCount)
					//// in order to determine if any images have been missed.  A difference
					//// greater than 1 indicates that an image has been missed.

					//if (iImages == 0)
					//{
					//	// This is the first image, set up the variables
					//	iFrameNumberPrev[uiCamera] = imFrameCount[uiCamera].embeddedFrameCounter;

					//	iFrameNumberDelta[uiCamera] = 1;
					//}
					//else
					//{
					//	// Get the difference in sequence numbers between the current
					//	// image and the last image we received
					//	iFrameNumberDelta[uiCamera] = imFrameCount[uiCamera].embeddedFrameCounter - iFrameNumberPrev[uiCamera];
					//}

					//if (iFrameNumberDelta[uiCamera] != 1)
					//{
					//	iCountMissedIm += iFrameNumberDelta[uiCamera] - 1;
					//}

					//iFrameNumberPrev[uiCamera] = imFrameCount[uiCamera].embeddedFrameCounter;
				}
					if (is_shutting_down_) {
						break;
					}
					iImages++;
					//if (iImages > 500) {
					//	break;
					//}
			//}

		//std::cout << endl;

		//std::cout << "We missed " << iCountMissedIm << " images!" << endl << endl;
	}
	cleanup();

}