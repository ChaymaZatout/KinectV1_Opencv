#include "pch.h"
#include<iostream>
#include <Windows.h>

//opencv header:
#include<opencv2/opencv.hpp>

//kinect headers:
#include <NuiApi.h>
#include <NuiImageCamera.h>
#include <NuiSensor.h>

using namespace std;
using namespace cv;

#define width 640    
#define height 480       

//global variables for kinect:
HRESULT kinectHr; //kinect sensor.
HANDLE rgbStreamEvent, depthStreamEvent; //Events for cameras to wait 
HANDLE  rgbStream, depthStream; //The identifiers of the Kinect's RGB and IR Cameras

bool initKinect();
bool getRGBImage(Mat* rgbImage);
bool getDepthPixelImage(Mat* depthPixelImage); // get depth pixel (depth+ squelette bits)
bool getDepthImage(Mat* depthImage); // get from depth pixel, the depthonly in mm
bool getDepthIntensityImage(Mat * depthImage); // get depth intensity in CV_8UC3
void fromImageDepthToDepthImageIntensityImage(const Mat depthImage, Mat* depthIntensityImage);

int main(int argc, char * argv[]){

	//initialise kinect and terminate if it's not initialized:
	rgbStream = NULL;
	depthStream = NULL;
	kinectHr = NULL;
	if (!initKinect()) {
		cout << "main: Kinect initialization error!" << endl;
		return 1;
	}
	cout << "main: Kinect initialized with succes!" << endl;
	
	//initialize opencv images:
	Mat rgbImage = Mat(Size(width, height), CV_8UC4);
	Mat depthPixelImage = Mat(Size(width, height), CV_16U);
	Mat depthImage = Mat(Size(width, height), CV_16U);
	Mat depthIntensityImage = Mat(Size(width, height), CV_8UC3);
	Mat dIntensityImage = Mat(Size(width, height), CV_8UC3);
	

	namedWindow("depth image", WINDOW_AUTOSIZE);
	namedWindow("intensity image", WINDOW_AUTOSIZE);
	namedWindow("color image", CV_WINDOW_AUTOSIZE);
	while (1)
	{
		getRGBImage(&rgbImage);
		imshow("color image", rgbImage);

		getDepthImage(&depthImage);
		imshow("depth image", depthImage);

		fromImageDepthToDepthImageIntensityImage(depthImage, &dIntensityImage);
		imshow("intensity image", dIntensityImage);

		//exit
		int c = cvWaitKey(1);
		if (c == 27 || c == 'q' || c == 'Q')
			break;
	}

	cvDestroyAllWindows();
	NuiShutdown();

	return 0;

}

//init kinect to get both: rgb and depth image:
bool initKinect() {
	// initialize kinect:
	kinectHr = NuiInitialize(
		NUI_INITIALIZE_FLAG_USES_DEPTH_AND_PLAYER_INDEX
		| NUI_INITIALIZE_FLAG_USES_COLOR);
	if (kinectHr != S_OK){
		cout << "initKinect: NuiInitialize failed" << endl;
		return false;
	}

	//rgb stream initialization:
	rgbStreamEvent = CreateEvent(NULL, TRUE, FALSE, NULL);// initialize an event to wait for rgb stream.
	kinectHr = NuiImageStreamOpen(
		NUI_IMAGE_TYPE_COLOR, // Depth camera or rgb camera?
		NUI_IMAGE_RESOLUTION_640x480, // Image resolution
		0, // Image stream flags, e.g. near mode
		2, // Number of frames to buffer
		rgbStreamEvent, // Event handle
		&rgbStream);
	if (FAILED(kinectHr)){
		cout << "initKinect: Could not open image stream" << endl;
		return false;
	}

	//depth stream initialization:
	depthStreamEvent = CreateEvent(NULL, TRUE, FALSE, NULL); // initialize an event to wait for depth stream.
	kinectHr = NuiImageStreamOpen(
		NUI_IMAGE_TYPE_DEPTH_AND_PLAYER_INDEX, // Depth camera or rgb camera?
		NUI_IMAGE_RESOLUTION_640x480, // Image resolution
		0, // Image stream flags, e.g. near mode
		2, // Number of frames to buffer
		depthStreamEvent, // Event handle
		&depthStream);
	if (FAILED(kinectHr))
	{
		cout << "initKinect: Could not open depth stream" << endl;
		return false;
	}

	return true;
}

bool getRGBImage(Mat* rgbImage)
{
	WaitForSingleObject(rgbStreamEvent, INFINITE);

	//fetch and lock the frame to prevent corruption while reading:
	const NUI_IMAGE_FRAME * imageFrame = NULL; // contains all the metadata about the frame: the number, resolution, etc
	NUI_LOCKED_RECT LockedRect; // contains a pointer to the actual data
	if (FAILED(NuiImageStreamGetNextFrame(rgbStream, 0, &imageFrame))){ //Get Image Frame Failed
		cout << "getRGBImage: Get Image Frame Failed" << endl;
		return false;
	}
	INuiFrameTexture * texture = imageFrame->pFrameTexture; //manages the frame data
	texture->LockRect(0, &LockedRect, NULL, 0);

	//Read values and fill the imgColor:
	INT colorPitch = LockedRect.Pitch;
	if (colorPitch != 0){ //check frame is not empty.
		BYTE * pBuffer = (BYTE*)LockedRect.pBits; //get color data.

		// Copy image information into Mat:
		for (INT y = 0; y < height; ++y){
			// Get row pointer for color Mat
			Vec4b* pColorRow = rgbImage->ptr<Vec4b>(y); // Get row pointer for color Mat
			for (INT x = 0; x < width; ++x){
				pColorRow[x] = Vec4b(pBuffer[y * colorPitch + x * 4 + 0],
					pBuffer[y * colorPitch + x * 4 + 1],
					pBuffer[y * colorPitch + x * 4 + 2],
					pBuffer[y * colorPitch + x * 4 + 3]);
			}
		}
	}

	//release the frame so that the Kinect can use it again:
	texture->UnlockRect(0);
	NuiImageStreamReleaseFrame(rgbStream, imageFrame);
	return true;
}

bool getDepthImage(Mat* depthImage) {
	WaitForSingleObject(depthStreamEvent, INFINITE);

	const NUI_IMAGE_FRAME * imageFrame = NULL; // contains all the metadata about the frame: the number, resolution, etc
	NUI_LOCKED_RECT LockedRect; // contains a pointer to the actual data
	if (FAILED(NuiImageStreamGetNextFrame(depthStream, 0, &imageFrame))) {//Get Image Frame Failed
		cout << "getDepthPixelImage: Get Image Frame Failed" << endl;
		return false;
	}
	INuiFrameTexture * texture = imageFrame->pFrameTexture; //manages the frame data
	texture->LockRect(0, &LockedRect, NULL, 0);

	if (LockedRect.Pitch != 0) { //check frame is not empty.
		USHORT * pBuff = (USHORT*)LockedRect.pBits;
		for (INT y = 0; y < height; ++y) {
			// Get row pointer for depth Mat
			USHORT* pDepthRow = depthImage->ptr<USHORT>(y);
			for (INT x = 0; x < width; ++x) {
				pDepthRow[x] = NuiDepthPixelToDepth(pBuff[y * width + x]); //get depth value in mm.
			}
		}
	}

	//release the frame so that the Kinect can use it again:
	texture->UnlockRect(0);
	NuiImageStreamReleaseFrame(depthStream, imageFrame);

	return true;
}

bool getDepthIntensityImage(Mat * depthImage) {
	WaitForSingleObject(depthStreamEvent, INFINITE);

	const NUI_IMAGE_FRAME * imageFrame = NULL; // contains all the metadata about the frame: the number, resolution, etc
	NUI_LOCKED_RECT LockedRect; // contains a pointer to the actual data
	if (FAILED(NuiImageStreamGetNextFrame(depthStream, 0, &imageFrame))) {//Get Image Frame Failed
		cout << "getDepthImage: Get Image Frame Failed" << endl;
		return false;
	}
	INuiFrameTexture * texture = imageFrame->pFrameTexture; //manages the frame data
	texture->LockRect(0, &LockedRect, NULL, 0);

	if (LockedRect.Pitch != 0) { //check frame is not empty.
		USHORT * pBuff = (USHORT*)LockedRect.pBits; //get pixel depth map
		for (INT y = 0; y < height; ++y) {
			// Get row pointer for depth Mat
			Vec3b* pDepthRow = depthImage->ptr<Vec3b>(y);
			for (INT x = 0; x < width; ++x) {
				USHORT depthPixel = pBuff[y * width + x]; //get depth pixel
				if (depthPixel != 65535) {
					SHORT realDepth = NuiDepthPixelToDepth(depthPixel); //get depth value in mm.
					// Convert depth info into an intensity for display
					BYTE  depth = 255 - static_cast<BYTE>(256 * realDepth / 0x0fff);
					pDepthRow[x] = Vec3b(depth, depth, depth);
				}
				else {
					pDepthRow[x] = 0;
				}
			}
		}
	}

	//release the frame so that the Kinect can use it again:
	texture->UnlockRect(0);
	NuiImageStreamReleaseFrame(depthStream, imageFrame);

	return true;
}

bool getDepthPixelImage(Mat* depthPixelImage)
{
	WaitForSingleObject(depthStreamEvent, INFINITE);

	const NUI_IMAGE_FRAME * imageFrame = NULL; // contains all the metadata about the frame: the number, resolution, etc
	NUI_LOCKED_RECT LockedRect; // contains a pointer to the actual data
	if (FAILED(NuiImageStreamGetNextFrame(depthStream, 0, &imageFrame))) {//Get Image Frame Failed
		cout << "getDepthPixelImage: Get Image Frame Failed" << endl;
		return false;
	}
	INuiFrameTexture * texture = imageFrame->pFrameTexture; //manages the frame data
	texture->LockRect(0, &LockedRect, NULL, 0);

	if (LockedRect.Pitch != 0){ //check frame is not empty.
		USHORT * pBuff = (USHORT*)LockedRect.pBits;
		for (INT y = 0; y < height; ++y) {
			// Get row pointer for depth Mat
			USHORT* pDepthRow = depthPixelImage->ptr<USHORT>(y);
			for (INT x = 0; x < width; ++x){
				pDepthRow[x] = pBuff[y * width + x];
			}
		}
	}

	//release the frame so that the Kinect can use it again:
	texture->UnlockRect(0);
	NuiImageStreamReleaseFrame(depthStream, imageFrame);

	return true;
}

void fromImageDepthToDepthImageIntensityImage(const Mat depthImage, Mat* depthIntensityImage) {
	for (int i = 0; i < height; i++) {
		//Vec3b* pDepthRow = depthIntensityImage->ptr<Vec3b>(i);
		for (int j = 0; j < width; j++) {
			BYTE value = 255 - static_cast<BYTE>(256 * depthImage.at<USHORT>(i, j) / 0x0fff);
			(*depthIntensityImage).at<Vec3b>(i, j) = Vec3b(value, value, value);
		}
	}
}