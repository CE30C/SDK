//********************************************************************************************************
//* Copyright (C) Benewake (Beijing) Co., Ltd.  All rights reserved.
//* Author: TAN Wenquan
//********************************************************************************************************
#include <windows.h>

#ifdef CE30C_WIN_SDK_EXPORTS
#define CE30CSDK_API __declspec(dllexport)
#else
#define CE30CSDK_API __declspec(dllimport)
#endif

#define WIDTH 320
#define HEIGHT 24
#define TOTALDATA 2 * WIDTH * HEIGHT

namespace benewake
{
#ifdef CE30C_WIN_SDK_EXPORTS
	EXTERN_C CE30CSDK_API double gCameraMatrix[9] = { 0.0 }, gDistCoeffs[4] = { 0.0 };
	EXTERN_C CE30CSDK_API unsigned short gRawDistMatrix[HEIGHT * WIDTH] = { 0 }, gRawAmpMatrix[HEIGHT * WIDTH] = { 0 };
#else
	EXTERN_C CE30CSDK_API double gCameraMatrix[9], gDistCoeffs[4];
	EXTERN_C CE30CSDK_API unsigned short gRawDistMatrix[HEIGHT * WIDTH], gRawAmpMatrix[HEIGHT * WIDTH];
#endif

	// @brief Establish Ethernet connection and initiallize LiDAR.
	// @param _host Socket handle.
	// @param _ip IP address of LiDAR.
	EXTERN_C CE30CSDK_API bool __stdcall initDevice(SOCKET &_host, char *_ip);

	// @brief Start several times measurement
	// @param _host Socket handle.
	// @param _times The times of measurement. When set to 0, the measurement will be continuous untill the LiDAR receive stop 
	// measurement command. Otherwise, the measurement will automatically stop when the times of measurement is achieved.
	EXTERN_C CE30CSDK_API bool __stdcall startMeasurement(SOCKET &_host, int _times);

	// @brief Get distance and signal strength of LiDAR's FoV.
	// @param _host Socket handle.
	// @param _dist Distance data buffer, the size should be 24[height] * 660[width].
	// @param _amp Distance data buffer, the size should be 24[height] * 660[width].
	// The data sequence is from field of view's left to right and then from top to bottom.
	EXTERN_C CE30CSDK_API bool __stdcall getDistanceData(SOCKET _host, 
		unsigned short *_dist, unsigned short *_amp);

	// @brief Stop measurement.
	// @param _host Socket handle.
	EXTERN_C CE30CSDK_API bool __stdcall stopMeasurement(SOCKET _host);

	// @brief Disconnect.
	// @param _host Socket handle.
	EXTERN_C CE30CSDK_API bool __stdcall closeDevice(SOCKET &_host);

	// @brief Transform distance data into point cloud .
	// @param _depth Distance data buffer.
	// @param _coordX x-axis coordinates of point cloud, the direction is from left to right.
	// @param _coordY y-axis coordinates of point cloud, the direction is from bottom to top.
	// @param _coordZ z-axis coordinates of point cloud, the direction is from near to far.
	// All params should have the same size of 24[height] * 660[width], and the same sequence number params construct a space point 
	// (_coordX[n], _coordY[n], _coordZ[n]). The data sequence is from field of view's left to right and then from top to bottom.
	EXTERN_C CE30CSDK_API bool __stdcall getPointCloud(unsigned short *_depth,
		float *_coordX, float *_coordY, float *_coordZ);

	// @brief Change LiDAR's IP Address.
	// @param _host Socket handle.
	// @param _newIP New IP address.
	EXTERN_C CE30CSDK_API bool __stdcall changeIPAddress(SOCKET _host, char *_newIP);
}