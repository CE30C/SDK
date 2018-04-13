//********************************************************************************************************
//* Copyright (C) Benewake (Beijing) Co., Ltd.  All rights reserved.
//* Author: TAN Wenquan
//********************************************************************************************************

#include "CE30-C_SDK.h"

#include <stdio.h>
#include <tchar.h>
#include <math.h>
#pragma comment(lib, "ws2_32.lib")

// define command buff
const char kStart[51] = "getDistanceAndAmplitudeSorted                     "; // start command
const char kStop[51] = "join                                              "; // stop command
const char kDisconnect[51] = "disconnect                                        "; // disconnect command
const char kGreyImage[51] = "enableFeatures 131072                             "; // enable grey image
const char kSetROI[51] = "roi 0 0 3                                         "; // set roi

bool TCP_connect(SOCKET _host, char *_ip)
{
	// set server address
	SOCKADDR_IN server_addr;
	server_addr.sin_family = AF_INET;
	server_addr.sin_addr.s_addr = inet_addr(_ip);
	server_addr.sin_port = htons((short)50660);

	// connect server
	int ret = connect(_host, (LPSOCKADDR)&server_addr, sizeof(server_addr));
	if (ret == SOCKET_ERROR)
	{
		return false;
	}
	else
	{
		return true;
	}
}

bool recv_data(SOCKET _host, char *_buff, int _len)
{
	int total = 0, ret = 0;

	do
	{
		ret = recv(_host, _buff + total, (_len - total), 0);
		if (ret <= 0)
		{
			printf("recv failed! Windows sockets error code: %d\n", WSAGetLastError());
			printf("Please search Windows Socket API WSAGetLastError() for more information.\n");
			return false;
		}

		total += ret;
	} while (total != _len);

	return true;
}

bool send_command(SOCKET _host, const char *_command, int _len)
{
	int ret = 0;

	do
	{
		ret = send(_host, _command, _len, 0);
		if (ret < 0)
		{
			printf("send failed! Windows sockets error code: %d\n", WSAGetLastError());
			printf("Please search Windows Socket API WSAGetLastError() for more information.\n");
			return false;
		}
	} while (ret != _len);

	return true;
}

/******************************************************************************************************/
/* Calibration remap 
/******************************************************************************************************/
float bilinear_interpolation(float _targetX, float _targetY, float _P11, float _P12, float _P21, float _P22)
{
	float alpha_x = _targetX - (int)_targetX;
	float alpha_y = _targetY - (int)_targetY;

	float interpl = _P11 * _P12 * _P21 * _P22;
	if (interpl != 0)
	{
		return ((1 - alpha_x) * (1 - alpha_y) * _P11 + alpha_x * (1 - alpha_y) * _P12 +
			(1 - alpha_x) * alpha_y * _P21 + alpha_x * alpha_y * _P22);
	}
	else
	{
		if (alpha_x < 0.5 && alpha_y < 0.5)
			return _P11;
		else if (alpha_x >= 0.5 && alpha_y < 0.5)
			return _P12;
		else if (alpha_x < 0.5 && alpha_y >= 0.5)
			return _P21;
		else
			return _P22;
	}
}

void remap_CE30(unsigned short *_src, unsigned short *_dst, float *_mapX, float *_mapY, int _h, int _w)
{
	for (int i = 0; i < _h; i++)
	{
		for (int j = 0; j < _w; j++)
		{
			float targetX = _mapX[i * _w + j], targetY = _mapY[i * _w + j];
			int x = (int)targetX, y = (int)targetY;
			if (y >= 0 && y <= HEIGHT - 2)
			{
				unsigned short p11 = _src[y * WIDTH + x];
				unsigned short p12 = _src[y * WIDTH + (x + 1)];
				unsigned short p21 = _src[(y + 1) * WIDTH + x];
				unsigned short p22 = _src[(y + 1) * WIDTH + (x + 1)];
				_dst[i * _w + j] = (unsigned short)bilinear_interpolation(targetX, targetY, p11, p12, p21, p22);
			}
			else
			{
				_dst[i * _w + j] = 0;
			}
		}
	}
}

// inverse matrix (simplified for camera matrix)
void inverse_matrix(double *_A, double *_B, int _n)
{
	int i, j, k;
	float mx, temp, iT;
	double *C = new double[_n * _n];
	for (i = 0; i < _n; i++){
		for (j = 0; j < _n; j++){
			C[i * _n + j] = _A[i * _n + j];
			_B[i * _n + j] = i == j ? 1.0 : 0.0;
		}
	}
	for (i = 0; i < _n; i++)
	{
		/*mx = C[i][i];
		k = i;
		for (j = i + 1; j < _n; j++)
		{
		if (fabs(C[j][i]) > fabs(mx))
		{
		mx = C[j][i];
		k = j;
		}
		}
		if (k != i)
		{
		for (j = 0; j < _n; j++)
		{
		temp = C[i][j];
		C[i][j] = C[k][j];
		C[k][j] = temp;
		temp = _B[i][j];
		_B[i][j] = _b[k][j];
		_B[k][j] = temp;
		}
		}*/
		temp = C[i * _n + i];
		for (j = 0; j < _n; j++)
		{
			C[i * _n + j] = C[i * _n + j] / temp;
			_B[i * _n + j] = _B[i * _n + j] / temp;
		}
		for (j = 0; j < _n; j++)
		{
			if (j != i)
			{
				iT = C[j * _n + i];
				for (k = 0; k < _n; k++)
				{
					C[j * _n + k] = C[j * _n + k] - C[i * _n + k] * iT;
					_B[j * _n + k] = _B[j * _n + k] - _B[i * _n + k] * iT;
				}
			}
		}
	}
}
void init_fisheye_map(double *_CameraMatrix, double *_Coeffs, float *_mapx, float *_mapy, int _height, int _width)
{
	double *NewCameraMatrix = (double*)malloc(sizeof(double) * 3 * 3);
	memcpy(NewCameraMatrix, _CameraMatrix, 9 * sizeof(double));
	NewCameraMatrix[2] = (_width - 1)* 0.5;
	NewCameraMatrix[5] = (_height - 1) * 0.5;
	double *ir = (double*)malloc(sizeof(double) * 3 * 3);
	inverse_matrix(NewCameraMatrix, ir, 3);
	double u0 = _CameraMatrix[2], v0 = _CameraMatrix[5];
	double fx = _CameraMatrix[0], fy = _CameraMatrix[4];
	for (int i = 0; i < _height; i++)
	{
		double _x = i * ir[1] + ir[2], _y = i * ir[4] + ir[5], _w = i * ir[7] + ir[8];
		for (int j = 0; j < _width; j++, _x += ir[0], _y += ir[3], _w += ir[6])
		{
			double x = _x / _w, y = _y / _w;
			double r = sqrt(x*x + y*y);
			double theta = atan(r);
			double theta2 = theta*theta, theta4 = theta2*theta2, theta6 = theta2*theta4, theta8 = theta4*theta4;
			double theta_d = theta * (1 + _Coeffs[0] * theta2 + _Coeffs[1] * theta4 + _Coeffs[2] * theta6 + _Coeffs[3] * theta8);
			double scale = (r == 0) ? 1.0 : theta_d / r;
			double u = fx * x * scale + u0;
			double v = fy * y * scale + v0;
			_mapx[i * _width + j] = (float)u;
			_mapy[i * _width + j] = (float)v;
		}
	}
}

int char_to_int(char *_c)
{
	int value = 0;
	int len = strlen(_c);
	if (len <= 0)
		return INT_MIN;

	for (int i = len - 1; i >= 0; i--)
	{
		if (i == 0 && _c[i] == '-')
		{
			value *= -1;
		}
		else if (_c[i] >= '0' && _c[i] <= '9')
		{
			value += (int)(_c[i] - '0') * pow(10.0, (len - 1 - i));
		}
		else
		{
			return INT_MIN;
		}
	}

	return value;
}

//********************************************************************************************************************
//* SDK function
//********************************************************************************************************************
namespace benewake
{
	char gDistData[TOTALDATA] = { 0 };
	char gAmpData[TOTALDATA] = { 0 };
	char gRecv[4] = { 0 };
	const int gHeight = 24, gWidth = 660;
	float gMapX[gHeight * gWidth] = { 0 }, gMapY[gHeight * gWidth] = { 0 };

	__declspec(dllexport) bool __stdcall initDevice(SOCKET &_host, char *_ip)
	{
		gCameraMatrix[0] = 149.905;
		gCameraMatrix[1] = 0;
		gCameraMatrix[2] = 159.5;
		gCameraMatrix[4] = 150.24;
		gCameraMatrix[5] = 11.5;
		gCameraMatrix[8] = 1.0;

		gDistCoeffs[0] = -0.059868055;
		gDistCoeffs[1] = -0.001303471;
		gDistCoeffs[2] = 0.010260736;
		gDistCoeffs[3] = -0.006102915;

		init_fisheye_map(gCameraMatrix, gDistCoeffs, gMapX, gMapY, gHeight, gWidth);

		// init winsock2 ------------------------------------------------------------------------------------
		WSADATA wsd;
		if (WSAStartup(MAKEWORD(2, 2), &wsd) != 0)
		{
			printf("WSAStartup failed!\n");
			return false;
		}

		// creat socket and connet ---------------------------------------------------------------------------
		_host = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
		if (INVALID_SOCKET == _host)
		{
			WSACleanup();
			printf("Creat socket failed!\n");
			return false;
		}
		
		if (!TCP_connect(_host, _ip))
		{
			closesocket(_host);
			WSACleanup();
			printf("Failed to connect device!\n");
			return false;
		}
		// set device -----------------------------------------------------------------------------------------
		if (!send_command(_host, kSetROI, strlen(kSetROI)))
		{
			closesocket(_host);
			WSACleanup();
			printf("Failed to send set roi command!\n");
			return false;
		}
		if (!recv_data(_host, gRecv, 4))
		{
			closesocket(_host);
			WSACleanup();
			printf("Failed to receive set roi response!\n");
			return false;
		}
		else if (gRecv[0] != 0x00 && gRecv[1] != 0x00 && gRecv[2] != 0x00 && gRecv[3] != 0x00)
		{
			printf("Set ROI error!");
		}
		if (!send_command(_host, kStart, strlen(kStart)))
		{
			closesocket(_host);
			WSACleanup();
			printf("Failed to send start command!\n");
			return false;
		}

		return true;
	}

	__declspec(dllexport) bool __stdcall getDistanceData(SOCKET _host, unsigned short *_dist, unsigned short *_amp)
	{
		if (!recv_data(_host, gDistData, TOTALDATA))
		{
			printf("Failed to receive distance data!\n");
			return false;
		}
		if (!recv_data(_host, gAmpData, TOTALDATA))
		{
			printf("Failed to receive amp data!\n");
			return false;
		}
		recv_data(_host, gRecv, 3);

		const unsigned short* raw_dist = reinterpret_cast<const unsigned short*>(&gDistData[0]);
		const unsigned short* raw_amp = reinterpret_cast<const unsigned short*>(&gAmpData[0]);
		for (int i = 0; i < HEIGHT; i++)
		{
			for (int j = 0; j < WIDTH; j++)
			{
				gRawDistMatrix[i * WIDTH + j] = raw_dist[i * WIDTH + (WIDTH - 1 - j)];
				gRawAmpMatrix[i * WIDTH + j] = raw_amp[i * WIDTH + (WIDTH - 1 - j)];
			}
		}
		remap_CE30(gRawDistMatrix, _dist, gMapX, gMapY, gHeight, gWidth);
		remap_CE30(gRawAmpMatrix, _amp, gMapX, gMapY, gHeight, gWidth);

		return true;
	}

	__declspec(dllexport) bool __stdcall closeDevice(SOCKET &_host)
	{
		if (!send_command(_host, kStop, strlen(kStop)))
		{
			closesocket(_host);
			WSACleanup();
			printf("Failed to send stop command! Please cut off device's power directly.\n");
			return false;
		}
		Sleep(100);
		if (!send_command(_host, kDisconnect, strlen(kDisconnect)))
		{
			closesocket(_host);
			WSACleanup();
			printf("Failed to send disconnect command! Please cut off device's power directly.\n");
			return false;
		}
		return true;
	}

	__declspec(dllexport) bool __stdcall getPointCloud(unsigned short *_depth,
		float *_coordX, float *_coordY, float *_coordZ)
	{
		float center_x = (gWidth - 1) / 2, center_y = (gHeight - 1) / 2;
		float tmp_x = 0, tmp_y = 0;
		for (int i = 0; i < gHeight; i++)
		{
			for (int j = 0; j < gWidth; j++)
			{
				if (_depth[i * gWidth + j] != 0)
				{
					tmp_x = (j - center_x) / gCameraMatrix[0];
					tmp_y = (center_y - i) / gCameraMatrix[4];


					_coordZ[i * gWidth + j] = (float)_depth[i * gWidth + j];
					_coordX[i * gWidth + j] = tmp_x * _coordZ[i * gWidth + j];
					_coordY[i * gWidth + j] = tmp_y * _coordZ[i * gWidth + j];
				}
				else
				{

					_coordZ[i * gWidth + j] = 0;
					_coordX[i * gWidth + j] = 0;
					_coordY[i * gWidth + j] = 0;
				}
			}
		}

		return true;
	}

	__declspec(dllexport) bool __stdcall changeIPAddress(SOCKET _host, char *_newIP)
	{
		send_command(_host, kStop, strlen(kStop));

		const char *delim = " .";
		char *nextField = NULL;
		char *field1 = strtok_s(_newIP, delim, &nextField);
		char *field2 = strtok_s(NULL, delim, &nextField);
		char *field3 = strtok_s(NULL, delim, &nextField);
		char *field4 = strtok_s(NULL, delim, &nextField);
		if (field1 == NULL || field2 == NULL || field3 == NULL || field4 == NULL)
		{
			printf("ERROR: Failed to change IP address! The new IP address has incorrect format!\n");
			send_command(_host, kStart, strlen(kStart));
			return false;
		}
		else
		{
			int f1 = char_to_int(field1);
			int f2 = char_to_int(field2);
			int f3 = char_to_int(field3);
			int f4 = char_to_int(field4);
			if (f1 < 0 || f1 > 255 ||
				f2 < 0 || f2 > 255 ||
				f3 < 0 || f3 > 255 ||
				f4 < 0 || f4 > 255)
			{
				printf("ERROR: Failed to change IP address! The fields must be value from 0 to 255!\n");
				send_command(_host, kStart, strlen(kStart));
				return false;
			}
			else
			{
				char setIP[51] = "ipconfig ";
				strcat_s(setIP, field1);
				strcat_s(setIP, " ");
				strcat_s(setIP, field2);
				strcat_s(setIP, " ");
				strcat_s(setIP, field3);
				strcat_s(setIP, " ");
				strcat_s(setIP, field4);
				int fillSpaceNum = 50 - 12 - strlen(field1) - strlen(field2) - strlen(field3) - strlen(field4);
				for (int i = 0; i < fillSpaceNum; i++)
				{
					strcat_s(setIP, " ");
				}

				if (!send_command(_host, setIP, 50))
				{
					closesocket(_host);
					WSACleanup();
					printf("ERROR: Failed to send change IP command! Please cut off device's power and try again.\n");
					return false;
				}
				recv_data(_host, gRecv, 4);
				if (gRecv[0] == 0x00 && gRecv[1] == 0x00 && gRecv[2] == 0x00 && gRecv[3] == 0x00)
				{
					printf("New IP address is set! The LiDAR is rebooting.\n");
					return true;
				}
				else
				{
					printf("ERROR: Failed to change IP address! The command is rejected.");
					send_command(_host, kStart, strlen(kStart));
					return false;
				}
			}
		}
	}
}