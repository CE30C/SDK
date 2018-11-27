#include "Header.h"
#include <atltime.h>


#define DEBUG
//#define OPENCV
#ifdef OPENCV
#include <opencv2\highgui.hpp>
#include <opencv2\imgproc.hpp>

void defineRGB(unsigned short _dist, uchar* _R, uchar* _G, uchar* _B)
{
	if (_dist <= 0)
	{
		*_R = 0;
		*_G = 0;
		*_B = 0;
	}
	else if (_dist <= 60)
	{
		*_R = 0;
		*_G = 0;
		*_B = (_dist - 0) * 4.25;
	}
	else if (_dist <= 120)
	{
		*_R = 0;
		*_G = (_dist - 60) * 4.25;
		*_B = 255;
	}
	else if (_dist <= 180)
	{
		*_R = 0;
		*_G = 255;
		*_B = 255 - (_dist - 120) * 4.25;
	}
	else if (_dist <= 240)
	{
		*_R = (_dist - 180) * 4.25;
		*_G = 255;
		*_B = 0;
	}
	else if (_dist <= 300)
	{
		*_R = 255;
		*_G = 255 - (_dist - 240) * 4.25;
		*_B = 0;
	}
	else if (_dist > 300)
	{
		*_R = 255;
		*_G = 0;
		*_B = 0;
	}
}
#endif // OPENCV

int main()
{
	char ip[20];
	std::printf("Enter LiDAR's IP address: ");
	std::cin.clear();
	std::cin.sync();
	std::cin >> ip;
	printf("Init device ...\n\n");
	SOCKET host;
	if (!benewake::initDevice(host, ip))
	{
		printf("Initialization error!\n");
		printf("Press ENTER to exit.\n");
		get_enter();
		return -1;
	}
#if 1
	printf("Camera intrinsic matrix:\n");
	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			printf("%.3f  ", benewake::gCameraMatrix[i * 3 + j]);
		}
		printf("\n");
	}
	printf("Distortion coefficients:\n");
	for (int i = 0; i < 4; i++)
	{
		printf("%.6f  ", benewake::gDistCoeffs[i]);
	}
	printf("\n");
#endif 

	unsigned short distData[24 * 660], ampData[24 * 660];
	int i = 0;
	float coord_x[24 * 660], coord_y[24 * 660], coord_z[24 * 660];

	int times = 0;
	std::printf("Enter measurement times (0 for continuous): ");
	std::cin.clear();
	std::cin.sync();
	std::cin >> times;
	if (!benewake::startMeasurement(host, times))
	{
		printf("Start measurement failed!\n");
		printf("Press ENTER to exit.\n");
		get_enter();
		return -1;
	}

	printf("\nStart processing ...\n");
	printf("Press [Q] to stop processing.\n");
	printf("Press [C] to change LiDAR's IP address.\n");
	bool run = true;
	DWORD Tstart, Tend;
	int nFrame = 0;
	while (run && (nFrame < times || times == 0))
	{
#ifdef DEBUG
		Tstart = GetTickCount();
#endif //DEBUG
		if (!benewake::getDistanceData(host, distData, ampData))
		{
			benewake::closeDevice(host);
			printf("Press ENTER to exit.\n");
			get_enter();
			return -1;
		}
		nFrame++;

		// TODO: 
		// do something with distance and amp data
		// ...
#ifdef OPENCV
		cv::Mat depthImg = cv::Mat::zeros(cv::Size(660, 24), CV_8UC3);;
		uchar* pt = depthImg.ptr<uchar>(0);
		for (int i = 0; i < 24; i++) {
			pt = depthImg.ptr<uchar>(i);
			for (int j = 0; j < 660; j++) {
				defineRGB(distData[i * 660 + j], &pt[j * 3 + 2], &pt[j * 3 + 1], &pt[j * 3 + 0]);
			}
		}
		cv::imshow("Depth image", depthImg);
		char cvkey = cv::waitKey(10);
#endif // OPENCV

		
		benewake::getPointCloud(distData, coord_x, coord_y, coord_z);

		// TODO: 
		// do something with point cloud
		// ...
		// In real use, it is not recommend that call getPointCloud() in a loop, for it is timeconsuming.
		// It is better to use another thread to process it or transplant it to GPU 

		if (_kbhit())
		{
			char key = _getch();
			switch (key)
			{
			case 'q':
			case 'Q':
				run = false;
				break;
			case 'c':
			case 'C':
			{
				char ip[20];
				std::printf("Enter New IP address: ");
				std::cin.clear();
				std::cin.sync();
				std::cin >> ip;

				if (benewake::changeIPAddress(host, ip))
				{
					printf("Press ENTER to exit.");
					get_enter();
					return 0;
				}
				break;
			}
			default:
				break;
			}
		}

#ifdef DEBUG
		Tend = GetTickCount();
		printf("Time to receive No.%d frame: %d ms\n", nFrame, (Tend - Tstart));
#endif	// DEBUG
	}

	if (!benewake::stopMeasurement(host))
	{
		printf("Press ENTER to exit.\n");
		get_enter();
		return -1;
	}

	if (!benewake::closeDevice(host))
	{
		printf("Press ENTER to exit.\n");
		get_enter();
		return -1;
	}

	printf("Device is successfully stopped and disconnected!\n");
	printf("Press ENTER to exit.\n");
	get_enter();
	return 0;
}