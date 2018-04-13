#include "Header.h"
#include <atltime.h>

#define DEBUG

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

	printf("\nStart processing ...\n");
	printf("Press [Q] to stop processing.\n");
	printf("Press [C] to change LiDAR's IP address.\n");
	bool run = true;
	DWORD Tstart, Tend;
	while (run)
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

		// TODO: 
		// do something with distance and amp data
		// ...
		
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
		printf("time: %d\n", (Tend - Tstart));
#endif	// DEBUG
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