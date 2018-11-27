/**************************************************************************************************************************************
/* Benewake imaging LiDAR CE30-C SDK, Windows Version
/* Version 1.3.0
/**************************************************************************************************************************************

/**************************************************************************************************************************************
/* Files & directories
/**************************************************************************************************************************************
  build		SDK header and .dll & .lib files
  samples:	SDK application example
  sources:	SDK sources

/**************************************************************************************************************************************
/* Global values
/**************************************************************************************************************************************
  double gCameraMatrix[]			LiDAR intrinsic parameters, in order {f/dx, skew, u0, 0, f/dy, v0, 0, 0, 1}
  double gDistCoffs[]				LiDAR distortion parameters, in order {k1, k2, k3, k4}
  unsigned short gRawDistMatrix[]	LiDAR original distance data
  unsigned short gRawAmpMatrix[]	LiDAR original amp data

/**************************************************************************************************************************************
/* References
/**************************************************************************************************************************************
  bool initDevice(SOCKET &_host, char *_ip);
    _host	socket handle
	_ip		IP address of LiDAR
  Initialize TCP connection, camera matrix, distortion coefficients, projection map.
  
  bool startMeasurement(SOCKET _host, int _times);
    _host 	Socket handle.
    _times 	The times of measurement. 
  Start several times measurement. When set parameter _times to 0, the measurement will be continuous untill the LiDAR receive stop 
  measurement command. Otherwise, the measurement will automatically stop when the times of measurement is achieved.

  bool getDistanceData(SOCKET _host, unsigned short *_dist, unsigned short *_amp);
    _host	socket handle
    _dist	LiDAR distance data buffer, the size should be 24[height] * 660[width]
    _amp	LiDAR amp data buffer, the size should be 24[height] * 660[width]
  Get one frame of CE30's original data and put into global value 'gRawDistMatrix[]', and its resolution is 24*320. Calibrate the data 
  and put undistorted data into data buffer '_dst', and the resolution changes to 24*660 due to calibration.

  bool stopMeasurement(SOCKET _host);
    _host 	Socket handle
  Stop CE30's measurement.
  
  bool closeDevice(SOCKET &_host);
    _host	socket handle
  Disable TCP connection.
  
  bool getPointCloud(unsigned short *_dist, float *_coordX, float *_coordY, float *_coordZ);
    _dist	depth data, get from getDistanceData()
    _coordX	x-axis coordinates of point cloud, the direction is from left to right
    _coordY	y-axis coordinates of point cloud, the direction is from bottom to top
    _coordZ	z-axis coordinates of point cloud, the direction is from near to far
  All params should have the same size of 24[height] * 660[width], and the same sequence number params construct a space point (_coordX[n],
  _coordY[n], _coordZ[n]). 
  The data sequence is from field of view's left to right and then from top to bottom.
  
  bool changeIPAddress(int _sd, char *_newIP);
    _sd		socke handle
    _newIP	new IP address fields
  Change LiDAR's IP address to _newIP. The LiDAR will reboot if succeed.
  
/**************************************************************************************************************************************
/* How to use
/**************************************************************************************************************************************
Option 1:
  Add codes (.h and .cpp files under ./sources) in your projects.

Option 2:
  Compile the project to generate a dynamic link library (.dll & .lib). Add it to your projects as showed in the samples.