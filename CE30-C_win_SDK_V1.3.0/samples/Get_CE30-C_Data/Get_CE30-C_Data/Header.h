#include <stdio.h>
#include <signal.h>
#include <fstream>
#include <iostream>
#include <conio.h>

#include "..\..\..\build\include\CE30-C_SDK.h"

#ifdef _DEBUG
#pragma comment(lib, "..\\..\\..\\build\\x86\\vc12\\debug\\CE30-C_win_SDKd.lib")
#else
#pragma comment(lib, "..\\..\\..\\build\\x86\\vc12\\release\\CE30-C_win_SDK.lib")
#endif

void get_enter()
{
	std::cin.clear();
	std::cin.sync();
	while (_getch() != '\r')
	{
	};
}


