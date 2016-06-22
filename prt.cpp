// prt.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include <sstream>
#include "prt.h"

using namespace std;

int _tmain(int argc, _TCHAR* argv[])
{
	Model prtModel;
	prtModel.Direct();
	//prtModel.Heuristic2();
	//prtModel.ColGen();

	return 1;
}

