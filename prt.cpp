// prt.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include <sstream>
#include "prt.h"

using namespace std;

int main(int argc, char *argv[])
{
	Model prtModel;
	prtModel.InitModel(argc, argv);
	prtModel.Direct();
	//prtModel.Heuristic2();
	//prtModel.ColGen();

	prtModel.freeMem();
	return 1;
}

Model::Model(void)
{
	i = 0;
	j = 0;
	k = 0;
	v = 0;
	t = 0;
	tau = 0;
	l = 0;
	n = 0;
	N = 0;
	nArc= 0;
	nVeh = 0;
	T = 0;
	nTrack = 0;
	buf = new char [33];
	path = new char [200];
	directoryPath.clear();
}

void Model::InitModel(int argc, char **argv)
{
	Reader rd;

	for(i = 1; i < argc; i++)
	{
		directoryPath += argv[i];
	}
	if(directoryPath.empty())
	{
		directoryPath = "./";
	}

	cout << directoryPath << endl; 

	string arcFile = directoryPath + "input/Arc.csv";
	string vehFile = directoryPath + "input/Vehicle.csv";
	string trackFile = directoryPath + "input/Track.csv";
	string dmdFile = directoryPath + "input/Demand.csv";

	rd.readArcData(arcFile, arc, nArc, N);
	rd.readVehicleData(vehFile, vehicle, nVeh, maxC);
	rd.readTrackData(trackFile, track, nTrack, maxD, maxL);
	rd.readDemandData(dmdFile, dmd, T);
}

void Model::freeMem()
{
	directoryPath.clear();
	delete []path;
	delete []buf;
	arc.clear();
	vehicle.clear();
	track.clear();
}

