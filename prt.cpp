// prt.cpp : Solve scheduling problem for PRT system
// instruction for arguments: directory path (I:\git\prt), algorithm (-d: direct, -h: heuristic, -c: colgen)

#include "stdafx.h"
#include <sstream>
#include "prt.h"

using namespace std;

int main(int argc, char *argv[])
{
	Model prtModel;
	prtModel.InitModel(argc, argv);

	switch (prtModel._algo){
	case DIRECT:	prtModel.Direct();break;
	case HEURESTIC:	prtModel.Heuristic2();break;
	case COLGEN:	prtModel.ColGen();break;
	case GREEDY:	prtModel.GreedyAlgo(); break;
	default:		break;}

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
	
	for(i = 1; i < argc; i++)
	{
		if (string(argv[i]) == "-d")
		{
			_algo = DIRECT;
		}
		else if (string(argv[i]) == "-h")
		{
			_algo = HEURESTIC;
		}
		else if (string(argv[i]) == "-c")
		{
			_algo = COLGEN;
		}
		else if (string(argv[i]) == "-g")
		{
			_algo = GREEDY;
		}
		else
		{
			directoryPath += argv[i];
		}
	}
	if(directoryPath.empty())
	{
		directoryPath = "./";
	}

	Reader rd;
	string arcFile = directoryPath + "input/Arc.csv";
	string vehFile = directoryPath + "input/Vehicle.csv";
	string trackFile = directoryPath + "input/Track.csv";
	string dmdFile = directoryPath + "input/Demand.csv";

	rd.readArcData(arcFile, arc, nArc, N);
	rd.readVehicleData(vehFile, vehicle, nVeh, maxC);
	rd.readTrackData(trackFile, track, nTrack, maxD, maxL);
	rd.readDemandData(dmdFile, dmd, T);

	reviewData();
}

void Model::reviewData()
{
	// review arc data
	for (vector<Arc>::iterator iArc = arc.begin(); iArc != arc.end(); iArc++)
	{
		iArc->tracks.clear();
		for (vector<int>::iterator nTrk = iArc->track.begin(); nTrk != iArc->track.end(); nTrk++)
		{
			iArc->tracks.push_back(&track[*nTrk]);
		}
	}
	// build maps related to vehicles
	int index = 0;
	for (vector<Vehicle>::iterator iVeh = vehicle.begin(); iVeh != vehicle.end(); ++iVeh, ++index)
	{
		// vehicle map based on its intial station of each vehicle
		vehicleIdx.insert(pair<Vehicle*, int>(&(*iVeh), index));
		map<int, set<Vehicle*>>::iterator mapItr = vehicleMap.find(iVeh->to);
		if (mapItr != vehicleMap.end())
		{
			mapItr->second.insert(&(*iVeh));
		}
		else
		{
			set<Vehicle*> refSet;
			refSet.insert(&(*iVeh));
			vehicleMap.insert(pair<int, set<Vehicle*> >(iVeh->to, refSet));
		}
		// initialize vehicle status and route maps
		vector<bool> vecStatus;
		vector<Arc*> vecRoute;
		for (t = 0; t < T; t++)
		{
			vecStatus.push_back(false);						// initial status: idle
		}
		vehicleStatus.insert(pair<Vehicle*, vector<bool> >(&(*iVeh), vecStatus));
		vehicleRoute.insert(pair<Vehicle*, vector<Arc*> >(&(*iVeh), vecRoute));
	}
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

