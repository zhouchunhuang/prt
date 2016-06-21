#include "stdafx.h"
#include "prt.h"


Reader::Reader(void)
{
	nv = 0;				
	nd = 0;				
	nt = 0;				
	na = 0;				
	nl = 0;
}

bool Reader::readArcData(string fname, vector<Arc> &arc, int &nArc, int &N)
{
	int col = 0;
	int row = 0;
	ifstream f_arc(fname);
	if(!f_arc.good())
	{
		cerr << "FAIL TO READ ARC DATA!";
		return false;
	}

	row = 0; col = 0;

	while(f_arc.good())
	{
		row++;
		Arc newArc;
		string s;
		if(!getline(f_arc,s)) break;

		col = 0;
		istringstream ss(s);
		while(ss)
		{
			col++;
			string value;
			if( !getline( ss, value, ',') ) break;
			if( atoi(value.c_str()) < 0 ) break;
			if (row == 1 && col == 2)
			{
				na = atoi(value.c_str());
			}
			else if(row == 1 && col ==4)
			{
				nd = atoi(value.c_str());
			}
			else if (row > 2 && row <= 2 + na)
			{	
				if( col == 2){
					newArc.from = atoi(value.c_str());
				}
				else if( col == 3){
					newArc.to = atoi(value.c_str());
				}
				else if( col == 4){
					newArc.cost = atof(value.c_str());
				}
				else if( col == 5){
					newArc.time = atoi(value.c_str());
				}
				else if( col >= 6 ){
					newArc.track.push_back(atoi(value.c_str()));
				}
			}
		}
		if( row > 2)	arc.push_back(newArc);
	}
	nArc = na;
	N = nd;
	return true;
}

bool Reader::readVehicleData(string fname, vector<Vehicle> &vehicle, int &nVeh, double &maxC)
{
	int col = 0;
	int row = 0;
	ifstream f_veh(fname);
	if(!f_veh.good())
	{
		cerr << "FAIL TO READ VEHICLE DATA!";
		return false;
	}

	row = 0; col = 0;

	while(f_veh.good())
	{
		row++;
		Vehicle newVeh;
		string s;
		if(!getline(f_veh,s)) break;

		col = 0;
		istringstream ss(s);
		while(ss)
		{
			col++;
			string value;
			if( !getline( ss, value, ',') ) break;
			if (row == 1 && col == 2)
			{
				nv = atoi(value.c_str());
			}
			else if(row == 1 && col == 4)
			{
				maxC = atof(value.c_str());
			}
			if (row > 2 && row <= 2 + na)
			{		
				switch(col)
				{			
				case 1:
					break;
				case 2:
					newVeh.from = atoi(value.c_str());
					break;
				case 3:
					newVeh.to = atoi(value.c_str());
					break;
				case 4:
					newVeh.time = atof(value.c_str());
					break;
				case 5:
					newVeh.cap = atoi(value.c_str());
					break;
				case 6:
					newVeh.powLvl = atof(value.c_str());
					break;
				case 7:
					newVeh.maxPowLvl = atof(value.c_str());
					break;
				default:
					break;
				}
			}
		}
		if( row > 2)	vehicle.push_back(newVeh);
	}
	nVeh = nv;
	return true;
}

bool Reader::readTrackData(string fname, vector<Track> &track, int &nTrack, int &maxD, int &maxL)
{
	int col = 0;
	int row = 0;
	ifstream f_track(fname);
	if(!f_track.good())
	{
		cerr << "FAIL TO READ TRACK DATA!";
		return false;
	}

	row = 0; col = 0;

	while(f_track.good())
	{
		row++;
		Track newTrack;
		string s;
		if(!getline(f_track,s)) break;

		col = 0;
		istringstream ss(s);
		while(ss)
		{
			col++;
			string value;
			if( !getline( ss, value, ',') ) break;
			if (row == 1)
			{
				switch(col)
				{
				case 2:
					nl = atoi(value.c_str());
					break;
				case 4:
					maxL = atoi(value.c_str());
					break;
				case 6:
					maxD = atoi(value.c_str());
					break;
				default:
					break;
				}
				
			}
			if (row > 2 && row <= 2 + na)
			{		
				switch(col)
				{			
				case 1:
					break;
				case 2:
					newTrack.from = atoi(value.c_str());
					break;
				case 3:
					newTrack.to = atoi(value.c_str());
					break;
				default:
					break;
				}
			}
		}
		if(row > 2)
			track.push_back(newTrack);
	}
	nTrack = nl;
	return true;
}

bool Reader::readDemandData(string fname, vector2int &dmd, int &T)
{
	int col = 0;
	int row = 0;
	ifstream f_dmd(fname);
	if(!f_dmd.good())
	{
		cerr << "FAIL TO READ DEMAND DATA!";
		return false;
	}

	row = 0; col = 0;

	while(f_dmd.good())
	{
		row++;
		vector<int> newDmd;
		string s;
		if(!getline(f_dmd,s)) break;

		col = 0;
		istringstream ss(s);
		while(ss)
		{
			col++;
			string value;
			if( !getline( ss, value, ',') ) break;
			if(row > 1 && col > 1)
			{
				newDmd.push_back(atoi(value.c_str()));
			}
		}
		if(row > 1){
			dmd.push_back(newDmd);
		}
	}
	T = dmd.size();
	return true;
}