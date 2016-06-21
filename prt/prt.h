#pragma once

#include "stdafx.h"
#include<sstream>
#include<fstream>
#include<iostream>
#include<vector>
#include<string>

#include "ilcplex\cplex.h"
#include "ilcplex\ilocplex.h"

using namespace std;

typedef vector<vector<int>> vector2int;
typedef IloArray<IloBoolVarArray> IloBoolVarArray2;
typedef IloArray<IloBoolVarArray2> IloBoolVarArray3;
typedef IloArray<IloNumVarArray> IloNumVarArray2;

//#define TEST_MODE

class Vehicle
{
public:
	int from;
	int to;				//a vehicle's location: moving from a starting node toward a destination; stay at node i if i == j
	int time;			//time point when this vehicle is ready (arrives at node j)
	double powLvl;		//power level
	double cap;			//capacity
	double maxPowLvl;	//maximum power level
};

class Arc
{
public:
	int from;
	int to;
	double cost;
	int time;				//travel time incurred while traveling from i to j
	vector<int> track;		//store the number of each track which is contained in the arc
};

class Track
{
public:
	int from;
	int to;
};

class Reader
{
private:
	int nv;				//number of vehicles
	int nd;				//number of nodes
	int nt;				//number of time periods
	int na;				//number of arcs
	int nl;				//number of tracks
public:
	Reader(void);
	bool	Reader::readArcData(string fname, vector<Arc> &arc, int &nArc, int &N);
	bool	Reader::readVehicleData(string fname, vector<Vehicle> &vehicle, int &nVeh, double &maxC);
	bool	Reader::readTrackData(string fname, vector<Track> &track, int &nTrack, int &maxD, int &maxL);
	bool	Reader::readDemandData(string fname, vector2int &dmd, int &T);
};

class Model
{
private:
	int i, j;
	int k;
	int v;
	int t, tau;
	int l;
	int N;				//number of nodes
	int nArc;			//number of arcs
	int nVeh;			//number of vehicles
	int T;				//number of periods
	int nTrack;			//number of tracks
	char buf[33];

	Reader rd;
	vector<Arc> arc;
	vector<Vehicle> vehicle;
	vector<Track> track;
	vector2int dmd;		//demand[t][k]
	double maxC;		//maximum charge amount during a time period
	int maxD;			//maximum node (depot) capacity
	int maxL;			//maximum track capacity

public:
	Model(void);		//initialize model parameters
	int Direct();		//solve the extensive model directly by CPLEX
	int outputSol();	//write out solution to file
};