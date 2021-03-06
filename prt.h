#pragma once

#include "stdafx.h"
#include<sstream>
#include<fstream>
#include<iostream>
#include<vector>
#include<string>
#include<algorithm>
#include<map>
#include<set>

#include "ilcplex\cplex.h"
#include "ilcplex\ilocplex.h"

using namespace std;

typedef vector<vector<int>> vector2int;
typedef vector<vector2int> vector3int;
typedef IloArray<IloBoolVarArray> IloBoolVarArray2;
typedef IloArray<IloBoolVarArray2> IloBoolVarArray3;
typedef IloArray<IloNumVarArray> IloNumVarArray2;
typedef IloArray<IloNumVarArray2> IloNumVarArray3;
typedef IloArray<IloRangeArray> IloRangeArray2;

//#define TEST_MODE2
//#define MODEL_EXPORT
#define TimeWindow 2
#define penalty 2
#define WallTime 300				// maximum elapsed time
#define MinDelayedPaxAssigned 3
#define MinTotalPaxAssigned 6
#define powerLB 5

class Vehicle
{
public:
	int from;
	int to;				//a vehicle's location: moving from a starting node toward a destination; stay at node i if i == j
	int time;			//time point when this vehicle is ready (arrives at node j)
	int cap;			//capacity
	double powLvl;		//power level
	double maxPowLvl;	//maximum power level
	bool assigned;		//whether the vehicle has been assigned
};


class Track
{
public:
	int from;
	int to;
};

class Arc
{
public:
	int from;
	int to;
	double fcost;			//fixed cost
	double cost;
	int time;				//travel time incurred while traveling from i to j
	vector<int> track;		//store the number of each track which is contained in the arc
	vector<Track*> tracks;  // store the object tracks contained in the arc
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
	bool	Reader::readArcData(string &fname, vector<Arc> &arc, int &nArc, int &N);
	bool	Reader::readVehicleData(string &fname, vector<Vehicle> &vehicle, int &nVeh, double &maxC);
	bool	Reader::readTrackData(string &fname, vector<Track> &track, int &nTrack, int &maxD, int &maxL);
	bool	Reader::readDemandData(string &fname, vector2int &dmd, int &T);
};

class System
{
public:
	vector2int Demand;
	vector2int Remain;					//the number of remaining customers at each time
	vector<Vehicle> vehicle;
	map<Vehicle*, int> vehIndex;
	vector2int NodeLoad;				//node load
	vector2int TrackLoad;				//track load
	double cost;
	vector3int route;					//route for each vehicle at each time
	vector<double> routeCost;			//cost of each route
	vector3int customer;				//number of customers that are transported by the vehicle at a certain time from i to j
};

enum Algo
{
	DIRECT,
	HEURESTIC,
	COLGEN,
	GREEDY
};

class Model
{
private:
	int i, j;
	int k;
	int v;
	int t, tau;
	int l;
	int n;
	int N;				//number of nodes
	int nArc;			//number of arcs
	int nVeh;			//number of vehicles
	int T;				//number of periods
	int nTrack;			//number of tracks
	char* buf;
	char* path;

	clock_t _start;
	clock_t _end;
	double cmp_time;
	string directoryPath;
	ofstream output;

	vector<Arc> arc;
	vector<Vehicle> vehicle;
	vector<Track> track;
	map<int, set<Vehicle*>, less<int>>	vehicleMap;
	map<Vehicle*, vector<bool>>			vehicleStatus;	// true: assigned  false: not assigned
	map<Vehicle*, vector<Arc*>>			vehicleRoute;	// store arcs that each vehicle is assigned to at each time period

	map<Arc*, int> arcIndex;
	map<Vehicle*, int> vehIndex;

	vector2int dmd;		//demand[t][k]
	System sys;			//dynamic system status
	double maxC;		//maximum charge amount during a time period
	int maxD;			//maximum node (depot) capacity
	int maxL;			//maximum track capacity

	//extensive model
	IloEnv		env;
	IloModel	PRT;
	IloCplex	PRTSolver;
	double UB, LB, NLB, gap;
	int itn;
	
	//decision variables
	IloNumVarArray3		var_x;		//the number of customers transported by a vehicle from i to j at time t
	IloBoolVarArray3	var_z;		//whether to assign a vehicle from i to j at time t
	IloNumVarArray2		var_e;		//electricity level for vehicle k at the beginning of time t
	IloNumVarArray2		var_y;		//number of customers who haven't been served within the defined time window
	IloNumVarArray2		var_s;		//number of customers who haven't been served at each time period

	//column generation models
	IloModel	MP, SP;
	IloCplex	MPSolver, SPSolver;
	IloObjective objMP, objSP;

	//MP decision variables and constraints
	IloNumVarArray2	v_lambda;		//columns: route for each vehicle
	IloNumVarArray2	v_y;			//number of customers who haven't been served within the defined time window
	IloNumVarArray2	v_s;			//number of customers who haven't been served at each time period
	IloRangeArray2	NodeCap;		//node capacity constraints	
	IloRangeArray2	TrackCap;		//track road capacity
	IloRangeArray2	Demand;			//Demand requirement
	IloRangeArray2	TmWindow;		//time window constraints
	IloRangeArray	Convex;			//convexity constraints
	//Dual solutions
	IloNumArray2	DualNodeCap;
	IloNumArray2	DualTrackCap;
	IloNumArray2	DualDemand;
	IloNumArray2	DualTmWindow;
	IloNumArray		DualConvex;

	//Heuristic algorithm used
	vector<pair<Arc*, double>>		ArcDmd;				//store arc demands
	vector<pair<Arc*, double>>		ArcDelayPax;		//store arc delayed pax (who are served out of time window)
	vector<pair<Vehicle*, double>>	VehElvl;			//store vehicle electricity level
	map<Arc*, bool>					arcIsAssigned;		//whether the arc demands are satisfied/assigned

	//SP decision variables and constraints
	IloExpr				costXpr;	//cost of route solved from SP
	IloNumVarArray3		v_x;		//the number of customers transported by a vehicle from i to j at time t
	IloNumVarArray3		v_z;		//whether to assign a vehicle from i to j at time t
	IloNumVarArray2		v_e;		//electricity level for vehicle k at the beginning of time t
	IloRangeArray2		Battery;		//battery level constraints
	IloRangeArray2		VehCap;			//vehicle capacity constraints
	IloRangeArray2		FlowBalance;	//flow balance constraints
	//columns to be added
	IloNumArray3		val_x;
	IloNumArray3		val_z;


public:
	Model(void);		//initialize model parameters
	Algo _algo;			//options of algorithms
	void InitModel(int argc, char **argv);		//initialize model by reading files
	void reviewData();
	int Direct();		//solve the extensive model directly by CPLEX
	int Heuristic();	//heuristic algorithm
	int Heuristic2();	//improved heuristic algorithm
	
	int GreedyAlgo();	//greedy algorithm
	int initSystem();	//initialize system status
	void startAssignment();
	void reviewSystem();
	bool assignVehtoArc(Arc* &pArc, Vehicle* &pVeh);
	void updateSystem(Arc* pArc, Vehicle* pVeh, bool isVehtoArcAssigned);
	void computeTotalCost();

	int outputSol();	//write out solution to file
	int heuristicSol();	//write out heuristic solution

	int ColGen();		//column generation algorithm
	int crtMP();		//create master problem
	int crtSP();		//create subproblems
	int solveMP();		//solve MP
	int solveSP();		//solve SP
	int addColumns(int v);		//add columns to the MP

	int min(int a, int b);		//get the maximum value between a and b
	int max(int a, int b);

	void initOutputColgen();
	bool terminateColgen();
	void finalizeColgen();

	int getArcIndex(Arc* pArc);
	int getVehIndex(Vehicle* pVeh);

	void freeMem();
};