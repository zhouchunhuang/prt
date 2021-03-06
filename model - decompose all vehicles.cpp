#include "stdafx.h"
#include "prt.h"

Model::Model(void)
{
	i = 0;
	j = 0;
	k = 0;
	v = 0;
	t = 0;
	tau = 0;
	l = 0;

	string arcFile = "./input/Arc.csv";
	string vehFile = "./input/Vehicle.csv";
	string trackFile = "./input/Track.csv";
	string dmdFile = "./input/Demand.csv";

	rd.readArcData(arcFile, arc, nArc, N);
	rd.readVehicleData(vehFile, vehicle, nVeh, maxC);
	rd.readTrackData(trackFile, track, nTrack, maxD, maxL);
	rd.readDemandData(dmdFile, dmd, T);
}

int Model::Direct()
{
#ifdef TEST_MODE
	for(i = 0; i < nArc; i++){
		cout << "Arc[" << i << "]:" << arc[i].from << "-" << arc[i].to << " track:\t";
		for(vector<int>::iterator itr = arc[i].track.begin(); itr != arc[i].track.end(); itr++){
			cout << (*itr) << "\t";
		}
		cout << endl;
	}
	cout << "number of vehicles:" << nVeh << endl;
	for(v = 0; v < nVeh; v++){
		cout << "Vehicle " << v << ":\t" << vehicle[v].from << "\t" << vehicle[v].to << "\t" << vehicle[v].time << "\t" << vehicle[v].cap << "\t" << vehicle[v].powLvl << "\t" << vehicle[v].maxPowLvl << endl;
	}

	cout << "Demand:\n";
	for(k = 0; k < nArc; k++){
		if(arc[k].from == arc[k].to)	continue;
		for(t = 0; t < T; t++){
			cout << dmd[t][k] << "\t";
		}
		cout << endl;
	}
#endif
	
	//extensive model
	PRT = IloModel(env);
	PRTSolver = IloCplex(PRT);
	PRTSolver.setParam(IloCplex::TiLim, 300);
	PRTSolver.setParam(IloCplex::EpGap, 0.01);

	//decision variables
	var_x = IloNumVarArray3(env, nArc);					
	var_z = IloBoolVarArray3(env, nArc);					//whether to assign a vehicle from i to j at time t
	var_e = IloNumVarArray2(env, nVeh);						//electricity level for vehicle k at the beginning of time t
	var_y = IloNumVarArray2(env, nArc);						
	var_s = IloNumVarArray2(env, nArc);
	 
	for(k = 0; k < nArc; k++){
		var_x[k] = IloNumVarArray2(env, nVeh);
		var_z[k] = IloBoolVarArray2(env, nVeh);
		for(v = 0; v < nVeh; v++){
			var_x[k][v] = IloNumVarArray(env);
			var_z[k][v] = IloBoolVarArray(env);
			for(t = 0; t < T; t++){
				sprintf(buf, "x[%d,%d][%d][%d]", arc[k].from, arc[k].to, v, t);
				var_x[k][v].add(IloNumVar(env, 0, +IloInfinity, buf));
				sprintf(buf, "z[%d,%d][%d][%d]", arc[k].from, arc[k].to, v, t);
				var_z[k][v].add(IloBoolVar(env,buf));
			}
		}
	}

	for(v = 0; v < nVeh; v++){
		var_e[v] = IloNumVarArray(env);
		for(t = 0; t <= vehicle[v].time; t++){
			sprintf(buf, "e[%d][%d]", v, t);
			var_e[v].add(IloNumVar(env, 0, vehicle[v].powLvl, buf));
		}
		for(t = vehicle[v].time+1; t < T; t++){
			sprintf(buf, "e[%d][%d]", v, t);
			var_e[v].add(IloNumVar(env, 0, vehicle[v].maxPowLvl, buf));
		}
	}

	for(k = 0; k < nArc; k++){
		var_y[k] = IloNumVarArray(env);
		var_s[k] = IloNumVarArray(env);
		for(t = 0; t < T; t++){
			sprintf(buf, "y[%d,%d][%d]", arc[k].from, arc[k].to, t);
			var_y[k].add(IloNumVar(env, 0, +IloInfinity, buf));
			sprintf(buf, "s[%d,%d][%d]", arc[k].from, arc[k].to, t);
			var_s[k].add(IloNumVar(env, 0, +IloInfinity, buf));
		}
	}

	//objective function
	IloExpr objXpr(env);	
	for(k = 0; k < nArc; k++){		
		for(t = 0; t < T; t++){
			objXpr += penalty * arc[k].cost * var_y[k][t];
			for(v = 0; v < nVeh; v++){
				objXpr += arc[k].fcost * var_z[k][v][t] + arc[k].cost * var_x[k][v][t];
			}
		}
	}
	PRT.add(IloMinimize(env, objXpr));

	//electricity for travel constraints	
	for( v = 0; v < nVeh; v++){
		for(t = vehicle[v].time + 1; t < T; t++){
			IloExpr xpr(env);
			for( k = 0; k < nArc; k++){
				i = arc[k].from; j = arc[k].to;
				if(i != j){
					sprintf(buf, "MinPowLvl(%d-%d,%d,%d)", arc[k].from, arc[k].to, v, t);
					PRT.add(IloRange(env, -IloInfinity, arc[k].fcost * var_z[k][v][t] + arc[k].cost * var_x[k][v][t] - var_e[v][t], 0, buf));

					for(tau = max(t - arc[k].time, 0); tau < t; tau++){
						xpr -= (arc[k].fcost * var_z[k][v][tau] + arc[k].cost * var_x[k][v][tau]) / (double)(abs(i - j)) ;
					}
				}
				else{
					xpr += maxC * var_z[k][v][t-1];
				}
			}
			xpr += var_e[v][t-1] - var_e[v][t];
			sprintf(buf, "Dynamic E Level(%d,%d)", v, t);
			PRT.add(IloRange(env, 0, xpr, +IloInfinity, buf));
			xpr.end();
		}
	}
	//vehicle capacity constraints
	for(v = 0; v < nVeh; v++){
		for(k = 0; k < nArc; k++){
			for(t = 0; t < T; t++){
				sprintf(buf, "Vehicle Capacity(%d-%d,%d,%d)", arc[k].from, arc[k].to, v, t);
				PRT.add(IloRange(env, -IloInfinity, var_x[k][v][t] - vehicle[v].cap * var_z[k][v][t], 0, buf));
			}
		}
	}
	//demand satisfaction constraints
	for(k = 0; k < nArc; k++){
		if(arc[k].from == arc[k].to) continue; 
		for(t = 0; t < T; t++){
			IloExpr xpr(env);
			for(v = 0; v < nVeh; v++){
				xpr += var_x[k][v][t];
			}
			if(t == 0)	xpr += var_s[k][t];
			else		xpr += var_s[k][t] - var_s[k][t-1];
			sprintf(buf, "Dmd(%d-%d,%d)", arc[k].from, arc[k].to, t); 
			PRT.add(IloRange(env, dmd[t][k], xpr, dmd[t][k], buf));
			xpr.end();
		}
	}

	// the number of remaining customers at the last time period should be zero
	for(k = 0; k < nArc; k++){
		if(arc[k].from == arc[k].to) continue; 
		PRT.add(IloRange(env, 0, var_s[k][T-1], 0, "TtlDmd"));
	}

	//Time window constraints
	for(k = 0; k < nArc; k++){
		if(arc[k].from == arc[k].to) continue; 
		for(t = 0; t < T - TimeWindow; t++){
			IloExpr xpr(env);
			xpr = var_y[k][t] - var_s[k][t];
			for(tau = t + 1; tau <= t + TimeWindow && t < T; tau++){
				for( v = 0; v < nVeh; v++){
					xpr += var_x[k][v][tau];
				}				
			}
			sprintf(buf, "TW(%d-%d,%d)", arc[k].from, arc[k].to, t); 
			PRT.add(IloRange(env, 0, xpr, +IloInfinity, buf));
			xpr.end();
		}
	}

	//Depot buffer capacity constraints
	for(k = 0; k < nArc; k++){
		if(arc[k].from != arc[k].to) continue;
		for(t = 0; t < T; t++){
			IloExpr xpr(env);
			for(v = 0; v < nVeh; v++){
				xpr += var_z[k][v][t];
			}
			sprintf(buf, "Buffer capacity(%d-%d, %d)", arc[k].from, arc[k].to, t); 
			PRT.add(IloRange(env, -IloInfinity, xpr, maxD, buf));
			xpr.end();
		}
	}
	//track road capacity constraints
	for(l = 0; l < nTrack; l++){
		for(t = 0; t < T; t++){
			IloExpr xpr(env);
			for(k = 0; k < nArc; k++){
				if(arc[k].from == arc[k].to) continue;
				for(vector<int>::iterator itr = arc[k].track.begin(); itr != arc[k].track.end(); itr++){
					if(l == (*itr) && t - abs(track[l].from - arc[k].from) >= 0 ){
						for(v = 0; v < nVeh; v++){	
							xpr += var_z[k][v][t - abs(track[l].from - arc[k].from)];
						}						
					}	
				}
			}
			sprintf(buf, "Track cap(%d-%d,%d)", track[l].from, track[l].to, t); 
			PRT.add(IloRange(env, -IloInfinity, xpr, maxL));
			xpr.end();
		}
	}
	//self flow balance constraints
	for(i = 0; i < N; i++){
		for(v = 0; v < nVeh; v++){
			if(vehicle[v].from == vehicle[v].to)								//the vehicle stops at some station
			{	
				for(t = 0; t < T; t++){
					IloExpr xpr(env);
					for(k = 0; k < nArc; k++){
						if(arc[k].from == arc[k].to && arc[k].from == i){
							if(t == 0)	xpr += var_z[k][v][t];
							else		xpr += var_z[k][v][t] - var_z[k][v][t-1];
						}
						else if(arc[k].from == i){
							xpr += var_z[k][v][t];
						}	
						else if(arc[k].to == i){
							tau = t - arc[k].time;
							if(tau >= 0) xpr -= var_z[k][v][tau];
						}
					}
					sprintf(buf, "flow bal(%d,%d,%d)", i, v, t);
					if(t == 0 && vehicle[v].from == i)							//the vehicle stops at station i at t=0
						PRT.add(IloRange(env, 1, xpr, 1, buf));
					else	
						PRT.add(IloRange(env, 0, xpr, 0, buf));
					xpr.end();
				}
			}
			else																//the vehicle is moving
			{
				for(t = 0; t < vehicle[v].time; t++){							//vehicle is moving from t=0 to t=vehicle[v].time
					for(k = 0; k < nArc; k++){
						PRT.add(IloRange(env, 0, var_z[k][v][t], 0));
					}
				}
				for(t = vehicle[v].time; t < T; t++){
					IloExpr xpr(env);
					for(k = 0; k < nArc; k++){
						if(arc[k].from == arc[k].to && arc[k].from == i){
							if(t == vehicle[v].time)	xpr += var_z[k][v][t];
							else		xpr += var_z[k][v][t] - var_z[k][v][t-1];
						}
						else if(arc[k].from == i){
							xpr += var_z[k][v][t];
						}	
						else if(arc[k].to == i){
							tau = t - arc[k].time;
							if(tau >= vehicle[v].time) xpr -= var_z[k][v][tau];
						}
					}
					sprintf(buf, "flow bal(%d,%d,%d)", i, v, t);
					if(t == vehicle[v].time && vehicle[v].to == i)							//the vehicle stops at station i at t=0
						PRT.add(IloRange(env, 1, xpr, 1, buf));
					else	
						PRT.add(IloRange(env, 0, xpr, 0, buf));
					xpr.end();
				}
			}
		}
	}


#ifdef TEST_MODE
	PRTSolver.exportModel("./output/PRT.lp");
#endif

	_start = clock();
	PRTSolver.solve();
	_end = clock();
	cmp_time = (double)(_end - _start)/CLOCKS_PER_SEC;

	//PRTSolver.writeSolution("./output/solution.lp");
	outputSol();

	return 1;
}

int Model::Heuristic()
{
	_start = clock();
	initSystem();
	for(t = 0; t < T; t++){
		for(v = 0; v < nVeh; v++){
			sys.vehicle[v].assigned = false;															//assigned is false by default: true if either assigned at the current time or moving
		}
		for(k = 0; k < nArc; k++){
			if(arc[k].from == arc[k].to) continue;
			if(t)	sys.Demand[t][k] += sys.Demand[t-1][k];		
			if(!(sys.Demand[t][k]))	continue;
			for(v = 0; v < nVeh; v++){
				if(sys.vehicle[v].to != arc[k].from)	continue;									//don't assign the vehicle if it is not in the station
				if(sys.vehicle[v].time != t){														//don't use the vehicle if it is moving											
					sys.vehicle[v].assigned = true;
					continue;
				}
				if(sys.vehicle[v].powLvl < arc[k].fcost + min(sys.Demand[t][k], sys.vehicle[v].cap) * arc[k].cost)	continue;	//don't use the vehicle if battery level is not enough
				if(!(sys.Demand[t][k]))	break;																					//break if demand is none
				bool overload = false;			
				for(vector<int>::iterator itr = arc[k].track.begin(); itr != arc[k].track.end(); itr++){
					tau = t + abs(track[*itr].from - arc[k].from);
					if(tau < T && sys.TrackLoad[*itr][tau] >= maxL){
						overload = true;
						break;
					}		
				}
				if(overload)	break;

				sys.route[k][v][t] = 1;													//assign the vehicle which is available
				sys.vehicle[v].powLvl -= arc[k].fcost + arc[k].cost;					//update vehicle status: battery level
				sys.vehicle[v].to = arc[k].to;											//update vehicle status: arriving station
				sys.vehicle[v].time = t + arc[k].time;									//update vehicle status: arriving time
				sys.vehicle[v].assigned = true;											//update vehicle status: assigned or not
				sys.customer[k][v][t] = min(sys.Demand[t][k], sys.vehicle[v].cap);		//the number of customers sent out
				sys.Demand[t][k] -= sys.customer[k][v][t];								//update demand	
				for(vector<int>::iterator itr = arc[k].track.begin(); itr != arc[k].track.end(); itr++){		//update track load
					tau = t + abs(track[*itr].from - arc[k].from);
					if(tau < T)	sys.TrackLoad[*itr][tau]++;	
				}
			}
		}
		for(k = 0; k < nArc; k++){
			if(arc[k].from != arc[k].to)	continue;
			for(v = 0; v < nVeh; v++){
				if(!(sys.vehicle[v].assigned) && sys.vehicle[v].to == arc[k].to && sys.vehicle[v].time == t){
					sys.route[k][v][t] = 1;						//stay in the station if the vehicle is not assigned toward any other stations
					sys.NodeLoad[arc[k].to][t]++;				//update node load
					sys.vehicle[v].time = t+1;					//update time
				}
			}
		}
		//if node load maximum is reached -- cancel vehicles toward this station and/or cancel previous route
	}
	// compute the total cost
	sys.cost = 0;
	for(k = 0; k < nArc; k++){
		if(arc[k].from == arc[k].to)	continue;		
		for(t = 0; t < T; t++){
			int val_y = sys.Demand[t][k];
			for(v = 0; v < nVeh; v++){
				sys.cost += arc[k].fcost * sys.route[k][v][t] + arc[k].cost * sys.customer[k][v][t];
				for(tau = t+1; tau <= t+TimeWindow && tau < T; tau++){
					val_y -= sys.customer[k][v][tau];  
				}
			}
			val_y = max(val_y,0);
			sys.cost += penalty * arc[k].cost * val_y;
		}
	}

	for(v = 0; v < nVeh; v++){
		double rCost = 0;
		for(k = 0; k < nArc; k++){
			for(t = 0; t < T; t++){
				rCost += arc[k].fcost * sys.route[k][v][t] + arc[k].cost * sys.customer[k][v][t];
			}
		}
		sys.routeCost.push_back(rCost);
	}

	_end = clock();
	cmp_time = (double)(_end - _start)/CLOCKS_PER_SEC;
	heuristicSol();

	return 1;
}

int Model::ColGen()
{
	bool loop = true;
#ifdef TEST_MODE
	crtMP();
	MPSolver.solve();
	
	cout << MPSolver.getObjValue() << endl;
	for(v = 0; v < nVeh; v++){
		IloNumArray	sol_lambda(env);
		MPSolver.getValues(sol_lambda, v_lambda[v]);		
		cout << sol_lambda << "\t";
	}
	cout << endl;
	
	for(v = 0; v < nVeh; v++){
		MP.add(IloConversion(env, v_lambda[v], ILOINT));
	}	
	MPSolver.solve();
	cout << MPSolver.getObjValue() << endl;
	for(v = 0; v < nVeh; v++){
		IloNumArray	sol_lambda(env);
		MPSolver.getValues(sol_lambda, v_lambda[v]);		
		cout << sol_lambda << "\t";
	}
	cout << endl;
#endif
	sprintf(path, "./output/ColGen_%dV_%dN_%dArc_%dT.txt", nVeh, N, nArc, T);
	string outputFile(path);
	output.open(outputFile);
	output << "Iteration\tUB\tLB\tNLB\tGap\n";
	UB = +IloInfinity; LB = -IloInfinity; NLB = -IloInfinity; gap = 1.0;
	itn = 0;
	output << itn << "\t" << UB << "\t" << LB << "\t" << NLB << "\t" << gap << endl;
	crtMP();
	crtSP();
	while(loop){
		itn++;	
		solveMP();
		solveSP();

		gap = (NLB >LB) ? ((UB - NLB)/(1e-75 + UB)):((UB - LB)/(1e-75 + UB));
		output << itn << "\t" << UB << "\t" << LB << "\t" << NLB << "\t" << gap << endl;
		LB = (NLB >LB) ? NLB:LB;
		if(gap < 0.01)	break;
		if(itn >= 2) break;
	}
	output.close();
	return 1;
}

int Model::initSystem()
{
	sys.cost = 0;
	for( t = 0; t < T; t++){
		sys.Demand.push_back(dmd[t]);				//initialize system demand data
		vector<int> remain;
		for(k = 0; k < nArc; k++){
			if(arc[k].from != arc[k].to)	remain.push_back(0);
		}
		sys.Remain.push_back(remain);
	}
	for(v = 0; v < nVeh; v++){
		sys.vehicle.push_back(vehicle[v]);			//initialize vehicle status
	}
	for(i = 0; i < N; i++){
		vector<int> newLoad;
		for(t = 0; t < T; t++){
			newLoad.push_back(0);
		}
		sys.NodeLoad.push_back(newLoad);						//initialize node load such that all are 0's
	}
	for(l = 0; l < nTrack; l++){
		vector<int> newLoad;
		for(t = 0; t < T; t++){
			newLoad.push_back(0);
		}
		sys.TrackLoad.push_back(newLoad);						//initialize track load such that all are 0's
	}
	for(k = 0; k < nArc; k++){
		vector2int	routeArc;
		vector2int	cusArc;
		for(v = 0; v < nVeh; v++){
			vector<int>	routeVehicle;
			vector<int> cusVehicle;
			for(t = 0; t < T; t++){
				routeVehicle.push_back(0);
				cusVehicle.push_back(0);
			}
			routeArc.push_back(routeVehicle);
			cusArc.push_back(cusVehicle);
		}
		sys.route.push_back(routeArc);
		sys.customer.push_back(cusArc);
	}

	return 1;
}

int Model::outputSol()
{
	sprintf(path, "./output/routes_%dV_%dN_%dArc_%dT.txt", nVeh, N, nArc, T);
	string outputFile(path);
	//string outputFile = "./output/routes.txt";
	output.open(outputFile);
	if(PRTSolver.getStatus() == IloCplex::Infeasible){
		output << "Problem is Infeasible!" << endl;
		output.close();
		return 0;
	}

	output << "Computational Time: " << cmp_time << endl;
	output << "Optimal Electricity Consumption:" << PRTSolver.getObjValue() << "KWH" << endl;
	output << "Relative Gap = " << PRTSolver.getMIPRelativeGap() << endl;
	output << "The Routes for All PRT Vehicles:\n";

	for(v = 0; v < nVeh; v++){
		output << "Routes for Vehicle " << v << ":\n\t";
		if(vehicle[v].time == 0)									//the vehicle stops at a station at the beginning
		{
			for(t = 0; t < T; t++){
				for(k = 0; k < nArc; k++){
					if(PRTSolver.getValue(var_z[k][v][t]) > .99){
						if(t == 0)	output << arc[k].from;
						for(i = 1; i < abs(arc[k].from - arc[k].to); i++)
							output << "--*";
						output << "--" << arc[k].to;
					}
				}
			}
		}
		else														//the vehicle is moving at the beginning
		{
			for(t= 0; t < vehicle[v].time; t++){
				output << "*--";
			}
			for(t = vehicle[v].time; t < T; t++){
				for(k = 0; k < nArc; k++){
					if(PRTSolver.getValue(var_z[k][v][t]) > .99){
						if(t == vehicle[v].time)	output << arc[k].from;
						for(i = 1; i < abs(arc[k].from - arc[k].to); i++)
							output << "--*";
						output << "--" << arc[k].to;
					}
				}
			}
			
			
		}
		output << endl;
	}

	output << "Battery level over time:\n";
	for(v = 0; v < nVeh; v++){
		output << "V" << v << ":\t";
		for(t = 0; t < vehicle[v].time && t < T; t++){
			output << vehicle[v].powLvl << "\t";
		}
		for(t = vehicle[v].time; t < T; t++){
			output << PRTSolver.getValue(var_e[v][t]) << "\t";
		}
		output << endl;
	}


	output << "The number of customers transported by All PRT Vehicles:\n";
	for(v = 0; v < nVeh; v++){
		output << "Vehicle " << v << ":\n\t";
		for(t = 0; t < T; t++){
			for(k = 0; k < nArc; k++){
				if(PRTSolver.getValue(var_z[k][v][t]) > .99)
					output << PRTSolver.getValue(var_x[k][v][t]) << "\t";
			}
		}
		output << endl;
	}

	output << "The number of customers who haven't been served within the time window:" << endl;
	for(t = 0; t < T; t++){
		output << "t = " << t << ":\t";
		for(k = 0; k < nArc; k++){
			if(arc[k].from == arc[k].to) continue;
			output << PRTSolver.getValue(var_y[k][t]) << "\t";
		}
		output << endl;
	}

	output << "The number of customers who haven't been served at each time:" << endl;
	for(t = 0; t < T; t++){
		output << "t = " << t << ":\t";
		for(k = 0; k < nArc; k++){
			if(arc[k].from == arc[k].to) continue;
			output << PRTSolver.getValue(var_s[k][t]) << "\t";
		}
		output << endl;
	}

	output << "Track load over time: \n";
	for(t = 0; t < T; t++){
		output << "t = " << t << ":\t";
		for(l = 0; l < nTrack; l++){
			int trackLoad = 0;
			for(k = 0; k < nArc; k++){
				if(arc[k].from == arc[k].to) continue;
				for(vector<int>::iterator itr = arc[k].track.begin(); itr != arc[k].track.end(); itr++){
					if(l == (*itr) && t - abs(track[l].from - arc[k].from) >= 0 ){
						for(v = 0; v < nVeh; v++){	
							trackLoad += PRTSolver.getValue(var_z[k][v][t - abs(track[l].from - arc[k].from)]);
						}						
					}	
				}
			}
			output << trackLoad << "\t";
		}
		output << endl;
	}

	output.close();
	return 1;
}

int Model::heuristicSol()
{
	ofstream heusol;
	sprintf(path, "./output/heuristic_%dV_%dN_%dArc_%dT.txt", nVeh, N, nArc, T);
	string outputFile(path);
	heusol.open(outputFile);

	heusol << "Computational Time: " << cmp_time << endl;
	heusol << "Optimal Electricity Consumption:" << sys.cost << "KWH" << endl;
	heusol << "The Routes for All PRT Vehicles:\n";

	for(v = 0; v < nVeh; v++){
		heusol << "Routes for Vehicle " << v << ":\n\t";
		if(vehicle[v].time == 0)									//the vehicle stops at a station at the beginning
		{
			for(t = 0; t < T; t++){
				for(k = 0; k < nArc; k++){
					if(sys.route[k][v][t] > .99){
						if(t == 0)	heusol << arc[k].from;
						for(i = 1; i < abs(arc[k].from - arc[k].to); i++)
							heusol << "--*";
						heusol << "--" << arc[k].to;
					}
				}
			}
		}
		else														//the vehicle is moving at the beginning
		{
			for(t= 0; t < vehicle[v].time; t++){
				heusol << "*--";
			}
			for(t = vehicle[v].time; t < T; t++){
				for(k = 0; k < nArc; k++){
					if(sys.route[k][v][t] > .99){
						if(t == vehicle[v].time)	heusol << arc[k].from;
						for(i = 1; i < abs(arc[k].from - arc[k].to); i++)
							heusol << "--*";
						heusol << "--" << arc[k].to;
					}
				}
			}
			
			
		}
		heusol << endl;
	}

	heusol << "Track Load over time:\n";
	for(t = 0; t < T; t++){
		heusol << "t = " << t << ":\t";
		for(l = 0; l < nTrack; l++){
			int trackLoad = 0;
			for(k = 0; k < nArc; k++){
				if(arc[k].from == arc[k].to) continue;
				for(vector<int>::iterator itr = arc[k].track.begin(); itr != arc[k].track.end(); itr++){
					if(l == (*itr) && t - abs(track[l].from - arc[k].from) >= 0 ){
						for(v = 0; v < nVeh; v++){	
							trackLoad += sys.route[k][v][t - abs(track[l].from - arc[k].from)];
						}						
					}	
				}
			}
			heusol << trackLoad << "\t";
		}
		heusol << endl;
	}

	heusol.close();
	return 1;
}

int Model::min(int a, int b)
{
	return (a > b) ? b : a;
}
int Model::max(int a, int b)
{
	return (a > b) ? a : b;
}

int Model::crtMP()
{
	MP = IloModel(env);
	MPSolver = IloCplex(MP);
	//MPSolver.setParam(IloCplex::RootAlg, IloCplex::Barrier);
	//MPSolver.setParam(IloCplex::BarCrossAlg, -1);
	//dual solutions
	DualNodeCap = IloNumArray2(env, N);	
	DualTrackCap = IloNumArray2(env, nTrack);
	DualDemand = IloNumArray2(env, nArc);			
	DualTmWindow = IloNumArray2(env, nArc);
	DualConvex = IloNumArray(env, nArc);

	Convex = IloRangeArray(env);
	//decision variables
	v_lambda = IloNumVarArray2(env, nVeh);
	v_y = IloNumVarArray2(env, nArc);
	v_s = IloNumVarArray2(env, nArc);
	for(v = 0; v < nVeh; v++){
		v_lambda[v] = IloNumVarArray(env);
		sprintf(buf, "Convex(%d)", v);
		Convex.add(IloRange(env,1,1,buf));
	}
	MP.add(Convex);
	for(k = 0; k < nArc; k++){
		v_y[k] = IloNumVarArray(env);
		v_s[k] = IloNumVarArray(env);
		for(t = 0; t < T; t++){
			sprintf(buf, "y[%d,%d][%d]", arc[k].from, arc[k].to, t);
			v_y[k].add(IloNumVar(env, 0, +IloInfinity, buf));
			sprintf(buf, "s[%d,%d][%d]", arc[k].from, arc[k].to, t);
			v_s[k].add(IloNumVar(env, 0, +IloInfinity, buf));
		}
	}

	//Objective function
	objMP = IloAdd(MP, IloMinimize(env));
	IloExpr objXpr(env);	
	for(k = 0; k < nArc; k++){		
		for(t = 0; t < T; t++){
			objXpr += penalty * arc[k].cost * v_y[k][t];
		}
	}
	objMP.setExpr(objXpr);
	
	//Constraints
	NodeCap = IloRangeArray2(env, N);	
	TrackCap = IloRangeArray2(env, nTrack);
	Demand = IloRangeArray2(env, nArc);			
	TmWindow = IloRangeArray2(env, nArc);
	for(i = 0; i < N; i++){
		DualNodeCap[i] = IloNumArray(env,T);
		NodeCap[i] = IloRangeArray(env, T);
		for(t = 0; t < T; t++){
			sprintf(buf, "Node Cap(%d,%d)", i, t);
			NodeCap[i][t] = IloAdd(MP, IloRange(env, -IloInfinity, maxD, buf));
		}
	}
	for(l = 0; l < nTrack; l++){
		DualTrackCap[l] = IloNumArray(env, T);
		TrackCap[l] = IloRangeArray(env, T);
		for(t = 0; t < T; t++){
			sprintf(buf, "Track Cap(%d,%d)", l, t);
			TrackCap[l][t] = IloAdd(MP, IloRange(env, -IloInfinity, maxL, buf));
		}
	}

	for(k = 0; k < nArc; k++){
		if(arc[k].from == arc[k].to) continue; 
		MP.add(IloRange(env, 0, v_s[k][T-1], 0, "TtlDmd"));
		DualDemand[k] = IloNumArray(env, T);
		DualTmWindow[k] = IloNumArray(env, T - TimeWindow);
		Demand[k] = IloRangeArray(env, T);
		TmWindow[k] = IloRangeArray(env, T - TimeWindow);
		for(t = 0; t < T; t++){
			sprintf(buf, "Demand(%d-%d,%d)", arc[k].from, arc[k].to, t);
			if(t == 0)	Demand[k][t] = IloAdd(MP, IloRange(env, dmd[t][k], v_s[k][t], dmd[t][k], buf));
			else		Demand[k][t] = IloAdd(MP, IloRange(env, dmd[t][k], v_s[k][t] - v_s[k][t-1], dmd[t][k], buf));

			if(t < T - TimeWindow){
				sprintf(buf, "TW(%d-%d,%d)", arc[k].from, arc[k].to, t); 
				TmWindow[k][t] = IloAdd(MP, IloRange(env, 0, v_y[k][t] - v_s[k][t], +IloInfinity, buf));
			}
		}
	}

	//initialize the MP
	Heuristic();							//apply the heuristic algorithm to get the intial columns
	for(v = 0; v < nVeh; v++){
		sprintf(buf, "lambda(%d,0)", v);
		v_lambda[v].add(IloNumVar(objMP(sys.routeCost[v])+Convex[v](1),0,+IloInfinity,ILOFLOAT,buf));
		for(t = 0; t < T; t++){
			for(k = 0; k < nArc; k++){
				i = arc[k].from; j = arc[k].to;
				if(i != j){
					Demand[k][t].setLinearCoef(v_lambda[v][0], sys.customer[k][v][t]);
					if(t < T - TimeWindow){
						int numCustomer = 0;
						for(tau = t+1; tau <= t+TimeWindow; tau++){
							numCustomer += sys.customer[k][v][tau];
						}
						TmWindow[k][t].setLinearCoef(v_lambda[v][0], numCustomer);
					}
				}
				else{
					NodeCap[i][t].setLinearCoef(v_lambda[v][0],sys.route[k][v][t]);
				}
			}

			for(l = 0; l < nTrack; l++){
				for(k = 0; k < nArc; k++){
					if(arc[k].from == arc[k].to) continue;
					for(vector<int>::iterator itr = arc[k].track.begin(); itr != arc[k].track.end(); itr++){
						if(l == (*itr) && t - abs(track[l].from - arc[k].from) >= 0 ){
							TrackCap[l][t].setLinearCoef(v_lambda[v][0], sys.route[k][v][t - abs(track[l].from - arc[k].from)]);						
						}	
					}
				}				
			}

		}
	}
	
#ifdef MODEL_EXPORT
	MPSolver.exportModel("output/MP.lp");
#endif
	return 1;
}

int Model::crtSP()
{
	//model, variables and parameter definitions
	SP = IloModel(env);
	SPSolver = IloCplex(SP);
	v_x = IloNumVarArray2(env, nArc);
	v_z = IloNumVarArray2(env, nArc);
	v_e = IloNumVarArray(env);

	val_x = IloNumArray2(env, nArc);
	val_z = IloNumArray2(env, nArc);
	
	for(k = 0; k < nArc; k++){
		v_x[k] = IloNumVarArray(env);
		v_z[k] = IloNumVarArray(env);
		val_x[k] = IloNumArray(env, T);
		val_z[k] = IloNumArray(env, T);
		for(t = 0; t < T; t++){
			sprintf(buf, "x[%d,%d][%d]", arc[k].from, arc[k].to, t);
			v_x[k].add(IloNumVar(env, 0, +IloInfinity, ILOFLOAT, buf));
			sprintf(buf, "z[%d,%d][%d]", arc[k].from, arc[k].to, t);
			v_z[k].add(IloNumVar(env, 0, 1, ILOINT, buf));
		}
	}
	for(t = 0; t < T; t++){
		sprintf(buf, "e[%d]", t);
		v_e.add(IloNumVar(env, 0, vehicle[0].maxPowLvl, buf));
	}

	//objective function
	objSP = IloAdd(SP, IloMinimize(env));
	costXpr = IloExpr(env);
	for(k = 0; k < nArc; k++){		
		for(t = 0; t < T; t++){
			costXpr += arc[k].fcost * v_z[k][t] + arc[k].cost * v_x[k][t];
		}
	}

	//Constraints
	FlowBalance = IloRangeArray2(env, N);
	//electricity for travel constraints	
	for(t = 1; t < T; t++){
		IloExpr xpr(env);
		for( k = 0; k < nArc; k++){
			i = arc[k].from; j = arc[k].to;
			if(i != j){
				sprintf(buf, "MinPowLvl(%d-%d,%d)", arc[k].from, arc[k].to, t);
				SP.add(IloRange(env, -IloInfinity, arc[k].fcost * v_z[k][t] + arc[k].cost * v_x[k][t] - v_e[t], 0, buf));

				for(tau = max(t - arc[k].time, 0); tau < t; tau++){
					xpr -= (arc[k].fcost * v_z[k][tau] + arc[k].cost * v_x[k][tau]) / (double)(abs(i - j)) ;
				}
			}
			else{
				xpr += maxC * v_z[k][t-1];
			}
		}
		xpr += v_e[t-1] - v_e[t];
		sprintf(buf, "Battery Level(%d)", t);
		SP.add(IloRange(env, 0, xpr, +IloInfinity, buf));
		xpr.end();
	}
	//vehicle capacity constraints
	for(k = 0; k < nArc; k++){
		for(t = 0; t < T; t++){
			sprintf(buf, "Vehicle Capacity(%d-%d,%d)", arc[k].from, arc[k].to, t);
			SP.add(IloRange(env, -IloInfinity, v_x[k][t] - vehicle[0].cap * v_z[k][t], 0, buf));
		}
	}

	//self flow balance constraints
	for(i = 0; i < N; i++){
		FlowBalance[i] = IloRangeArray(env);
		for(t = 0; t < T; t++){
			IloExpr xpr(env);
			for(k = 0; k < nArc; k++){
				if(arc[k].from == arc[k].to && arc[k].from == i){
					if(t == 0)	xpr += v_z[k][t];
					else		xpr += v_z[k][t] - v_z[k][t-1];
				}
				else if(arc[k].from == i){
					xpr += v_z[k][t];
				}	
				else if(arc[k].to == i){
					tau = t - arc[k].time;
					if(tau >= 0) xpr -= v_z[k][tau];
				}
			}
			sprintf(buf, "flow bal(%d,%d)", i,t);
			if(t == 0)															//the vehicle stops at station i at t=0
				FlowBalance[i].add(IloRange(env, 1, xpr, 1, buf));
			else	
				FlowBalance[i].add(IloRange(env, 0, xpr, 0, buf));
			xpr.end();
		}					
	}
	
	return 1;
}

int Model::solveMP()
{
	MPSolver.solve();
	UB = MPSolver.getObjValue();
	//get duals
	for(i = 0; i < N; i++){
		MPSolver.getDuals(DualNodeCap[i], NodeCap[i]);
	}	
	for(l = 0; l < nTrack; l++){
		MPSolver.getDuals(DualTrackCap[l], TrackCap[l]);
	}	
	for(k = 0; k < nArc; k++){
		if(arc[k].from == arc[k].to)	continue;
		MPSolver.getDuals(DualDemand[k], Demand[k]);
		MPSolver.getDuals(DualTmWindow[k], TmWindow[k]);
	}
	MPSolver.getDuals(DualConvex, Convex);
	return 1;
}

int Model::solveSP()
{
	//update the ofv of SP
	for(k = 0; k < nArc; k++){
		if(arc[k].from != arc[k].to){
			for(t = 0; t < T; t++){
				if(t < T - TimeWindow){
					objSP.setLinearCoef(v_x[k][t], arc[k].cost - DualDemand[k][t] - DualTmWindow[k][t]);
				}
				else{
					objSP.setLinearCoef(v_x[k][t], arc[k].cost - DualDemand[k][t]);
				}

				IloNum temp = arc[k].fcost;
				for(vector<int>::iterator itr = arc[k].track.begin(); itr != arc[k].track.end(); itr++){
					temp -= DualTrackCap[*itr][t];
				}
				objSP.setLinearCoef(v_z[k][t], temp);
			}
			
		}
		else{
			i = arc[k].to;
			for(t = 0; t < T; t++){
				objSP.setLinearCoef(v_z[k][t], DualNodeCap[i][t]);
			}
		}
	}

	//add respective constraints to SP's and solve them
	NLB = UB;
	for(n = 0; n < N; n++){
		SP.add(FlowBalance[n]);
		for(v = 0; v < nVeh; v++){
			if(vehicle[v].to != n)	continue;
			v_e[0].setUB(vehicle[v].powLvl);
			SPSolver.solve();
			//obtain columns and add to MP
			for(k = 0; k < nArc; k++){
				SPSolver.getValues(v_z[k],val_z[k]);
				SPSolver.getValues(v_x[k],val_x[k]);				
				SPSolver.addMIPStart(v_z[k],val_z[k]);
				SPSolver.addMIPStart(v_x[k],val_x[k]);
			}
			if(!addColumns(v))	
				cerr << "Fail to add columns!";
			NLB += SPSolver.getObjValue() - DualConvex[v];
		}
		SP.remove(FlowBalance[n]);
	}

	return 1;
}

int Model::addColumns(int v)
{
	sprintf(buf, "lambda(%d,%d)", v, itn);
	v_lambda[v].add(IloNumVar(objMP(SPSolver.getValue(costXpr))+Convex[v](1), 0, +IloInfinity, ILOFLOAT, buf));
	for(t = 0; t < T; t++){
		for(k = 0; k < nArc; k++){
			i = arc[k].from; j = arc[k].to;
			if(i != j){
				Demand[k][t].setLinearCoef(v_lambda[v][itn], val_x[k][t]);
				if(t < T - TimeWindow){
					int numCustomer = 0;
					for(tau = t+1; tau <= t+TimeWindow; tau++){
						numCustomer += val_x[k][tau];
					}
					TmWindow[k][t].setLinearCoef(v_lambda[v][itn],numCustomer);
				}
			}
			else{
				NodeCap[i][t].setLinearCoef(v_lambda[v][itn],val_z[k][t]);
			}
		}

		for(l = 0; l < nTrack; l++){
			for(k = 0; k < nArc; k++){
				if(arc[k].from == arc[k].to) continue;
				for(vector<int>::iterator itr = arc[k].track.begin(); itr != arc[k].track.end(); itr++){
					if(l == (*itr) && t - abs(track[l].from - arc[k].from) >= 0 ){
						TrackCap[l][t].setLinearCoef(v_lambda[v][itn], val_z[k][t - abs(track[l].from - arc[k].from)]);						
					}	
				}
			}				
		}
	}
	return 1;
}