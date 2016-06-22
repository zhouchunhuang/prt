#include "stdafx.h"
#include "prt.h"

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
			var_e[v].add(IloNumVar(env, 5, vehicle[v].maxPowLvl, buf));
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
		objXpr += penalty * arc[k].cost * var_s[k][T-1];
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
	PRTSolver.exportModel("../output/PRT.lp");
#endif

	_start = clock();
	PRTSolver.solve();
	_end = clock();
	cmp_time = (double)(_end - _start)/CLOCKS_PER_SEC;

	//PRTSolver.writeSolution("../output/solution.lp");
	outputSol();

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
	sprintf(path, "%soutput/routes_%dV_%dN_%dArc_%dT.txt", directoryPath.c_str(), nVeh, N, nArc, T);
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

int Model::min(int a, int b)
{
	return (a > b) ? b : a;
}
int Model::max(int a, int b)
{
	return (a > b) ? a : b;
}

