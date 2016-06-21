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
#endif
	
	//extensive model
	IloEnv		env;
	IloModel	PRT(env);
	IloCplex	PRTSolver(PRT);

	//decision variables
	IloBoolVarArray3	var_x(env, nArc);		//whether to assign a vehicle from i to j at time t
	IloNumVarArray2		var_e(env, nVeh);		//electricity level for vehicle k at the beginning of time t

	for(k = 0; k < nArc; k++){
		var_x[k] = IloBoolVarArray2(env, nVeh);
		for(v = 0; v < nVeh; v++){
			var_x[k][v] = IloBoolVarArray(env);
			for(t = 0; t < T; t++){
				sprintf(buf, "x[%d,%d][%d][%d]", arc[k].from, arc[k].to, v, t);
				var_x[k][v].add(IloBoolVar(env,buf));
			}
		}
	}

	for(v = 0; v < nVeh; v++){
		var_e[v] = IloNumVarArray(env);
		for(t = 0; t < T; t++){
			sprintf(buf, "e[%d][%d]", v, t);
			if(t == 0)	
				var_e[v].add(IloNumVar(env, 0, vehicle[v].powLvl, buf));
			else	
				var_e[v].add(IloNumVar(env, 0, vehicle[v].maxPowLvl, buf));
		}
	}

	//objective function
	IloExpr objXpr(env);	
	for(k = 0; k < nArc; k++){
		for(v = 0; v < nVeh; v++){
			for(t = 0; t < T; t++){
				objXpr += arc[k].cost * var_x[k][v][t];
			}
		}
	}
	PRT.add(IloMinimize(env, objXpr));

	//electricity for travel constraints	
	for( v = 0; v < nVeh; v++){
		for(t = 1; t < T; t++){
			IloExpr xpr(env);
			for( k = 0; k < nArc; k++){
				i = arc[k].from; j = arc[k].to;
				if(i != j){
					sprintf(buf, "MinPowLvl(%d-%d,%d,%d)", arc[k].from, arc[k].to, v, t);
					PRT.add(IloRange(env, -IloInfinity, arc[k].cost * var_x[k][v][t] - var_e[v][t], 0, buf));

					for(tau = t - arc[k].time; tau >=0 && tau < t; tau++){
						xpr -= arc[k].cost / (double)(abs(i - j)) * var_x[k][v][tau];
					}
				}
				else{
					xpr += maxC * var_x[k][v][t-1];
				}
			}
			xpr += var_e[v][t-1] - var_e[v][t];
			sprintf(buf, "Dynamic E Level(%d,%d)", v, t);
			PRT.add(IloRange(env, 0, xpr, +IloInfinity, buf));
			xpr.end();
		}
	}
	//demand satisfaction constraints
	for(k = 0; k < nArc; k++){
		if(arc[k].from == arc[k].to) continue; 
		for(t = 0; t < T; t++){
			IloExpr xpr(env);
			for(v = 0; v < nVeh; v++){
				xpr += vehicle[v].cap * var_x[k][v][t];
			}
			sprintf(buf, "Dmd(%d-%d,%d)", arc[k].from, arc[k].to, t); 
			PRT.add(IloRange(env, dmd[t][k], xpr, +IloInfinity, buf));
			xpr.end();
		}
	}
	//Depot buffer capacity constraints
	for(k = 0; k < nArc; k++){
		if(arc[k].from != arc[k].to) continue;
		for(t = 0; t < T; t++){
			IloExpr xpr(env);
			for(v = 0; v < nVeh; v++){
				xpr += var_x[k][v][t];
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
							xpr += var_x[k][v][t - abs(track[l].from - arc[k].from)];
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
			for(t = 0; t < T; t++){
				IloExpr xpr(env);
				for(k = 0; k < nArc; k++){
					if(arc[k].from == arc[k].to && arc[k].from == i){
						if(t == 0)	xpr += var_x[k][v][t];
						else		xpr += var_x[k][v][t] - var_x[k][v][t-1];
					}
					else if(arc[k].from == i){
						xpr += var_x[k][v][t];
					}	
					else if(arc[k].to == i){
						tau = t - arc[k].time;
						if(tau >= 0) xpr -= var_x[k][v][tau];
					}
				}
				sprintf(buf, "flow bal(%d,%d,%d)", i, v, t);
				if(t == 0)	PRT.add(IloRange(env, 1, xpr, 1, buf));
				else	PRT.add(IloRange(env, 0, xpr, 0, buf));
			}
		}
	}


#ifdef TEST_MODE
	PRTSolver.exportModel("./output/PRT.lp");
#endif

	PRTSolver.solve();
	//PRTSolver.writeSolution("./output/solution.lp");
	return 1;
}