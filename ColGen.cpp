#include "stdafx.h"
#include "prt.h"

//---------------------------------------------------------------------//
//Column generation is applied in this cpp: The subproblem is to obtain routes for each vehicle and are decomposed to subproblems corresponding
//to each group of vehicles which stays at a certain station at time 0
//----------------------------------------------------------------------//


int Model::ColGen()
{
	UB = +IloInfinity; LB = -IloInfinity; NLB = -IloInfinity; gap = 1.0;
	itn = 0;
	initOutputColgen();
	_start = clock();
	crtMP();
	crtSP();

	while(1)
	{
		itn++;	
		solveMP();
		solveSP();

		if (terminateColgen())
		{
			break;
		}
	}

	finalizeColgen();
	
	return 1;
}

int Model::crtMP()
{
	MP = IloModel(env);
	MPSolver = IloCplex(MP);
	MPSolver.setParam(IloCplex::TiLim,120);
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
		objXpr += penalty * arc[k].cost * v_s[k][T - 1];
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
		//MP.add(IloRange(env, 0, v_s[k][T-1], 0, "TtlDmd"));
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
	Heuristic2();							//apply the heuristic algorithm to get the intial columns
	for(v = 0; v < nVeh; v++){
		sprintf(buf, "lambda(%d,0)", v);
		v_lambda[v].add(IloNumVar(objMP(sys.routeCost[v])+Convex[v](1),0,1,ILOFLOAT,buf));
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
	sprintf(buf, "%soutput/CG_MP.lp", directoryPath.c_str());
	MPSolver.exportModel(buf);
#endif
	return 1;
}

int Model::crtSP()
{
	//model, variables and parameter definitions
	SP = IloModel(env);
	SPSolver = IloCplex(SP);
	v_x = IloNumVarArray3(env, nArc);
	v_z = IloNumVarArray3(env, nArc);
	v_e = IloNumVarArray2(env, nVeh);

	val_x = IloNumArray3(env, nArc);
	val_z = IloNumArray3(env, nArc);
	
	for(k = 0; k < nArc; k++){
		v_x[k] = IloNumVarArray2(env, nVeh);
		v_z[k] = IloNumVarArray2(env, nVeh);
		val_x[k] = IloNumArray2(env, nVeh);
		val_z[k] = IloNumArray2(env, nVeh);
		for(v = 0; v < nVeh; v++){
			v_x[k][v] = IloNumVarArray(env);
			v_z[k][v] = IloNumVarArray(env);
			val_x[k][v] = IloNumArray(env, T);
			val_z[k][v] = IloNumArray(env, T);
			for(t = 0; t < T; t++){
				sprintf(buf, "x[%d,%d][%d][%d]", arc[k].from, arc[k].to, v, t);
				v_x[k][v].add(IloNumVar(env, 0, +IloInfinity, ILOFLOAT, buf));
				sprintf(buf, "z[%d,%d][%d][%d]", arc[k].from, arc[k].to, v, t);
				v_z[k][v].add(IloNumVar(env, 0, 1, ILOINT, buf));
			}
		}		
	}
	for(v = 0; v < nVeh; v++){
		v_e[v] = IloNumVarArray(env);
		for(t = 0; t <= vehicle[v].time; t++){
			sprintf(buf, "e[%d][%d]", v, t);
			v_e[v].add(IloNumVar(env, 0, vehicle[v].powLvl, buf));
		}
		for(t = vehicle[v].time+1; t < T; t++){
			sprintf(buf, "e[%d][%d]", v, t);
			v_e[v].add(IloNumVar(env, 0, vehicle[v].maxPowLvl, buf));
		}
	}

	//objective function
	objSP = IloAdd(SP, IloMinimize(env));
	costXpr = IloExpr(env);

	//Constraints
	Battery = IloRangeArray2(env, N);
	VehCap = IloRangeArray2(env, N);
	FlowBalance = IloRangeArray2(env, N);
	for(n = 0; n < N; n++){
		Battery[n] = IloRangeArray(env);
		VehCap[n] = IloRangeArray(env);
		FlowBalance[n] = IloRangeArray(env);
	}
	//electricity for travel constraints	
	for(v = 0; v < nVeh; v++){
		n = vehicle[v].to;
		for(t = vehicle[v].time + 1; t < T; t++){
			IloExpr xpr(env);
			for( k = 0; k < nArc; k++){
				i = arc[k].from; j = arc[k].to;
				if(i != j){
					sprintf(buf, "MinPowLvl(%d-%d,%d)", arc[k].from, arc[k].to, t);
					Battery[n].add(IloRange(env, -IloInfinity, arc[k].fcost * v_z[k][v][t] + arc[k].cost * v_x[k][v][t] - v_e[v][t], 0, buf));

					for(tau = max(t - arc[k].time, 0); tau < t; tau++){
						xpr -= (arc[k].fcost * v_z[k][v][tau] + arc[k].cost * v_x[k][v][tau]) / (double)(abs(i - j)) ;
					}
				}
				else{
					xpr += maxC * v_z[k][v][t-1];
				}
			}
			xpr += v_e[v][t-1] - v_e[v][t];
			sprintf(buf, "Battery Level(%d,%d)",v, t);
			Battery[n].add(IloRange(env, 0, xpr, +IloInfinity, buf));
			xpr.end();
		}
	}
	//vehicle capacity constraints
	for(v = 0; v < nVeh; v++){
		n = vehicle[v].to;
		for(k = 0; k < nArc; k++){
			for(t = 0; t < T; t++){
				sprintf(buf, "Vehicle Capacity(%d-%d,%d,%d)", arc[k].from, arc[k].to, v, t);
				VehCap[n].add(IloRange(env, -IloInfinity, v_x[k][v][t] - vehicle[v].cap * v_z[k][v][t], 0, buf));
			}
		}
	}

	//self flow balance constraints
	for(i = 0; i < N; i++){
		for(v = 0; v < nVeh; v++){
			n = vehicle[v].to;
			if(vehicle[v].from == vehicle[v].to)								//the vehicle stops at some station
			{	
				for(t = 0; t < T; t++){
					IloExpr xpr(env);
					for(k = 0; k < nArc; k++){
						if(arc[k].from == arc[k].to && arc[k].from == i){
							if(t == 0)	xpr += v_z[k][v][t];
							else		xpr += v_z[k][v][t] - v_z[k][v][t-1];
						}
						else if(arc[k].from == i){
							xpr += v_z[k][v][t];
						}	
						else if(arc[k].to == i){
							tau = t - arc[k].time;
							if(tau >= 0) xpr -= v_z[k][v][tau];
						}
					}
					sprintf(buf, "flow bal(%d,%d,%d)", i, v, t);
					if(t == 0 && vehicle[v].from == i)							//the vehicle stops at station i at t=0
						FlowBalance[n].add(IloRange(env, 1, xpr, 1, buf));
					else	
						FlowBalance[n].add(IloRange(env, 0, xpr, 0, buf));
					xpr.end();
				}
			}
			else																//the vehicle is moving
			{
				for(t = 0; t < vehicle[v].time; t++){							//vehicle is moving from t=0 to t=vehicle[v].time
					for(k = 0; k < nArc; k++){
						FlowBalance[n].add(IloRange(env, 0, v_z[k][v][t], 0));
					}
				}
				for(t = vehicle[v].time; t < T; t++){
					IloExpr xpr(env);
					for(k = 0; k < nArc; k++){
						if(arc[k].from == arc[k].to && arc[k].from == i){
							if(t == vehicle[v].time)	xpr += v_z[k][v][t];
							else		xpr += v_z[k][v][t] - v_z[k][v][t-1];
						}
						else if(arc[k].from == i){
							xpr += v_z[k][v][t];
						}	
						else if(arc[k].to == i){
							tau = t - arc[k].time;
							if(tau >= vehicle[v].time) xpr -= v_z[k][v][tau];
						}
					}
					sprintf(buf, "flow bal(%d,%d,%d)", i, v, t);
					if(t == vehicle[v].time && vehicle[v].to == i)							//the vehicle stops at station i at t=0
						FlowBalance[n].add(IloRange(env, 1, xpr, 1, buf));
					else	
						FlowBalance[n].add(IloRange(env, 0, xpr, 0, buf));
					xpr.end();
				}
			}
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
	NLB = UB;
	//update decomposed SP one by one and then solve them respectively
	for(n = 0; n < N; n++){
		//update OF
		IloExpr objXpr(env);
		for(v = 0; v < nVeh; v++){
			if(vehicle[v].to != n)	continue;		
			for(k = 0; k < nArc; k++){
				if(arc[k].from != arc[k].to){
					for(t = 0; t < T; t++){
						IloNum temp1 = arc[k].cost - DualDemand[k][t];
						for(tau = t - 1; tau >= 0 && tau >= t - TimeWindow && tau < T - TimeWindow; tau--){
							temp1 -= DualTmWindow[k][tau];
						}
						objXpr.setLinearCoef(v_x[k][v][t], temp1);

						IloNum temp2 = arc[k].fcost;
						for(vector<int>::iterator itr = arc[k].track.begin(); itr != arc[k].track.end(); itr++){
							tau = t + abs(track[*itr].from - arc[k].from);
							if(tau < T)	temp2 -= DualTrackCap[*itr][tau];
						}
						objXpr.setLinearCoef(v_z[k][v][t], temp2);
					}
			
				}
				else{
					i = arc[k].to;
					for(t = vehicle[v].time; t < T; t++){
						objXpr.setLinearCoef(v_z[k][v][t], -DualNodeCap[i][t]);
					}
				}
			}
		}
		objSP.setExpr(objXpr);
		objXpr.end();
		//update constraints
		SP.add(Battery[n]);
		SP.add(VehCap[n]);
		SP.add(FlowBalance[n]);

		//solve SP and get columns
#ifdef TEST_MODE
		if(itn >= 40 && itn <= 42)
		{
			sprintf(buf, "output/SP%d_%d.lp", itn, n);
			SPSolver.exportModel(buf);
		}
#endif
		SPSolver.solve();
		NLB += SPSolver.getObjValue();
		for(v = 0; v < nVeh; v++){
			if(vehicle[v].to != n)	continue;
#ifndef TEST_MODE
			if(itn >= 55 && itn <=57 && (v == 2 || v==6 || v == 12)){
				output << "route for vehicle " << v << ":\t";			
				for(t = 0; t < T; t++){
					for(k = 0; k < nArc; k++){
						if(SPSolver.getValue(v_z[k][v][t]) > .99){
							if(t == 0)	output << arc[k].from;
							for(i = 1; i < abs(arc[k].from - arc[k].to); i++)
								output << "--*";
							output << "--" << arc[k].to;
						}
					}
				}
				output << endl;
				output << "customers for vehicle " << v << ":\t";			
				for(t = 0; t < T; t++){
					for(k = 0; k < nArc; k++){
						if(SPSolver.getValue(v_z[k][v][t]) > .99){
							if(t == 0)	output << "0";
							for(i = 1; i < abs(arc[k].from - arc[k].to); i++)
								output << "--*";
							output << "--" << SPSolver.getValue(v_x[k][v][t]);
						}
					}
				}
				output << endl;
			}		
#endif
			for(k = 0; k < nArc; k++){
				SPSolver.getValues(v_z[k][v],val_z[k][v]);
				SPSolver.getValues(v_x[k][v],val_x[k][v]);				
				//SPSolver.addMIPStart(v_z[k][v],val_z[k][v]);
				//SPSolver.addMIPStart(v_x[k][v],val_x[k][v]);
			}
			if(!addColumns(v))	cerr << "Fail to add columns!";
			NLB -= DualConvex[v];
		}

		//remove constraints
		SP.remove(Battery[n]);
		SP.remove(VehCap[n]);
		SP.remove(FlowBalance[n]);
	}

	return 1;
}

int Model::addColumns(int v)
{
	double colCost = 0;
	for(k = 0; k < nArc; k++){		
		for(t = 0; t < T; t++){
			colCost += arc[k].fcost * val_z[k][v][t] + arc[k].cost * val_x[k][v][t];
		}
	}
	sprintf(buf, "lambda(%d,%d)", v, itn);
	v_lambda[v].add(IloNumVar(objMP(colCost) + Convex[v](1), 0, 1, ILOFLOAT, buf));
	for(t = 0; t < T; t++){
		for(k = 0; k < nArc; k++){
			i = arc[k].from; j = arc[k].to;
			if(i != j){
				Demand[k][t].setLinearCoef(v_lambda[v][itn], val_x[k][v][t]);
				if(t < T - TimeWindow){
					double numCustomer = 0;
					for(tau = t+1; tau <= t+TimeWindow; tau++){
						numCustomer += val_x[k][v][tau];
					}
					TmWindow[k][t].setLinearCoef(v_lambda[v][itn],numCustomer);
				}
			}
			else{
				NodeCap[i][t].setLinearCoef(v_lambda[v][itn],val_z[k][v][t]);
			}
		}

		for(l = 0; l < nTrack; l++){
			for(k = 0; k < nArc; k++){
				if(arc[k].from == arc[k].to) continue;
				for(vector<int>::iterator itr = arc[k].track.begin(); itr != arc[k].track.end(); itr++){
					if(l == (*itr) && t - abs(track[l].from - arc[k].from) >= 0 ){
						TrackCap[l][t].setLinearCoef(v_lambda[v][itn], val_z[k][v][t - abs(track[l].from - arc[k].from)]);						
					}	
				}
			}				
		}
	}
	return 1;
}

void Model::initOutputColgen()
{
	sprintf(path, "%soutput/ColGen_%dV_%dN_%dArc_%dT.txt", directoryPath.c_str(), nVeh, N, nArc, T);
	string outputFile(path);
	output.open(outputFile);
	output << "Iteration\tUB\tLB\tNLB\tGap\n";
	output << itn << "\t" << UB << "\t" << LB << "\t" << NLB << "\t" << gap << endl;
}

bool Model::terminateColgen()
{
	gap = (NLB > LB) ? ((UB - NLB) / (1e-75 + UB)) : ((UB - LB) / (1e-75 + UB));
	output << itn << "\t" << UB << "\t" << LB << "\t" << NLB << "\t" << gap << endl;
	LB = (NLB > LB) ? NLB : LB;
	if (gap < 0.01)	
	{ 
		return true; 
	}
	_end = clock();
	cmp_time = (double)(_end - _start) / (double)CLOCKS_PER_SEC;
	if (cmp_time > WallTime)
	{
		return true;
	}

	return false;
}

void Model::finalizeColgen()
{
	MPSolver.solve();
	output << "MP OFV = " << MPSolver.getObjValue() << endl;
	output << "Solution from CG:\n";
	for (v = 0; v < nVeh; v++){
		output << "Vehicle " << v << ":\t";
		IloNumArray val_lambda(env, v_lambda[v].getSize());
		MPSolver.getValues(v_lambda[v], val_lambda);
		output << val_lambda << endl;
		/*j = v_lambda[v].getSize();
		for(i = 0; i < j; i++){
		if(MPSolver.getBasisStatus(v_lambda[v][i]) == IloCplex::AtLower){
		objMP.setLinearCoef(v_lambda[v][i], 0);
		for(t = 0; t < T; t++){
		for(k = 0; k < nArc; k++){
		if(arc[k].from == arc[k].to)	continue;
		if(t < T - TimeWindow)	TmWindow[k][t].setLinearCoef(v_lambda[v][i], 0);
		Demand[k][t].setLinearCoef(v_lambda[v][i], 0);
		}
		for(n = 0; n < N; n++){
		NodeCap[n][t].setLinearCoef(v_lambda[v][i], 0);
		}
		for(l = 0; l < nTrack; l++){
		TrackCap[l][t].setLinearCoef(v_lambda[v][i], 0);
		}
		}
		Convex[v].setLinearCoef(v_lambda[v][i], 0);
		}
		}*/
	}

	for (v = 0; v < nVeh; v++){
		MP.add(IloConversion(env, v_lambda[v], ILOBOOL));
	}
	MPSolver.solve();

	output << "Computational Time =" << cmp_time << "sec" << endl;
	output << "Real UB = " << MPSolver.getObjValue() << endl;
	output.close();
}