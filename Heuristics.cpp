#include "stdafx.h"
#include "prt.h"

int Model::Heuristic()
{
	_start = clock();
	initSystem();
	for (t = 0; t < T; t++){
		for (v = 0; v < nVeh; v++){
			sys.vehicle[v].assigned = false;															//assigned is false by default: true if either assigned at the current time or moving
		}
		for (k = 0; k < nArc; k++){
			if (arc[k].from == arc[k].to) continue;
			if (t)	sys.Demand[t][k] += sys.Demand[t - 1][k];
			if (!(sys.Demand[t][k]))	continue;
			for (v = 0; v < nVeh; v++){
				if (sys.vehicle[v].to != arc[k].from)	continue;									//don't assign the vehicle if it is not in the station
				if (sys.vehicle[v].time != t){														//don't use the vehicle if it is moving											
					sys.vehicle[v].assigned = true;
					continue;
				}
				if (sys.vehicle[v].powLvl < arc[k].fcost + min(sys.Demand[t][k], sys.vehicle[v].cap) * arc[k].cost)	continue;	//don't use the vehicle if battery level is not enough
				if (!(sys.Demand[t][k]))	break;																					//break if demand is none
				bool overload = false;
				for (vector<int>::iterator itr = arc[k].track.begin(); itr != arc[k].track.end(); itr++){
					tau = t + abs(track[*itr].from - arc[k].from);
					if (tau < T && sys.TrackLoad[*itr][tau] >= maxL){
						overload = true;
						break;
					}
				}
				if (overload)	break;

				sys.route[k][v][t] = 1;																			//assign the vehicle which is available
				sys.customer[k][v][t] = min(sys.Demand[t][k], sys.vehicle[v].cap);								//the number of customers sent out
				sys.vehicle[v].powLvl -= arc[k].fcost + arc[k].cost * sys.customer[k][v][t];					//update vehicle status: battery level
				sys.vehicle[v].to = arc[k].to;																	//update vehicle status: arriving station
				sys.vehicle[v].time = t + arc[k].time;															//update vehicle status: arriving time
				sys.vehicle[v].assigned = true;																	//update vehicle status: assigned or not				
				sys.Demand[t][k] -= sys.customer[k][v][t];														//update demand	
				for (vector<int>::iterator itr = arc[k].track.begin(); itr != arc[k].track.end(); itr++){		//update track load
					tau = t + abs(track[*itr].from - arc[k].from);
					if (tau < T)	sys.TrackLoad[*itr][tau]++;
				}
			}
		}
		for (k = 0; k < nArc; k++){
			if (arc[k].from != arc[k].to)	continue;
			for (v = 0; v < nVeh; v++){
				if (!(sys.vehicle[v].assigned) && sys.vehicle[v].to == arc[k].to && sys.vehicle[v].time == t){
					sys.route[k][v][t] = 1;						//stay in the station if the vehicle is not assigned toward any other stations
					sys.NodeLoad[arc[k].to][t]++;				//update node load
					sys.vehicle[v].time = t + 1;					//update time
					sys.vehicle[v].powLvl += maxC;
				}
			}
		}
		//if node load maximum is reached -- cancel vehicles toward this station and/or cancel previous route
	}
	// compute the total cost
	sys.cost = 0;
	for (k = 0; k < nArc; k++){
		if (arc[k].from == arc[k].to)	continue;
		for (t = 0; t < T; t++){
			int val_y = sys.Demand[t][k];
			for (v = 0; v < nVeh; v++){
				sys.cost += arc[k].fcost * sys.route[k][v][t] + arc[k].cost * sys.customer[k][v][t];
				for (tau = t + 1; tau <= t + TimeWindow && tau < T; tau++){
					val_y -= sys.customer[k][v][tau];
				}
			}
			val_y = max(val_y, 0);
			sys.cost += penalty * arc[k].cost * val_y;
		}
	}

	for (v = 0; v < nVeh; v++){
		double rCost = 0;
		for (k = 0; k < nArc; k++){
			for (t = 0; t < T; t++){
				rCost += arc[k].fcost * sys.route[k][v][t] + arc[k].cost * sys.customer[k][v][t];
			}
		}
		sys.routeCost.push_back(rCost);
	}

	_end = clock();
	cmp_time = (double)(_end - _start) / CLOCKS_PER_SEC;

	sprintf(path, "%soutput/heuristic_%dV_%dN_%dArc_%dT.txt", directoryPath.c_str(), nVeh, N, nArc, T);
	heuristicSol();

	return 1;
}

int Model::Heuristic2()
{
	_start = clock();
	initSystem();
	for (t = 0; t < T; t++){
		//sort the arcs according to its demand
		vector<pair<int, double>> ArcDmd;
		for (k = 0; k < nArc; k++){
			if (arc[k].from == arc[k].to)	continue;
			if (t)	sys.Demand[t][k] += sys.Demand[t - 1][k];
			ArcDmd.push_back(pair<int, double>(k, penalty * arc[k].cost * sys.Demand[t][k]));
		}
		sort(ArcDmd.begin(), ArcDmd.end(),
			[](const pair<int, double>& lhs, const pair<int, double>& rhs) {
			return lhs.second > rhs.second; });
		//sort the vehicle battery level
		vector<pair<int, double>> Elvl;
		for (v = 0; v < nVeh; v++){
			sys.vehicle[v].assigned = false;															//reset all vehicles
			Elvl.push_back(pair<int, double>(v, sys.vehicle[v].powLvl));
		}
		sort(Elvl.begin(), Elvl.end(),
			[](const pair<int, double>& lhs, const pair<int, double>& rhs) {
			return lhs.second > rhs.second; });

		for (vector<pair<int, double>>::iterator vecItr = ArcDmd.begin(); vecItr != ArcDmd.end(); vecItr++){
			k = vecItr->first;
			if (arc[k].from == arc[k].to) continue;
			if (!(sys.Demand[t][k]))	continue;
			for (vector<pair<int, double>>::iterator vecItr2 = Elvl.begin(); vecItr2 != Elvl.end(); vecItr2++){
				v = vecItr2->first;
				if (sys.vehicle[v].to != arc[k].from)	continue;									//don't assign the vehicle if it is not in the station
				if (sys.vehicle[v].time != t){														//don't use the vehicle if it is moving											
					sys.vehicle[v].assigned = true;
					continue;
				}
				if (sys.vehicle[v].powLvl < arc[k].fcost + min(sys.Demand[t][k], sys.vehicle[v].cap) * arc[k].cost)	continue;	//don't use the vehicle if battery level is not enough
				if (!(sys.Demand[t][k]))	break;																					//break if demand is none
				bool overload = false;
				for (vector<int>::iterator itr = arc[k].track.begin(); itr != arc[k].track.end(); itr++){
					tau = t + abs(track[*itr].from - arc[k].from);
					if (tau < T && sys.TrackLoad[*itr][tau] >= maxL){
						overload = true;
						break;
					}
				}
				if (overload)	break;																			//no more vehicle assignment for this arc if overloaded

				sys.route[k][v][t] = 1;																			//assign the vehicle which is available
				sys.customer[k][v][t] = min(sys.Demand[t][k], sys.vehicle[v].cap);								//the number of customers sent out
				sys.vehicle[v].powLvl -= arc[k].fcost + arc[k].cost * sys.customer[k][v][t];					//update vehicle status: battery level
				sys.vehicle[v].to = arc[k].to;																	//update vehicle status: arriving station
				sys.vehicle[v].time = t + arc[k].time;															//update vehicle status: arriving time
				sys.vehicle[v].assigned = true;																	//update vehicle status: assigned or not

				sys.Demand[t][k] -= sys.customer[k][v][t];														//update demand	
				for (vector<int>::iterator itr = arc[k].track.begin(); itr != arc[k].track.end(); itr++){		//update track load
					tau = t + abs(track[*itr].from - arc[k].from);
					if (tau < T)	sys.TrackLoad[*itr][tau]++;
				}
			}
		}
		for (k = 0; k < nArc; k++){
			if (arc[k].from != arc[k].to)	continue;
			for (v = 0; v < nVeh; v++){
				if (!(sys.vehicle[v].assigned) && sys.vehicle[v].to == arc[k].to && sys.vehicle[v].time == t){
					sys.route[k][v][t] = 1;						//stay in the station if the vehicle is not assigned toward any other stations
					sys.NodeLoad[arc[k].to][t]++;				//update node load
					sys.vehicle[v].time = t + 1;					//update time
					sys.vehicle[v].powLvl += maxC;				//update battery level
				}
			}
		}
		//if node load maximum is reached -- cancel vehicles toward this station and/or cancel previous route
	}
	// compute the total cost
	sys.cost = 0;
	for (k = 0; k < nArc; k++){
		if (arc[k].from == arc[k].to)	continue;
		for (t = 0; t < T; t++){
			int val_y = sys.Demand[t][k];
			for (v = 0; v < nVeh; v++){
				sys.cost += arc[k].fcost * sys.route[k][v][t] + arc[k].cost * sys.customer[k][v][t];
				for (tau = t + 1; tau <= t + TimeWindow && tau < T; tau++){
					val_y -= sys.customer[k][v][tau];
				}
			}
			val_y = max(val_y, 0);
			sys.cost += penalty * arc[k].cost * val_y;
		}
	}

	for (v = 0; v < nVeh; v++){
		double rCost = 0;
		for (k = 0; k < nArc; k++){
			for (t = 0; t < T; t++){
				rCost += arc[k].fcost * sys.route[k][v][t] + arc[k].cost * sys.customer[k][v][t];
			}
		}
		sys.routeCost.push_back(rCost);
	}

	_end = clock();
	cmp_time = (double)(_end - _start) / CLOCKS_PER_SEC;

	sprintf(path, "%soutput/heuristic2_%dV_%dN_%dArc_%dT.txt", directoryPath.c_str(), nVeh, N, nArc, T);
	heuristicSol();

	return 1;
}

int Model::GreedyAlgo()
{


	return 1;
}


int Model::initSystem()
{
	sys.cost = 0;
	for (t = 0; t < T; t++){
		sys.Demand.push_back(dmd[t]);				//initialize system demand data
		vector<int> remain;
		for (k = 0; k < nArc; k++){
			if (arc[k].from != arc[k].to)	remain.push_back(0);
		}
		sys.Remain.push_back(remain);
	}
	for (v = 0; v < nVeh; v++){
		sys.vehicle.push_back(vehicle[v]);			//initialize vehicle status
	}
	for (i = 0; i < N; i++){
		vector<int> newLoad;
		for (t = 0; t < T; t++){
			newLoad.push_back(0);
		}
		sys.NodeLoad.push_back(newLoad);						//initialize node load such that all are 0's
	}
	for (l = 0; l < nTrack; l++){
		vector<int> newLoad;
		for (t = 0; t < T; t++){
			newLoad.push_back(0);
		}
		sys.TrackLoad.push_back(newLoad);						//initialize track load such that all are 0's
	}
	for (k = 0; k < nArc; k++){
		vector2int	routeArc;
		vector2int	cusArc;
		for (v = 0; v < nVeh; v++){
			vector<int>	routeVehicle;
			vector<int> cusVehicle;
			for (t = 0; t < T; t++){
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

int Model::heuristicSol()
{
	ofstream heusol;
	string outputFile(path);
	heusol.open(outputFile);

	if (!heusol.is_open())
	{
		cerr << "Fail to open the output file!\n";
		return 0;
	}

	heusol << "Computational Time: " << cmp_time << endl;
	heusol << "Optimal Electricity Consumption:" << sys.cost << "KWH" << endl;
	heusol << "The Routes for All PRT Vehicles:\n";

	for (v = 0; v < nVeh; v++){
		heusol << "Routes for Vehicle " << v << ":\n\t";
		if (vehicle[v].time == 0)									//the vehicle stops at a station at the beginning
		{
			for (t = 0; t < T; t++){
				for (k = 0; k < nArc; k++){
					if (sys.route[k][v][t] > .99){
						if (t == 0)	heusol << arc[k].from;
						for (i = 1; i < abs(arc[k].from - arc[k].to); i++)
							heusol << "--*";
						heusol << "--" << arc[k].to;
					}
				}
			}
		}
		else														//the vehicle is moving at the beginning
		{
			for (t = 0; t < vehicle[v].time; t++){
				heusol << "*--";
			}
			for (t = vehicle[v].time; t < T; t++){
				for (k = 0; k < nArc; k++){
					if (sys.route[k][v][t] > .99){
						if (t == vehicle[v].time)	heusol << arc[k].from;
						for (i = 1; i < abs(arc[k].from - arc[k].to); i++)
							heusol << "--*";
						heusol << "--" << arc[k].to;
					}
				}
			}


		}
		heusol << endl;
	}

	heusol << "The number of customers transported by Each PRT Vehicles:\n";
	for (v = 0; v < nVeh; v++){
		heusol << "Vehicle " << v << ":\n\t";
		for (t = 0; t < T; t++){
			for (k = 0; k < nArc; k++){
				if (arc[k].from == arc[k].to)	continue;
				heusol << sys.customer[k][v][t] << "\t";
			}
		}
		heusol << endl;
	}

	heusol << "The number of customers who haven't been served within the time window:" << endl;
	for (t = 0; t < T; t++){
		heusol << "t = " << t << ":\t";
		for (k = 0; k < nArc; k++){
			if (arc[k].from == arc[k].to) continue;
			int val_y = sys.Demand[t][k];
			for (v = 0; v < nVeh; v++){
				for (tau = t + 1; tau <= t + TimeWindow && tau < T; tau++){
					val_y -= sys.customer[k][v][tau];
				}
			}
			val_y = max(val_y, 0);
			heusol << val_y << "\t";
		}
		heusol << endl;
	}

	heusol << "The number of customers who haven't been served at each time:" << endl;
	for (t = 0; t < T; t++){
		heusol << "t = " << t << ":\t";
		for (k = 0; k < nArc; k++){
			if (arc[k].from == arc[k].to) continue;
			heusol << sys.Demand[t][k] << "\t";
		}
		heusol << endl;
	}

	heusol << "Track Load over time:\n";
	for (t = 0; t < T; t++){
		heusol << "t = " << t << ":\t";
		for (l = 0; l < nTrack; l++){
			int trackLoad = 0;
			for (k = 0; k < nArc; k++){
				if (arc[k].from == arc[k].to) continue;
				for (vector<int>::iterator itr = arc[k].track.begin(); itr != arc[k].track.end(); itr++){
					if (l == (*itr) && t - abs(track[l].from - arc[k].from) >= 0){
						for (v = 0; v < nVeh; v++){
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
