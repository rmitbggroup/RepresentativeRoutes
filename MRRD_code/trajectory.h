#ifndef _TRAJECTORY_H_
#define _TRAJECTORY_H_

#include "definition.h"
#include "vertex.h"

typedef unsigned int EdgeID;

struct trajectory_t
{
	int tid; // trajectory id
	int oldid; // it is from trajectory number
	int start; // its start point from the old trajectory
	int end; // its end point from the old trajectory
	vector<VertexID> points; // store point id in the trajectory
	vector<vertex*> vertices; // store points in the trajectory
	double length; // trajectory length
};

struct result_t
{
	vector<edge> curroute;
	map<EdgeID, vector<edge>> curroute2seg;// current best route: first segment: all segment
	map<EdgeID, double> curbudgets; // current used budget
};

double measure_dist_segment2trajectory(edge e, trajectory_t* trajectory);
void read_trajectory(map<int, trajectory_t*>& trajectories);
void connect_first(map<EdgeID, edge> edges, map<int, trajectory_t*> trajectories);
void coverage_first(map<EdgeID, edge> edges, map<int, trajectory_t*> trajectories);
void maximum_coverage(map<EdgeID, edge> segments, map<int, trajectory_t*> trajectories);

#endif
