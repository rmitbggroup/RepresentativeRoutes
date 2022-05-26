#ifndef _STATIC_GRAPH_H
#define _STATIC_GRAPH_H

#include "definition.h"
#include "performanceTimer.h"
#include "vertex.h"



pair<string, vertex*> check_region(vertex* a, vertex* b);
void graph_init(vector<vertex*>& vertices, map<VertexID, vertex*>& vertexList, map<EdgeID, edge>& edges, string data_dir);
double measure_distance_point2point(vertex* v1, vertex* v2);
double measure_dist_segment2segment(vertex* ps, vertex* pe, vertex* ts, vertex* te);
double measure_perpendicular_distance(vertex* s1, vertex* e1, vertex* s2, vertex* e2);

#endif
