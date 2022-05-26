#ifndef DEFINITION_H_
#define DEFINITION_H_

#include <iostream>
#include <string>
#include <fstream>
#include <sstream>
#include <vector>
#include <set>
#include <map>
#include <list>
#include <deque>
#include <algorithm>
#include <utility>
#include <limits>
#include <cmath>
#include <cstdlib>
#include <cassert>
#include <cstring>

using namespace std;

#define UINT_INFINITY 1073741823       //1024*1024*1024-1
#define INT_INFINITY numeric_limits<int>::max()
#define FLT_INFINITY numeric_limits<float>::max()
#define DBL_INFINITY numeric_limits<double>::max()

typedef unsigned int VertexID;
typedef unsigned int EdgeID;
typedef double Weight;

struct edge
{
	EdgeID id; // edge id
	VertexID s_vert; // start vertex id
	VertexID e_vert; // end vertex id
	Weight weight; // edge length
	long tnum; // covered trajectory number
	vector<EdgeID> rightconnect; // store edges connecting with its right.
	vector<EdgeID> leftconnect; // store edges connecting with its left.
};
extern map<EdgeID, edge> edges;



#endif
