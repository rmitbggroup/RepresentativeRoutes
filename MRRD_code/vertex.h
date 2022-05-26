#ifndef _VERTEX_H_
#define _VERTEX_H_

#include "includes.h"
using namespace std;

struct vertex
{
	vector<vertex *> neighbors;
	double x, y; // plane coordinates
	double lat, lng; // latitude and longitude
	VertexID id;
};
extern vector<vertex*> vertices;

#endif // !_VERTEX_H_
