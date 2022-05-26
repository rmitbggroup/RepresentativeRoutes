#include "main.h"

// input/output path
ofstream outputfile;
string data_dir = "/research/data/porto";
string cover_dir = "/research/data/porto/res";

//simulation parameter
double epsilon = 1000; 
double budget = 10000; // the budget 
int K = 15;

//query region
double rtlat = 41.1; //starting latitude of the region  
double rtlng = -8.6; //starting longitude of the region  

double rlength = 4000; // the length of the region 
double rwidth = 4000; // the width of the region

bool ifregion = true;

//data structures
vector<vertex*> vertices; // all vertices in the road network
map<VertexID, vertex*> vertexList; // vertices in the region
map<EdgeID, edge> edges; // all possible edges
map<int, trajectory_t*> trajectories; // trajectories in the region

int main(int argc, char** argv)
{

	outputfile.open(cover_dir + "/log_graph.txt");
	graph_init(vertices, vertexList, edges, data_dir);
	read_trajectory(trajectories);
	outputfile.close();
	connect_first(edges, trajectories);

	return 0;
}

