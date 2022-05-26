#include "main.h"
#include "graph.h"

double compute_inner_product(vertex* v1, vertex* v2)
{
	double innerProduct = 0.0;
	innerProduct = (v1->x * v2->x) + (v1->y * v2->y);

	return innerProduct;
}

double compute_vector_length(vertex* v)
{
	double squareSum = 0.0;
	squareSum = pow(v->x, 2) + pow(v->y, 2);

	return sqrt(squareSum);
}

double measure_distance_point2point(vertex* v1, vertex* v2)
{
	double squareSum = 0.0;
	squareSum = pow((v2->x - v1->x), 2) + pow((v2->y - v1->y), 2);

	return sqrt(squareSum);
}

double measure_distance_point2segment(vertex* s, vertex* e, vertex* p, double coefficient, vertex* projectPoint)
{
	vertex* v1 = new vertex;
	vertex* v2 = new vertex;

	// the vector connecting the start point of the line segment and a given point
	v1->x = p->x - s->x;
	v1->y = p->y - s->y;

	// the vector representing the line segment
	v2->x = e->x - s->x;
	v2->y = e->y - s->y;

	// a coefficient (0 <= b <= 1)
	coefficient = compute_inner_product(v1, v2) / compute_inner_product(v2, v2);

	// the projection on the line segment from a given point
	projectPoint->x = s->x + coefficient * v2->x;
	projectPoint->y = s->y + coefficient * v2->y;

	delete v1;
	delete v2;

	// return the distance between the projection point and the given point
	return measure_distance_point2point(p, projectPoint);
}

double measure_angle_disntance(vertex* s1, vertex* e1, vertex* s2, vertex* e2)
{

	vertex* v1 = new vertex;
	vertex* v2 = new vertex;

	// construct two vectors representing the given line segment and another line segment, respectively
	v1->x = e1->x - s1->x;
	v1->y = e1->y - s1->y;
	v2->x = e2->x - s2->x;
	v2->y = e2->y - s2->y;

	// we assume that the first line segment is longer than the second one
	// i.e., vectorLength1 >= vectorLength2
	double vectorLength1 = compute_vector_length(v1);
	double vectorLength2 = compute_vector_length(v2);

	// if one of two vectors is a point, the angle distance becomes zero
	if (vectorLength1 == 0.0 || vectorLength2 == 0.0) return 0.0;

	// compute the inner product of the two vectors
	double innerProduct = compute_inner_product(v1, v2);

	// compute the angle between two vectors by using the inner product
	double cosTheta = innerProduct / (vectorLength1 * vectorLength2);
	// compensate the computation error (e.g., 1.00001)
	// cos(theta) should be in the range [-1.0, 1.0]
	// START ...
	if (cosTheta > 1.0) cosTheta = 1.0;
	if (cosTheta < -1.0) cosTheta = -1.0;
	// ... END
	double sinTheta = sqrt(1 - pow(cosTheta, 2));
	// if 90 <= theta <= 270, the angle distance becomes the length of the line segment
	// if (cosTheta < -1.0) sinTheta = 1.0;

	delete v1;
	delete v2;

	return (vectorLength2 * sinTheta);
}

double measure_perpendicular_distance(vertex* s1, vertex* e1, vertex* s2, vertex* e2)
{
	double perDistance1, perDistance2;
	double length1, length2;
	double perpendicularDistance;

	// the length of the first line segment
	length1 = measure_distance_point2point(s1, e1);
	// the length of the second line segment
	length2 = measure_distance_point2point(s2, e2);

	// compute the perpendicular distance and the parallel distance
	// START ...
	double coefficient = 0;
	vertex* projectPoint = new vertex;
	if (length1 > length2)
	{
		perDistance1 = measure_distance_point2segment(s1, e1, s2, coefficient, projectPoint);
		perDistance2 = measure_distance_point2segment(s1, e1, e2, coefficient, projectPoint);
	}
	else
	{
		perDistance1 = measure_distance_point2segment(s2, e2, s1, coefficient, projectPoint);
		perDistance2 = measure_distance_point2segment(s2, e2, e1, coefficient, projectPoint);
	}

	// compute the perpendicular distance; take (d1^2 + d2^2) / (d1 + d2)
	if (!(perDistance1 == 0.0 && perDistance2 == 0.0))
		perpendicularDistance = ((pow(perDistance1, 2) + pow(perDistance2, 2)) / (perDistance1 + perDistance2));
	else
		perpendicularDistance = 0.0;

	delete projectPoint;

	return perpendicularDistance;
}

pair<double, double> measure_perpendicular_and_parallel_distance(vertex* s1, vertex* e1, vertex* s2, vertex* e2)
{
	double perDistance1, perDistance2;
	double parDistance1, parDistance2;
	double length1, length2;
	double perpendicularDistance, parallelDistance;
	pair<double, double> perp_paral;

	// the length of the first line segment
	length1 = measure_distance_point2point(s1, e1);
	// the length of the second line segment
	length2 = measure_distance_point2point(s2, e2);

	// compute the perpendicular distance and the parallel distance
	// START ...
	double coefficient = 0;
	vertex* projectPoint = new vertex;
	if (length1 > length2)
	{
		perDistance1 = measure_distance_point2segment(s1, e1, s2, coefficient, projectPoint);
		if (coefficient < 0.5) parDistance1 = measure_distance_point2point(s1, projectPoint);
		else parDistance1 = measure_distance_point2point(e1, projectPoint);

		perDistance2 = measure_distance_point2segment(s1, e1, e2, coefficient, projectPoint);
		if (coefficient < 0.5) parDistance2 = measure_distance_point2point(s1, projectPoint);
		else parDistance2 = measure_distance_point2point(e1, projectPoint);
	}
	else
	{
		perDistance1 = measure_distance_point2segment(s2, e2, s1, coefficient, projectPoint);
		if (coefficient < 0.5) parDistance1 = measure_distance_point2point(s2, projectPoint);
		else parDistance1 = measure_distance_point2point(e2, projectPoint);

		perDistance2 = measure_distance_point2segment(s2, e2, e1, coefficient, projectPoint);
		if (coefficient < 0.5) parDistance2 = measure_distance_point2point(s2, projectPoint);
		else parDistance2 = measure_distance_point2point(e2, projectPoint);
	}

	// compute the perpendicular distance; take (d1^2 + d2^2) / (d1 + d2)
	if (!(perDistance1 == 0.0 && perDistance2 == 0.0))
		perpendicularDistance = ((pow(perDistance1, 2) + pow(perDistance2, 2)) / (perDistance1 + perDistance2));
	else
		perpendicularDistance = 0.0;

	// compute the parallel distance; take the minimum
	parallelDistance = (parDistance1 < parDistance2) ? parDistance1 : parDistance2;
	// ... END

	perp_paral.first = perpendicularDistance;
	perp_paral.second = parallelDistance;

	delete projectPoint;

	return perp_paral;
}

double measure_dist_segment2segment(vertex* ps, vertex* pe, vertex* ts, vertex* te) { // calculate distance between two line segments

	double slength = measure_distance_point2point(ps, pe);
	double tlength = measure_distance_point2point(ts, te);
	double perpdist = 0;
	double paradist = 0;
	double angledist = 0;
	if (slength > tlength) {
		angledist = measure_angle_disntance(ts, te, ps, pe);
	}
	else {
		angledist = measure_angle_disntance(ps, pe, ts, te);
	}
	pair<double, double> perp_paral = measure_perpendicular_and_parallel_distance(ps, pe, ts, te);
	perpdist = perp_paral.first;
	paradist = perp_paral.second;

	double dist = perpdist + angledist + paradist;

	return dist;
}

double multi(double p1x, double p1y, double p2x, double p2y, double p0x, double p0y)
{
	return (p1x - p0x) * (p2y - p0y) - (p2x - p0x) * (p1y - p0y);
}
pair<double, double> intersection(double ax, double ay, double bx, double by,
	double cx, double cy, double dx, double dy) // the intersection point of two line segments
{
	pair<double, double> p;
	p.first = (multi(ax, ay, dx, dy, cx, cy) * bx - multi(bx, by, dx, dy, cx, cy) * ax)
		/ (multi(ax, ay, dx, dy, cx, cy) - multi(bx, by, dx, dy, cx, cy));
	p.second = (multi(ax, ay, dx, dy, cx, cy) * by - multi(bx, by, dx, dy, cx, cy) * ay)
		/ (multi(ax, ay, dx, dy, cx, cy) - multi(bx, by, dx, dy, cx, cy));
	return p;
}

pair<string, vertex*> check_region(vertex* a, vertex* b) {
	pair<string, vertex*> con;
	GeographyCoordinateTransform GCT;
	double rtx;
	double rty;
	GCT.BL2XY(rtlat, rtlng, rtx, rty);
	double ax = a->x;
	double ay = a->y;
	double bx = b->x;
	double by = b->y;
	//segment region
	if (ax >= rtx - rlength / 2 && ax <= rtx + rlength / 2
		&& ay >= rty - rwidth / 2 && ay <= rty + rwidth / 2) {
		// a in the regoin, b in the region
		if (bx >= rtx - rlength / 2 && bx <= rtx + rlength / 2
			&& by >= rty - rwidth / 2 && by <= rty + rwidth / 2) {
			con.first = "ainbin";
			con.second = a;
		}
		// a in the regoin, b out the region (less longitude)
		if (bx < rtx - rlength / 2) {
			pair<double, double> nv = intersection(ax, ay, bx, by,
				rtx - rlength / 2, rty - rwidth / 2, rtx - rlength / 2, rty + rwidth / 2);
			vertex* nvertex = new vertex;
			nvertex->x = nv.first;
			nvertex->y = nv.second;
			double lat;
			double lng;
			GCT.XY2BL(nv.first, nv.second, lat, lng);
			nvertex->lat = lat;
			nvertex->lng = lng;
			con.first = "ainbout";
			con.second = nvertex;
		}
		// a in the regoin, b out the region (greater longitude)
		if (bx > rtx + rlength / 2) {
			pair<double, double> nv = intersection(ax, ay, bx, by,
				rtx + rlength / 2, rty - rwidth / 2, rtx + rlength / 2, rty + rwidth / 2);
			vertex* nvertex = new vertex;
			nvertex->x = nv.first;
			nvertex->y = nv.second;
			double lat;
			double lng;
			GCT.XY2BL(nv.first, nv.second, lat, lng);
			nvertex->lat = lat;
			nvertex->lng = lng;
			con.first = "ainbout";
			con.second = nvertex;
		}
		// a in the regoin, b out the region (less latitude)
		if (by < rty - rwidth / 2) {
			pair<double, double> nv = intersection(ax, ay, bx, by,
				rtx - rlength / 2, rty - rwidth / 2, rtx + rlength / 2, rty - rwidth / 2);
			vertex* nvertex = new vertex;
			nvertex->x = nv.first;
			nvertex->y = nv.second;
			double lat;
			double lng;
			GCT.XY2BL(nv.first, nv.second, lat, lng);
			nvertex->lat = lat;
			nvertex->lng = lng;
			con.first = "ainbout";
			con.second = nvertex;
		}
		// a in the regoin, b out the region (greater latitude)
		if (by > rty + rwidth / 2) {
			pair<double, double> nv = intersection(ax, ay, bx, by,
				rtx - rlength / 2, rty + rwidth / 2, rtx + rlength / 2, rty + rwidth / 2);
			vertex* nvertex = new vertex;
			nvertex->x = nv.first;
			nvertex->y = nv.second;
			double lat;
			double lng;
			GCT.XY2BL(nv.first, nv.second, lat, lng);
			nvertex->lat = lat;
			nvertex->lng = lng;
			con.first = "ainbout";
			con.second = nvertex;
		}
	}
	if (bx >= rtx - rlength / 2 && bx <= rtx + rlength / 2
		&& by >= rty - rwidth / 2 && by <= rty + rwidth / 2) {
		// b in the regoin, a out the region (less longitude)
		if (ax < rtx - rlength / 2) {
			pair<double, double> nv = intersection(ax, ay, bx, by,
				rtx - rlength / 2, rty - rwidth / 2, rtx - rlength / 2, rty + rwidth / 2);
			vertex* nvertex = new vertex;
			nvertex->x = nv.first;
			nvertex->y = nv.second;
			double lat;
			double lng;
			GCT.XY2BL(nv.first, nv.second, lat, lng);
			nvertex->lat = lat;
			nvertex->lng = lng;
			con.first = "binaout";
			con.second = nvertex;
		}
		// b in the regoin, a out the region (greater longitude)
		if (ax > rtx + rlength / 2) {
			pair<double, double> nv = intersection(ax, ay, bx, by,
				rtx + rlength / 2, rty - rwidth / 2, rtx + rlength / 2, rty + rwidth / 2);
			vertex* nvertex = new vertex;
			nvertex->x = nv.first;
			nvertex->y = nv.second;
			double lat;
			double lng;
			GCT.XY2BL(nv.first, nv.second, lat, lng);
			nvertex->lat = lat;
			nvertex->lng = lng;
			con.first = "binaout";
			con.second = nvertex;
		}
		// b in the regoin, a out the region (less latitude)
		if (ay < rty - rwidth / 2) {
			pair<double, double> nv = intersection(ax, ay, bx, by,
				rtx - rlength / 2, rty - rwidth / 2, rtx + rlength / 2, rty - rwidth / 2);
			vertex* nvertex = new vertex;
			nvertex->x = nv.first;
			nvertex->y = nv.second;
			double lat;
			double lng;
			GCT.XY2BL(nv.first, nv.second, lat, lng);
			nvertex->lat = lat;
			nvertex->lng = lng;
			con.first = "binaout";
			con.second = nvertex;
		}
		// b in the regoin,a out the region (greater latitude)
		if (by > rty + rwidth / 2) {
			pair<double, double> nv = intersection(ax, ay, bx, by,
				rtx - rlength / 2, rty + rwidth / 2, rtx + rlength / 2, rty + rwidth / 2);
			vertex* nvertex = new vertex;
			nvertex->x = nv.first;
			nvertex->y = nv.second;
			double lat;
			double lng;
			GCT.XY2BL(nv.first, nv.second, lat, lng);
			nvertex->lat = lat;
			nvertex->lng = lng;
			con.first = "binaout";
			con.second = nvertex;
		}
	}
	if (ax < rtx - rlength / 2 || ax > rtx + rlength / 2
		|| ay < rty - rwidth / 2 || ay > rty + rwidth / 2) {
		if (bx < rtx - rlength / 2 || bx > rtx + rlength / 2
			|| by < rty - rwidth / 2 || by > rty + rwidth / 2) {
			con.first = "aoutbout";
		}
	}

	return con;
}

void read_graph(vector<vertex*>& vertices, map<VertexID, vertex*>& vertexList, map<EdgeID, edge>& edges, string data_dir)
{
	GeographyCoordinateTransform GCT;
	double rtx;
	double rty;
	GCT.BL2XY(rtlat, rtlng, rtx, rty);
	//rtx = rtx + rlength*0.6;
	ifstream vertexFile;
	int nr_vertex, nr_edge;
	vertexFile.open((data_dir + "/vertices.out").c_str());
	vertexFile >> nr_vertex;
	for (int i = 0; i < nr_vertex; i++) {
		double lat;
		double lng;
		double x;
		double y;
		vertexFile >> lat >> lng;
		GCT.BL2XY(lat, lng, x, y);
		vertex* newVertex = new vertex;
		newVertex->id = i;
		newVertex->x = x;
		newVertex->y = y;
		newVertex->lng = lng;
		newVertex->lat = lat;
		if (ifregion) {
			if (x >= rtx - rlength / 2 && x <= rtx + rlength / 2
				&& y >= rty - rwidth / 2 && y <= rty + rwidth / 2) {
				// vertex in the region
				vertexList.emplace(newVertex->id, newVertex);
			}
		}
		else {
			vertexList.emplace(newVertex->id, newVertex);
		}
		vertices.push_back(newVertex);
	}
	vertexFile.close();
	cout << "Read Vertices: done" << endl;
	outputfile << "Read Vertices: done " << vertices.size() << endl;

	ifstream edgeFile;
	edgeFile.open((data_dir + "/edges.out").c_str());
	edgeFile >> nr_edge;
	for (int i = 0; i < nr_edge; i++) {
		int a, b;
		double distance;
		edgeFile >> a >> b >> distance;
		if (ifregion) {
			// check if the edge is in the region 
			pair<string, vertex*> check = check_region(vertices[a], vertices[b]);
			if (check.first == "ainbin") {
				vertices[a]->neighbors.push_back(vertices[b]);
				edge e;
				e.id = i;
				e.s_vert = a;
				e.e_vert = b;
				e.weight = distance;
				edges.emplace(i, e);
			}
			if (check.first == "ainbout") {
				vertex* newVertex = new vertex;
				newVertex->id = nr_vertex;
				nr_vertex++;
				newVertex->x = check.second->x;
				newVertex->y = check.second->y;
				double lat;
				double lng;
				GCT.XY2BL(check.second->x, check.second->y, lat, lng);
				newVertex->lat = lat;
				newVertex->lng = lng;
				vertices.push_back(newVertex);
				vertexList.emplace(newVertex->id, newVertex);
				vertices[a]->neighbors.push_back(newVertex);
				double distance = measure_distance_point2point(vertices[a], newVertex);
				edge e;
				e.id = i;
				e.s_vert = a;
				e.e_vert = newVertex->id;
				e.weight = distance;
				edges.emplace(i, e);
			}
			if (check.first == "binaout") {
				vertex* newVertex = new vertex;
				newVertex->id = nr_vertex;
				nr_vertex++;
				newVertex->x = check.second->x;
				newVertex->y = check.second->y;
				double lat;
				double lng;
				GCT.XY2BL(check.second->x, check.second->y, lat, lng);
				newVertex->lat = lat;
				newVertex->lng = lng;
				vertices.push_back(newVertex);
				vertexList.emplace(newVertex->id, newVertex);
				newVertex->neighbors.push_back(vertices[b]);
				double distance = measure_distance_point2point(vertices[b], newVertex);
				edge e;
				e.id = i;
				e.s_vert = newVertex->id;
				e.e_vert = b;
				e.weight = distance;
				edges.emplace(i, e);
			}
		}
		else {
			vertices[a]->neighbors.push_back(vertices[b]);
			edge e;
			e.id = i;
			e.s_vert = a;
			e.e_vert = b;
			e.weight = distance;
			edges.emplace(i, e);
		}

	}


	// find the connected line segments
	for (auto e1 : edges) {
		vector<EdgeID> leftconnect;
		vector<EdgeID> rightconnect;
		for (auto e2 : edges) {
			if (e1.second.e_vert == e2.second.s_vert) { // connected edge e2 on the right
				rightconnect.push_back(e2.first);
			}
			if (e1.second.s_vert == e2.second.e_vert) { // connected edge e2 on the left
				leftconnect.push_back(e2.first);
			}
		}
		edges[e1.first].rightconnect = rightconnect;
		edges[e1.first].leftconnect = leftconnect;
	}

	edgeFile.close();
	cout << "Read Edges: done" << endl;
	outputfile << "Read Edges: done " << edges.size() << endl;
}

void graph_init(vector<vertex*>& vertices, map<VertexID, vertex*>& vertexList, map<EdgeID, edge>& edges, string data_dir)
{
	read_graph(vertices, vertexList, edges, data_dir);
}