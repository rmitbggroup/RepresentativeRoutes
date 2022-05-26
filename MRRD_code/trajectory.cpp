#include "main.h"
#include "trajectory.h"


#define inf 10e9
#define LOG2(_x)	(float)(log((float)(_x)) / log((float)2))

PerformanceTimer read_time, segment_time, dist_time, connect_time, olap_time;



double measure_dist_segment2trajectory(edge e, trajectory_t* trajectory) { // calculate distance between a line segment and a trajectory

	vertex* ps = vertices[e.s_vert];
	vertex* pe = vertices[e.e_vert];

	vector<vertex*> points = trajectory->vertices;

	double mindist = inf;
	for (int i = 0; i < points.size() - 1; i++) {
		vertex* ts = points[i];
		vertex* te = points[i + 1];
		double dist = measure_dist_segment2segment(ts, te, ps, pe);
		if (dist < mindist) {
			mindist = dist;
		}
	}

	return mindist;
}

vector<double> min_median_max(vector<double> t, int n)
{
	vector<double> res;
	int i, j, max;
	double temp;
	for (i = 0; i < n - 1; i++)
	{
		max = i;
		for (j = i + 1; j < n; j++)
			if (t[j] > t[max]) max = j;
		temp = t[i]; t[i] = t[max]; t[max] = temp;
	}
	double min = t[t.size() - 1];
	double maximum = t[0];
	if (n % 2 == 0) {
		double median = (t[n / 2] + t[(n / 2) - 1]) / 2;
		res.push_back(min);
		res.push_back(median);
		res.push_back(maximum);
		return res;
	}
	double median = t[(n - 1) / 2];
	res.push_back(min);
	res.push_back(median);
	res.push_back(maximum);
	return res;
}

void read_trajectory(map<int, trajectory_t*>& trajectories) {
	GeographyCoordinateTransform GCT;
	double rtx;
	double rty;
	GCT.BL2XY(rtlat, rtlng, rtx, rty);
	ifstream trajectory_file((data_dir + "/trajectories.out").c_str());
	int count_traj = 0;
	int count_point = 0;
	double totallength = 0;
	double totalsize = 0;
	vector<double> lengthList;
	vector<double> sizeList;
	map<int, trajectory_t*> alltrajectories;
	map<double, map<double, VertexID>> xyID;
	map<VertexID, vertex*> pointList;
	read_time.resume();
	while (!trajectory_file.eof()) {
		double tlength = 0;
		vector<vertex*> vtraj;
		vector<VertexID> points;
		char c;
		int id;
		string s_lat;
		string s_lng;
		double lat;
		double lng;
		do {
			trajectory_file >> s_lat >> s_lng;
			trajectory_file.get(c);
			lat = atof(s_lat.c_str());
			lng = atof(s_lng.c_str());
			double x;
			double y;
			GCT.BL2XY(lat, lng, x, y);
			VertexID id;
			vertex* v = new vertex;
			if (xyID.find(x) == xyID.end()) {
				v->id = count_point;
				v->lat = lat;
				v->lng = lng;
				v->x = x;
				v->y = y;
				pointList.emplace(v->id, v);
				map<double, VertexID> temp;
				temp.emplace(y, v->id);
				xyID.emplace(x, temp);
				id = v->id;
				count_point++;
			}
			else {
				map<double, VertexID> temp = xyID[x];
				if (temp.find(y) == temp.end()) {
					v->id = count_point;
					v->lat = lat;
					v->lng = lng;
					v->x = x;
					v->y = y;
					pointList.emplace(v->id, v);
					temp.emplace(y, v->id);
					xyID.emplace(x, temp);
					id = v->id;
					count_point++;
				}
				else {
					id = temp[y];
					v = pointList[id];
				}
			}
			points.push_back(id);
			vtraj.push_back(v);
			if (points.size() > 1) {
				tlength = tlength + measure_distance_point2point(pointList[points[points.size() - 2]], pointList[points[points.size() - 1]]);
				//tlength = tlength + 1;
			}
		} while (c == ' ');
		trajectory_t* trajectory = new trajectory_t;
		trajectory->tid = count_traj;
		trajectory->vertices = vtraj;
		trajectory->points = points;
		trajectory->length = tlength;
		alltrajectories.emplace(trajectory->tid, trajectory);
		if (count_traj % 10000 == 0) {
			cout << "Read " << count_traj << " trajectories: done" << endl;
		}
		lengthList.push_back(tlength);
		totallength = totallength + tlength;
		tlength = 0;
		count_traj++;

	}
	outputfile << "Read " << count_traj << " trajectories: done" << endl;
	outputfile << "Avarage: " << totallength / count_traj << endl;

	if (ifregion) {
		totallength = 0;
		totalsize = 0;
		count_traj = 0;
		lengthList.clear();
		for (auto t : alltrajectories) {
			vector<VertexID> points = t.second->points;
			vector<vertex*> vtraj = t.second->vertices;
			vector<VertexID> newpoints;
			vector<vertex*> newvtraj;
			double tlength = 0;
			int start = 0;
			int end = points.size() - 1;
			for (int i = 0; i < points.size() - 1; i = i + 2) {
				VertexID v1 = points[i];
				VertexID v2 = points[i + 1];
				pair<string, vertex*> check = check_region(pointList[v1], pointList[v2]);// check if the point is in the region 
				if (check.first == "aoutbout") {
					if (newpoints.size() > 1 && tlength > 0) {
						trajectory_t* trajectory = new trajectory_t;
						trajectory->tid = count_traj;
						trajectory->oldid = t.second->tid;
						trajectory->points = newpoints;
						trajectory->vertices = newvtraj;
						trajectory->length = tlength;
						end = i - 1;
						trajectory->start = start;
						trajectory->end = end;
						trajectories.emplace(trajectory->tid, trajectory);
						lengthList.push_back(tlength);
						sizeList.push_back(newvtraj.size() - 1);
						if (count_traj % 10000 == 0) {
							cout << "Read " << count_traj << " trajectories: done" << endl;
						}
						totallength = totallength + tlength;
						totalsize = totalsize + newvtraj.size() - 1;
						count_traj++;
					}
					tlength = 0;
					newpoints.clear();
					newvtraj.clear();
				}
				if (check.first == "ainbin") { // two adjacent points both in the region
					newpoints.push_back(v1); // add the first point
					newpoints.push_back(v2); // add the last point
					newvtraj.push_back(vtraj[i]);
					newvtraj.push_back(vtraj[i + 1]);
					if (newpoints.size() == 2) {
						start = i;
						double le = measure_distance_point2point(pointList[v1], pointList[v2]);
						tlength = tlength + le;
						//tlength = tlength + 1;
					}
					if (newpoints.size() > 2) {
						VertexID v0 = newpoints[newpoints.size() - 3];
						double le = measure_distance_point2point(pointList[v0], pointList[v1]) +
							measure_distance_point2point(pointList[v1], pointList[v2]);
						tlength = tlength + le;
						//tlength = tlength + 2;
					}
					if (i == points.size() - 2 && newpoints.size() > 1 && tlength > 0) { //left the last two points in the previous list
						trajectory_t* trajectory = new trajectory_t;
						trajectory->tid = count_traj;
						trajectory->oldid = t.second->tid;
						trajectory->points = newpoints;
						trajectory->vertices = newvtraj;
						trajectory->length = tlength;
						end = i + 1;
						trajectory->start = start;
						trajectory->end = end;
						trajectories.emplace(trajectory->tid, trajectory);
						lengthList.push_back(tlength);
						sizeList.push_back(newvtraj.size() - 1);
						if (count_traj % 10000 == 0) {
							cout << "Read " << count_traj << " trajectories: done" << endl;
						}
						totallength = totallength + tlength;
						totalsize = totalsize + newvtraj.size() - 1;
						count_traj++;
						tlength = 0;
						newpoints.clear();
						newvtraj.clear();
					}
				}
				if (check.first == "ainbout") { // the first point in the region; cut the trajectory
					if (check.second->x >= rtx - rlength / 2 && check.second->x <= rtx + rlength / 2
						&& check.second->y >= rty - rwidth / 2 && check.second->y <= rty + rwidth / 2) {
						if (xyID.find(check.second->x) == xyID.end()) {
							v2 = pointList.size();
							check.second->id = v2;
							pointList.emplace(v2, check.second);
							map<double, VertexID> temp;
							temp.emplace(check.second->y, v2);
							xyID.emplace(check.second->x, temp);
						}
						else {
							map<double, VertexID> temp = xyID[check.second->x];
							if (temp.find(check.second->y) == temp.end()) {
								v2 = pointList.size();
								check.second->id = v2;
								pointList.emplace(v2, check.second);
								temp.emplace(check.second->y, v2);
								xyID.emplace(check.second->x, temp);
							}
							else {
								v2 = temp[check.second->y];
							}
						}
						newpoints.push_back(v1);
						newpoints.push_back(v2); // add the interact points 
						newvtraj.push_back(vtraj[i]);
						newvtraj.push_back(check.second);
					}
					else {
						newpoints.push_back(v1);
						newvtraj.push_back(vtraj[i]);
					}

					if (newpoints.size() == 2) {
						start = i;
						double le = measure_distance_point2point(pointList[v1], pointList[v2]);
						tlength = tlength + le;
						//tlength = tlength + 1;
					}
					if (newpoints.size() > 2) {
						VertexID v0 = newpoints[newpoints.size() - 3];
						double le = measure_distance_point2point(pointList[v0], pointList[v1]) +
							measure_distance_point2point(pointList[v1], pointList[v2]);
						tlength = tlength + le;
						//tlength = tlength + 2;
					}
					if (newpoints.size() > 1 && tlength > 0) {
						trajectory_t* trajectory = new trajectory_t;
						trajectory->tid = count_traj;
						trajectory->oldid = t.second->tid;
						trajectory->points = newpoints;
						trajectory->vertices = newvtraj;
						trajectory->length = tlength;
						end = i + 1;
						trajectory->start = start;
						trajectory->end = end;
						trajectories.emplace(trajectory->tid, trajectory);
						lengthList.push_back(tlength);
						sizeList.push_back(newvtraj.size() - 1);
						if (count_traj % 10000 == 0) {
							cout << "Read " << count_traj << " trajectories: done" << endl;
						}
						totallength = totallength + tlength;
						totalsize = totalsize + newvtraj.size() - 1;
						count_traj++;
					}
					tlength = 0;
					newpoints.clear();
					newvtraj.clear();
				}
				if (check.first == "binaout") { // the last point in the region; cut the trajectory
					if (check.second->x >= rtx - rlength / 2 && check.second->x <= rtx + rlength / 2
						&& check.second->y >= rty - rwidth / 2 && check.second->y <= rty + rwidth / 2) {
						if (xyID.find(check.second->x) == xyID.end()) {
							v1 = pointList.size();
							check.second->id = v1;
							pointList.emplace(v1, check.second);
							map<double, VertexID> temp;
							temp.emplace(check.second->y, v1);
							xyID.emplace(check.second->x, temp);
						}
						else {
							map<double, VertexID> temp = xyID[check.second->x];
							if (temp.find(check.second->y) == temp.end()) {
								v1 = pointList.size();
								check.second->id = v1;
								pointList.emplace(v1, check.second);
								temp.emplace(check.second->y, v1);
								xyID.emplace(check.second->x, temp);
							}
							else {
								v1 = temp[check.second->y];
							}
						}
						newpoints.push_back(v1);
						newpoints.push_back(v2);
						newvtraj.push_back(check.second);
						newvtraj.push_back(vtraj[i + 1]);
						start = i;
						double le = measure_distance_point2point(pointList[v1], pointList[v2]);
						tlength = tlength + le;
						//tlength = tlength + 1;
					}
					else {
						newpoints.push_back(v2);
						newvtraj.push_back(vtraj[i + 1]);
						start = i + 1;
					}
					if (i == points.size() - 2 && newpoints.size() > 1 && tlength > 0) { //left the last two points in the previous list
						trajectory_t* trajectory = new trajectory_t;
						trajectory->tid = count_traj;
						trajectory->oldid = t.second->tid;
						trajectory->points = newpoints;
						trajectory->vertices = newvtraj;
						trajectory->length = tlength;
						end = i + 1;
						trajectory->start = start;
						trajectory->end = end;
						trajectories.emplace(trajectory->tid, trajectory);
						lengthList.push_back(tlength);
						sizeList.push_back(newvtraj.size() - 1);
						if (count_traj % 10000 == 0) {
							cout << "Read " << count_traj << " trajectories: done" << endl;
						}
						totallength = totallength + tlength;
						totalsize = totalsize + newvtraj.size() - 1;
						count_traj++;
						tlength = 0;
						newpoints.clear();
						newvtraj.clear();
					}
				}
			}
		}
	}
	else {
		trajectories = alltrajectories;
	}
	read_time.pause();
	outputfile << "Read " << count_traj << " trajectories: done" << endl;
	
	
	cout << "Read trajectory time: " << read_time.sum.count() << endl;
}