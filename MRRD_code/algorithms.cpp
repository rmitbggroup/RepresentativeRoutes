#include "main.h"

#define inf 10e9


void connect_first(map<EdgeID, edge> edges, map<int, trajectory_t*> trajectories) {
	ofstream outfile;
	outfile.open(cover_dir + "/con.txt");
	int ssize = edges.size();
	int tsize = trajectories.size();
	map<EdgeID, vector<edge>> curroute2seg;// current best route: first segment: all segment
	map<int, int> curcover; // whether current trajectories are covered (T: true/false) 
	map<EdgeID, double> curroute2budget;
	double curscore = 0;  // current best representative score
	map<EdgeID, map<int, int>> segbytrajdist; // l: {T: true/false}
	long indextime;

	dist_time.clear();
	connect_time.clear();
	olap_time.clear();
	segment_time.clear();
	segment_time.resume();

	while (curroute2seg.size() < K) {
		int size = edges.size();
		vector<edge> curroute;
		double curbudget;
		map<int, int> possiblecover; // T: dist (single segment)

		if (curroute2seg.size() == 0) {
			edge top1seg;
			segment_time.resume();
			for (auto e : edges) {
				if (e.second.weight > budget) {
					continue;
				}
				EdgeID eid = e.first;
				int svert = e.second.s_vert;
				int evert = e.second.e_vert;
				vertex* ls = vertices[svert];
				vertex* le = vertices[evert];
				double tdist = 0;
				map<int, int> cover;
				map<int, int> seg2trajdist;
				for (auto t : trajectories) {
					int tid = t.first;
					dist_time.resume();
					double d = measure_dist_segment2trajectory(e.second, t.second);
					dist_time.pause();
					if (d < epsilon) {
						tdist = tdist + 1;
						cover.emplace(t.first, 1);
						seg2trajdist.emplace(tid, 1);
					}
				}
				double score = tdist / tsize;
				segbytrajdist.emplace(eid, seg2trajdist);
				if (score > curscore) {
					curscore = score;
					top1seg = e.second;
					curcover = cover;
				}
			}
			curroute.push_back(top1seg);
			curbudget = top1seg.weight;
			segment_time.pause();
			outfile << "The first segment: " << top1seg.id << " " << top1seg.s_vert << " " << top1seg.e_vert
				<< " length: " << top1seg.weight << endl;
			indextime = segment_time.sum.count();
			outfile << "calculate time: " << segment_time.sum.count() << endl;
		}
		else {
			edge possibleseg;
			double utility = 0;
			bool flag = false;
			int size = edges.size();
			map<int, int> cover;
			segment_time.resume();
			for (auto e : edges) {
				if (e.second.weight > budget) {
					continue;
				}
				EdgeID eid = e.first;
				map<int, int> seg2trajdist = segbytrajdist[eid];
				map<int, int> tempcover = curcover;
				double newd = 0;
				for (auto t : seg2trajdist) {
					int tid = t.first;
					if (curcover.find(tid) == curcover.end()) {
						newd = newd + 1;
						tempcover.emplace(tid, 1);
					}
				}
				if (newd > utility) {
					flag = true;
					utility = newd;
					possibleseg = e.second;
					possiblecover = seg2trajdist;
					cover = tempcover;
				}
			}
			curroute.push_back(possibleseg);
			curbudget = possibleseg.weight;
			curcover = cover;
			segment_time.pause();
		}

		bool flag = true;
		bool ifupdate = false;
		map<EdgeID, map<int, bool>> checkedseg; // checked line segments when constructing a route
		segment_time.resume();
		connect_time.resume();
		while (flag) {
			vector<EdgeID> left = curroute[0].leftconnect;
			vector<EdgeID> right = curroute[curroute.size() - 1].rightconnect;
			int ifaddroute = -1; //1 : add left 2 : add right
			double utility = 0;
			vector<edge> newroute;
			double newbudget;
			map<int, double> newmindist;
			map<int, int> newcover;
			for (int i = 0; i < left.size(); i++) {
				EdgeID eid = left[i];
				if (edges.find(eid) == edges.end()) continue;
				edge tempseg = edges[eid];
				if (curbudget + tempseg.weight > budget) { // over budget
				//if (curroute.size() + 1 > budget) {
					continue;
				}
				bool ifcircle = false; //if have circle
				for (int j = 0; j < curroute.size(); j++) {
					if (tempseg.s_vert == curroute[j].s_vert || tempseg.s_vert == curroute[j].e_vert) {
						ifcircle = true;
						break;
					}
				}
				if (ifcircle) continue;

				vector<edge> temproute;
				temproute.push_back(tempseg);
				for (int j = 0; j < curroute.size(); j++) {
					temproute.push_back(curroute[j]);
				}

				map<int, int> tempcover = curcover;
				double nowd = 0;
				map<int, int> seg2trajdist = segbytrajdist[eid];

				for (auto t : seg2trajdist) {
					int tid = t.first;
					if (curcover.find(tid) == curcover.end()) {
						nowd = nowd + 1;
						tempcover.emplace(tid, 1);
					}
				}
				if (nowd > utility) {
					utility = nowd;
					newroute = temproute;
					newbudget = curbudget + tempseg.weight;
					newcover = tempcover;
					ifaddroute = 1;
				}
			}
			for (int i = 0; i < right.size(); i++) {
				EdgeID eid = right[i];
				if (edges.find(eid) == edges.end()) continue;
				edge tempseg = edges[eid];
				if (curbudget + tempseg.weight > budget) { // over budget
				//if (curroute.size() + 1 > budget) {
					continue;
				}
				bool ifcircle = false; //if have circle
				for (int j = 0; j < curroute.size(); j++) {
					if (tempseg.e_vert == curroute[j].s_vert || tempseg.e_vert == curroute[j].e_vert) {
						ifcircle = true;
						break;
					}
				}
				if (ifcircle) continue;

				vector<edge> temproute = curroute;
				temproute.push_back(tempseg);

				map<int, int> tempcover = curcover;
				double nowd = 0;
				map<int, int> seg2trajdist = segbytrajdist[eid];

				for (auto t : seg2trajdist) {
					int tid = t.first;
					if (curcover.find(tid) == curcover.end()) {
						nowd = nowd + 1;
						tempcover.emplace(tid, 1);
					}
				}
				if (nowd > utility) {
					utility = nowd;
					newroute = temproute;
					newbudget = curbudget + tempseg.weight;
					newcover = tempcover;
					ifaddroute = 2;
				}
			}
			connect_time.pause();

			if (ifaddroute > 0) {
				curroute = newroute;
				curbudget = newbudget;
				curcover = newcover;
				ifupdate = true;
			}
			else {
				flag = false;
			}
		}
		connect_time.pause();
		segment_time.pause();

		segment_time.resume();
		curroute2seg.emplace(curroute[0].id, curroute);
		curroute2budget.emplace(curroute[0].id, curbudget);
		for (int i = 0; i < curroute.size(); i++) {
			edges.erase(curroute[i].id);
		}
		double tdist = curcover.size();
		curscore = tdist / tsize;
		segment_time.pause();
		outfile << "Begin from: " << curroute2seg.size() << " segment size " << size << endl;
		outfile << "Current route: " << endl;
		map<EdgeID, vector<edge>> routes = curroute2seg;
		int c = 1;
		for (auto r : routes) {
			vector<edge> route = r.second;
			outfile << c << ":" << endl;
			for (int i = 0; i < route.size(); i++) {
				outfile << route[i].id << ": [" << route[i].s_vert << ", " << route[i].e_vert << "] ";
			}
			outfile << endl;
			for (int i = 0; i < route.size() - 1; i++) {
				outfile << " " << to_string(vertices[route[i].s_vert]->lat) << ", " << to_string(vertices[route[i].s_vert]->lng) << " ";
			}
			outfile << " " << to_string(vertices[route[route.size() - 1].s_vert]->lat) << ", " << to_string(vertices[route[route.size() - 1].s_vert]->lng) << " ";
			outfile << " " << to_string(vertices[route[route.size() - 1].e_vert]->lat) << ", " << to_string(vertices[route[route.size() - 1].e_vert]->lng);
			outfile << endl;
			outfile << " route size: " << route.size() << " used budget: " << curroute2budget[r.first] << endl;
			c++;
		}
		double dist = curcover.size();
		double rep = dist / trajectories.size();
		outfile << "representative score: " << rep << " " << dist << endl;
		outfile << "distance time: " << dist_time.sum.count() << endl;
		outfile << "connect time: " << connect_time.sum.count() << endl;
		outfile << "overlap time: " << olap_time.sum.count() << endl;
	}
	outfile << "Calculate time: " << segment_time.sum.count() << endl;
	outfile << "Index time: " << indextime << endl;
	outfile << "Route time: " << segment_time.sum.count()-indextime << endl;
	outfile << "Representative route: " << endl;
	map<EdgeID, vector<edge>> routes = curroute2seg;
	int c = 1;
	for (auto r : routes) {
		vector<edge> route = r.second;
		outfile << c << ":" << endl;
		for (int i = 0; i < route.size(); i++) {
			outfile << route[i].id << ": [" << route[i].s_vert << ", " << route[i].e_vert << "] ";
		}
		outfile << endl;
		for (int i = 0; i < route.size() - 1; i++) {
			outfile << " " << to_string(vertices[route[i].s_vert]->lat) << ", " << to_string(vertices[route[i].s_vert]->lng) << " ";
		}
		outfile << " " << to_string(vertices[route[route.size() - 1].s_vert]->lat) << ", " << to_string(vertices[route[route.size() - 1].s_vert]->lng) << " ";
		outfile << " " << to_string(vertices[route[route.size() - 1].e_vert]->lat) << ", " << to_string(vertices[route[route.size() - 1].e_vert]->lng);
		outfile << endl;
		outfile << " route size: " << route.size() << " used budget: " << curroute2budget[r.first] << endl;
		c++;
	}
	double dist = curcover.size();
	double rep = dist / trajectories.size();
	outfile << "representative score: " << rep << " " << dist << endl;
	outfile << "distance time: " << dist_time.sum.count() << endl;
	outfile << "connect time: " << connect_time.sum.count() << endl;
	outfile.close();
}
