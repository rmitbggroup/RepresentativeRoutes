#ifndef _MAIN_H_
#define _MAIN_H_

#include "includes.h"
#include "performanceTimer.h"
#include "geographyCoordinateTransform.h"
#include "graph.h"
#include "grid.h"
#include "trajectory.h"

extern ofstream outputfile;
extern string data_dir;
extern string cover_dir;

extern double epsilon;
extern double budget;
extern double overlap;
extern int K;

extern double rtlat; //starting latitude of the region
extern double rtlng; //starting longitude of the region
extern double rlength; // the length of the region
extern double rwidth; // the width of the region 
extern bool ifregion;

#endif // _MAIN_H_
