#ifndef UTILS_H
#define UTILS_H

#include <vector>

using namespace std;

// For converting back and forth between radians and degrees.
double deg2rad(double x);
double rad2deg(double x);

double distance(double x1, double y1, double x2, double y2);
int ClosestWaypoint(double x, double y, vector<double> maps_x, vector<double> maps_y);
int NextWaypoint(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y);

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y);

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, vector<double> maps_s, vector<double> maps_x, vector<double> maps_y);

double getPosition(const vector<double>& trajectoryCoefficients, double time);
double getSpeed(const vector<double>& trajectoryCoefficients, double time);
double getAcceleration(const vector<double>& trajectoryCoefficients, double time);

#endif // UTILS_H
